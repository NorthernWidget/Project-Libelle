//Libelle_Driver_ShortWave.ino
//v0.0.0
#include "SlowSoftI2CMaster.h"
#include "WireS.h"
#include <EEPROM.h>
//Commands

#define CTRL 0x46  //Define location of onboard control/confiuration register (Schema 1 Page 2 Config byte; was 0x00, which is now the Page 0 schema byte)

//Firmware patch version: bump on any behavioural change visible to the
//library. The hardware version lives in Page 0 (EEPROM), written at
//provisioning; the firmware writes this constant into the served copy of
//Page 0 at 0x0A and recomputes the CRC there (NW-Device-Specification).
#define FW_FW_PATCH 1

//The stored pages are the top 64 bytes of EEPROM in bus order: Page 0
//(identity) at 0x1C0-0x1DF on the ATtiny841's 512-byte EEPROM, Page 1
//(calibration; Libelle has none) at 0x1E0-0x1FF. Page 0 is written once by
//NW-Provision and read at boot; Page 1 is served as zeros.
#define PAGE0_BASE   (E2END + 1 - 64)
#define REG_I2C_ADDR 0x1F
#define ADR_DEFAULT  0x4C  //Schema 1 'L' (UP orientation); used when Page 0 byte 0x1F is 0xFF (was 0x40)
#define ADR_DOWN_XOR 0x40  //DOWN orientation (solder jumper): the UP address XOR 0x40, so 0x4C -> 0x0C

//Page 2 Block 0 (NW-Device-Specification): universal status and control.
#define REG_STATUS   0x40
#define REG_CTRL     0x41
#define REG_COUNTER  0x42
#define REG_REQUEST  0x44  //Readings requested, uint16 LE, writable; Libelle has no chip power to hold, so it only accepts the write
#define REG_REPORT   0x47
#define BIT_READY    0x01
#define BIT_PANFAULT 0x80
#define BIT_TRIGGER  0x01
#define CHIP_VEML6075 0x02  //Control chip-select bit and status fault bit: chip 0 (UV)
#define CHIP_VEML6030 0x04  //chip 1 (visible light)
#define CHIP_ADS1115  0x08  //chip 2 (IR and thermistor)
#define BIT_SLEEP    0x80
#define FAULT_VEML6075_NOACK 0x01  //chip 0, kind 1: no acknowledge during the reading
#define FAULT_VEML6030_NOACK 0x21  //chip 1, kind 1
#define FAULT_ADS1115_NOACK  0x41  //chip 2, kind 1
#define NOTICE_UNIT_RESET    0xE6  //unit (7), kind 6: reset since the controller last wrote Control (a notice: no status bit)
#define NOTICE_UNIT_PAGE0    0xE3  //unit (7), kind 3: Page 0 CRC did not match (unprovisioned or corrupt)

#define CONF_CMD 0x00
#define ALS_CMD 0x04
#define WHITE_CMD 0x05

#define UVA_CMD 0x07
#define UVB_CMD 0x09
#define COMP1_CMD 0x0A
#define COMP2_CMD 0x0B

#define ADC_CONF 0x01
#define ADC_CONV 0x00
#define ADC0 0x4200
#define ADC1 0x5200
#define ADC2 0x6200
#define ADC3 0x7200

#define VIS_ADR 0x48
#define UV_ADR 0x10
#define ADC_ADR 0x49

#define READ 0x01
#define WRITE 0x00

#define BUF_LENGTH 64 //Length of I2C Buffer, verify with documentation 

#define LOW_LIM_VIS 10000  //Lower limit to the auto ranging of the VEML6030
#define HIGH_LIM_VIS 55000 //Upper limit to the auto ranging of the VEML6030

#define ADR_SEL_PIN 7 //Digital pin 7 is used to test which device address should be used
// #define ADR_ALT 0x41 //Alternative device address

//Global values for gain and int time of visable light sensor
uint8_t Gain = 0;
unsigned int IntTime = 0;
uint8_t GainValsVis[4] = {0b10, 0b11, 0b00, 0b01}; //Gain values for visible sensor
uint8_t GainsVis[4] = {1, 2, 8, 16}; //Gain multipliers for visable sensor 
uint8_t IntTimeValsVis[6] = {0b1100, 0b1000, 0b0000, 0b0001, 0b0010, 0b0011}; //Integration time values for visible sensor

//Compensation constants
float a = 1.92;
float b = 0.55;
float c = 2.46;
float d = 0.63;

volatile uint8_t ADR = ADR_DEFAULT; //I2C address: Page 0 byte 0x1F (EEPROM), or ADR_DEFAULT if unprogrammed; XOR ADR_DOWN_XOR when the jumper says DOWN

uint8_t Config = 0; //Global config value

uint8_t Reg[96] = {0}; //Initialize registers; 0x00-0x1F = Page 0 (identity), 0x20-0x3F = Page 1 (calibration: none; served from EEPROM as stored), 0x40-0x47 = Page 2 Block 0 (status/control), 0x48-0x5F = Page 2 sensor data
#define DATA_BASE 0x48 //First sensor data register (Page 2 Block 1)
#define DATA_LEN  22   //0x48-0x5D: the bytes a reading writes
uint8_t Staged[DATA_LEN] = {0}; //A reading is assembled here over ~2 s and copied into Reg with the counter, so a page read never sees half a reading
bool page0Valid = false; //Page 0 CRC matched what NW-Provision wrote
bool uvNoAck = false;  //VEML6075 did not acknowledge during the last reading
bool visNoAck = false; //VEML6030 did not acknowledge during the last reading
bool adcNoAck = false; //ADS1115 did not acknowledge during the last reading

//CRC-8/SMBUS (poly 0x07, init 0x00), the NW-Device-Specification reference.
uint8_t crc8smbus(const uint8_t* data, uint8_t len) {
	uint8_t crc = 0x00;
	for(uint8_t i = 0; i < len; i++) {
		crc ^= data[i];
		for(uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
	}
	return crc;
}

//Copy Page 0 from EEPROM into the served register array, check its CRC,
//then substitute this firmware's patch version at 0x0A and recompute the
//CRC of the served copy (EEPROM is left as provisioned).
void loadPage0() {
	for(uint8_t i = 0; i < 64; i++) Reg[i] = EEPROM.read(PAGE0_BASE + i); //The stored half, Page 0 and Page 1, byte for byte
	page0Valid = (crc8smbus(Reg, 0x1E) == Reg[0x1E]) && Reg[0x00] == 0x01;
	Reg[0x0A] = FW_FW_PATCH;
	Reg[0x1E] = crc8smbus(Reg, 0x1E);
}

//Registers a controller may write. Everything else is read-only and writes
//to it are ignored (NW-Device-Specification, Page 2 rules).
bool isWritable(uint8_t pos) {
	return pos == REG_CTRL || pos == CTRL || pos == REG_I2C_ADDR
	    || pos == REG_REQUEST || pos == REG_REQUEST + 1;
}
bool StartSample = true; //Flag used to start a new converstion, make a conversion on startup
// const unsigned int UpdateRate = 5; //Rate of update
const unsigned int UpdateRate[] = {5, 10, 60, 300}; //FIX with better numbers! 

SlowSoftI2CMaster si = SlowSoftI2CMaster(PIN_B2, PIN_A7, true);  //Initialize software I2C

volatile bool StopFlag = false; //Used to indicate a stop condition 
volatile uint8_t RegID = 0; //Used to denote which register will be read from
volatile bool RepeatedStart = false; //Used to show if the start was repeated or not

void setup() {
  Serial.begin(115200); //DEBUG!
  Serial.println("begin"); //DEBUG!
  Reg[CTRL] = 0x00; //Set Config to POR value
  pinMode(ADR_SEL_PIN, INPUT_PULLUP);
  pinMode(10, OUTPUT); //DEBUG!
  pinMode(9, OUTPUT); //DEBUG!
  digitalWrite(10, HIGH); //DEBUG!
  digitalWrite(9, LOW); //DEBUG!
  loadPage0();
  if(Reg[REG_I2C_ADDR] != 0xFF) ADR = Reg[REG_I2C_ADDR]; //Provisioned address; 0xFF = use default
  if(!digitalRead(ADR_SEL_PIN)) ADR ^= ADR_DOWN_XOR; //If solder jumper is bridged, DOWN orientation: secondary address //DEBUG!
  Reg[REG_STATUS] = 0; //Not ready: no reading yet
  Reg[REG_CTRL] = CHIP_VEML6075 | CHIP_VEML6030 | CHIP_ADS1115; //Power-up: every chip selected
  Reg[REG_REPORT] = page0Valid ? NOTICE_UNIT_RESET : NOTICE_UNIT_PAGE0; //Latched until the controller writes Control
  Wire.begin(ADR);  //Begin slave I2C
  // EEPROM.write(0, ADR);
  InitVEML(0x48); //Init Vis (VEML6030)
  InitVEML(0x10); //Init UV (VEML6075)
  InitADC(); //Init ADC (ADS1115)

  //Setup I2C slave
	Wire.onAddrReceive(addressEvent); // register event
	Wire.onRequest(requestEvent);     // register event
	Wire.onReceive(receiveEvent);
	Wire.onStop(stopEvent);

  si.i2c_init(); //Begin I2C master

  // AutoRange_Vis(); //Auto range for given light conditions
	digitalWrite(9, HIGH); //DEBUG!
	digitalWrite(10, LOW); //DEBUG!
}

void loop() {
	// static unsigned int Count = 0; //Counter to determine update rate
	// uint8_t Ctrl = Reg[CTRL]; //Store local value to improve efficiency
	uint8_t UpdateRateBits = Reg[CTRL] & 0x03; 
	static unsigned long Timeout = millis() % (UpdateRate[3]*1000); //Take mod with longest update rate 

	// digitalWrite(10, HIGH); //DEBUG!
	if(Reg[REG_CTRL] & BIT_TRIGGER) StartSample = true; //Controller trigger, in addition to the free-running timer
	if(StartSample == true) {
		//A reading begins: clear ready, take the chip selection, consume the trigger.
		Reg[REG_STATUS] &= ~BIT_READY;
		bool doUV = Reg[REG_CTRL] & CHIP_VEML6075;
		bool doVis = Reg[REG_CTRL] & CHIP_VEML6030;
		bool doADC = Reg[REG_CTRL] & CHIP_ADS1115;
		Reg[REG_CTRL] &= ~(BIT_TRIGGER | BIT_SLEEP); //trigger consumed; sleep not implemented
		uvNoAck = visNoAck = adcNoAck = false;

		// Config = Reg[CTRL]; //Update local register val
		//Read new values in
		if(doVis && BitRead(Reg[CTRL], 2) == 0) {  //Only auto range if configured in Ctrl register 
			AutoRange_Vis();  //Run auto range
			delay(800); //Wait for new sample
		}
		// digitalWrite(9, HIGH); //DEBUG!
		if(doVis) {
			SplitAndLoad(0x48, GetALS()); //Load ALS value (Schema 1 Block 1: uint16 raw VEML6030 counts)
			SplitAndLoad(0x4A, GetWhite()); //Load white value (Block 1: uint16 raw counts)
			SplitAndLoad(0x4C, GetLuxGain()); //Load lux multiplier (Block 1: uint16 auto-range scaler)
		}
		if(doUV) {
			SplitAndLoad(0x50, long(GetUV(0))); //Load UVA (Block 2: int32 compensated counts)
			SplitAndLoad(0x54, long(GetUV(1))); //Load UVB (Block 2: int32; the legacy map wrote this at 0x07 while the library read 0x06)
		}
		if(doADC) {
			SplitAndLoad(0x5A, GetADC(0)); //IR mid (Block 3: uint16 raw ADS1115 counts)
			SplitAndLoad(0x58, GetADC(1)); //IR short (Block 3)
			SplitAndLoad(0x5C, GetADC(2)); //Thermistor (Block 3)
		}

		//Reading complete: copy the staged data in, load status and fault, bump
		//the counter, set ready. Atomic so a controller's page read never
		//straddles the update or sees a reading half written. A chip
		//that did not acknowledge its address (NoteNoAck) gets its status bit
		//and the latched code; a data check per chip is not yet done.
		uint8_t status = BIT_READY;
		if(doUV && uvNoAck) { status |= CHIP_VEML6075; Reg[REG_REPORT] = FAULT_VEML6075_NOACK; }
		if(doVis && visNoAck) { status |= CHIP_VEML6030; Reg[REG_REPORT] = FAULT_VEML6030_NOACK; }
		if(doADC && adcNoAck) { status |= CHIP_ADS1115; Reg[REG_REPORT] = FAULT_ADS1115_NOACK; }
		if(status & 0x7E) status |= BIT_PANFAULT;
		uint16_t count = Reg[REG_COUNTER] | (Reg[REG_COUNTER + 1] << 8);
		count++;
		cli();
		memcpy(Reg + DATA_BASE, Staged, DATA_LEN); //The whole reading appears at once, with its counter
		Reg[REG_COUNTER] = count & 0xFF; Reg[REG_COUNTER + 1] = count >> 8;
		Reg[REG_STATUS] = status;
		sei();
		digitalWrite(9, LOW); //DEBUG!
		StartSample = false; //Clear flag when new values updated  
	}

	//Make sure there is not a protnetial logic problem when changing update rate!!!!!!
	if(millis() % (UpdateRate[3]*1000) - Timeout > UpdateRate[UpdateRateBits]*1000) {  
		StartSample = true; //Set flag if number of updates have rolled over 
		Timeout = millis() % (UpdateRate[3]*1000); //Restart timer
		// digitalWrite(10, LOW); //DEBUG!
	}

	if(BitRead(Reg[CTRL], 3) == 1) {  //If manual autorange is commanded
		AutoRange_Vis(); //Call autorange
		delay(800); //Wait for new data
		Reg[CTRL] &= 0xF7; //Clear auto range bit to inform user autorange is complete
	}

	// if(Reg[CTRL] != Config) {
	// 	Config = Reg[CTRL]; //Update local register 
	// 	Timeout = millis() % (UpdateRate[3]*1000); //Reset counter if control register changes
	// }
	delay(100);
}

uint8_t InitVEML(uint8_t Adr) 
{
	uint8_t CMD = 0; 
	if(Adr == 0x48) CMD = 0x13;  //DEBUG! Replace w/0x23 or 0x13 
	else CMD = 0;
    si.i2c_start((Adr << 1) | WRITE);
    si.i2c_write(0x00);
    si.i2c_write(0x00);
    si.i2c_write(CMD);
    si.i2c_stop(); 

    return 0; //Fix dummy
    //Setup gain and integration values manually! 
}

uint8_t InitADC() 
{
	//Setup ADC system 
	WriteWord_LE(ADC_ADR, ADC_CONF, ADC0); //Set to single shot mode with 
}

unsigned int GetADC(unsigned int Num)
{
	unsigned int ADC_Config = ADC0 | (Num << 12); //Use to select which ADC to get data from
	WriteWord_LE(ADC_ADR, ADC_CONF, ADC_Config); //Setup registers
	delay(300);  //Wait for next sample to be read
	return ReadWord_LE(ADC_ADR, ADC_CONV); //Read from register
}

float GetUV(uint8_t Sel) //Select A or B using Sel value (0 or 1) 
{
	float Comp1 = ReadWord(UV_ADR, COMP1_CMD);
	float Comp2 = ReadWord(UV_ADR, COMP2_CMD);

	float UV = 0;
	float UV_Comp = 0;
	if(Sel == 0) {
		UV = ReadWord(UV_ADR, UVA_CMD);
		UV_Comp = UV - a*Comp1 - b*Comp2;
		// Serial.println(UV);
	}

	if(Sel == 1) {
		UV = ReadWord(UV_ADR, UVB_CMD);
		UV_Comp = UV - c*Comp1 - d*Comp2;
	}

	return UV_Comp;
}

unsigned int GetALS() 
{
	return ReadWord(VIS_ADR, ALS_CMD);
}

unsigned int GetWhite() 
{
	return ReadWord(VIS_ADR, WHITE_CMD);
}

unsigned int GetLuxGain() 
{	//Add non-linear correction! 
	float GainTemp = float(Gain); //Hardcode max range for gain and int time 
	float IntTimeTemp = float(IntTime);  
	float Resolution = (1.8432/IntTimeTemp)/GainTemp;
	// unsigned int Resolution = (512/(IntTime/25.0))/(Gain/0.125);
	Serial.println(int(Resolution/0.0036)); //DEBUG!
	return int(Resolution/0.0036); //Return Lux scaler
}


uint8_t AutoRange_Vis() 
{	
	// WriteWord(VIS_ADR, CONF_CMD, (GainValsVis[0] << 11) | (IntTimeValsVis[0] << 6)); //Write min gain vals
	WriteWord(VIS_ADR, CONF_CMD, 0x1300); //DEBUG! Replace w/ 0x1300
	delay(800); //Wait for new sample
	unsigned int Val = GetALS();
	Serial.print("Val = "); Serial.println(Val); //DEBUG!
	// Serial.println(Val); //DEBUG!
	if(Val > HIGH_LIM_VIS) {
		Gain = GainsVis[0];  //Set global values
		IntTime = pow(2, 0); 
	}

	bool InRange = false; //Flag to test for range
	unsigned int ValTest = 0;
	uint8_t GainTemp = 0;  //Index of desired gain
	uint8_t IntTimeTemp = 0; //Index of desired integration time
	//DEBUG!!!!!!
	// while(!InRange && IntTimeTemp < 6) {
	// 	ValTest = (Val*int(GainsVis[GainTemp])*(1 << int(IntTimeTemp)));
	// 	Serial.print("Val = "); Serial.println(ValTest); //DEBUG!
	// 	Serial.print("D = "); Serial.print(GainsVis[GainTemp]); Serial.print("\t"); Serial.println(IntTimeTemp);
	// 	if(ValTest < LOW_LIM_VIS) {
	// 		GainTemp++;
	// 	}
	// 	if(GainTemp > 3 && IntTimeTemp < 5) {
	// 		GainTemp = 0;
	// 		IntTimeTemp++;
	// 	}

	// 	if(GainTemp == 3 && IntTimeTemp == 5) InRange = true; //Set if max value is reached 
	// 	if(ValTest > LOW_LIM_VIS && ValTest < HIGH_LIM_VIS) InRange = true; //Found correct gain and int vals
	// }
	GainTemp = 0; //DEBUG!
	IntTimeTemp = 0;  //DEBUG!
	//DEBUG!
	// Serial.print(GainValsVis[GainTemp], BIN); Serial.print("\t"); Serial.println(IntTimeValsVis[IntTimeTemp], BIN);  //DEBUG!
	// WriteWord(VIS_ADR, CONF_CMD, (GainValsVis[GainTemp] << 11) | (IntTimeValsVis[IntTimeTemp] << 6));  //Write new gain value
	// Serial.print((GainValsVis[GainTemp] << 11) | (IntTimeValsVis[IntTimeTemp] << 6), HEX); //DEBUG!
	Gain = GainsVis[GainTemp];  //Set global values
	IntTime = pow(2, IntTimeTemp); 
}
// float GetGain(uint8_t GainBits)
// {
// 	uint8_t X0 = ReadBit(GainBits, 1);
// 	uint8_t X1 = ReadBit(GainBits, 0);
// 	Gain = pow(2, X0 - 3*X1);
// 	return Gain; 
// }

// unsigned int Gain2Bits(float GainVal)
// {
// 	for(int i = 0; i < 4; i++) {  //Use linear search to avoid float math and increase speed
// 		if(GainVals[i] == GainVal) {
// 			return (i << 11); //if entries match, return bits
// 		}
// 	}
// 	return 0x1000; //Return gain of 1/8 if not a valid gain value 
// }

// unsigned int GetIntTime()
// {
// 	Config = ReadWord(CONF_CMD); //Update global config value
// 	int X2X1 = (Config >> 6) & 0x03; 
// 	int X3 = ReadBit(Config, 8);
// 	int X4 = ReadBit(Config, 9);
// 	IntTime = 100*pow(2, X2X1)/pow(2, X3 + X4);  //Do some ugly math to go from bit pattern to value
// 	return IntTime; 
// }

// unsigned int IntTime2Bits(unsigned int Time)
// {
// 	uint8_t X2X1 = (int)(log(Time/100)/log(2));
// 	uint8_t X4X3 = (int)(((Time % 100) / 25) + 2*(Time % 50)/25);
// 	return ((X4X3 << 2) | (X2X1)) << 6; 
// }

// uint8_t SetGain(unsigned int GainVal)
// {
// 	Config = ReadWord(CONF_CMD); //Update global config value
// 	// Serial.print("Config = "); //DEBUG!
// 	// Serial.print(Config); //DEBUG! 
// 	// Serial.print(" "); //DEBUG!
// 	// Serial.println((Config & 0xE7FF) | GainVal); //DEBUG!
// 	return WriteConfig((Config & 0xE7FF) | GainVal);
// }

// uint8_t AutoRange()  //Automatically finds maximum gain and resolution values for given irradiance
// {
// 	// PowerSaveOff(); //Turn power save off for fastest reading
// 	SetIntTime(IT25); //Set to minimum integration time
// 	SetGain(GAIN_1_8); //Set to minmum gain
// 	// Serial.print("UnRanged Result ="); Serial.println(ReadWord(CONF_CMD), HEX); //DEBUG!
// 	// PowerOn();
// 	// long StartTime = millis(); //DEBUG! 
// 	// while(millis() - StartTime < 50) {  //DEBUG!
// 	// 	Serial.println(ReadWord(INT_CMD), HEX); //DEBUG!
// 	// }  //DEBUG!
// 	delay(30); //Wait for new sample
// 	float TestLux = GetLux(); //Get new lux value
// 	unsigned long HighLux = 120796;  //Start at max value
// 	unsigned int NewIntTime = 25; //Default to min value
// 	float NewGainHigh = 0.125; //Default to min value
// 	float NewGainLow = 0.25; //Default to 2nd lowest value
// 	float NewGain = 0.125; //Default to min value
// 	// Serial.print("LuxTest = "); //DEBUG!
// 	// Serial.println(TestLux); //DEBUG!
// 	//Increment through lux ranges to find desired gain range
// 	if(TestLux < 236) {  //If Lux is too small to measure at max values (<1.8432) or in minimum range, simply set to highest gain and integration time
// 		NewIntTime = 800;
// 		NewGain = 2;
// 	}
// 	else {  //If lux is not outside low range, search for a value
// 		for(int i = 0; i < 6; i++) {
// 			if(TestLux < HighLux && TestLux >= HighLux/2.0) {
// 				NewIntTime = NewIntTime * ceil(pow(2, i));
// 				break; //breakout of for loop since result is found
// 			}
// 			else HighLux = ceil(HighLux/2.0); //If not found, go to next lux range
// 		}

// 		if(TestLux < HighLux * 0.0625) NewGain = 2; //If below the lowest max for integration range, set to max gain
// 		else {  //Otherwise search for a new value 
// 			for(int g = 1; g < 3; g++) {
// 				if(TestLux < HighLux * (0.125/NewGainHigh) && TestLux >= HighLux * (0.125/NewGainLow)) {
// 					NewGain = NewGainHigh;
// 					break; //Break loop once gain is found
// 				}
// 				else {
// 					NewGainHigh = NewGainLow;
// 					NewGainLow = GetGain(GainValBits[g + 1]);
// 				}
// 			}
// 		}
// 	}

// 	// Serial.print("AutoRange Vals = "); Serial.print(NewGain); Serial.print(" "); Serial.println(NewIntTime); //DEBUG!
// 	unsigned int GainBits = Gain2Bits(NewGain); //Convert new gain value
// 	unsigned int IntBits = IntTime2Bits(NewIntTime); //Convert to new integration time
// 	// Serial.print("Bits = "); Serial.print(GainBits, HEX); Serial.print(" "); Serial.println(IntBits, HEX); //DEBUG!

// 	SetGain(GainBits);
// 	SetIntTime(IntBits);
// 	// Serial.print("AutoRanged Result ="); Serial.println(ReadWord(CONF_CMD), HEX); //DEBUG!
// }

// float GetGain()
// {
// 	Config = ReadWord(CONF_CMD); //Update global config value
// 	// Serial.println(Config); //DEBUG!
// 	int X0 = ReadBit(Config, 11);
// 	int X1 = ReadBit(Config, 12);
// 	Gain = pow(2, X0 - 3*X1);
// 	return Gain; 
// }

// unsigned int GetIntTime()
// {
// 	Config = ReadWord(CONF_CMD); //Update global config value
// 	int X2X1 = (Config >> 6) & 0x03; 
// 	int X3 = ReadBit(Config, 8);
// 	int X4 = ReadBit(Config, 9);
// 	IntTime = 100*pow(2, X2X1)/pow(2, X3 + X4);  //Do some ugly math to go from bit pattern to value
// 	return IntTime; 
// }

// float GetLux() 
// {	//Add non-linear correction! 
// 	GetGain(); //Update global values
// 	GetIntTime(); 
// 	float Resolution = (1.8432/((float)IntTime/25.0))*(0.125/Gain);
// 	// Serial.print("TEST = "); //DEBUG!
// 	// Serial.print(Resolution); //DEBUG!
// 	// Serial.print(" "); //DEBUG!
// 	// Serial.print(IntTime); //DEBUG! 
// 	// Serial.print(" "); //DEBUG!
// 	// Serial.println(Gain); //DEBUG!
// 	return GetALS()*Resolution; //Return scaled Lux mesurment
// }

// uint8_t ReadBit(unsigned int Data, uint8_t Pos)
// {
// 	return (Data >> Pos) & 0x01; 
// }

bool BitRead(uint8_t Val, uint8_t Pos) //Read the bit value at the specified position
{
	return (Val >> Pos) & 0x01;
}

void NoteNoAck(uint8_t Adr) //A chip did not acknowledge its address: its fault bit for this reading
{
	if(Adr == UV_ADR) uvNoAck = true;
	if(Adr == VIS_ADR) visNoAck = true;
	if(Adr == ADC_ADR) adcNoAck = true;
}

uint8_t SendCommand(uint8_t Adr, uint8_t Command)
{
    if(!si.i2c_start((Adr << 1) | WRITE)) NoteNoAck(Adr);
    bool Error = si.i2c_write(Command);
    return 1; //DEBUG!
}

uint8_t WriteWord(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
	if(!si.i2c_start((Adr << 1) | WRITE)) NoteNoAck(Adr);
	si.i2c_write(Command); //Write Command value
	si.i2c_write(Data & 0xFF); //Write LSB
	uint8_t Error = si.i2c_write((Data >> 8) & 0xFF); //Write MSB
	si.i2c_stop();
	return Error;  //Invert error so that it will return 0 is works
}

uint8_t WriteWord_LE(uint8_t Adr, uint8_t Command, unsigned int Data)  //Writes value to 16 bit register
{
	if(!si.i2c_start((Adr << 1) | WRITE)) NoteNoAck(Adr);
	si.i2c_write(Command); //Write Command value
	si.i2c_write((Data >> 8) & 0xFF); //Write MSB
	si.i2c_write(Data & 0xFF); //Write LSB
	si.i2c_stop();
	// return Error;  //Invert error so that it will return 0 is works
}

uint8_t WriteConfig(uint8_t Adr, uint8_t NewConfig)
{
	if(!si.i2c_start((Adr << 1) | WRITE)) NoteNoAck(Adr);
	si.i2c_write(CONF_CMD);  //Write command code to Config register
	uint8_t Error = si.i2c_write(NewConfig);
	si.i2c_stop();
	if(Error == true) {
		Config = NewConfig; //Set global config if write was sucessful 
		return 0;
	}
	else return -1; //If write failed, return failure condition
}

int ReadByte(uint8_t Adr, uint8_t Command, uint8_t Pos) //Send command value, and high/low byte to read, returns desired byte
{
	bool Error = SendCommand(Adr, Command);
	si.i2c_rep_start((Adr << 1) | READ);
	uint8_t ValLow = si.i2c_read(false);
	uint8_t ValHigh = si.i2c_read(false);
	si.i2c_stop();
	Error = true; //DEBUG!
	if(Error == true) {
		if(Pos == 0) return ValLow;
		if(Pos == 1) return ValHigh;
	}
	else return -1; //Return error if read failed

}

int ReadWord(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
	bool Error = SendCommand(Adr, Command);
	Serial.print("Error = "); Serial.println(Error); //DEBUG!
	si.i2c_rep_start((Adr << 1) | READ);
	uint8_t ByteLow = si.i2c_read(false);  //Read in high and low bytes (big endian)
	uint8_t ByteHigh = si.i2c_read(false);
	si.i2c_stop();
	// if(Error == true) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
	// else return -1; //Return error if read failed
	return ((ByteHigh << 8) | ByteLow); //DEBUG!
}

int ReadWord_LE(uint8_t Adr, uint8_t Command)  //Send command value, returns entire 16 bit word
{
	bool Error = SendCommand(Adr, Command);
	si.i2c_stop();
	si.i2c_start((Adr << 1) | READ);
	uint8_t ByteHigh = si.i2c_read(false);  //Read in high and low bytes (big endian)
	uint8_t ByteLow = si.i2c_read(false);
	si.i2c_stop();
	// if(Error == true) return ((ByteHigh << 8) | ByteLow); //If read succeeded, return concatonated value
	// else return -1; //Return error if read failed
	return ((ByteHigh << 8) | ByteLow); //DEBUG!
}

void SplitAndLoad(uint8_t Pos, unsigned int Val) //Write 16 bits into the staged reading; Pos is the Page 2 register address
{
	uint8_t Len = sizeof(Val);
	for(int i = Pos; i < Pos + Len; i++) {
		Staged[i - DATA_BASE] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
	}
}

void SplitAndLoad(uint8_t Pos, long Val)  //Write 32 bits into the staged reading
{
	uint8_t Len = sizeof(Val);
	for(int i = Pos; i < Pos + Len; i++) {
		Staged[i - DATA_BASE] = (Val >> (i - Pos)*8) & 0xFF; //Pullout the next byte
	}
}

boolean addressEvent(uint16_t address, uint8_t count)
{
	RepeatedStart = (count > 0 ? true : false);
	return true; // send ACK to master
}

void requestEvent()
{	
	//Serve up to one full page from the requested register with auto-increment.
	//WireS clocks out only as many bytes as the controller asks for; the rest
	//of the buffer is discarded at the stop condition. Reads past the end of
	//the array wrap, so a controller never receives bytes from outside it.
	for(uint8_t i = 0; i < 32; i++) {
		uint16_t k = (uint16_t)RegID + i;
		Wire.write(k < sizeof(Reg) ? Reg[k] : 0x00); //Past the last page: zeros, never a wrap onto Page 0
	}
}

void receiveEvent(int DataLen) 
{
    //Write data to appropriate location
    if(DataLen == 2){
	    //Remove while loop?? 
	    while(Wire.available() < 2); //Only option for writing would be register address, and single 8 bit value
	    uint8_t Pos = Wire.read();
	    uint8_t Val = Wire.read();
	    if(!isWritable(Pos)) return; //Read-only register: ignore the write
	    Reg[Pos] = Val; //Set register value
	    if(Pos == REG_CTRL) Reg[REG_REPORT] = 0; //A control write acknowledges the report
	    if(Pos == REG_I2C_ADDR) EEPROM.update(PAGE0_BASE + REG_I2C_ADDR, Val); //Persist I2C address (compare-before-write); takes effect on next boot
	}

	if(DataLen == 1){
		RegID = Wire.read(); //Read in the register ID to be used for subsequent read
	}
}

void stopEvent() 
{
	StopFlag = true;
	//End comunication
}
