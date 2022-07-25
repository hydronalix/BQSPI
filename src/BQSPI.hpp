#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <sstream>
#include "BQ769x2Header.h"

#define DEV_ADDR  0x10  // BQ769x2 address is 0x10 including R/W bit or 0x8 as 7-bit address
#define CRC_Mode 0  // 0 for disabled, 1 for enabled
#define MAX_BUFFER_SIZE 10
#define R 0 // Read; Used in DirectCommands and Subcommands functions
#define W 1 // Write; Used in DirectCommands and Subcommands functions
#define W2 2 // Write data with two bytes; Used in Subcommands function

uint8_t RX_data [2] = {0x00, 0x00};
uint8_t rxdata [2];
uint16_t CellVoltage [16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
float Temperature [3] = {0,0,0};
uint16_t Stack_Voltage = 0x00;
uint16_t Pack_Voltage = 0x00;
uint16_t LD_Voltage = 0x00;
uint16_t Pack_Current = 0x00;
uint16_t AlarmBits = 0x00;

int getCellVoltage(int cell,SPIClass SPI){ //function that returns cell voltage in mv 
//(more info page 15 of Software Developemen Guide

  int BUFF[3] = {}; //The write packet 
  int low, high; //Both of the voltage bytes from BQchip which will get concatenated
  int command = 0x14 + cell - 1;

    while(BUFF[0] != command){ //sending same packet until slave echoes command
        BUFF[0] = SPI.transfer(command);
        BUFF[1] = SPI.transfer(0xFF);
        BUFF[2] = SPI.transfer(0xF0);  //TODO create CRC8 func and pass into transfer func
    }
    
    low = BUFF[1]; //receiving lower byte of voltage 

    for(int i = 0; i < 3; i++){ //claering out packet buffer
        BUFF[i] = 0;
    }

    while(BUFF[0] != command + 1){ //calling next command
        BUFF[0] = SPI.transfer(command + 1);
        BUFF[1] = SPI.transfer(0xFF);
        BUFF[2] = SPI.transfer(0xE5);   
    }

    high = BUFF[1]; //receiving high byte

    return (high << 8 | low); //returning concatenated voltage bits 


}
void delayUS(uint32_t usec){
    delay(usec / 1000);
}
unsigned char CRC8(unsigned char *ptr, unsigned char len)
//Calculates CRC8 for passed bytes.
{
	unsigned char i;
	unsigned char crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= 0x107;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= 0x107;
		}
		ptr++;
	}
	return(crc);
}
const char* intTohex(int i){
    std::ostringstream ss;
    ss << "0x" << std::hex << i;
    std::string result = ss.str();
 
    //std::cout << result << std::endl;  
    
    return result.c_str();
}
void SPI_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
	// SPI Write. Includes retries in case HFO has not started or if wait time is needed. See BQ76952 Software Development Guide for examples
  uint8_t addr; 
  uint8_t TX_Buffer [MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned int i;
	unsigned int match;
	unsigned int retries = 10;
    
	match = 0;
  addr = 0x80 | reg_addr;
    
  for(i=0; i<count; i++) {
		TX_Buffer[0] = addr;
		TX_Buffer[1] = reg_data[i];
		
        SPI.beginTransaction(SPISettings(2000000,MSBFIRST,SPI_MODE0));
        digitalWrite(10, 0);
        rxdata[0] = SPI.transfer(TX_Buffer[0]);
        rxdata[1] = SPI.transfer(TX_Buffer[1]);
        digitalWrite(10, 1);
        SPI.endTransaction();

		while ((match == 0) & (retries > 0)) {
			delayMicroseconds(500);

            SPI.beginTransaction(SPISettings(2000000,MSBFIRST,SPI_MODE0));
            digitalWrite(10, 0);
            rxdata[0] = SPI.transfer(TX_Buffer[0]);
            rxdata[1] = SPI.transfer(TX_Buffer[1]);
            digitalWrite(10, 1);
            SPI.endTransaction();

			if ((rxdata[0] == addr) & (rxdata[1] == reg_data[i]))
				match = 1;
			retries --;
		}    
    match = 0;
    addr += 1;
		delayMicroseconds(500);
  }
}
void SPI_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count) {
	// SPI Read. Includes retries in case HFO has not started or if wait time is needed. See BQ76952 Software Development Guide for examples
  uint8_t addr; 
  uint8_t TX_Buffer [MAX_BUFFER_SIZE] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  unsigned int i;
  unsigned int match;
  unsigned int retries = 10;
    
  match = 0;
  addr = reg_addr;
    
  for(i=0; i<count; i++) {
		TX_Buffer[0] = addr;
		TX_Buffer[1] = 0xFF;

        SPI.beginTransaction(SPISettings(2000000,MSBFIRST,SPI_MODE0));
        digitalWrite(10, 0);
        rxdata[0] = SPI.transfer(TX_Buffer[0]);
        rxdata[1] = SPI.transfer(TX_Buffer[1]);		
        digitalWrite(10, 1);
        SPI.endTransaction();

		while ((match == 0) & (retries > 0)) {
			delayMicroseconds(500);

            SPI.beginTransaction(SPISettings(2000000,MSBFIRST,SPI_MODE0));
		    digitalWrite(10, 0);
            rxdata[0] = SPI.transfer(TX_Buffer[0]);
            rxdata[1] = SPI.transfer(TX_Buffer[1]);		
            digitalWrite(10, 1);
            SPI.endTransaction();

			if (rxdata[0] == addr) {
				match = 1;
				reg_data[i] = rxdata[1];
			}
			retries --;
		}    
    match = 0;
    addr += 1;
		delayMicroseconds(500);
  }
}
void CommandSubcommands(uint16_t command) //For Command only Subcommands
{	//For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively
	
	uint8_t TX_Reg[2] = {0x00, 0x00};

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	SPI_WriteReg(0x3E,TX_Reg,2); 
	delayMicroseconds(2000);
}
void DirectCommands(uint8_t command, uint16_t data, uint8_t type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands
{	//type: R = read, W = write
	uint8_t TX_data[2] = {0x00, 0x00};

	//little endian format
	TX_data[0] = data & 0xff;
	TX_data[1] = (data >> 8) & 0xff;

	if (type == R) {//Read
		SPI_ReadReg(command, RX_data, 2); //RX_data is a global variable
		delayMicroseconds(2000);
	}
	if (type == W) {//write
    //Control_status, alarm_status, alarm_enable all 2 bytes long
		SPI_WriteReg(command,TX_data,2);
		delayMicroseconds(2000);
	}
}
uint16_t BQ769x2_ReadVoltage(uint8_t command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
	//RX_data is global var
	DirectCommands(command, 0x00, R);
	if(command >= Cell1Voltage && command <= Cell16Voltage) {//Cells 1 through 16 (0x14 to 0x32)
		return (RX_data[1]*256 + RX_data[0]); //voltage is reported in mV
	}
	else {//stack, Pack, LD
		return 10 * (RX_data[1]*256 + RX_data[0]); //voltage is reported in 0.01V units
	}

}
void BQ769x2_ReadAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
  int cellvoltageholder = Cell1Voltage; //Cell1Voltage is 0x14
  for (int x = 0; x < 16; x++){//Reads all cell voltages
    CellVoltage[x] = BQ769x2_ReadVoltage(cellvoltageholder);
    cellvoltageholder = cellvoltageholder + 2;
  }
  Stack_Voltage = BQ769x2_ReadVoltage(StackVoltage);
  Pack_Voltage = BQ769x2_ReadVoltage(PACKPinVoltage);
  LD_Voltage = BQ769x2_ReadVoltage(LDPinVoltage);
}
unsigned char Checksum(unsigned char *ptr, unsigned char len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.	
{
	unsigned char i;
	unsigned char checksum = 0;

	for(i=0; i<len; i++)
		checksum += ptr[i];

	checksum = 0xff & ~checksum;

	return(checksum);
}
void BQ769x2_SetRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
{
	uint8_t TX_Buffer[2] = {0x00, 0x00};
	uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	//TX_RegData in little endian format
	TX_RegData[0] = reg_addr & 0xff; 
	TX_RegData[1] = (reg_addr >> 8) & 0xff;
	TX_RegData[2] = reg_data & 0xff; //1st byte of data

	switch(datalen)
    {
		case 1: //1 byte datalength
      		SPI_WriteReg(0x3E, TX_RegData, 3);
			delayMicroseconds(2000);
			TX_Buffer[0] = Checksum(TX_RegData, 3); 
			TX_Buffer[1] = 0x05; //combined length of register address and data
      		SPI_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
			delayMicroseconds(2000);
			break;
		case 2: //2 byte datalength
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			SPI_WriteReg(0x3E, TX_RegData, 4);
			delayMicroseconds(2000);
			TX_Buffer[0] = Checksum(TX_RegData, 4); 
			TX_Buffer[1] = 0x06; //combined length of register address and data
      		SPI_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
			delayMicroseconds(2000);
			break;
		case 4: //4 byte datalength, Only used for CCGain and Capacity Gain
			TX_RegData[3] = (reg_data >> 8) & 0xff;
			TX_RegData[4] = (reg_data >> 16) & 0xff;
			TX_RegData[5] = (reg_data >> 24) & 0xff;
			SPI_WriteReg(0x3E, TX_RegData, 6);
			delayMicroseconds(2000);
			TX_Buffer[0] = Checksum(TX_RegData, 6); 
			TX_Buffer[1] = 0x08; //combined length of register address and data
      		SPI_WriteReg(0x60, TX_Buffer, 2); // Write the checksum and length
			delayMicroseconds(2000);
			break;
    }
}

void BQ769x2_Init() {
	// Configures all parameters in device RAM

	// Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
	// See TRM for full description of CONFIG_UPDATE mode
	CommandSubcommands(SET_CFGUPDATE);

	// After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
	// programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
	// An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
	// a full description of the register and the bits will pop up on the screen.

	// 'Power Config' - 0x9234 = 0x2D80
	// Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
  	// Set wake speed bits to 00 for best performance
	BQ769x2_SetRegister(PowerConfig, 0x2D80, 2);

	// 'REG0 Config' - set REG0_EN bit to enable pre-regulator
	BQ769x2_SetRegister(REG0Config, 0x01, 1);

	// 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
	BQ769x2_SetRegister(REG12Config, 0x0D, 1);

	// Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
	BQ769x2_SetRegister(DFETOFFPinConfig, 0x42, 1);

	// Set up ALERT Pin - 0x92FC = 0x2A
	// This configures the ALERT pin to drive high (REG1 voltage) when enabled.
	// The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
	BQ769x2_SetRegister(ALERTPinConfig, 0x2A, 1);

	// Set TS1 to measure Cell Temperature - 0x92FD = 0x07
	BQ769x2_SetRegister(TS1Config, 0x07, 1);

	// Set TS3 to measure FET Temperature - 0x92FF = 0x0F
	BQ769x2_SetRegister(TS3Config, 0x0F, 1);

	// Set HDQ to measure Cell Temperature - 0x9300 = 0x07
	BQ769x2_SetRegister(HDQPinConfig, 0x00, 1);   // No thermistor installed on EVM HDQ pin, so set to 0x00

	// 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
	BQ769x2_SetRegister(VCellMode, 0x00FF, 2); // did this

	// Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
	// Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
	// COV (over-voltage), CUV (under-voltage)
	BQ769x2_SetRegister(EnabledProtectionsA, 0xBC, 1);

	// Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
	// Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
	// OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
	BQ769x2_SetRegister(EnabledProtectionsB, 0xF7, 1);

	// 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
	BQ769x2_SetRegister(DefaultAlarmMask, 0xF882, 2);

	// Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
	// Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
	BQ769x2_SetRegister(BalancingConfiguration, 0x03, 1);

	// Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
	// CUV Threshold is this value multiplied by 50.6mV
	BQ769x2_SetRegister(CUVThreshold, 0x31, 1);

	// Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
	// COV Threshold is this value multiplied by 50.6mV
	BQ769x2_SetRegister(COVThreshold, 0x55, 1);

	// Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
	BQ769x2_SetRegister(OCCThreshold, 0x05, 1);

	// Set up OCD1 Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
	BQ769x2_SetRegister(OCD1Threshold, 0x0A, 1);

	// Set up SCD Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
	BQ769x2_SetRegister(SCDThreshold, 0x05, 1);

	// Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 \B5s; min value of 1    
	BQ769x2_SetRegister(SCDDelay, 0x03, 1);

	// Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
	// If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
	BQ769x2_SetRegister(SCDLLatchLimit, 0x01, 1);

	// Exit CONFIGUPDATE mode  - Subcommand 0x0092
	CommandSubcommands(EXIT_CFGUPDATE);
}
float BQ769x2_ReadTemperature(uint8_t command) 
{
	DirectCommands(command, 0x00, R);
	//RX_data is a global var
	return (0.1 * (float)(RX_data[1]*256 + RX_data[0])) - 273.15;  // converts from 0.1K to Celcius
}
uint16_t BQ769x2_ReadAlarmStatus() { 
	// Read this register to find out why the ALERT pin was asserted
	DirectCommands(AlarmStatus, 0x00, R);
	return (RX_data[1]*256 + RX_data[0]);
}
