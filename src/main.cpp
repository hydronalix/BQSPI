#include "BQSPI.hpp"

void setup() {

  SPI.begin();
  

  pinMode(10, OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);

  digitalWrite(10,1);
  digitalWrite(4, 0);
  digitalWrite(5,0);

  delayMicroseconds(10000);

  
  Serial.begin(9600);
  Serial.setTimeout(1000);

	CommandSubcommands(BQ769x2_RESET);  // Resets the BQ769x2 registers
	delayMicroseconds(60000);

  BQ769x2_Init();
  delayMicroseconds(10000);

  CommandSubcommands(FET_ENABLE); // Enable the CHG and DSG FETs
	delayMicroseconds(10000);

  CommandSubcommands(SLEEP_DISABLE);
  delayMicroseconds(60000); delayMicroseconds(60000); delayMicroseconds(60000); delayMicroseconds(60000);
  
}

void loop() {
  
  AlarmBits = BQ769x2_ReadAlarmStatus();
	if (AlarmBits & 0x80) {  // Check if FULLSCAN is complete. If set, new measurements are available
    BQ769x2_ReadAllVoltages();
   // Pack_Current = BQ769x2_ReadCurrent();
    //Temperature[0] = BQ769x2_ReadTemperature(TS1Temperature);
    //Temperature[1] = BQ769x2_ReadTemperature(TS3Temperature);
		DirectCommands(AlarmStatus, 0x0080, W);  // Clear the FULLSCAN bit

    Serial.println("Cell Voltages:");
    for(int i = 0; i < 16;i++){
      Serial.print(i); Serial.print(". ");
      Serial.println(CellVoltage[i]);
    }

	}

  delay(1000);

}