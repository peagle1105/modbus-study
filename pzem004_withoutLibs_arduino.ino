#include <SoftwareSerial.h>

#define RX 10
#define TX 11
SoftwareSerial pzemSerial(RX, TX);

typedef struct PZEM_DATA{
  unsigned int voltageRaw;     
  unsigned int currentLow;     
  unsigned int currentHigh;    
  unsigned int powerLow;       
  unsigned int powerHigh;     
  unsigned int energyLow;      
  unsigned int energyHigh;    
  unsigned int frequencyRaw;   
  unsigned int powerFactorRaw; 
  unsigned int alarmStatus; 
};

struct PZEM_DATA currentPZEMData;

byte modbusRequest[8];
byte modbusResponse[30];

unsigned int calculateCRC16(byte *buf, int len);
unsigned int floatToRaw16Bit(float value, float resolution);
void floatToRaw32Bit(float value, float resolution, unsigned int &low16, unsigned int &high16);
void getSensorValuesFromUser();
void buildModbusResponse(int startAddress, int numRegisters);
void sendErrorResponse(byte address, byte functionCode, byte errorCode);
void handleModbusRequest();
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pzemSerial.begin(9600);
  while(pzemSerial.available()) pzemSerial.read();
  getSensorValuesFromUser();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(pzemSerial.available() >= 8){
    handleModbusRequest();
  }
  if (Serial.available()) {
    getSensorValuesFromUser();
  }
}

unsigned int calculateCRC16(byte *buf, int len) {
  unsigned int crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (unsigned int)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

unsigned int floatToRaw16Bit(float value, float resolution) {
  return (unsigned int)(value * resolution);
}

void floatToRaw32Bit(float value, float resolution, unsigned int &low16, unsigned int &high16) {
  long rawValue = (long)(value * resolution);
  low16 = (unsigned int)(rawValue & 0xFFFF);      
  high16 = (unsigned int)((rawValue >> 16) & 0xFFFF); 
}

void getSensorValuesFromUser() {
  float tempFloat;
  long tempLong;
  int tempInt;

  Serial.println("\n--- Enter New PZEM Values ---");

  Serial.print("Voltage (V, e.g., 220.5): ");
  while (Serial.available() == 0) delay(10);
  tempFloat = Serial.parseFloat();
  while (Serial.available()) Serial.read();
  currentPZEMData.voltageRaw = floatToRaw16Bit(tempFloat, 10.0);
  Serial.print("Raw Voltage: "); Serial.println(currentPZEMData.voltageRaw);

  Serial.print("Current (A, e.g., 1.234): ");
  while (Serial.available() == 0) delay(10);
  tempFloat = Serial.parseFloat();
  while (Serial.available()) Serial.read();
  floatToRaw32Bit(tempFloat, 1000.0, currentPZEMData.currentLow, currentPZEMData.currentHigh); // Resolution 0.001A
  Serial.print("Raw Current (High:Low): "); Serial.print(currentPZEMData.currentHigh, HEX); Serial.print(":"); Serial.println(currentPZEMData.currentLow, HEX);

  Serial.print("Power (W, e.g., 500.5): ");
  while (Serial.available() == 0) delay(10);
  tempFloat = Serial.parseFloat();
  while (Serial.available()) Serial.read();
  floatToRaw32Bit(tempFloat, 10.0, currentPZEMData.powerLow, currentPZEMData.powerHigh); // Resolution 0.1W
  Serial.print("Raw Power (High:Low): "); Serial.print(currentPZEMData.powerHigh, HEX); Serial.print(":"); Serial.println(currentPZEMData.powerLow, HEX);

  Serial.print("Energy (Wh, e.g., 123456): ");
  while (Serial.available() == 0) delay(10);
  tempFloat = Serial.parseFloat();
  // THÊM DÒNG NÀY ĐỂ XÓA BUFFER:
  while (Serial.available()) Serial.read();
  floatToRaw32Bit(tempFloat, 1.0, currentPZEMData.energyLow, currentPZEMData.energyHigh); // Resolution 1Wh
  Serial.print("Raw Energy (High:Low): "); Serial.print(currentPZEMData.energyHigh, HEX); Serial.print(":"); Serial.println(currentPZEMData.energyLow, HEX);

  Serial.print("Frequency (Hz, e.g., 50.0): ");
  while (Serial.available() == 0) delay(10);
  tempFloat = Serial.parseFloat();
  while (Serial.available()) Serial.read();
  currentPZEMData.frequencyRaw = floatToRaw16Bit(tempFloat, 10.0); // Resolution 0.1Hz
  Serial.print("Raw Frequency: "); Serial.println(currentPZEMData.frequencyRaw);

  Serial.print("Power Factor (e.g., 0.95): ");
  while (Serial.available() == 0) delay(10);
  tempFloat = Serial.parseFloat();
  while (Serial.available()) Serial.read();
  currentPZEMData.powerFactorRaw = floatToRaw16Bit(tempFloat, 100.0); // Resolution 0.01
  Serial.print("Raw Power Factor: "); Serial.println(currentPZEMData.powerFactorRaw);

  Serial.print("Alarm Status (0 for no alarm, 1 for alarm): ");
  while (Serial.available() == 0) delay(10);
  tempInt = Serial.parseInt();
  while (Serial.available()) Serial.read();
  if (tempInt == 1) {
    currentPZEMData.alarmStatus = 0xFFFF;
  } else {
    currentPZEMData.alarmStatus = 0x0000;
  }
  Serial.print("Raw Alarm Status: 0x"); Serial.println(currentPZEMData.alarmStatus, HEX);

  while (Serial.available()) Serial.read();

  Serial.println("Values updated. Waiting for Modbus request...");
}

void buildModbusResponse(int startAddress, int numRegisters) {
  int responseIndex = 0;
  modbusResponse[responseIndex++] = 0x01;
  modbusResponse[responseIndex++] = 0x03;

  unsigned int registers[10];

  registers[0] = currentPZEMData.voltageRaw;
  registers[1] = currentPZEMData.currentLow;
  registers[2] = currentPZEMData.currentHigh;
  registers[3] = currentPZEMData.powerLow;
  registers[4] = currentPZEMData.powerHigh;
  registers[5] = currentPZEMData.energyLow;
  registers[6] = currentPZEMData.energyHigh;
  registers[7] = currentPZEMData.frequencyRaw;
  registers[8] = currentPZEMData.powerFactorRaw;
  registers[9] = currentPZEMData.alarmStatus;

  int dataBytesCount = 0;
  for (int i = 0; i < numRegisters; i++) {
    int currentRegAddress = startAddress + i;
    if (currentRegAddress >= 0 && currentRegAddress <= 9) {
      modbusResponse[responseIndex++] = highByte(registers[currentRegAddress]);
      modbusResponse[responseIndex++] = lowByte(registers[currentRegAddress]);
      dataBytesCount += 2;
    } else {
      sendErrorResponse(0x01, 0x03, 0x02);
      Serial.print("Modbus Response: Illegal Data Address (0x");
      Serial.print(currentRegAddress, HEX); Serial.println(")");
      return;
    }
  }

  modbusResponse[2] = dataBytesCount;

  unsigned int crc = calculateCRC16(modbusResponse, responseIndex);
  modbusResponse[responseIndex++] = highByte(crc);
  modbusResponse[responseIndex++] = lowByte(crc);

  pzemSerial.write(modbusResponse, responseIndex);

  Serial.print("Modbus Response Sent ("); Serial.print(responseIndex); Serial.println(" bytes).");
  Serial.print("Response Bytes: ");
  for (int i = 0; i < responseIndex; i++) {
    if (modbusResponse[i] < 0x10) Serial.print("0");
    Serial.print(modbusResponse[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

void sendErrorResponse(byte address, byte functionCode, byte errorCode) {
  byte errorResponse[5];
  errorResponse[0] = address;
  errorResponse[1] = functionCode | 0x80;
  errorResponse[2] = errorCode;

  unsigned int crc = calculateCRC16(errorResponse, 3);
  errorResponse[3] = highByte(crc);
  errorResponse[4] = lowByte(crc);

  pzemSerial.write(errorResponse, 5);
}
void handleModbusRequest(){
  int byteRead = 0;
  unsigned long startTime = millis();
  while(byteRead<8 && (millis() - startTime<100)){
    if (pzemSerial.available()){
      modbusRequest[byteRead] = pzemSerial.read();
      byteRead += 1;
    }
  }
  if (byteRead == 8){
    unsigned int recievedCRC = (unsigned int)modbusRequest[7]<<8 | modbusRequest[6];
    unsigned int calculatedCRC = calculateCRC16(modbusRequest, 6);

    if(recievedCRC != calculatedCRC){
      Serial.println("Error: CRC does not match");
      return;
    }
    byte address = modbusRequest[0];
    byte functionCode = modbusRequest[1];
    uint16_t start = (modbusRequest[2] << 8)|modbusRequest[3];
    uint16_t len = (modbusRequest[4] << 8) | modbusRequest[5];

    if(address == 0x01 && functionCode == 0x03){
      buildModbusResponse(start, len);
    }
    else{
      sendErrorResponse(address, functionCode, 0x01);
      Serial.println("Modbus Request: Invalid Address or Function Code.");
    }
  }
}