#include <HardwareSerial.h>

#define RX 16
#define TX 17

HardwareSerial Pzem_serial(2);

typedef struct PZEM004{
  float voltage;
  float current;
  float power;
  float energy;
  float frequency;
  float pf;
  unsigned int alarm_status;
};
struct PZEM004 pzem;
const uint16_t REG_VOL = 0x0000;
const uint16_t REG_CURRENT_LOW = 0x0001;
const uint16_t REG_POWER_LOW = 0x0003;
const uint16_t REG_ENERGY_LOW = 0x0005;
const uint16_t REG_FREQUENCY = 0x0007;
const uint16_t REG_POWER_FACTOR = 0x0008;
const uint16_t REG_ALARM_STATUS = 0x0009;
const uint8_t NUM_REG = 10;

byte modbusRequest[8];
byte modbusRespond[30];

unsigned int calculateCRC16(byte *buf, int len);
bool SendCommand(byte slaveID, byte functionCode, uint16_t start, uint16_t len, byte *dataBuffer);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Pzem_serial.begin(9600, SERIAL_8N1, RX, TX);
  Serial.println("Pzem read manually");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (SendCommand(0x01, 0x03, REG_VOL, NUM_REG, modbusRespond)){
    int dataIndex = 2;
    //Voltage
    unsigned int rawVolt = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);
    pzem.voltage = (((float)rawVolt/20.0)-153.6)*2;
    dataIndex += 2;
    //Current
    unsigned int currentLow = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);
    dataIndex += 2;
    unsigned int currentHigh = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);
    dataIndex += 2;
    long rawCurrent = ((long)currentHigh << 16)|currentLow;
    pzem.current = (float)rawCurrent/1000.0;
    //power
    unsigned int powerLow = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);
    dataIndex += 2;
    unsigned int powerHigh = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);
    dataIndex += 2;
    long rawPower = ((long)powerHigh <<16) | powerLow;
    pzem.power = (float)rawPower/10.0;
    //energy
    unsigned int energyLow = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);
    dataIndex += 2;
    unsigned int energyHigh = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);
    dataIndex += 2;
    long rawEnergy = ((long)energyHigh<<16)|energyLow;
    pzem.energy = (float)rawEnergy/1.0;
    //frequency
    unsigned int rawFrequency = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);
    dataIndex += 2;
    pzem.frequency = (float)rawFrequency/10.0;
    //power factor
    unsigned int rawPf = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);
    dataIndex += 2;
    pzem.pf = (float)rawPf/100.0;
    //alarm status
    pzem.alarm_status = (unsigned int)((modbusRespond[dataIndex]<<8)|modbusRespond[dataIndex+1]);

    //Display
    Serial.print("Voltage: "); Serial.print(pzem.voltage); Serial.println(" V");
    Serial.print("Current: "); Serial.print(pzem.current); Serial.println(" A");
    Serial.print("Power: "); Serial.print(pzem.power); Serial.println(" W");
    Serial.print("Energy: "); Serial.print(pzem.energy); Serial.println(" Wh");
    Serial.print("Frequency: "); Serial.print(pzem.frequency); Serial.println(" Hz");
    Serial.print("Power Factor: "); Serial.print(pzem.pf); Serial.println("");
    Serial.print("Alarm Status: ");
    if (pzem.alarm_status == 0xFFFF) {
        Serial.println("Alarm!");
    } else if (pzem.alarm_status == 0x0000) {
        Serial.println("No Alarm.");
    } else {
        Serial.print("Unknown (0x"); Serial.print(pzem.alarm_status, HEX); Serial.println(")");
    }
  }
  delay(2000);
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
bool SendCommand(byte slaveID, byte functionCode, uint16_t start, uint16_t len, byte *dataBuffer){
  modbusRequest[0] = slaveID;
  modbusRequest[1] = functionCode;
  modbusRequest[2] = highByte(start);
  modbusRequest[3] = lowByte(start);
  modbusRequest[4] = highByte(len);
  modbusRequest[5] = lowByte(len);

  unsigned int crc16 = calculateCRC16(modbusRequest, 6);
  modbusRequest[6] = lowByte(crc16);
  modbusRequest[7] = highByte(crc16);

  while(Pzem_serial.available()) Pzem_serial.read();
  Pzem_serial.write(modbusRequest, 8);

  unsigned long startTime = millis();
  int byteRead = 0;
  int RespondByte = 1 + 1 + 1 + 2*len + 2;

  while(millis() - startTime <= 500){
    if(Pzem_serial.available()){
      dataBuffer[byteRead] = Pzem_serial.read();
      if(byteRead >= RespondByte) break;
      if(byteRead >= 3 && byteRead >= dataBuffer[2] + 5) break;
      byteRead += 1;
    }
  }
  if(byteRead == 0){
    Serial.println("No data response");
    return false;
  }
  else if(byteRead < 5){
    Serial.println("No CRC check");
    return false;
  }
  else{
    unsigned int recievedCRC = (unsigned int)dataBuffer[byteRead - 2] << 8 | dataBuffer[byteRead -1];
    unsigned int calculatedCRC = calculateCRC16(dataBuffer, byteRead - 2);
    if(recievedCRC != calculatedCRC){
      Serial.println("CRC does not match");
      return false;
    }
    else if (dataBuffer[1] == (functionCode | 0x80)){
      Serial.print("Error: "); Serial.println(dataBuffer[1], HEX);
      return false;
    }
    else if (dataBuffer[2] != (len * 2)) {
      Serial.print("Unexpected number of data bytes in response. Expected ");
      Serial.print(len * 2);
      Serial.print(", got ");
      Serial.println(dataBuffer[2]);
      return false;
    }
    else return true;
  }
}






