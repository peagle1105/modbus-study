#include <PZEM004Tv30.h>

#define RX_PZEM_004 16
#define TX_PZEM_004 17

PZEM004Tv30 pzem004(Serial1, RX_PZEM_004, TX_PZEM_004);

typedef struct
{
  float voltage;
  float current;
  float power;
  float energy;
  float frequency;
  float pf;
} _electricalParas;

_electricalParas station_2;

uint32_t lastMillis = 0;
uint16_t interval = 5000;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, RX_PZEM_004, TX_PZEM_004);


}

void loop() {

  uint32_t nowMillis = millis();

  if (nowMillis - lastMillis >= interval) {
    lastMillis = nowMillis;

    station_2.voltage = pzem004.voltage();
    station_2.current = pzem004.current();
    station_2.power = pzem004.power();
    station_2.energy = pzem004.energy();
    station_2.frequency = pzem004.frequency();
    station_2.pf = pzem004.pf();

    char station2_line1[30], station2_line2[30], station2_line3[30];
    sprintf(station2_line1, "U :%5.1fV  I :%6.3fA", station_2.voltage, station_2.current);
    sprintf(station2_line2, "P :%5.1fW  E :%5.2fWh", station_2.power, station_2.energy);
    sprintf(station2_line3, "F :%4.1fHz  PF:%5.2f", station_2.frequency, station_2.pf);

    Serial.println("STATION 2 ELECTRICAL PARAMETERS");
    Serial.println(station2_line1);
    Serial.println(station2_line2);
    Serial.println(station2_line3);
    Serial.println("\n==================================================\n");
  }
}
