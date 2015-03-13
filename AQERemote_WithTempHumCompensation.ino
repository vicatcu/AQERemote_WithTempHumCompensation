#include <Wire.h>
#include <EggBus.h>
#include <AQERF_Remote.h>
#include <RF12.h>
#include <NanodeMAC.h>
#include <SoftReset.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

#define FIRMWARE_REVISION 0x22

// each coefficient holds a float
#define MAGIC_NUMBER         (0x5A)
#define NO2_COEFF_VALID      (252)
#define CO_COEFF_VALID       (253)
#define O3_COEFF_VALID       (254)
#define NO2_C_COEFF_ADDR     (256)
#define NO2_RS_COEFF_ADDR    (260)
#define NO2_RS2_COEFF_ADDR   (264)
#define NO2_RS3_COEFF_ADDR   (268)
#define CO_C_COEFF_ADDR      (272)
#define CO_RS_COEFF_ADDR     (276)
#define CO_RS2_COEFF_ADDR    (280)
#define CO_RS3_COEFF_ADDR    (284)
#define O3_C_COEFF_ADDR      (288)
#define O3_RS_COEFF_ADDR     (292)
#define O3_RS2_COEFF_ADDR    (296)
#define O3_RS3_COEFF_ADDR    (300)
#define NO2_TEMP_COEFF_ADDR  (304)
#define NO2_HUM_COEFF_ADDR   (308)
#define CO_TEMP_COEFF_ADDR   (312)
#define CO_HUM_COEFF_ADDR    (316)
#define O3_TEMP_COEFF_ADDR   (320)
#define O3_HUM_COEFF_ADDR    (324)
#define DUST_COEFF_VALID     (328)
#define DUST_C_COEFF_ADDR    (332)
#define DUST_RS_COEFF_ADDR   (336)
#define DUST_RS2_COEFF_ADDR  (340)
#define DUST_RS3_COEFF_ADDR  (344)
#define DUST_TEMP_COEFF_ADDR (348)
#define DUST_HUM_COEFF_ADDR  (352)

#define SENSOR_PACKET_DELAY 5000L
#define TRANSMIT_STATE_SEND_TEMPERATURE 1
#define TRANSMIT_STATE_SEND_HUMIDITY    2
#define TRANSMIT_STATE_POLL_EGG_BUS     3
#define TRANSMIT_STATE_WAITING          4
static uint8_t transmit_state = TRANSMIT_STATE_WAITING;
char sensor_type_temperature[]  = "Temperature";
char sensor_units_temperature[] = "deg C";
char sensor_type_humidity[]     = "Humidity";
char sensor_units_humidity[]    = "%";
uint8_t eggbus_sensor_index     = 0;
char feed_name[32] = {0};

// read your MAC address
static uint8_t mymac[6] = {0,0,0,0,0,0};
NanodeMAC mac(mymac);
EggBus eggBus;

#define SEND_RAW        0
#define SEND_CALCULATED 1
#define SEND_R0         2

byte send_type = SEND_RAW;

// create an AQERF_Base object to handle the RF Link
// informing it what the unit MAC address is
AQERF_Remote rflink(mymac);

void printMAC(uint8_t * mac);

// support variables
byte first_time = 1;
long previous_millis = 0;
int need_to_send = 0;

boolean no2_coeff_valid = false, co_coeff_valid = false, o3_coeff_valid = false, dust_coeff_valid = false;
float temperature = 0.0f, humidity = 0.0f;
float no2_c_coeff = 0.0f, no2_rs_coeff = 0.0f, no2_rs2_coeff = 0.0f, no2_rs3_coeff = 0.0f;
float co_c_coeff = 0.0f, co_rs_coeff = 0.0f, co_rs2_coeff = 0.0f, co_rs3_coeff = 0.0f;
float o3_c_coeff = 0.0f, o3_rs_coeff = 0.0f, o3_rs2_coeff = 0.0f, o3_rs3_coeff = 0.0f;
float dust_c_coeff = 0.0f, dust_rs_coeff = 0.0f, dust_rs2_coeff = 0.0f, dust_rs3_coeff = 0.0f;
float no2_temperature_coeff = 0.0f, no2_humidity_coeff = 0.0f;
float co_temperature_coeff = 0.0f, co_humidity_coeff = 0.0f;
float o3_temperature_coeff = 0.0f, o3_humidity_coeff = 0.0f;
float dust_temperature_coeff = 0.0f, dust_humidity_coeff = 0.0f;

void setup(){
    randomSeed(analogRead(0));

    Serial.begin(115200);
    Serial.println(F("Here are the stored coefficients:"));
    printCoefficients();

    optionallyUpdateCoefficients();

    Serial.println(F("Here are the updated coefficients:"));
    printCoefficients();

    rflink.setTransmitInterval(120000L); // transmit every two minutes

    Serial.println(F("\n[Air Quality Egg - Remote - v2.02]"));
    Serial.print(F("Unit Address: "));
    printlnMAC(mymac);
    Serial.print(F("Last Paired Base: "));
    printlnMAC(rflink.getBaseStationAddress());

    for(;;){ // until some condition results in a break
        if(rflink.pair()){
            Serial.println(F("Pairing Successful"));
            Serial.print(F("Base Station Address: "));
            printlnMAC(rflink.getBaseStationAddress());

            // after pairing succeeds, ensure that the pairing phase is complete before
            // proceeding to runtime behavior (i.e. loop)
            delay(AQERF_PAIRING_DURATION_MS);

            break; // go to loop
        }
        else{
            Serial.println(F("Pairing Failed"));
            // check to see if we already know who are base station is from EEPROM
            // if we do, then use it and proceed to runtime behavior (i.e. loop)
            if(rflink.previouslyPaired()){
                Serial.println(F("Assuming previously paired Base Station"));
                break;
            }
            else{
                // otherwise fall through and re-attempt pairing
                Serial.println(F("Retrying"));
            }
        }
    }

    previous_millis = millis();
}

void loop(){

    unsigned long currentMillis = millis();

    if((first_time == 1) || (currentMillis - previous_millis > rflink.getTransmitInterval())) {

        if(transmit_state != TRANSMIT_STATE_WAITING){
          Serial.println(F("Something is taking longer than expected, Resetting"));
          Serial.flush();
          delay(1000);
          soft_restart();
        }

        first_time = 0;
        previous_millis = currentMillis;
        need_to_send = 0;
        transmit_state = TRANSMIT_STATE_SEND_TEMPERATURE;

        eggBus.init();
    }

    if(need_to_send == 0){
      switch(transmit_state){
      case TRANSMIT_STATE_SEND_TEMPERATURE:
        setupTemperaturePacket();
        need_to_send = 1;
        transmit_state = TRANSMIT_STATE_SEND_HUMIDITY;
        break;
      case TRANSMIT_STATE_SEND_HUMIDITY:
        setupHumidityPacket();
        need_to_send = 1;
        eggbus_sensor_index = 0;
        if(eggBus.next()){
          transmit_state = TRANSMIT_STATE_POLL_EGG_BUS;
        }
        else{
          transmit_state = TRANSMIT_STATE_WAITING;
        }
        break;
      case TRANSMIT_STATE_POLL_EGG_BUS:
        if(eggbus_sensor_index < eggBus.getNumSensors()){ // there are more sensors at the current address
          if(SEND_RAW == send_type){
            setupEggBusPacketRaw();
            send_type = SEND_R0;
          }
          else if(SEND_R0 == send_type){
            setupEggBusPacketR0();
            send_type = SEND_CALCULATED;
          }
          else{
            setupEggBusPacket();
            send_type = SEND_RAW;
            eggbus_sensor_index++;
          }

          need_to_send = 1;
        }
        else if(eggBus.next()){ // there are more sensors on the bus
          eggbus_sensor_index = 0;
        }
        else{ // there are no sensors left on the bus
          need_to_send = 0;
          transmit_state = TRANSMIT_STATE_WAITING;
        }
        break;
      case TRANSMIT_STATE_WAITING:
        // nothing to do here ...
        break;
      default:
        Serial.print(F("Transmit State = "));
        Serial.print(transmit_state);
        Serial.println(F("... Inconceivable!"));
        break;
      }
    }

    if(rflink.clearToSend() && need_to_send){

        // transmit the packet
        rflink.transmit();

        need_to_send = 0;
        Serial.print(F("Transmitted Packet: "));
        Serial.println(feed_name);

        delay(SENSOR_PACKET_DELAY);
    }
}

void printlnMAC(uint8_t * mac){
    for(uint8_t i = 0; i < 6; i++){
        if(mac[i] < 10) Serial.print(F("0"));
        Serial.print(mac[i], HEX);
        if(i != 5){
            Serial.print(F(":"));
        }
    }
    Serial.println();

}

void setupTemperaturePacket(){
    memcpy(feed_name, 0, 32);
    strncpy(feed_name, sensor_type_temperature, 20);
    rflink.setPacketType(AQERF_PACKET_TYPE_REMOTE_STATION_DATUM);
    rflink.setRemoteFirmwareVersion(FIRMWARE_REVISION);
    rflink.setRemoteStationAddress(mymac);
    rflink.setSourceSensorAddress(mymac);
    rflink.setSensorIndex(0);
    rflink.setSensorType(sensor_type_temperature);
    rflink.setSensorUnits(sensor_units_temperature);
    rflink.setSensorValue(getTemperature());
}

void setupHumidityPacket(){
    memcpy(feed_name, 0, 32);
    strncpy(feed_name, sensor_type_humidity, 20);
    rflink.setPacketType(AQERF_PACKET_TYPE_REMOTE_STATION_DATUM);
    rflink.setRemoteFirmwareVersion(FIRMWARE_REVISION);
    rflink.setRemoteStationAddress(mymac);
    rflink.setSourceSensorAddress(mymac);
    rflink.setSensorIndex(1);
    rflink.setSensorType(sensor_type_humidity);
    rflink.setSensorUnits(sensor_units_humidity);
    rflink.setSensorValue(getHumidity());
}

void setupEggBusPacket(){
    float sensor_val = 0, sensor_r = 0.0f, sensor_r_squared = 0.0f, sensor_r_cubed = 0.0f;
    memcpy(feed_name, 0, 32);
    strncpy(feed_name, eggBus.getSensorType(eggbus_sensor_index), 20);
    rflink.setPacketType(AQERF_PACKET_TYPE_REMOTE_STATION_DATUM);
    rflink.setRemoteFirmwareVersion(eggBus.getFirmwareVersion());
    rflink.setRemoteStationAddress(mymac);
    rflink.setSourceSensorAddress(eggBus.getSensorAddress());
    rflink.setSensorIndex(eggbus_sensor_index);
    rflink.setSensorType(eggBus.getSensorType(eggbus_sensor_index));
    rflink.setSensorUnits(eggBus.getSensorUnits(eggbus_sensor_index));

    sensor_r = eggBus.getSensorResistance(eggbus_sensor_index);
    sensor_r /= 1000.0; // convert  to kohms
    sensor_r_squared = sensor_r * sensor_r;
    sensor_r_cubed = sensor_r_squared * sensor_r;

    if(no2_coeff_valid && strcmp_P(eggBus.getSensorType(eggbus_sensor_index), PSTR("NO2")) == 0){
      sensor_val = no2_c_coeff
        + (no2_temperature_coeff * temperature)
        + (no2_humidity_coeff * humidity)
        + (no2_rs_coeff * sensor_r)
        + (no2_rs2_coeff * sensor_r_squared)
        + (no2_rs3_coeff * sensor_r_cubed);
    }
    else if(co_coeff_valid && strcmp_P(eggBus.getSensorType(eggbus_sensor_index), PSTR("CO")) == 0){
      sensor_val = co_c_coeff
        + (co_temperature_coeff * temperature)
        + (co_humidity_coeff * humidity)
        + (co_rs_coeff * sensor_r)
        + (co_rs2_coeff * sensor_r_squared)
        + (co_rs3_coeff * sensor_r_cubed);
    }
    else if(o3_coeff_valid && strcmp_P(eggBus.getSensorType(eggbus_sensor_index), PSTR("O3")) == 0){
      sensor_val = o3_c_coeff
        + (o3_temperature_coeff * temperature)
        + (o3_humidity_coeff * humidity)
        + (o3_rs_coeff * sensor_r)
        + (o3_rs2_coeff * sensor_r_squared)
        + (o3_rs3_coeff * sensor_r_cubed);
    }
    else if(dust_coeff_valid && strcmp_P(eggBus.getSensorType(eggbus_sensor_index), PSTR("Dust")) == 0){
      sensor_val = dust_c_coeff
        + (dust_temperature_coeff * temperature)
        + (dust_humidity_coeff * humidity)
        + (dust_rs_coeff * sensor_r)
        + (dust_rs2_coeff * sensor_r_squared)
        + (dust_rs3_coeff * sensor_r_cubed);
    }
    else{
      sensor_val = eggBus.getSensorValue(eggbus_sensor_index);
    }

    if(sensor_val < 0) sensor_val = 0;
    rflink.setSensorValue(sensor_val);
}

void setupEggBusPacketRaw(){
    char * ohms = "ohms";

    strncpy(feed_name, eggBus.getSensorType(eggbus_sensor_index), 20);
    strcat(feed_name, "_raw");

    rflink.setPacketType(AQERF_PACKET_TYPE_REMOTE_STATION_DATUM);
    rflink.setRemoteFirmwareVersion(eggBus.getFirmwareVersion());
    rflink.setRemoteStationAddress(mymac);
    rflink.setSourceSensorAddress(eggBus.getSensorAddress());
    rflink.setSensorIndex(eggbus_sensor_index);
    rflink.setSensorType(feed_name);
    rflink.setSensorUnits(ohms);
    rflink.setSensorValue((uint32_t) eggBus.getSensorResistance(eggbus_sensor_index));
}

void setupEggBusPacketR0(){
    char * ohms = "ohms";
    uint32_t resistance = eggBus.getSensorR0(eggbus_sensor_index);
    memcpy(feed_name, 0, 32);

    strncpy(feed_name, eggBus.getSensorType(eggbus_sensor_index), 20);
    strcat(feed_name, "_r0");

    rflink.setPacketType(AQERF_PACKET_TYPE_REMOTE_STATION_DATUM);
    rflink.setRemoteFirmwareVersion(eggBus.getFirmwareVersion());
    rflink.setRemoteStationAddress(mymac);
    rflink.setSourceSensorAddress(eggBus.getSensorAddress());
    rflink.setSensorIndex(eggbus_sensor_index);
    rflink.setSensorType(feed_name);
    rflink.setSensorUnits(ohms);
    rflink.setSensorValue(resistance);
}

void printCoefficients(void){

  uint8_t val = 0;
  val = eeprom_read_byte((const uint8_t *) NO2_COEFF_VALID);
  if(val == MAGIC_NUMBER) no2_coeff_valid = true;
  val = eeprom_read_byte((const uint8_t *) CO_COEFF_VALID);
  if(val == MAGIC_NUMBER) co_coeff_valid = true;
  val = eeprom_read_byte((const uint8_t *) O3_COEFF_VALID);
  if(val == MAGIC_NUMBER) o3_coeff_valid = true;
  val = eeprom_read_byte((const uint8_t *) DUST_COEFF_VALID);
  if(val == MAGIC_NUMBER) dust_coeff_valid = true;  

  eeprom_read_block(&no2_c_coeff,   (const void *) NO2_C_COEFF_ADDR,   4);
  eeprom_read_block(&no2_rs_coeff,  (const void *) NO2_RS_COEFF_ADDR,  4);
  eeprom_read_block(&no2_rs2_coeff, (const void *) NO2_RS2_COEFF_ADDR, 4);
  eeprom_read_block(&no2_rs3_coeff, (const void *) NO2_RS3_COEFF_ADDR, 4);

  eeprom_read_block(&co_c_coeff,   (const void *) CO_C_COEFF_ADDR,   4);
  eeprom_read_block(&co_rs_coeff,  (const void *) CO_RS_COEFF_ADDR,  4);
  eeprom_read_block(&co_rs2_coeff, (const void *) CO_RS2_COEFF_ADDR, 4);
  eeprom_read_block(&co_rs3_coeff, (const void *) CO_RS3_COEFF_ADDR, 4);

  eeprom_read_block(&o3_c_coeff,   (const void *) O3_C_COEFF_ADDR,   4);
  eeprom_read_block(&o3_rs_coeff,  (const void *) O3_RS_COEFF_ADDR,  4);
  eeprom_read_block(&o3_rs2_coeff, (const void *) O3_RS2_COEFF_ADDR, 4);
  eeprom_read_block(&o3_rs3_coeff, (const void *) O3_RS3_COEFF_ADDR, 4);

  eeprom_read_block(&dust_c_coeff,   (const void *) DUST_C_COEFF_ADDR,   4);
  eeprom_read_block(&dust_rs_coeff,  (const void *) DUST_RS_COEFF_ADDR,  4);
  eeprom_read_block(&dust_rs2_coeff, (const void *) DUST_RS2_COEFF_ADDR, 4);
  eeprom_read_block(&dust_rs3_coeff, (const void *) DUST_RS3_COEFF_ADDR, 4);

  eeprom_read_block(&no2_temperature_coeff, (const void *) NO2_TEMP_COEFF_ADDR, 4);
  eeprom_read_block(&no2_humidity_coeff, (const void *) NO2_HUM_COEFF_ADDR, 4);
  eeprom_read_block(&co_temperature_coeff, (const void *) CO_TEMP_COEFF_ADDR, 4);
  eeprom_read_block(&co_humidity_coeff, (const void *) CO_HUM_COEFF_ADDR, 4);
  eeprom_read_block(&o3_temperature_coeff, (const void *) O3_TEMP_COEFF_ADDR, 4);
  eeprom_read_block(&o3_humidity_coeff, (const void *) O3_HUM_COEFF_ADDR, 4);
  eeprom_read_block(&dust_temperature_coeff, (const void *) DUST_TEMP_COEFF_ADDR, 4);
  eeprom_read_block(&dust_humidity_coeff, (const void *) DUST_HUM_COEFF_ADDR, 4);

  Serial.print(F("NO2 Temperature coefficient [ppb/degC]: "));
  Serial.println(no2_temperature_coeff, 8);
  Serial.print(F("NO2 Humidity coefficient [ppb/(%RH)]: "));
  Serial.println(no2_humidity_coeff, 8);
  Serial.print(F("NO2 C coefficient [ppb]: "));
  Serial.println(no2_c_coeff, 8);
  Serial.print(F("NO2 RS coefficient [ppb/kohm]: "));
  Serial.println(no2_rs_coeff, 8);
  Serial.print(F("NO2 RS^2 coefficient [ppb/kohm^2]: "));
  Serial.println(no2_rs2_coeff, 8);
  Serial.print(F("NO2 RS^3 coefficient [ppb/kohm^3]: "));
  Serial.println(no2_rs3_coeff, 8);

  Serial.print(F("CO Temperature coefficient [ppb/degC]: "));
  Serial.println(co_temperature_coeff, 8);
  Serial.print(F("CO Humidity coefficient [ppb/(%RH)]: "));
  Serial.println(co_humidity_coeff, 8);
  Serial.print(F("CO C coefficient [ppb]: "));
  Serial.println(co_c_coeff, 8);
  Serial.print(F("CO RS coefficient [ppb/kohm]: "));
  Serial.println(co_rs_coeff, 8);
  Serial.print(F("CO RS^2 coefficient [ppb/kohm^2]: "));
  Serial.println(co_rs2_coeff, 8);
  Serial.print(F("CO RS^3 coefficient [ppb/kohm^3]: "));
  Serial.println(co_rs3_coeff, 8);

  Serial.print(F("O3 Temperature coefficient [ppb/degC]: "));
  Serial.println(o3_temperature_coeff, 8);
  Serial.print(F("O3 Humidity coefficient [ppb/(%RH)]: "));
  Serial.println(o3_humidity_coeff, 8);
  Serial.print(F("O3 C coefficient [ppb]: "));
  Serial.println(o3_c_coeff, 8);
  Serial.print(F("O3 RS coefficient [ppb/kohm]: "));
  Serial.println(o3_rs_coeff, 8);
  Serial.print(F("O3 RS^2 coefficient [ppb/kohm^2]: "));
  Serial.println(o3_rs2_coeff, 8);
  Serial.print(F("O3 RS^3 coefficient [ppb/kohm^3]: "));
  Serial.println(o3_rs3_coeff, 8);

  Serial.print(F("Dust Temperature coefficient [(ug/m^3)/degC]: "));
  Serial.println(dust_temperature_coeff, 8);
  Serial.print(F("Dust Humidity coefficient [(ug/m^3)/(%RH)]: "));
  Serial.println(dust_humidity_coeff, 8);
  Serial.print(F("Dust C coefficient [(ug/m^3)]: "));
  Serial.println(dust_c_coeff, 8);
  Serial.print(F("Dust RS coefficient [(ug/m^3)/(%Occupancy)]: "));
  Serial.println(dust_rs_coeff, 8);
  Serial.print(F("Dust RS^2 coefficient [(ug/m^3)/(%Occupancy)^2]: "));
  Serial.println(dust_rs2_coeff, 8);
  Serial.print(F("Dust RS^3 coefficient [(ug/m^3)/(%Occupancy)^3]: "));
  Serial.println(dust_rs3_coeff, 8);
}

void optionallyUpdateCoefficients(void){
  boolean change_coefficients = false;
  char ch = 0;
  uint32_t now = millis();
  uint32_t timeout = now + 10000; // 10 seconds to type something

  Serial.print(F("Do you want to change the NO2 coefficients? Y/N: "));
  for(;;){

    if(millis() > timeout){
      Serial.print(F("Skipping coefficient entry"));
      return;
    }

    while(Serial.available() == 0){;};
    ch = Serial.read();
    if(ch == 'N' || ch == 'n'){
      change_coefficients = false;
      break;
    }
    else if(ch == 'Y' || ch == 'y'){
      change_coefficients = true;
      break;
    }
  }
  Serial.println(ch);

  if(change_coefficients){
    Serial.print(F("Enter New NO2 Temperature coefficient [ppb/degC]: "));
    while(Serial.available() == 0){;};
    no2_temperature_coeff = Serial.parseFloat();
    eeprom_write_block(&no2_temperature_coeff, (void *) NO2_TEMP_COEFF_ADDR, 4);
    Serial.println(no2_temperature_coeff, 8);

    Serial.print(F("Enter New NO2 Humidity coefficient [ppb/(%RH)]: "));
    while(Serial.available() == 0){;};
    no2_humidity_coeff = Serial.parseFloat();
    eeprom_write_block(&no2_humidity_coeff, (void *) NO2_HUM_COEFF_ADDR, 4);
    Serial.println(no2_humidity_coeff, 8);

    Serial.print(F("Enter New NO2 C coefficient [ppb]: "));
    while(Serial.available() == 0){;};
    no2_c_coeff = Serial.parseFloat();
    eeprom_write_block(&no2_c_coeff, (void *) NO2_C_COEFF_ADDR, 4);
    Serial.println(no2_c_coeff, 8);

    Serial.print(F("Enter New NO2 RS coefficient [ppb/kohm]: "));
    while(Serial.available() == 0){;};
    no2_rs_coeff = Serial.parseFloat();
    eeprom_write_block(&no2_rs_coeff, (void *) NO2_RS_COEFF_ADDR, 4);
    Serial.println(no2_rs_coeff, 8);

    Serial.print(F("Enter New NO2 RS^2 coefficient [ppb/kohm^2]: "));
    while(Serial.available() == 0){;};
    no2_rs2_coeff = Serial.parseFloat();
    eeprom_write_block(&no2_rs2_coeff, (void *) NO2_RS2_COEFF_ADDR, 4);
    Serial.println(no2_rs2_coeff, 8);

    Serial.print(F("Enter New NO2 RS^3 coefficient [ppb/kohm^3]: "));
    while(Serial.available() == 0){;};
    no2_rs3_coeff = Serial.parseFloat();
    eeprom_write_block(&no2_rs3_coeff, (void *) NO2_RS3_COEFF_ADDR, 4);
    Serial.println(no2_rs3_coeff, 8);

    eeprom_write_byte((uint8_t *) NO2_COEFF_VALID, MAGIC_NUMBER);
  }

  Serial.print(F("Do you want to change the CO coefficients? Y/N: "));
  for(;;){
    while(Serial.available() == 0){;};
    ch = Serial.read();
    if(ch == 'N' || ch == 'n'){
      change_coefficients = false;
      break;
    }
    else if(ch == 'Y' || ch == 'y'){
      change_coefficients = true;
      break;
    }
  }
  Serial.println(ch);

  if(change_coefficients){
    Serial.print(F("Enter New CO Temperature coefficient [ppb/degC]: "));
    while(Serial.available() == 0){;};
    co_temperature_coeff = Serial.parseFloat();
    eeprom_write_block(&co_temperature_coeff, (void *) CO_TEMP_COEFF_ADDR, 4);
    Serial.println(co_temperature_coeff, 8);

    Serial.print(F("Enter New CO Humidity coefficient [ppb/(%RH)]: "));
    while(Serial.available() == 0){;};
    co_humidity_coeff = Serial.parseFloat();
    eeprom_write_block(&co_humidity_coeff, (void *) CO_HUM_COEFF_ADDR, 4);
    Serial.println(co_humidity_coeff, 8);

    Serial.print(F("Enter New CO C coefficient [ppb]: "));
    while(Serial.available() == 0){;};
    co_c_coeff = Serial.parseFloat();
    eeprom_write_block(&co_c_coeff, (void *) CO_C_COEFF_ADDR, 4);
    Serial.println(co_c_coeff, 8);

    Serial.print(F("Enter New CO RS coefficient [ppb/kohm]: "));
    while(Serial.available() == 0){;};
    co_rs_coeff = Serial.parseFloat();
    eeprom_write_block(&co_rs_coeff, (void *) CO_RS_COEFF_ADDR, 4);
    Serial.println(co_rs_coeff, 8);

    Serial.print(F("Enter New CO RS^2 coefficient [ppb/kohm^2]: "));
    while(Serial.available() == 0){;};
    co_rs2_coeff = Serial.parseFloat();
    eeprom_write_block(&co_rs2_coeff, (void *) CO_RS2_COEFF_ADDR, 4);
    Serial.println(co_rs2_coeff, 8);

    Serial.print(F("Enter New CO RS^3 coefficient [ppb/kohm^3]: "));
    while(Serial.available() == 0){;};
    co_rs3_coeff = Serial.parseFloat();
    eeprom_write_block(&co_rs3_coeff, (void *) CO_RS3_COEFF_ADDR, 4);
    Serial.println(co_rs3_coeff, 8);

    eeprom_write_byte((uint8_t *) CO_COEFF_VALID, MAGIC_NUMBER);
  }

  Serial.print(F("Do you want to change the O3 coefficients? Y/N: "));
  for(;;){
    while(Serial.available() == 0){;};
    char ch = Serial.read();
    if(ch == 'N' || ch == 'n'){
      change_coefficients = false;
      break;
    }
    else if(ch == 'Y' || ch == 'y'){
      change_coefficients = true;
      break;
    }
  }
  Serial.println(ch);

  if(change_coefficients){
    Serial.print(F("Enter New O3 Temperature coefficient [ppb/degC]: "));
    while(Serial.available() == 0){;};
    o3_temperature_coeff = Serial.parseFloat();
    eeprom_write_block(&o3_temperature_coeff, (void *) O3_TEMP_COEFF_ADDR, 4);
    Serial.println(o3_temperature_coeff, 8);

    Serial.print(F("Enter New O3 Humidity coefficient [ppb/(%RH)]: "));
    while(Serial.available() == 0){;};
    o3_humidity_coeff = Serial.parseFloat();
    eeprom_write_block(&o3_humidity_coeff, (void *) O3_HUM_COEFF_ADDR, 4);
    Serial.println(o3_humidity_coeff, 8);

    Serial.print(F("Enter New O3 C coefficient [ppb]: "));
    while(Serial.available() == 0){;};
    o3_c_coeff = Serial.parseFloat();
    eeprom_write_block(&o3_c_coeff, (void *) O3_C_COEFF_ADDR, 4);
    Serial.println(o3_c_coeff, 8);

    Serial.print(F("Enter New O3 RS coefficient [ppb/kohm]: "));
    while(Serial.available() == 0){;};
    o3_rs_coeff = Serial.parseFloat();
    eeprom_write_block(&o3_rs_coeff, (void *) O3_RS_COEFF_ADDR, 4);
    Serial.println(o3_rs_coeff, 8);

    Serial.print(F("Enter New O3 RS^2 coefficient [ppb/kohm^2]: "));
    while(Serial.available() == 0){;};
    o3_rs2_coeff = Serial.parseFloat();
    eeprom_write_block(&o3_rs2_coeff, (void *) O3_RS2_COEFF_ADDR, 4);
    Serial.println(o3_rs2_coeff, 8);

    Serial.print(F("Enter New O3 RS^3 coefficient [ppb/kohm^3]: "));
    while(Serial.available() == 0){;};
    o3_rs3_coeff = Serial.parseFloat();
    eeprom_write_block(&o3_rs3_coeff, (void *) O3_RS3_COEFF_ADDR, 4);
    Serial.println(o3_rs3_coeff, 8);

    eeprom_write_byte((uint8_t *) O3_COEFF_VALID, MAGIC_NUMBER);

  }

  Serial.print(F("Do you want to change the Dust coefficients? Y/N: "));
  for(;;){
    while(Serial.available() == 0){;};
    char ch = Serial.read();
    if(ch == 'N' || ch == 'n'){
      change_coefficients = false;
      break;
    }
    else if(ch == 'Y' || ch == 'y'){
      change_coefficients = true;
      break;
    }
  }
  Serial.println(ch);

  if(change_coefficients){
    Serial.print(F("Enter New Dust Temperature coefficient [(ug/m^3)/degC]: "));
    while(Serial.available() == 0){;};
    dust_temperature_coeff = Serial.parseFloat();
    eeprom_write_block(&dust_temperature_coeff, (void *) DUST_TEMP_COEFF_ADDR, 4);
    Serial.println(dust_temperature_coeff, 8);

    Serial.print(F("Enter New O3 Humidity coefficient [(ug/m^3)/(%RH)]: "));
    while(Serial.available() == 0){;};
    dust_humidity_coeff = Serial.parseFloat();
    eeprom_write_block(&dust_humidity_coeff, (void *) DUST_HUM_COEFF_ADDR, 4);
    Serial.println(dust_humidity_coeff, 8);

    Serial.print(F("Enter New O3 C coefficient [ug/m^3]: "));
    while(Serial.available() == 0){;};
    dust_c_coeff = Serial.parseFloat();
    eeprom_write_block(&dust_c_coeff, (void *) DUST_C_COEFF_ADDR, 4);
    Serial.println(dust_c_coeff, 8);

    Serial.print(F("Enter New O3 RS coefficient [(ug/m^3)/(%Occupancy)]: "));
    while(Serial.available() == 0){;};
    dust_rs_coeff = Serial.parseFloat();
    eeprom_write_block(&dust_rs_coeff, (void *) DUST_RS_COEFF_ADDR, 4);
    Serial.println(dust_rs_coeff, 8);

    Serial.print(F("Enter New O3 RS^2 coefficient [(ug/m^3)/(%Occupancy)^2]: "));
    while(Serial.available() == 0){;};
    dust_rs2_coeff = Serial.parseFloat();
    eeprom_write_block(&dust_rs2_coeff, (void *) DUST_RS2_COEFF_ADDR, 4);
    Serial.println(dust_rs2_coeff, 8);

    Serial.print(F("Enter New O3 RS^3 coefficient [(ug/m^3)/(%Occupancy)^3]: "));
    while(Serial.available() == 0){;};
    dust_rs3_coeff = Serial.parseFloat();
    eeprom_write_block(&dust_rs3_coeff, (void *) DUST_RS3_COEFF_ADDR, 4);
    Serial.println(dust_rs3_coeff, 8);

    eeprom_write_byte((uint8_t *) DUST_COEFF_VALID, MAGIC_NUMBER);

  }
} 
