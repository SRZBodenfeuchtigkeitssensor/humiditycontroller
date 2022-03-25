#include <Arduino.h>
#include <avr/dtostrf.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <Adafruit_BME280.h>
#include <lmic.h>
#include <hal/hal.h>
#include <FlashStorage.h>

#define I2C_OLED_ADDR 0x3C     // 0x3C or 0x3D
#define I2C_BME280_ADDR 0x76   // 0x76 or 0x77

#define VBATPIN A7

#define switchpin 12
#define maxpin 11
#define minpin 10

#define maxSensorPin A3
#define minSensorPin A0

#define SensorId (SensorWatching-minSensorPin)
#define SUmformungMax 75
#define SUmformungMin 0  

#define DisplayZeit 30

typedef struct{
  byte data[4];
}Data;

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={0xdb, 0x17, 0x8c, 0x2c, 0xdc, 0x18, 0xdb, 0xd9};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xbb, 0xfd, 0xda, 0x02, 0xa7, 0x5b, 0x9c, 0x07, 0xef, 0xba, 0xf8, 0xd5, 0x32, 0x2b, 0x5e, 0x28};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

U8X8_SH1106_128X64_NONAME_HW_I2C display(/* reset=*/ U8X8_PIN_NONE);
Adafruit_BME280 bme280;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

const String hzvorlage= "Frequez: fhz";
const String prozentvorlage = "Feuchtigkeit: p%";
const String tempvorlage = "Temperatur: i^C";

int maxWert[4] = {200,200,200,200};
int minWert[4] = {0,0,0,0};
int SensorWatching = minSensorPin;
FlashStorage(maxWert_fs, Data);
FlashStorage(minWert_fs, Data);

long unsigned int lastsetmin = 0;
long unsigned int lastsetmax = 0;
long unsigned int lastPressedSwitch = 0;
long unsigned int displayTime =0;
bool lastStatus = false;
bool powerSaveMode = true;

uint8_t leero[104];
uint8_t leeru[104];
uint8_t varo[sizeof(leero)];
uint8_t varu[sizeof(leero)];

void readData(FlashStorageClass<Data> fs, int *n, int plus){
  Data x = fs.read();
  for(int i=0;i<4;i++) 
    if( x.data[i] != 0 ) n[i] = x.data[i] + plus;
}

void writeData(FlashStorageClass<Data> fs, int *n, int minus){
  Data x;
  for(int i=0;i<4;i++) 
    x.data[i] = n[i] - minus;
  fs.write(x);
}

int frequenz(int pin){
  int start = micros();
  int wechsel =0;
  int last = digitalRead(pin);
  for(int i=0;i<100000;i++){
    int wert = digitalRead(pin);
    if(wert!=last){
      if(wert==1)wechsel++;
      last = wert;
    }
  }
  return (int)((wechsel*1000.0)/(micros()-start));
}

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {3, 6, 5},
};

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
  Serial.print('0');
  Serial.print(v, HEX);
}

void displayupdate(){  
  float vbat = analogRead(VBATPIN); 
  vbat *= 2;    // resistor on board divided by 2, so multiply back
  vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  vbat /= 1024; // convert to voltage
  int pbat = ((vbat-3.3)/0.9)*100;
  float temperature = bme280.readTemperature();
  if(isnan(temperature)){
    if(!bme280.begin(I2C_BME280_ADDR)) display.drawString(0,0,"BME280 not found");
    else temperature = bme280.readTemperature();
  }
  int frequenzfeutigkeit = frequenz(SensorWatching);
  int prozentfeuchtigkeit = (1-(((double)frequenzfeutigkeit-minWert[SensorId]))/(maxWert[SensorId]-minWert[SensorId]))*100;

  String hz = hzvorlage;
  hz.replace("f",String(frequenzfeutigkeit));
  String ps = String("p%");
  ps.replace("p",String(prozentfeuchtigkeit));

  String ts = tempvorlage;
  ts.replace("i",String(temperature));

  char datastr[17]; 
  hz.toCharArray(datastr,6,9);  
  display.clearLine(2);
  display.drawString(0,2,datastr);

  ps.toCharArray(datastr,5);
  display.drawString(8,2,datastr);

  ts.toCharArray(datastr,9,12); 
  display.clearLine(4);
  int i = 0;
  while((int)datastr[i]!=0)i++;
  memset(&datastr[i],' ',sizeof(datastr)-(i));
  String(pbat).toCharArray(&datastr[9],10);             
  display.drawString(0,4,datastr);

  memcpy(varo,leero,sizeof(leero));
  memcpy(varu,leeru,sizeof(leeru));

  for(int i=2;i<prozentfeuchtigkeit+2&&i<102;i++){
    varo[i]=B11111101;
    varu[i]=B10111111;
  } 
  display.drawTile(0,6,13,varo);
  display.drawTile(0,7,13,varu);
}

void do_send(osjob_t* j){
  // Check if there is not a current TX/RX job running
  displayupdate();
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } 
  else {
    // Prepare upstream data transmission at the next possible time.
    float vbat;
    
    float temperature = bme280.readTemperature();
    float pressure = bme280.readPressure()/100.0f;
    float humidity = bme280.readHumidity();
    int prozentfeuchtigkeit[4];

    vbat = analogRead(VBATPIN); 
    vbat *= 2;    // resistor on board divided by 2, so multiply back
    vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    vbat /= 1024; // convert to voltage

    for (int i = 0;i+minSensorPin<maxSensorPin;i++){
      int frequenzfeutigkeit = frequenz(i);
      if(frequenzfeutigkeit!=0){
        prozentfeuchtigkeit[i] = (1-(((double)frequenzfeutigkeit-minWert[i]))/(maxWert[i]-minWert[i]))*100;

        String hz = hzvorlage; 
        hz.replace("f",String(frequenzfeutigkeit));
        String ps = prozentvorlage;
        ps.replace("p",String(prozentfeuchtigkeit[i]));

        Serial.println(hz);
        Serial.println(ps);
      }
      
    }

    
    String ts = tempvorlage;
    ts.replace("i",String(temperature));

    Serial.println(ts);
    Serial.println("------------------");
    
    byte batvalue = (byte)(vbat * 10); ; // no problem putting it into a int.   
    unsigned int t = (unsigned int)((temperature + 40.05) * 10.0); 
    // t = t + 40; => t [-40..+85] => [0..125] => t = t * 10; => t [0..125] => [0..1250] => round to 0.1 degree
    unsigned int p = (unsigned int)((pressure + 5) * 10.0);  // p [300..1100]
    unsigned int h = (unsigned int)((humidity + 0.05) * 10.0);

    unsigned char mydata[19];
    mydata[0] = batvalue;
    mydata[1] = (h >> 8) & 0xFF;      
    mydata[2] = h & 0xFF; 
    mydata[3] = (t >> 8) & 0xFF;
    mydata[4] = t & 0xFF;
    mydata[5] = (p >> 8) & 0xFF;
    mydata[6] = p & 0xFF;
    for(int i=0;i+minSensorPin<maxSensorPin;i++){
      mydata[3*i+7] = (unsigned short)(minWert[i]) & 0xff;
      mydata[3*i+8] = (unsigned int)(maxWert[i] - 75) & 0xff;
      mydata[3*i+9] = (unsigned int)(prozentfeuchtigkeit[i]) & 0xFF; 
    }
    LMIC_setTxData2(5, mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
    break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
    break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
    break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
    break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];  
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i=0; i<sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i=0; i<sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      LMIC_setLinkCheckMode(0);
    break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
    break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
    break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
    break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
    break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
    break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
    break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
    break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
    break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
    break;
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
    break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting"));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  display.begin();
  display.setPowerSave(0);
  display.setFont(u8x8_font_chroma48medium8_r);
  display.draw1x2String(2,3,"Giessomat");
  delay(2000);
  display.clearDisplay();
  display.draw1x2String(0,0,"Feuchtigkeit");
  
  if(!bme280.begin(I2C_BME280_ADDR)) display.drawString(0,0,"BME280 not found");

  digitalWrite(LED_BUILTIN, LOW);

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset(); 
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
  
  pinMode(switchpin,INPUT_PULLDOWN); 
  pinMode(maxpin,INPUT_PULLDOWN);
  pinMode(minpin,INPUT_PULLDOWN);
  for (int i=minSensorPin; i<maxSensorPin; i++) pinMode(i,INPUT_PULLDOWN);

  memset(leero,B00000001,sizeof(leero));
  memset(leeru,B10000000,sizeof(leeru));
  leero[0]=leeru[0]=leero[103]=leeru[103]=B11111111;
  digitalWrite(LED_BUILTIN, LOW);
  
  //Flashspeicher lesen
  readData(maxWert_fs, maxWert, SUmformungMax);
  readData(minWert_fs, minWert, SUmformungMin);
  
  displayTime = millis() + 15000;
}

void activateDisplay(int sec){
  display.setPowerSave(0);
  displayupdate();
  displayTime=millis() + sec * 1000;
  powerSaveMode = false;
  Serial.println("display on");
}

void loop() {
  os_runloop_once();

  if(digitalRead(maxpin) && millis() > lastsetmax){//set Frequenzy with most humidity
    maxWert[SensorId] = frequenz(SensorWatching);
    writeData(maxWert_fs, maxWert, SUmformungMax);
    Serial.print("Max: ");
    Serial.println(maxWert[SensorId]);
    activateDisplay(DisplayZeit);
    display.drawString(5,2,"Max");
    lastsetmax = millis()+1000;
  }

  if(digitalRead(minpin) && millis() > lastsetmin){//set Frequenz mit geringster Feuchtigkeit
    minWert[SensorId] = frequenz(SensorWatching);
    writeData(minWert_fs, minWert, SUmformungMin);
    Serial.print("Min: ");
    Serial.println(minWert[SensorId]);
    activateDisplay(DisplayZeit);
    display.drawString(5,2,"Min");
    lastsetmin = millis()+1000;
  } 

  if(lastStatus){
    
    if(millis() > lastPressedSwitch){
      if(!digitalRead(switchpin)){
        lastStatus=false;
        lastPressedSwitch=millis()+100; 
      }
    }

    else if(!digitalRead(switchpin)){
      if(++SensorWatching>maxSensorPin) SensorWatching=minSensorPin;
      display.drawString(0,3,"Sensor: ");
      char buffer[2];
      String(SensorId).toCharArray(buffer,2,0); 
      display.drawString(8,3,buffer);
      lastPressedSwitch=millis()+100;
      lastStatus=false; 
    }
  }

  else if(digitalRead(switchpin) && millis() > lastPressedSwitch){ 
    Serial.print("display ...");
    lastStatus = true;
    lastPressedSwitch=millis()+500;
    activateDisplay(DisplayZeit);
  }

  if(!powerSaveMode && millis() > displayTime){
    display.setPowerSave(1); //display off
    powerSaveMode = true;
    Serial.println("Display off");
  }
} 