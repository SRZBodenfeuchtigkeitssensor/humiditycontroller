#include <Arduino.h>
#include <avr/dtostrf.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <Adafruit_BME280.h>
#include <lmic.h>
#include <hal/hal.h>

#define I2C_OLED_ADDR 0x3C     // 0x3C or 0x3D
#define I2C_BME280_ADDR 0x76   // 0x76 or 0x77

#define VBATPIN        A7

#define fpin 12
#define maxpin 11
#define minpin 10

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
// static const u1_t PROGMEM DEVEUI[8]={ 0x8b, 0x86, 0x9a, 0xf2, 0x1a, 0xc9, 0x2c, 0x4a };
static const u1_t PROGMEM DEVEUI[8]={ 0xd9, 0xdb, 0x18, 0xdc, 0x2c, 0x8c, 0x17, 0xdb };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x40, 0x55, 0xd4, 0xb9, 0xf4, 0xa1, 0x9a, 0x45, 0x98, 0x3d, 0x60, 0x4b, 0x0e, 0xbb, 0x0f, 0x5f };
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

int maxWert = 50;
int minWert = 200;

uint8_t leero[104];
uint8_t leeru[104];
uint8_t varo[sizeof(leero)];
uint8_t varu[sizeof(leero)];

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
    delayMicroseconds(1);
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
  int frequenzfeutigkeit = frequenz(fpin);
  int prozentfeuchtigkeit = (1-(((double)frequenzfeutigkeit-minWert))/(maxWert-minWert))*100;

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
    char floatStr[17];
    float vbat;
    
    float temperature = bme280.readTemperature();
    float pressure = bme280.readPressure()/100.0f;
    float humidity = bme280.readHumidity();
    int frequenzfeutigkeit = frequenz(fpin);
    int prozentfeuchtigkeit = (1-(((double)frequenzfeutigkeit-minWert))/(maxWert-minWert))*100;

    vbat = analogRead(VBATPIN); 
    vbat *= 2;    // resistor on board divided by 2, so multiply back
    vbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    vbat /= 1024; // convert to voltage

    String hz = hzvorlage;
    hz.replace("f",String(frequenzfeutigkeit));
    String ps = prozentvorlage;
    ps.replace("p",String(prozentfeuchtigkeit));
    String ts = tempvorlage;
    ts.replace("i",String(temperature));

    Serial.println(hz);
    Serial.println(ps);
    Serial.println(ts);
    Serial.println("------------------");
    
    byte batvalue = (byte)(vbat * 10); ; // no problem putting it into a int.   
    unsigned int t = (unsigned int)((temperature + 40.05) * 10.0); 
    // t = t + 40; => t [-40..+85] => [0..125] => t = t * 10; => t [0..125] => [0..1250] => round to 0.1 degree
    unsigned int p = (unsigned int)((pressure + 5) * 10.0);  // p [300..1100]
    unsigned int h = (unsigned int)((humidity + 0.05) * 10.0);

    unsigned char mydata[9];
    mydata[0] = batvalue;
    mydata[1] = (h >> 8) & 0xFF;      
    mydata[2] = h & 0xFF; 
    mydata[3] = (t >> 8) & 0xFF;
    mydata[4] = t & 0xFF;
    mydata[5] = (p >> 8) & 0xFF;
    mydata[6] = p & 0xFF; 
    mydata[7] = (unsigned int)(prozentfeuchtigkeit) & 0xFF;

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
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
    break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_RFU1:
    ||     Serial.println(F("EV_RFU1"));
    ||     break;
    */
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
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_SCAN_FOUND:
    ||    Serial.println(F("EV_SCAN_FOUND"));
    ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
    break;
    //        case EV_TXCANCELED:
    //            Serial.println(F("EV_TXCANCELED"));
    //            break;
    //        case EV_RXSTART:
    //            /* do not print anything -- it wrecks timing */
    //            break;
    //        case EV_JOIN_TXCOMPLETE:
    //            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
    //            break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
    break;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("Starting"));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  display.begin();
  display.setPowerSave(0);
  display.setFont(u8x8_font_chroma48medium8_r);
  display.draw1x2String(2,3,"Hello World!");
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
  
  pinMode(fpin,INPUT_PULLUP); 
  pinMode(maxpin,INPUT_PULLDOWN);
  pinMode(minpin,INPUT_PULLDOWN);

  memset(leero,B00000001,sizeof(leero));
  memset(leeru,B10000000,sizeof(leeru));
  leero[0]=leeru[0]=leero[103]=leeru[103]=B11111111;
  digitalWrite(LED_BUILTIN,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  os_runloop_once();
  digitalWrite(LED_BUILTIN,LOW);
  if(digitalRead(maxpin)){ 
    digitalWrite(LED_BUILTIN,HIGH);  
    maxWert = frequenz(fpin);
    Serial.print("Max: ");
    Serial.println(maxWert);
    displayupdate();
    display.drawString(5,2,"Max");
  }
  if(digitalRead(minpin)){
    digitalWrite(LED_BUILTIN,HIGH);
    minWert = frequenz(fpin);
    Serial.print("Min: ");
    Serial.println(minWert);
    displayupdate();
    display.drawString(5,2,"Min");
  } 
}
