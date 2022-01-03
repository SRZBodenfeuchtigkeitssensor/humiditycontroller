#include <Arduino.h>
#include <U8x8lib.h>
#include <Adafruit_BME280.h>


#define I2C_OLED_ADDR 0x3C     
#define I2C_BME280_ADDR 0x76   
#define fpin 12
#define minf 50  //max. feuchtigkeits
#define maxf 200.0 //min. feuchtigkeits 

U8X8_SH1106_128X64_NONAME_HW_I2C display(U8X8_PIN_NONE);
Adafruit_BME280 bme280;

const String hzvorlage= "Frequez: fhz";
const String prozentvorlage = "Feuchtigkeit: p%";
const String tempvorlage = "Temperatur: i^C";

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
      wechsel++;
      last = wert;
    }
    delayMicroseconds(1);
  }
  return (int)((wechsel*1000.0)/(micros()-start));
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  display.begin();
  display.setFont(u8x8_font_chroma48medium8_r);
  display.draw1x2String(0,0,"Feuchtigkeit");
  if(!bme280.begin(I2C_BME280_ADDR)) Serial.println("BME280 not found");
  pinMode(fpin,INPUT);  

  memset(leero,B00000001,sizeof(leero));
  memset(leeru,B10000000,sizeof(leeru));
  leero[0]=leeru[0]=leero[103]=leeru[103]=B11111111;
}

void loop() {
  int frequenzfeutigkeit = frequenz(fpin);
  int prozentfeuchtigkeit = (1-(frequenzfeutigkeit-minf)/maxf)*100;
  float temp = bme280.readTemperature();

  String hz = hzvorlage;
  hz.replace("f",String(frequenzfeutigkeit));
  String p = prozentvorlage;
  p.replace("p",String(prozentfeuchtigkeit));
  String t = tempvorlage;
  t.replace("i",String(temp));

  Serial.println(hz);
  Serial.println(p);
  Serial.println(t);
  Serial.println("------------------");

  char datastr[17]; 
  hz.toCharArray(datastr,6,9);
  display.clearLine(2);
  display.drawString(0,2,datastr);

  p.toCharArray(datastr,4,14);
  display.drawString(9,2,datastr);

  t.toCharArray(datastr,9,12); 
  display.clearLine(4);
  display.drawString(0,4,datastr);

  memcpy(varo,leero,sizeof(leero));
  memcpy(varu,leeru,sizeof(leeru));
  for(int i=2;i<prozentfeuchtigkeit+2;i++){
    varo[i]=B11111101;
    varu[i]=B10111111;
  } 
  display.drawTile(0,6,13,varo);
  display.drawTile(0,7,13,varu);

  for(int i = 4;i>=0;i--){
    display.clearLine(5);
    display.drawGlyph(12,5,String(i).charAt(0));
    delay(60000);
  }
}