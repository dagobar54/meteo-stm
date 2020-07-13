#include <Arduino.h>
#include "UARTTransfer.h"
#include <RTClock.h>
#include <Wire.h>
#include "home_meteo.h"
#include <SPI.h>
#include "nRF24L01-STM.h"
#include "RF24-STM.h"
#include "MyTimeLib.h" 
#include "LiquidCrystal_I2C.h"
//#include "LiquidCrystal_PCF8574.h"
#include <stdio.h>
#include <stdlib.h>
//#include <exti.h>
#include <BH1750.h>

#include "DHT.h"


#define DHTTYPE DHT11   // DHT 11
#define DHTPIN PA1     // Digital pin connected to the DHT sensor

#define radioSignal PA0
#define LED_RADIO PA4
#define SERIAL_DEBUG 1
RTClock rt (RTCSEL_LSE); // initialise

// message id's for STM32 <-> ESP communication
#define SET_NTP_TIME 51
#define GET_NTP_TIME 52

// check NTP time every
#define NTP_CHECK_INT (1000*60)*20 //(1000*3600*1)  // six hours


#define BMPEP  280
//#define BMEP  180
//#define DHT   11
#define pinRF24 3
#define paramButton 4


#if (BMPEP==180) 
  #include <SFE_BMP180.h>
#endif
#if (BMPEP==280)
  #include <BME280I2C.h>
  BME280I2C::Settings settings(
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_Off,
    BME280::SpiEnable_False,
    BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
  );
  BME280I2C bme(settings);
#endif
// --------------------------------------------------------
LiquidCrystal_I2C lcd(0x27,16,2);
//LiquidCrystal_PCF8574 lcd(0x27);

DHT dht(DHTPIN, DHTTYPE);

uint32_t tt;
uint32_t prev_ntp_check = 0;
UARTTransfer UT;

const int analogInPin = PB1; // Analog input pin that the potentiometer
const int pwmOutPin = PA8;    // PWM pin that the LED is attached to
// These variables will change:
int lightValue = 0;        // value read from the pot
int backLightValue = 0;        // value output to the PWM
BH1750 lightMeter;
uint16_t lux = 0;

// table of brightness levels 
#define NUM_LEVELS 5
uint16_t luxlevels[NUM_LEVELS] = {10, 50, 100, 500, 2000};
uint8_t brightlevels[NUM_LEVELS] = {3, 25, 40, 100, 150};

void DisplayTime(DateTime dt);
void second_isr();
void radioUp_isr();
void GetMeteoData();
void serial_showSensors(String s);
void CreateStartAck();
void CreateAck(int pipeNo);
void showReceiverState(unsigned int pipeNo);
void showRemoteState(int pipeNo);
void showRemoteMeteoData(unsigned int pipeNo);
void ShowSecond(DateTime dt);
void ShowLocalLcdMeteo();
void ShowRemoteLcdMeteo();
void pwmBackLight();

volatile bool second_flag=true;
float T,P,H,Ph,H11;
//RTCDateTime dt;

RF24 radio(9,10);

pipe_data pipeData;
meteo_data_struct receivedData;

bool paramMode =false;

char str1[50] = {0,};
char buf[100]={0,};


const uint32_t max_exchange_interval = 4000;
uint32_t max_ex_counter = 10;
uint32_t unixtime = 0;

RF24EventStack evStack;

uint8_t symD[8]   = { 0x07, 0x09, 0x09, 0x09, 0x09, 0x1F, 0x11 }; // Д
//uint8_t symZH[8]  = { 0x11, 0x15, 0x15, 0x0E, 0x15, 0x15, 0x11 }; // Ж
uint8_t symB[8]   = { 0x1E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x1E }; // Б--
uint8_t sym4[8]   = { 0x11, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x01 }; // Ч
uint8_t symL[8]   = { 0x0F, 0x09, 0x09, 0x09, 0x09, 0x11, 0x11 }; // Л
uint8_t symP[8]   = { 0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }; // П
uint8_t symSHi[8] = { 0x10, 0x15, 0x15, 0x15, 0x15, 0x1F, 0x03 }; // Щ
uint8_t symJU[8]  = { 0x11, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E }; // У
uint8_t symGradus[8]  = { 0x0C, 0x12, 0x12, 0x0C, 0x00, 0x00, 0x00 }; // Я

const uint64_t pipe_addresses[2] = {
	0xF0F0F0F0D2LL,
	0xF0F0F0F0E1LL,
	//0xF0F0F0F0E2LL,
	//0xF0F0F0F0E3LL,
	//0xF0F0F0F0E4LL,
	//0xF0F0F0F0E5LL
};

void setup(void)
{ 
      // Configure the ADC pin
  pinMode(analogInPin, INPUT_ANALOG);
    // Configure LED pin
  pinMode(pwmOutPin, PWM);
  pinMode(radioSignal,INPUT);
  delay(3000);
  Serial.begin(115200);
  dht.begin();
  Wire.begin();
	if(!bme.begin())
	{
		Serial.println("Could not find BME280I2C sensor!");
		delay(1000);
	}

	// Change some settings before using.
	settings.tempOSR = BME280::OSR_X4;
	bme.setSettings(settings);
  Serial.println(bme.chipModel());
  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(106);

    
  radio.enableDynamicPayloads();                    // Ack payloads are dynamic payloads
  radio.enableAckPayload();                         // We will be using the Ack Payload feature, so please enable it
  //radio.setAutoAck(true);
  

  
  // Open a writing and reading pipe on each radio, with opposite addresses
  radio.openWritingPipe(pipe_addresses[0]);
  radio.openReadingPipe(1,pipe_addresses[1]);
  //radio.openReadingPipe(2,pipe_addresses[2]);
  
 
  // Start the radio listening for data
  radio.startListening();


  #if defined(SERIAL_DEBUG) 
  radio.printDetails();
  #endif

  tt = rt.getTime();
  unixtime = tt;
  CreateStartAck();

  lightMeter.begin(); //BH1750_CONTINUOUS_LOW_RES_MODE);
  lux = lightMeter.readLightLevel();

  lcd.init();
  pwmBackLight();
  lcd.load_custom_character(0,symD); // Д
  lcd.load_custom_character(1,symB); // Б
  lcd.load_custom_character(2,sym4); // Ч
  lcd.load_custom_character(3,symL); // Л
  lcd.load_custom_character(4,symP); // П
  lcd.load_custom_character(5,symSHi); // Щ
  lcd.load_custom_character(6,symJU); // Ю
  lcd.load_custom_character(7,symGradus); // Я
  //lcd.setBacklight(255);
  //lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Start meteo");
  //delay(2000);
  //lcd.clear();
  
  int nDevices = 0;
  int address;
  int error;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  Serial2.begin(9600);
  UT.begin(&Serial2);
  //tt = rt.getTime();
  //DisplayTime(tt);
  GetMeteoData();
  serial_showSensors("Local data ");
  pinMode(PC13, OUTPUT);
  pinMode(LED_RADIO,OUTPUT);
  digitalWrite(PC13,LOW);
  digitalWrite(LED_RADIO,LOW);
  rt.attachSecondsInterrupt(second_isr);
  attachInterrupt(0, radioUp_isr, FALLING);
  //digitalPinToInterrupt(PA0)
 }



void loop() {

    uint8_t pipeNo,st=0;

    paramMode = digitalRead(paramButton)!=0;
    while (!evStack.isEmpty()){
      delay(100);
      digitalWrite(PC13,HIGH);
      pipeNo = evStack.pop();
      pipeData.exchange_counter++;
      if (receivedData.round_tripDelay > max_exchange_interval)
        pipeData.longer_exchange_counter++;
      receivedData.client_time += 3600;      

      st++; 
      #if defined(SERIAL_DEBUG) 
      Serial.println();
      //Serial.print("Reveiver ");
      //Serial.print(pipeNo);
      //Serial.print("    ");
      //print_time(clock.getDateTime());
      #endif
      showRemoteMeteoData(pipeNo);
      showRemoteState(pipeNo);
      showReceiverState(pipeNo);
      CreateAck(pipeNo);
      ShowRemoteLcdMeteo();
      //strcpy(str1,"#");
      //lcd.setCursor(0, 0);
      //lcd.print(str1);
    } 
  if (UT.receiveData()) {
    uint8_t m_id = UT.getMessageID();
    //Serial.println(m_id);
    if ((m_id == SET_NTP_TIME) && (UT.getDataLength() ==  4 )) {
      UT.getData((uint8_t*)&tt, sizeof(tt));
      Serial.print("NTP time ");
      Serial.println(tt);
        //unixtimeToString(tt,str1);
        //Serial.println(str1);
      rt.setTime(tt);
      prev_ntp_check = millis();
    }
    UT.clearMessage();
  }
  
  if (millis() - prev_ntp_check > NTP_CHECK_INT ) {
     UT.sendData(GET_NTP_TIME, 4, (uint8_t*) &tt);
     prev_ntp_check = millis();
     Serial.print("request on ");
     Serial.println(tt);
  }
  if (second_flag){
    second_flag=false;
    pwmBackLight();
    tt = rt.getTime();
    unixtime = tt;
    int diffT= tt % 30;
    DateTime dt = UnixtimeToDateTime(tt);
    if (diffT ==20){
      DisplayTime(dt);
    }//else if (diffT <=4)
      //ShowSecond(dt);
    else if (diffT == 0)
      ShowLocalLcdMeteo();      
    if ((tt % 60)==0){
      //DisplayTime(tt);

      GetMeteoData();
      serial_showSensors("Local data ");
    }
  }
  delay(100);
  digitalWrite(PC13,HIGH);
  digitalWrite(LED_RADIO,HIGH);
  //strcpy(str1," ");
  //lcd.setCursor(0, 0);
  //lcd.print(str1);
}
void second_isr(){
  second_flag=true;
}
void radioUp_isr()
{
 
  //detachInterrupt(1);

  digitalWrite(PC13,LOW);
  digitalWrite(LED_RADIO,LOW);
  uint8_t pipeNo;                          // Declare variables for the pipe and the byte received
  while( radio.available(&pipeNo)){              // Read all available payloads
      uint8_t s = radio.getDynamicPayloadSize();
      //Serial.println(s);
      radio.read( &receivedData, s );    
      //radio.flush_tx();  
      radio.writeAckPayload(1,&pipeData.ackData, sizeof(server_ack )); 
      evStack.push(pipeNo); 
      //flagReceiver = pipeNo;    
  }
}
void ShowSecond(DateTime dt)
{
  char buffer[5]={0,};
  dateFormat(buffer,"s",unixtime);
  lcd.setCursor(6,1);
  lcd.print(buffer);
}
void pwmBackLight()
{
          // read the analog in value:
    lightValue = analogRead(analogInPin);
    // map it to the range of the analog out:
    backLightValue = map(lightValue, 0, 4095, 0, 65535);
    // change the analog out value:
    pwmWrite(pwmOutPin, backLightValue);
}
void DisplayTime(DateTime dt) {
  static uint8_t prev_h = 0;
  static uint8_t prev_m = 0;
  static uint8_t prev_s = 0;
  uint8_t h = dt.hour;
  uint8_t m = dt.minute;
  uint8_t s = dt.second;


  char buffer[255]={0,};
  dateFormat(buffer,"H:i d.m.Y N D",unixtime);
  Serial.println(buffer);
  dateFormat(buffer,"H:i d.m.y",unixtime);
  lcd.setCursor(0,1);
  lcd.print(buffer);
   lcd.setCursor(14,1);
   switch(dt.dayOfWeek)
   {
      case 1: lcd.write(4);
        lcd.print("H");
        break;
      case 2: 
        lcd.print("BT");
        break;
      case 3:
        lcd.print("CP");
        break;
      case 4: lcd.write(2);
        lcd.print("T");
        break;
      case 5: lcd.write(4);   //4T
        lcd.print("T");
        break;
      case 6: lcd.print("C");lcd.write(1);
        break;
      case 7:
        lcd.print("BC");
        break;
   }

  if ((h == prev_h) && (m == prev_m)&& (s == prev_s)) return;
  prev_h = h;
  prev_m = m;
  prev_s = s;
}

void CreateStartAck()
{
  pipeData.ackData.command=start_command;
  pipeData.ackData.time_interval = 0;
  pipeData.ackData.server_time = unixtime-3600;
  //ackData.state =transmit_state;
  pipeData.ackData.channel=radio.getChannel();
  pipeData.ackData.data_rate=radio.getDataRate();
  pipeData.ackData.power = radio.getPALevel();
  pipeData.power_tune = power_lowing;
  pipeData.longer_exchange_counter =0;
  pipeData.exchange_counter = 0;
  pipeData.pipe_status = start_pipe;
}
void CreateAck(int pipeNo)
{
  unixtime = rt.getTime();
  pipeData.ackData.server_time=unixtime-3600;
  //uint8_t channel,data_rate,power;
  pipeData.ackData.command=none_command;
  pipeData.ackData.channel=radio.getChannel();
  pipeData.ackData.power = radio.getPALevel();
  pipeData.pipe_status = connected_pipe;
  pipeData.ackData.command = none_command; //test_command;
}

void showReceiverState(unsigned int pipeNo)
{
  //  sensors.state=transmit_state;
  //int channel=radio.getChannel();
  //int data_rate=radio.getDataRate();

  int power = radio.getPALevel();
      #if defined(SERIAL_DEBUG) 
        //выводим на серийный порт
      Serial.print("Pipe=");
      Serial.print(pipeNo);
      Serial.print(" local state   ");
      tt = rt.getTime();
      char buffer[255]={0,};
      dateFormat(buffer,"H:i:s",tt);
      Serial.print(buffer);
      Serial.print("  ");
      Serial.print(pipeData.pipe_status);
      Serial.print("-");
      Serial.println(power);  
      #endif
}
void showRemoteState(int pipeNo)
{
  #if defined(SERIAL_DEBUG) 
        //выводим на серийный порт
      //Serial.print("Pipe=");
      //Serial.print(pipeNo);
      Serial.print("Remote data "); 
      Serial.print(" Query:");
      Serial.print(receivedData.query);
      Serial.print(" TypeOfData:");
      Serial.print(receivedData.type_of_data);
      Serial.print(" State:");
      Serial.print(receivedData.state);
      Serial.print(" Power:");
      Serial.print(receivedData.power);
      Serial.print("    ");
      uint32_t dt = receivedData.client_time;
      char str[10];
      unixtimeToString(dt,str);
      Serial.print(str);
      Serial.print(" delay:");
      Serial.print(receivedData.round_tripDelay);  
      Serial.print(" Vcc=");
      Serial.println(receivedData.vcc);
      //Serial.print(F("Loaded next response "));
      //Serial.println(receivedData[pipeNo].query);  
  #endif
}
void showRemoteMeteoData(unsigned int pipeNo)
{
  #if defined(SERIAL_DEBUG) 
        //выводим на серийный порт
      Serial.print("Remote meteo T=");
      Serial.print(receivedData.meteo_data.T);
      Serial.print("   P=");
      Serial.print(receivedData.meteo_data.P);
      Serial.print("   H=");
      Serial.println(receivedData.meteo_data.H);
      delay(50);
  #endif
}
void ShowLocalLcdMeteo()
{
  int row = 1;
  lcd.setCursor(0,row);
  lcd.print("                ");

  dtostrf(T, 5, 1, str1);
   //if (abs(receivedData.meteo_data.T) > 100)
  lcd.setCursor(0, row);
  lcd.print(str1);
  strcpy(str1,"C");
  lcd.setCursor(5, 1);
  lcd.write(7);
  lcd.print(str1);
  lcd.setCursor(0,row);
  if (T > 0)
      strcpy(str1,"+");
     else
      strcpy(str1,"-");
   lcd.print(str1);

   uint32_t Pr = lroundf(P*0.750062);
   itoa (Pr, str1,10);
   lcd.setCursor(8, row);
   lcd.print(str1);
   strcpy(str1,"mm");
   lcd.setCursor(11, row);
   lcd.print(str1);

   itoa (lroundf(H/10.0), str1,10);
   lcd.setCursor(14, row);
   lcd.print(str1);

   
}
void ShowRemoteLcdMeteo()
{
  int row = 0;
  lcd.setCursor(0,row);
  lcd.print("                ");

  dtostrf(receivedData.meteo_data.T/10.0, 5, 1, str1);
   //if (abs(receivedData.meteo_data.T) > 100)
  lcd.setCursor(0, row);
  lcd.print(str1);
  strcpy(str1,"C");
  lcd.setCursor(5, row);
  lcd.write(7);
  lcd.print(str1);
  lcd.setCursor(0,row);
  if (receivedData.meteo_data.T > 0)
      strcpy(str1,"+");
     else
      strcpy(str1,"-");
   lcd.print(str1);

   uint32_t Pr = lroundf(receivedData.meteo_data.P*0.0750062);
   itoa (Pr, str1,10);
   lcd.setCursor(8, row);
   lcd.print(str1);
   strcpy(str1,"mm");
   lcd.setCursor(11, row);
   lcd.print(str1);

   itoa (lroundf(receivedData.meteo_data.H/10.0), str1,10);
   lcd.setCursor(14, row);
   lcd.print(str1);
}

void GetMeteoData()
{
  #if (BMPEP==180) 
  long int timePress;
  H = 0;
  char status;
  status = pressure.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    {
        status = pressure.startPressure(3);
        if (status != 0)
        {
          delay(status);
          status = pressure.getPressure(P,T);
          if (status != 0)
          {
            Ph = P /1.333224;
          }
          else {
            P=0.0;  
            }
        }
        else{
            T=0.0;
            P=0.0;
        }
    }
    else{
      T=0.0;
    }
    //local_data.T = t; 
  }
  else{ 
    T=0.0;P=0.0;H=0.0;
  }
  #endif
  #if (BMPEP==280)
    float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  //  float _T,_P,_H;
  bme.read(pres, temp, hum, tempUnit, presUnit);
  //bme.read(&_P, &_T, &_H, tempUnit, presUnit);
  //Serial.println(pres);
  P= pres/100;
  T = temp;
  H = hum*10;
  Ph = pres /133.3224;
  #endif
  #if (DHTTYPE == 11)
    H11 = dht.readHumidity();
  #endif
  lux = lightMeter.readLightLevel();
}
#if defined(SERIAL_DEBUG) 
void  serial_showSensors(String s)
{
  Serial.print(s);
    Serial.print(":  T=");
    //float t = ((float)sensors.meteo_data.T) / 10.0;
    Serial.print(T,1);
    //float p = ((float)sensors.meteo_data.P) / 10.0;
    Serial.print("   P=");
    Serial.print(P,1);
    Serial.print("-");
    Serial.print(Ph,1);
    Serial.print("   H=");
    //float h = sensors.meteo_data.H / 10.0;
    Serial.print(H/10,1);
    Serial.print("   H11=");
    Serial.print(H11,1);
    Serial.print("  lux=");
    Serial.print(lux);  
    Serial.print(" light=");
    Serial.print(lightValue);  
    Serial.print(" bl=");
    Serial.println(backLightValue);  

    delay(50);
}
#endif
