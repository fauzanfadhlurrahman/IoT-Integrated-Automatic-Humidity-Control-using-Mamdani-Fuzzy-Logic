#include <WiFi.h>
#include <PubSubClient.h>
#include <HTTPClient.h>// hanya keperluan data log
#include "time.h"
#include <EEPROM.h>
#include "DHT.h"
#include <Arduino.h>
#include <analogWrite.h>
#define DHTTYPE DHT22
#define DHTPIN 0
DHT dht(DHTPIN, DHTTYPE);
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 0;

const int trigPin = 25;// oranye
const int echoPin = 26;//kuning
int setA,setB;
long duration;
float distance;
int air;
float setpoint;
String humstate, dehumstate;


String WiFiSSID = "Link";
String WiFiPassword = "AndroidAP999C";
String mqttBroker = "test.mosquitto.org";
WiFiClient client;
PubSubClient mqtt(client);
String GOOGLE_SCRIPT_ID = "AKfycbwLkD1M-OcQWZuFbN4pTiGrJvUfDVlXYfPsW8ohVSHmpXLs_bEjCqMbG2MIIrvPtElu"; 
void connectWiFi();
void connect_mqtt();
//String randomTemp();
String perintah = "";
String perintah2 = "";
void mqttReceivedMessage(char* topic, byte *msg, unsigned int msgLength);
unsigned long gtime=0;
int R_IS = 12;
int R_EN = 13;
int R_PWM = 5;
int L_IS = 23;
int L_EN = 19;
int L_PWM = 18;

int in1 = 27;
int in2 = 16;
int enA = 14;
#define kipas 17
#define kipas2 2

#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE(*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);

// Number of inputs to the fuzzy inference system
const int fis_gcI = 2;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 2;
// Number of rules to the fuzzy inference system
const int fis_gcR = 12;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// Setup routine runs once when you press reset:
void setup()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(R_IS, OUTPUT);
 pinMode(R_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
 pinMode(L_IS, OUTPUT);
 pinMode(L_EN, OUTPUT);
 pinMode(L_PWM, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(kipas,OUTPUT);
  pinMode(kipas2,OUTPUT);
 digitalWrite(R_IS, LOW);
 digitalWrite(L_IS, LOW);
 digitalWrite(R_EN, HIGH);
 digitalWrite(L_EN, HIGH);
 digitalWrite(L_PWM, 0);
 digitalWrite(in1, HIGH);
 digitalWrite(in2, LOW);
 lcd.begin();
  Serial.begin(9600);
  connectWiFi();
  mqtt.setServer(mqttBroker.c_str(),1883);
  mqtt.setCallback(mqttReceivedMessage);

 dht.begin();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  setA=40;
  setB=60;
}

// Loop routine runs over and over again forever:
void loop() {
  float hum = dht.readHumidity();
  float tem = dht.readTemperature();
  float hic = dht.computeHeatIndex(tem, hum, false);
 if (!mqtt.connected())
 {
   connect_mqtt();
   lcd.clear();
   Serial.println("MQTT Connected");
   lcd.print("MQTT connected");
   delay(1000);
   lcd.clear();
 }
    g_fisInput[0] = tem;
    // Read Input: Kelembapan
    g_fisInput[1] = hum;

    g_fisOutput[0] = 0;
    g_fisOutput[1] = 0;

    fis_evaluate();
 mqtt.loop();
 //mqtt.publish("fauzan/test", randomTemp().c_str());
 digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  air = (((21-distance)/15)*100);

 if (air>100){
 air=100;
 }
 else if (air<0){
air=0;
 }

lcd.setCursor(0,1);
 lcd.print("Air =");
 lcd.setCursor(6,1);

 lcd.print(air);
//lcd.setCursor(14,1);
 lcd.print("%");
lcd.print("   ");
if (millis()-gtime>=10000){
  gtime=millis();
gsheetlogdata();  
}


 lcd.backlight();
 lcd.setCursor(0,0);
 lcd.print("K=");
 lcd.setCursor(3,0);
 lcd.print(hum);
   lcd.print("%");
 lcd.setCursor(10,0);
 lcd.print("S=");
 lcd.setCursor(12,0);
 lcd.print(tem);
 lcd.print(char(223));
   lcd.print("C");
 lcd.setCursor(0,2);
 lcd.print("H =");
 lcd.print(g_fisOutput[0]);
 lcd.setCursor(12,2);
 lcd.print("D=");
 lcd.print(g_fisOutput[1]);
 humstate=g_fisOutput[0];
 dehumstate=g_fisOutput[1];
 analogWrite(enA, g_fisOutput[0]);
 analogWrite(R_PWM, g_fisOutput[1]);
 if (g_fisOutput[0]>0){
   digitalWrite(kipas,HIGH);
 }
 else if (g_fisOutput[0]=0){
   digitalWrite(kipas,LOW);
 }
 if (g_fisOutput[1]>0){
   digitalWrite(kipas2,HIGH);
 }
 else if (g_fisOutput[1]=0){
   digitalWrite(kipas2,LOW);
 }
/*
 if (air>100){
 lcd.setCursor(0,1);
 air=100;
 lcd.print("Air = 100 %");  
 }
 else {
 lcd.setCursor(0,1);
 lcd.print("Air =");
 lcd.setCursor(6,1);
 lcd.print(air);
 lcd.setCursor(14,1);
 lcd.print("%");
}*/
  delay(2000);
  

 static char temperature[7];
  dtostrf(tem, 6, 2, temperature);
  static char humidity[7];
  dtostrf(hum, 6,2, humidity);
  static char tangki[7];
  dtostrf(air, 6,2, tangki);
  static char pwmH[7];
  dtostrf(g_fisOutput[0], 6,2, pwmH);
  static char pwmD[7];
  dtostrf(g_fisOutput[1], 6,2, pwmD);

 mqtt.publish("fauzan/test", humidity);
 mqtt.publish("fauzan/test2", temperature);
 mqtt.publish("fauzan/test3", tangki);
 mqtt.publish("fauzan/test6", pwmH);
 mqtt.publish("fauzan/test7", pwmD);

}
//Atur Setpoint
void mqttReceivedMessage(char* topic, byte *msg, unsigned int msgLength) {
  if (String(topic)== "fauzan/test4"){
    Serial.println(topic);
    String perintah="";
    for (int i= 0; i<msgLength; i++){
      Serial.print(char(msg[i]));
      perintah+=String (char(msg[i]));
    }
    Serial.println("");
    Serial.println(perintah);
    setpoint=perintah.toInt();
    Serial.println(setpoint);
    EEPROM.write(0,setpoint);
    EEPROM.commit();
    }
    
     else if (String(topic)== "fauzan/test5"){
    Serial.println(topic);
    String perintah="";
    for (int i= 0; i<msgLength; i++){
      Serial.print(char(msg[i]));
      perintah+=String (char(msg[i]));
    }
    Serial.println("");
    Serial.println(perintah);
    setpoint=perintah.toInt();
    Serial.println(setpoint);
    EEPROM.write(4,setpoint);
    EEPROM.commit();
     }
  }

//String randomTemp() {
  //int randTemp = random(20,40);
  //return String(randTemp);
  //delay(1000);
//}

// Menghubungkan Wifi
void connectWiFi()
{lcd.setCursor(0,0);
  lcd.print("connecting to wifi");
  Serial.println("Connecting To WiFi");
  WiFi.begin(WiFiSSID.c_str(), WiFiPassword.c_str());
  while (WiFi.status() != WL_CONNECTED)
  {
    lcd.print(".");
    Serial.print(".");
    delay(500);
  }
  lcd.clear();
  Serial.println("Wifi Connected");
  Serial.println(WiFi.SSID());
  Serial.println(WiFi.RSSI());
  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());  
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.dnsIP());
  
}

//Menghubungkan MQTT
void connect_mqtt()
{
  while (!mqtt.connected())
  {
    Serial.println("Connecting MQTT...");
    lcd.setCursor(0,0);
  lcd.print("connecting to mqtt...");
    if (mqtt.connect("fauzan"))
    {
      mqtt.subscribe("fauzan/test");
      mqtt.subscribe("fauzan/test2");
      mqtt.subscribe("fauzan/test3");
    mqtt.subscribe("fauzan/test4");
      mqtt.subscribe("fauzan/test5");
    }

  }
}
// Menghubungkan Googlesheet
void gsheetlogdata(){
 static bool flag = false;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
      return;
    }
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%m %d %Y %H:%M:%S", &timeinfo);
    String asString(timeStringBuff);
    asString.replace(" ", "-");
    //Serial.print("Time:");
    //Serial.println(asString);
  float hum = dht.readHumidity();
  float tem = dht.readTemperature();
    String urlFinal = "https://script.google.com/macros/s/"+GOOGLE_SCRIPT_ID+"/exec?"+"date="+asString+"&hum="+String(hum)+"&temp="+String(tem)+"&humstate="+humstate+"&dehumstate="+dehumstate+"&setA="+String(setA)+"&setB="+String(setB)+"&air="+String(air);
    //Serial.print("POST data to spreadsheet:");
    //Serial.println(urlFinal);
    HTTPClient http;
    http.begin(urlFinal.c_str());
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int httpCode = http.GET(); 
    //Serial.print("HTTP Status Code: ");
    //Serial.println(httpCode);
    //---------------------------------------------------------------------
 
    String payload;
    if (httpCode > 0) {
        payload = http.getString();
        //Serial.println("Payload: "+payload);    
    }
    //---------------------------------------------------------------------
    http.end();
}

// Library Fuzzy Logic
//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Trapezoidal Member Function
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    return (FIS_TYPE) min(t1, t2);
}

// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0.f);
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trapmf, fis_trimf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 4, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 4, 4 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { 0, 0, 18, 25 };
FIS_TYPE fis_gMFI0Coeff2[] = { 18, 24, 30 };
FIS_TYPE fis_gMFI0Coeff3[] = { 27, 30, 33 };
FIS_TYPE fis_gMFI0Coeff4[] = { 30, 35, 60, 60 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4 };
FIS_TYPE fis_gMFI1Coeff1[] = { 0, 0, 40, 50 };
FIS_TYPE fis_gMFI1Coeff2[] = { 40, 50, 60 };
FIS_TYPE fis_gMFI1Coeff3[] = { 50, 60, 100, 100 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 50, 100 };
FIS_TYPE fis_gMFO0Coeff2[] = { 50, 100, 150 };
FIS_TYPE fis_gMFO0Coeff3[] = { 100, 150, 200 };
FIS_TYPE fis_gMFO0Coeff4[] = { 150, 200, 255, 255 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4 };
FIS_TYPE fis_gMFO1Coeff1[] = { 0, 0, 60, 110 };
FIS_TYPE fis_gMFO1Coeff2[] = { 85, 135, 185 };
FIS_TYPE fis_gMFO1Coeff3[] = { 160, 205, 250 };
FIS_TYPE fis_gMFO1Coeff4[] = { 225, 245, 255, 255 };
FIS_TYPE* fis_gMFO1Coeff[] = { fis_gMFO1Coeff1, fis_gMFO1Coeff2, fis_gMFO1Coeff3, fis_gMFO1Coeff4 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff, fis_gMFO1Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 1, 1, 0 };
int fis_gMFI1[] = { 0, 1, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1};

// Output membership function set
int fis_gMFO0[] = { 0, 1, 1, 0 };
int fis_gMFO1[] = { 0, 1, 1, 0 };
int* fis_gMFO[] = { fis_gMFO0, fis_gMFO1};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1 };
int fis_gRI1[] = { 1, 2 };
int fis_gRI2[] = { 1, 3 };
int fis_gRI3[] = { 2, 1 };
int fis_gRI4[] = { 2, 2 };
int fis_gRI5[] = { 2, 3 };
int fis_gRI6[] = { 3, 1 };
int fis_gRI7[] = { 3, 2 };
int fis_gRI8[] = { 3, 3 };
int fis_gRI9[] = { 4, 1 };
int fis_gRI10[] = { 4, 2 };
int fis_gRI11[] = { 4, 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11 };

// Rule Outputs
int fis_gRO0[] = { 3, 1 };
int fis_gRO1[] = { 1, 2 };
int fis_gRO2[] = { 1, 3 };
int fis_gRO3[] = { 4, 1 };
int fis_gRO4[] = { 1, 2 };
int fis_gRO5[] = { 1, 4 };
int fis_gRO6[] = { 4, 1 };
int fis_gRO7[] = { 1, 3 };
int fis_gRO8[] = { 1, 4 };
int fis_gRO9[] = { 4, 1 };
int fis_gRO10[] = { 2, 3 };
int fis_gRO11[] = { 1, 4 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 60, 100 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0, 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 255, 255 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0 };
    FIS_TYPE fuzzyOutput1[] = { 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, fuzzyOutput1, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}