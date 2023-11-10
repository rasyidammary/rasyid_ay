#include <PID_v1.h>
#include <ESPRotary.h>
#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#elif defined(ESP32)
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#endif
#include <ESPDash.h>
#define ppr 7
#define encA  D2
#define encB D1
#define ENCODEROUTPUT 35                     // radius kyone
ESPRotary r;
volatile long encoderValue = 0;
int interval = 60;                          //kudune 1000 tak ganti sek
long previousMillis = 0;
long currentMillis = 0;
int rpm = 0;
const int PWM = D8;
const int DIR1 = D7;
const int DIR2 = D6;
int PWM_ = 0;
int tekan = 0;
int tombol = 0;
double Setpoint, Input, Output;

const char* ssid = "Putar Motor"; // SSID
const char* password = "12345678"; // Password

double Kp, Ki, Kd;//2,5,1
float sp;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

AsyncWebServer server(80);

ESPDash dashboard(&server);

Card slider1(&dashboard, SLIDER_CARD, "ki", "", 0, 100);
Card slider2(&dashboard, SLIDER_CARD, "kp", "", 0, 100);
Card slider3(&dashboard, SLIDER_CARD, "kd", "", 0, 100);
Card slider4(&dashboard, SLIDER_CARD, "sp", "", 0, 500);
String XAxis[] = {"100", "200", "300", "400", "500", "600", "700"};
int YAxis[] = {0};
Chart power(&dashboard, BAR_CHART, "RPM");

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 77, 1), IPAddress(192, 168, 77, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  power.updateX(XAxis, 7);
  server.begin();

  r.begin(encA, encB, ppr);

  r.setChangedHandler(rotate);
  myPID.SetMode(AUTOMATIC);

  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(tombol, INPUT_PULLUP);
}

void rotate(ESPRotary& r) {
  encoderValue = r.getPosition();
}

void loop() {
  RPM();
  Input = rpm;
  Setpoint = sp;
  myPID.Compute();
  analogWrite(PWM, Output);

  slider1.attachCallback([&](int value) {
    Ki = value;
    slider1.update(value);
    dashboard.sendUpdates();
    myPID.SetTunings(Kp, Ki, Kd);

  });
  slider2.attachCallback([&](int value) {
    Kp = value;
    slider2.update(value);
    dashboard.sendUpdates();
    myPID.SetTunings(Kp, Ki, Kd);

  });
  slider3.attachCallback([&](int value) {
    Kd = value;
    slider3.update(value);
    dashboard.sendUpdates();
    myPID.SetTunings(Kp, Ki, Kd);

  });
  slider4.attachCallback([&](int value) {
    sp = value;
    slider4.update(sp);
    dashboard.sendUpdates();
  });

//     for(int i=0; i < 7; i++){
//      YAxis[i] = int()(0, 50000);
//    }
//    power.updateY(YAxis, 1);
//     dashboard.sendUpdates();

  r.loop();
  tekan = digitalRead(tombol);
  if ( tekan == 1) {
    digitalWrite(DIR1, 1);
    digitalWrite(DIR2, 0);
  }
  else {
    digitalWrite(DIR1, 0);
    digitalWrite(DIR2, 1);
  }
}
void RPM () {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    rpm = (float)(encoderValue * 60 / ENCODEROUTPUT);//60
    Serial.println(rpm);
    encoderValue = 0;
    r.resetPosition();
  }
}
