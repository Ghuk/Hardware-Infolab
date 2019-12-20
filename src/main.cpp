
#include <Arduino.h>
#include <HTTPClient.h>
#include <../lib/ArduinoJson-v6.11.3.h>
#include <../lib/NTPClient/NTPClient.h>
#include <WiFiUdp.h>

#include <../lib/DHT/DHT.h>
#include <HardwareSerial.h>
#include "../lib/DFRobotDFPlayerMini/DFRobotDFPlayerMini.h"
#include <../lib/NewPingESP8266/NewPingESP8266.h>
#include <SD.h>
#include <AsyncWebSocket.h>
#include "WiFi.h"
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <../lib/Time/Time.h>
#include <../lib/Adafruit_APDS9960/Adafruit_APDS9960.h>
#include <../lib/MPU9250/MPU9250.h>
#include <Wire.h>
#include <SPI.h>
#include <../lib/Adafruit_BMP280_Library/Adafruit_BMP280.h>

#include <../lib/FastLED/FastLED.h>


#define SERIAL2_RX 17
#define SERIAL2_TX 21

#define DHTTYPE DHT22   // DHT 22
const int DHTPin = 5; // pin digital donde esta conectado el sensor de temperatura  y humedad
DHT dht(DHTPin, DHTTYPE);

const int VENTANA_MUESTREO = 50; // ventana de muestreo en ms (50 ms = 20Hz)


const int MICROFONO = A2; // pin analogico donde esta conectado el microfono
const int RESOLUCION_MUESTRA = 4095; // resolucion de muestreo
const float VOLTAJE_MUESTRA = 3.3; // voltaje de referencia para el ESP32

int VENTANA_POSTEO = 1000;

const int SENSORLLAMA = 32; // pin digital donde esta conectado el sensor de llama

const int SENSOR_CALIDAD_AIRE = A3;

const int SENSOR_ULTRASONIDO_TRIGGER = 15;
const int SENSOR_ULTRASONIDO_ECHO = 33;
const int maxDistanciaUltrasonido = 200;

//const int SENSOR_PIR = 4;
const int SENSOR_PIR = 27;

Adafruit_APDS9960 apds;
MPU9250 mpu = MPU9250(); 
Adafruit_BMP280 bmp;


NewPingESP8266 sonar(SENSOR_ULTRASONIDO_TRIGGER, SENSOR_ULTRASONIDO_ECHO, maxDistanciaUltrasonido);

// https://electronics.stackexchange.com/questions/96205/how-to-convert-volts-in-db-spl
// La sensibilidad del microfono es de -58db, entonces V RMS / PA es 0.001259
// conversion usando : http://www.sengpielaudio.com/calculator-gainloss.htm
const float VOLTAJE_GANANCIA_PERDIDA = 0.001259;
//const float VOLTAJE_GANANCIA_PERDIDA = 0.00631; // -44dB https://cdn-shop.adafruit.com/datasheets/CMA-4544PF-W.pdf
//const float VOLTAJE_GANANCIA_PERDIDA = 0.0004;
int muestra;

//Configuracion del WiFi, nombre y contraseña

const char* ssid = "WiFi-UAO";
const char* password =  "";

String URL;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

//Define variable del mp3
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

float promedioVolts_db = 0;
int ctrRuido = 0;
float sumaVolts_db = 0;

String postMessage = "";

bool sonido = false;
static unsigned long timerLed;
static unsigned long timerParlante;

char actuador;
char nodo;
char nodoRecibido;

//Configuracion Socket

AsyncWebServer server(80);
AsyncWebSocket ws("/test");

// Numero de leds en la tira
#define NUM_LEDS 60

// Configuracion tira led
#define DATA_PIN 19
#define CLOCK_PIN 16

// Crea arreglo de leds
CRGB leds[NUM_LEDS];

//Socket
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){

  if(type == WS_EVT_CONNECT){

    Serial.println("Websocket client connection received");

  } else if(type == WS_EVT_DISCONNECT){
    Serial.println("Client disconnected");
    Serial.println("-----------------------");

  } else if(type == WS_EVT_DATA){

    Serial.println("Data received");

    nodoRecibido = (char)data[0];
    actuador = (char)data[1];

    Serial.println(nodoRecibido);
    Serial.println(actuador);

  }
}


void Post(String fechaPost, float promedioVolts_dbPost, float humedad, float temperatura, int llama, float calidadAire, int ultrasonido, int PIR, uint16_t iluminacion, int16_t acelerometro [], int16_t giroscopio [], int16_t magnetometro [], int altitud){
  
  // Crea JSON
  StaticJsonDocument<30> sensor1;
  sensor1["idSensor"] = 1;
  sensor1["valor"] = promedioVolts_dbPost;

  StaticJsonDocument<30> sensor2;
  sensor2["idSensor"] = 2;
  sensor2["valor"] = humedad;

  StaticJsonDocument<30> sensor3;
  sensor3["idSensor"] = 3;
  sensor3["valor"] = temperatura;

  StaticJsonDocument<30> sensor4;
  sensor4["idSensor"] = 4;
  sensor4["valor"] = llama;

  StaticJsonDocument<30> sensor5;
  sensor5["idSensor"] = 5;
  sensor5["valor"] = calidadAire;

  StaticJsonDocument<30> sensor6;
  sensor6["idSensor"] = 6;
  sensor6["valor"] = ultrasonido;

  StaticJsonDocument<30> sensor7;
  sensor7["idSensor"] = 7;
  sensor7["valor"] = PIR;

  StaticJsonDocument<30> sensor8;
  sensor8["idSensor"] = 8;
  sensor8["valor"] = iluminacion;

  StaticJsonDocument<30> sensor9;
  sensor9["idSensor"] = 9;
  sensor9["valor"] = acelerometro[0];

  StaticJsonDocument<30> sensor10;
  sensor10["idSensor"] = 10;
  sensor10["valor"] = acelerometro[1];

  StaticJsonDocument<30> sensor11;
  sensor11["idSensor"] = 11;
  sensor11["valor"] = acelerometro[2];

  StaticJsonDocument<30> sensor12;
  sensor12["idSensor"] = 12;
  sensor12["valor"] = giroscopio[0];

  StaticJsonDocument<30> sensor13;
  sensor13["idSensor"] = 13;
  sensor13["valor"] = giroscopio[1];

  StaticJsonDocument<30> sensor14;
  sensor14["idSensor"] = 14;
  sensor14["valor"] = giroscopio[2];

  StaticJsonDocument<30> sensor15;
  sensor15["idSensor"] = 15;
  sensor15["valor"] = magnetometro[0];

  StaticJsonDocument<30> sensor16;
  sensor16["idSensor"] = 16;
  sensor16["valor"] = magnetometro[1];

  StaticJsonDocument<30> sensor17;
  sensor17["idSensor"] = 17;
  sensor17["valor"] = magnetometro[2];

  StaticJsonDocument<30> sensor18;
  sensor18["idSensor"] = 18;
  sensor18["valor"] = altitud;

  StaticJsonDocument<950> doc;
  doc["marcaTiempo"] = fechaPost;
  doc["idNodo"] = 1;

  JsonArray sensores = doc.createNestedArray("sensores");
  sensores.add(sensor1);
  sensores.add(sensor2);
  sensores.add(sensor3);
  sensores.add(sensor4);
  sensores.add(sensor5);
  sensores.add(sensor6);
  sensores.add(sensor7);
  sensores.add(sensor8);
  sensores.add(sensor9);
  sensores.add(sensor10);
  sensores.add(sensor11);
  sensores.add(sensor12);
  sensores.add(sensor13);
  sensores.add(sensor14);
  sensores.add(sensor15);
  sensores.add(sensor16);
  sensores.add(sensor17);
  sensores.add(sensor18);
  
  serializeJson(doc, postMessage);
  Serial.println(postMessage);
  if(WiFi.status()== WL_CONNECTED){
    HTTPClient http;
    http.begin(URL);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(postMessage);
    
    Serial.print("http result:");
    Serial.println(httpCode);
    
    http.writeToStream(&Serial);
    String payload = http.getString();
    http.end();
    delay(70);
  }else{
    Serial.print("Error in Wifi connection");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.println("Connecting to WiFi..");
    }
  }
  delay(70);
  postMessage = "";
  doc.clear();
  sensor1.clear();
  sensor2.clear();
  sensor3.clear();
  sensor4.clear();
  sensor5.clear();
  sensor6.clear();
  sensor7.clear();
  sensor8.clear();
  sensor9.clear();
  sensor10.clear();
  sensor11.clear();
  sensor12.clear();
  sensor13.clear();
  sensor14.clear();
  sensor15.clear();
  sensor16.clear();
  sensor17.clear();
  sensor18.clear();
}

void setup() {
  LEDS.addLeds<P9813, DATA_PIN, CLOCK_PIN>(leds, NUM_LEDS);
  nodo = '1';
  URL = "http://11.11.6.30:3000/registros/insert-register";

  actuador = '0';

  // Establece la velocidad de la transmisiÃ³n mediante puerto serial
  Serial.begin(9600);
  // Establece la configuracion del WiFi
  WiFi.begin(ssid, password);

  Serial2.begin( 9600, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX );

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  // Muestra la IP local
  Serial.println(WiFi.localIP());

  //Configuracion socket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/ws.html", "text/html");
  });
  server.begin();

  //Inicia dht sensor
  dht.begin();

  if(!apds.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");

  apds.enableColor(true);

  uint8_t temp = mpu.begin();

  bmp.begin(BMP280_ADDRESS);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  timeClient.begin();
  timeClient.setTimeOffset(-18000);
  
  while(!timeClient.update()) {
    Serial.println("Actualizando fecha y hora");
    timeClient.forceUpdate();
  }

  String formattedDate = timeClient.getFormattedDate();
  int splitT = formattedDate.indexOf("T");
  // Extract day
  String dayStamp = formattedDate.substring(0, splitT);
  // Extract time
  String timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-1);

  String ano = dayStamp.substring(0, 4);
  String mes = dayStamp.substring(5, 7);
  String dia = dayStamp.substring(8, 10);

  String hora = timeStamp.substring(0, 2);
  String minuto = timeStamp.substring(3, 5);
  String segundo = timeStamp.substring(6, 8);

  setTime(hora.toInt(), minuto.toInt(), segundo.toInt(), dia.toInt(), mes.toInt(), ano.toInt());

  if (!myDFPlayer.begin(Serial2)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("No pudo iniciar el mp3:"));
    Serial.println(F("1. Verificar coneccion"));
    Serial.println(F("2. Insertar un tarjeta SD"));
  }
}

String print2digits(int number) {
  String number2 = "";
  if (number >= 0 && number < 10) {
      number2 = "0" + String(number);
  }else{
    number2 = String(number);
  }
  return number2;
}

void loop() {
    
    long tiempoInicioMuestreo = millis(); // inicio de la ventana de muestreo
    int picoPico = 0; // nivel pico a pico

    int muestraMaxima = 0;
    int muestraMinima = 4094;

    while (millis() - tiempoInicioMuestreo < VENTANA_MUESTREO) {
      muestra = analogRead(MICROFONO);
      if (muestra < RESOLUCION_MUESTRA) {  // Omite lectura falsas
        if (muestra > muestraMaxima) {
          muestraMaxima = muestra; // almacena el nivel mÃ¡ximo
        } else if (muestra < muestraMinima) {
          muestraMinima = muestra; // almacena el nivel mÃ­nimo
        }
      }
    }

    if(millis() <= VENTANA_POSTEO) {
      picoPico = muestraMaxima - muestraMinima; 
      double voltaje = (picoPico * VOLTAJE_MUESTRA) / RESOLUCION_MUESTRA; // convierte el valor a voltaje
      double volts_db = 20*log10(voltaje/VOLTAJE_GANANCIA_PERDIDA);
      ctrRuido++;
      sumaVolts_db += volts_db;
    } else {
      
      String fecha = String(year()) + "-" + print2digits(month()) + "-" + print2digits(day()) + " " + print2digits(hour()) + ":" + print2digits(minute()) + ":" + print2digits(second());
      
      //Actuadores
      sonido = digitalRead(18);

      if(actuador == '1' && sonido == HIGH  && nodo == nodoRecibido){
        timerParlante = millis();
        myDFPlayer.volume(10);
        myDFPlayer.play(1);
        actuador = 0;
      }

      if(actuador == '2' && nodo == nodoRecibido){
        timerLed = millis();
        actuador = 0;
      }

      if(millis() - timerLed < 10000){
          leds[0] = CRGB::Red;
          FastLED.show();
      }else{
          leds[0] = CRGB::Black;
          FastLED.show();
          actuador = 0;
      }

      //Lectura de sensores
      //Sensor ruido
      promedioVolts_db = sumaVolts_db/ctrRuido;
      //Sensor humedad
      float humedad = dht.readHumidity();
      //Sensor temperatura
      float temperatura = dht.readTemperature();
      //Sensor llama
      int llama = digitalRead(SENSORLLAMA);
      //Sensor aire
      int raw_adc = analogRead(SENSOR_CALIDAD_AIRE);
      float value_adc = raw_adc * (5.0 / 4092);
      //Sensor distancia por ultrasonido
      int ultrasonido = sonar.ping_cm();
      //Sensor PIR
      int PIR = digitalRead(SENSOR_PIR);
      //Sensor altitud
      int altitud = bmp.readAltitude(1018);
      //Sensor iluminacion
      uint16_t r, g, b, c;
      if(apds.colorDataReady()){
        apds.getColorData(&r, &g, &b, &c);
      }
      uint16_t iluminacion = apds.calculateLux(r, g, b);
      //Sensor acelerometro
      mpu.set_accel_range(RANGE_4G);
      mpu.get_accel();
      int16_t acelerometro []= {mpu.x, mpu.y, mpu.z};
      //Sensor giroscopio
      mpu.set_gyro_range(RANGE_GYRO_250);
      mpu.get_gyro();
      int16_t giroscopio [] = {mpu.gx, mpu.gy, mpu.gz};
     //Sensor magnetometro
      mpu.set_mag_scale(SCALE_14_BITS);
      mpu.set_mag_speed(MAG_8_Hz);
      mpu.get_mag();
      int16_t magnetometro [] = {mpu.mx, mpu.my, mpu.mz};

      if(promedioVolts_db > 0){
        //Envio de datos al web service
        Post(fecha, promedioVolts_db, humedad, temperatura, llama, value_adc, ultrasonido, PIR, iluminacion, acelerometro, giroscopio, magnetometro, altitud);
      }

      sumaVolts_db = 0;
      ctrRuido = 0;
      VENTANA_POSTEO += 1000;

    }
}

