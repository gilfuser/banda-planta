#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif
#include <DNSServer.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <OSCMessage.h> // instalar via IDE
// #include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager


int sensorPin = A0;
int sensorValue = 0;
// int outputValue = 0;

const char* ssid = "Popoloninis"; // VARIÁVEL QUE ARMAZENA O NOME DA REDE SEM FIO
const char* password = "cibpopfu5"; // 24378Skmecs VARIÁVEL QUE ARMAZENA A SENHA DA REDE SEM FIO

WiFiUDP Udp;

// MUDE AQUI MUDE AQUI MUDE AQUI MUDE AQUI MUDE AQUI MUDE
//     AQUI MUDE AQUI MUDE AQUI MUDE AQUI MUDE AQUI MU
//          DE AQUI MUDE AQUI MUDE AQUI MUDE AQUI MU
//              DE AQUI MUDE AQUI MUDE AQUI MUDE A
//                  QUI MUDE AQUI MUDE AQUI MUD
//                      E AQUI MUDE AQUI MUD
//                           E AQUI MUDE
//                                AQUI
//                                 V

const IPAddress outIp(192, 168, 0, 55);  // ENDEREÇO IP DO COMPUTADOR

/////////////////////////////////////////////////////////////////////

const unsigned int outPort = 57120;         // remote port to receive OSC

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


const unsigned int localPort = 8888;      // local port to listen for OSC packets (actually not used for sending)

// DEFINIÇÃO DE IP FIXO PARA O NODEMCU
IPAddress ip(192,168,0,106); //COLOQUE UMA FAIXA DE IP DISPONÍVEL DO SEU ROTEADOR. EX: 192.168.1.110 **** ISSO VARIA, NO MEU CASO É: 192.168.0.175
IPAddress gateway(192,168,0,1); //GATEWAY DE CONEXÃO (ALTERE PARA O GATEWAY DO SEU ROTEADOR)
IPAddress subnet(255,255,255,0); //MASCARA DE REDE
WiFiServer server(80); //CASO OCORRA PROBLEMAS COM A PORTA 80, UTILIZE OUTRA (EX:8082,8089) E A CHAMADA DA URL FIC


void setup() {
    // put your setup code here, to run once:
    // Serial.begin(115200);

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    // WiFiManager wifiManager;
    //reset saved settings
    //wifiManager.resetSettings();

    pinMode(sensorPin, INPUT);
    
    //set custom ip for portal
    // wifiManager.setAPStaticIPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

    //fetches ssid and pass from eeprom and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    // wifiManager.autoConnect("AutoConnectAP");
    //or use this for auto generated name ESP + ChipID
    //wifiManager.autoConnect();

  WiFi.mode(WIFI_STA);
  WiFi.hostname("esposc");
  delay(1000);
  // Serial.println();
  // Serial.println();
  // Serial.print("Connecting to ");
  // Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }

  //Udp.begin(localPort);
  //Serial.print("Local port: ");
  //Serial.println(Udp.localPort());

  // Serial.print(F("WiFi connected! IP address: "));
  // Serial.println(WiFi.localIP());
    
    //if you get here you have connected to the WiFi
    // Serial.println("connected...yeey :)");
}
// fim de setup()

void loop() {
  sensorValue = analogRead(sensorPin);
    OSCMessage msg("/bromelia");
    msg.add(sensorValue);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
  // outputValue = map( sensorValue, 0, 255, 0, 255);
  //analogWrite(LED_BUILTIN, outputValue);
  //Serial.print("sensor = ");
  // Serial.print(sensorValue);
   delay(3);
  //ESP.reset();
}