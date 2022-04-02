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

int interruptPin = 4; //galvanometer input
const byte samplesize = 4; //set sample array size
const byte analysize = samplesize - 1;  //trim for analysis array
volatile unsigned long microseconds; //sampling timer
volatile byte ind = 0;
volatile word samples[samplesize];
float threshold = 0.1;   //2.3;  //change threshold multiplier

const char* ssid = "rede_miceliar"; // VARIÁVEL QUE ARMAZENA O NOME DA REDE SEM FIO
const char* password = "2478Esta"; // 24378Skmecs VARIÁVEL QUE ARMAZENA A SENHA DA REDE SEM FIO

WiFiUDP Udp;

// MUDE AQUI MUDE AQUI MUDE AQUI MUDE 
//     AQUI MUDE AQUI MUDE AQUI MUDE A
//          QUI MUDE AQUI MUDE AQUI MUD
//              E AQUI MUDE AQUI MUDE AQ
//                  QUI MUDE AQUI MUDE A
//                      QUI MUDE AQUI M
//                           UDE AQUI M
//                                UDE A
//                                  QU

const IPAddress outIp(192,168,1,103);  // ENDEREÇO IP DO COMPUTADOR

/////////////////////////////////////////////////////////////////////

const unsigned int outPort = 57121;         // remote port to receive OSC

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


const unsigned int localPort = 8890;      // local port to listen for OSC packets (actually not used for sending)

// DEFINIÇÃO DE IP FIXO PARA O NODEMCU
IPAddress ip(192,168,1,255); //COLOQUE UMA FAIXA DE IP DISPONÍVEL DO SEU ROTEADOR. EX: 192.168.1.110 **** ISSO VARIA, NO MEU CASO É: 192.168.0.175
IPAddress gateway(192,168,1,1); //GATEWAY DE CONEXÃO (ALTERE PARA O GATEWAY DO SEU ROTEADOR)
IPAddress subnet(255,255,255,0); //MASCARA DE REDE
WiFiServer server(80); //CASO OCORRA PROBLEMAS COM A PORTA 80, UTILIZE OUTRA (EX:8082,8089) E A CHAMADA DA URL FIC

IRAM_ATTR void sample();
void analyzeSample();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.hostname("floresta");
  delay(2000);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }

  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());

  Serial.print(F("WiFi connected! IP address: "));
  Serial.println(WiFi.localIP());
    
  //   //if you get here you have conected to the WiFi
    Serial.println("connected...yeey :)");
    attachInterrupt(interruptPin, sample, CHANGE);  //begin sampling from interrupt
}
// fim de setup()

word value = 0;
unsigned long antes = 0;
const byte intervalo = 10;

void loop() {
  unsigned long agora = millis();
    // delay(500);
  if( ind >= samplesize )  { analyzeSample(); }
  if ( agora - antes >= intervalo )  { antes = agora; 
    OSCMessage msg("/suculenta");
    msg.add(value);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    // Serial.println(value);
  }
}

//interrupt timing sample array
void sample()
{
  if(ind < samplesize) {
    samples[ind] = micros() - microseconds;
    microseconds = samples[ind] + microseconds;
    ind += 1;
    // Serial.println(samples[ind]);
  }
}

void analyzeSample()
{
  //eating up memory, one long at a time!
  word averg = 0;
  word maxim = 0;
  word minim = 65535;
  float stdevi = 0;
  word delta = 0;
  byte change = 0;

  if (ind == samplesize) { //array is full
    word sampanalysis[analysize];
    for (byte i=0; i<analysize; i++){ 
      //skip first element in the array
      sampanalysis[i] = samples[i+1];  //load analysis table (due to volitle)
      //manual calculation
      if(sampanalysis[i] > maxim) { maxim = sampanalysis[i]; }
      if(sampanalysis[i] < minim) { minim = sampanalysis[i]; }
      averg += sampanalysis[i];
      stdevi += sampanalysis[i] * sampanalysis[i];  //prep stdevi
    }
    //manual calculation
    averg = averg/analysize;
    stdevi = sqrt(stdevi / analysize - averg * averg); //calculate stdevu
    if (stdevi < 1) { stdevi = 1.0; } //min stdevi of 1
    delta = maxim - minim; 
    //**********perform change detection 
    if (delta > (stdevi * threshold)){
      change = 1;
    }
    //*********
    if(change){// set note and control vector
      value = averg;
    // Serial.println(value);
     }
     //reset array for next sample
      ind = 0;
  }
}
