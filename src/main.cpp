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
// #include <EEPROMex.h>

int interruptPin = 4; //galvanometer input
int pulseRate = 350; //base pulse rateunsigned long durationHigh;
const byte samplesize = 10; //set sample array size
const byte analysize = samplesize - 1;  //trim for analysis array
byte controlVoltage = 1; //output PWM CV on controlLED, pin 17, PB3, digital 11 *lowpass filter
byte timeout = 0;
int value = 0;
int velocity;
long unsigned duration;
long period;
int prevValue = 0;
volatile unsigned long microseconds; //sampling timer
volatile byte ind = 0;
volatile unsigned long samples[samplesize];
float threshold = 1.7;   //2.3;  //change threshold multiplier
float threshMin = 1.61; //scaling threshold min
float threshMax = 3.71; //scaling threshold max
unsigned long previousMillis = 0;
unsigned long currentMillis = 1;

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

const unsigned int outPort = 57121;         // remote port to receive OSC

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


const unsigned int localPort = 8889;      // local port to listen for OSC packets (actually not used for sending)

// DEFINIÇÃO DE IP FIXO PARA O NODEMCU
IPAddress ip(192,168,0,106); //COLOQUE UMA FAIXA DE IP DISPONÍVEL DO SEU ROTEADOR. EX: 192.168.1.110 **** ISSO VARIA, NO MEU CASO É: 192.168.0.175
IPAddress gateway(192,168,0,1); //GATEWAY DE CONEXÃO (ALTERE PARA O GATEWAY DO SEU ROTEADOR)
IPAddress subnet(255,255,255,0); //MASCARA DE REDE
WiFiServer server(80); //CASO OCORRA PROBLEMAS COM A PORTA 80, UTILIZE OUTRA (EX:8082,8089) E A CHAMADA DA URL FIC

void sample();
void setControl(int thisValue, int thisVelocity, long thisDuration);
void checkControl();
void analyzeSample();
void checkControl();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    
  WiFi.mode(WIFI_STA);
  WiFi.hostname("espoca");
  delay(2000);
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());

  Serial.print(F("WiFi connected! IP address: "));
  Serial.println(WiFi.localIP());
    
    //if you get here you have conected to the WiFi
    Serial.println("connected...yeey :)");
    attachInterrupt(interruptPin, sample, RISING);  //begin sampling from interrupt
}
// fim de setup()

void loop() {
  currentMillis = millis();   //manage time
  if(ind >= samplesize)  { analyzeSample(); }  //if samples array full, also checked in analyzeSample(), call sample analysis
    checkControl();  //update control value

    // OSCMessage msg("/suculenta");
    // msg.add(value);
    // Udp.beginPacket(outIp, outPort);
    // msg.send(Udp);
    // Udp.endPacket();
    // msg.empty();
  // outputValue = map( sensorValue, 0, 255, 0, 255);
  //analogWrite(LED_BUILTIN, outputValue);

  //  delay(3);
  //ESP.reset();
}

//interrupt timing sample array
void sample()
{
  if(ind < samplesize) {
    samples[ind] = micros() - microseconds;
    microseconds = samples[ind] + microseconds; //rebuild micros() value w/o recalling
    //micros() is very slow
    //try a higher precision counter
    //samples[index] = ((timer0_overflow_count << 8) + TCNT0) - microseconds;
    ind += 1;
  }
}

void analyzeSample()
{
  //eating up memory, one long at a time!
  unsigned long averg = 0;
  unsigned long maxim = 0;
  unsigned long minim = 100000;
  float stdevi = 0;
  unsigned long delta = 0;
  byte change = 0;

  if (ind == samplesize) { //array is full
    unsigned long sampanalysis[analysize];
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
       int dur = 150+(map(delta%127,1,127,100,2500)); //length of note
       int ramp = 3 + (dur%100) ; //control slide rate, min 25 (or 3 ;)
       
       //set scaling, root key, note
      //  int setnote = map(averg%127,1,127,noteMin,noteMax);  //derive note, min and max note
  
       //derive control parameters and set    
       setControl(averg, delta%127, ramp); //set the ramp rate for the control
     }
     //reset array for next sample
    ind = 0;
  }
}

void setControl(int thisValue, int thisVelocity, long thisDuration)
{
  value = thisValue;
  velocity = thisVelocity;
  period = thisDuration;
  duration = currentMillis + thisDuration; //schedule for update cycle
}

void checkControl()
{
  //need to make this a smooth slide transition, using high precision 
  //distance is current minus goal
  signed int distance =  velocity - value; 
  //if still sliding
  if(distance != 0) {
    //check timing
    if(currentMillis>duration) { //and duration expired
        duration = currentMillis + period; //extend duration
        //update value
       if(distance > 0) { value += 1; } else { value -=1; }
       
       //send MIDI control message after ramp duration expires, on each increment
      //  midiSerial(176, channel, controlMessage.type, controlMessage.value); 
        
        //send out control voltage message on pin 17, PB3, digital 11
        if(controlVoltage) { if(distance > 0) {
          // OSCMessage msg("/suculenta");
          // msg.add(value);
          // Udp.beginPacket(outIp, outPort);
          // msg.send(Udp);
          // Udp.endPacket();
          // msg.empty();  
          Serial.print("sensor = ");
          Serial.println(value);
        }
        }
    }
  }
}