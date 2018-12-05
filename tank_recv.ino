//This is the tank reciever code, select Arduino Mega when compiling

/*
PWM Info for Arduino Mega
0x01  1 62500
0x02  8 7812.5
0x03  64  976.5625
0x04  256 244.140625
0x05  1024  61.03515625
TCCR0B = TCCR0B & 0b11111000 | <setting>;[/
PWM 
timer 0 (controls pin 13, 4)
timer 1 (controls pin 12, 11)
timer 2 (controls pin 10, 9)
timer 3 (controls pin 5, 3, 2)
timer 4 (controls pin 8, 7, 6)
http://forum.arduino.cc/index.php?topic=16612#msg121031
*/
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h" //for the NRF library

uint8_t address[] = "Tiger";
uint8_t packet[16];

const int fanpin = A7;
const int thermistor_vcc = A6;
const int thermistor = A5;
const int thermistor_gnd = A4;
const int thermistor_threshold = 560;
int temperature = 0;

const int leftfwpin   = 12;
const int leftbackpin = 11;
const int rightfwpin  = 10;
const int rightbackpin  = 9;
const int turretleftpin = 8;
const int turretrightpin= 7;
const int turretuppin = 6;
const int turretdownpin = 5;
const int firepin   = 4;
const int shotfeedbackpin = 3; 
const int lightpin    = 2;
const int smokepin    = 44;

const int debugin = 46;
const int debugout = 47;
bool debug = 1;


String serialpacket = ""

RF24 radio(9, 10); //CE & CS


unsigned long lastshottime = millis();
const unsigned long shotdelay = 3000;

unsigned long lastpacketreceived = millis();
const unsigned long packettimeout = 250;

void stopmotion(){
  analogWrite(leftfwpin    , 0);
  analogWrite(leftbackpin  , 0);
  analogWrite(rightfwpin   , 0);
  analogWrite(rightbackpin , 0);
  analogWrite(turretleftpin ,0);
  analogWrite(turretrightpin,0);
  analogWrite(turretuppin  , 0);
  analogWrite(turretdownpin ,0);
}
int good_analog(int pin, const byte samples){
  int tmp = 0;
  for (byte b=0; b<samples;b++){
    tmp = tmp + analogRead(pin);
  }
  return (tmp/samples) ; //[0-1023]
}

void setup(){
  pinMode(debugout,OUTPUT);
  pinMode(debugin,INPUT_PULLUP);
  digitalWrite(debugout,LOW);
  if (digitalRead(debugin) == 1){
    debug = 1;
    }
  if (debug) 
  Serial.begin(115200);

  pinMode(thermistor_vcc,OUTPUT);
  digitalWrite(thermistor_vcc,HIGH);
  pinMode(thermistor_gnd,OUTPUT);
  digitalWrite(thermistor_gnd,LOW);
  pinMode(thermistor,INPUT);
  pinMode(fanpin,OUTPUT);
  digitalWrite(fanpin,LOW);
  analogRead(thermistor);
  temperature = good_analog(thermistor,8);

  
  printf_begin();
  pinMode(leftfwpin   , OUTPUT);
  pinMode(leftbackpin   , OUTPUT);
  pinMode(rightfwpin    , OUTPUT);
  pinMode(rightbackpin  , OUTPUT);
  pinMode(turretleftpin , OUTPUT);
  pinMode(turretrightpin  , OUTPUT);
  pinMode(turretuppin   , OUTPUT);
  pinMode(turretdownpin , OUTPUT);
  pinMode(firepin     , OUTPUT);
  pinMode(lightpin    , OUTPUT);
  pinMode(smokepin    , OUTPUT);
  pinMode(shotfeedbackpin, INPUT_PULLUP);
  
  stopmotion();
  digitalWrite(firepin    , LOW);
  digitalWrite(lightpin   , HIGH);
  digitalWrite(smokepin   , LOW);

  //We need 120hz pwm, even though it might be far from ideal
  // note 120hz pwm is better with fast decay, but still judders like mad
  // better to use 30khz combo'd with slow decay.
  TCCR1B = TCCR1B & 0b11111000 | 0x01 ;  //
  //For timer 1,3,4,5:[0;7]:[X, 1, 8, 64, 256,1024, EXT_FALL,EXT_RISE]
  TCCR2B = TCCR2B & 0b11111000 | 0x01; //1
  // For timer2: [0;7]:[X,1,8,32,64,128,256,1024]
  // Note that phase-correctness doubles period
  
  
  TCCR3B = TCCR3B & 0b11111000 | 0x01;//4
  TCCR4B = TCCR4B & 0b11111000 | 0x01;//4
  TCCR5B = TCCR5B & 0b11111000 | 0x01;//4

  
  if( radio.begin()) Serial.println("radio.begin() Success");
  else Serial.println("radio.begin() FAILED");
  
  radio.setChannel(125);
  radio.setPayloadSize(sizeof(packet));
  radio.setPALevel(RF24_PA_MAX);
  if( ! radio.setDataRate(RF24_250KBPS)){
    if (debug) Serial.println("Failed to set data rate to 250kbps");
  }
  
  radio.openReadingPipe(1,address);
  radio.startListening();
  if (debug)
    radio.printDetails();
  Serial.println("rdy");
  serialpacket.reserve(128);
  for (byte i =0; i<sizeof(packet); i++) packet[i] = 0; //init packet
}

void printpacket(){
  if (!debug) return;
      for (byte i =0; i<sizeof(packet); i++){
        Serial.print(i);
        Serial.print(":");
        Serial.print(packet[i]);
        Serial.print("\t" );
      }
      Serial.println();
  
  }

void printtimers(){
  if (!debug) return;
  Serial.print("TCCR1A ");
  Serial.println(TCCR1A,BIN);
  Serial.print("TCCR1B ");
  Serial.println(TCCR1B,BIN);
  Serial.print("TCCR1C ");
  Serial.println(TCCR1C,BIN);
  
  Serial.print("OCR1B_16 ");
  Serial.println(OCR1BL + 256*OCR1BH);

  Serial.print("TCCR2A ");
  Serial.println(TCCR2A,BIN);
  Serial.print("TCCR2B ");
  Serial.println(TCCR2B,BIN);
  
  }
  
void ramp(int pin, int target, int inertia){
  //Add some inertia to the system, like maybe a full sec at full swing?
  //To prevent shoot-through, targets of 0 will be set immedatiely!
  if (target == 0){
    if (decaymode == SLOW){
      packet[pin] = 255;
    }
    if (decaymode == FAST){
      packet[pin] = 0;
      }
    return;
  }
 
  if (packet[pin] - target > 2*inertia){ // slow down ramp is faster
    packet[pin] = packet[pin] - 2*inertia;
  }else if(target - packet[pin] > inertia){
    packet[pin] = packet[pin]+inertia;    
  }else{
    packet[pin]=target;
  }
}
void process(int pin, int pack){
  if (pack==0){ digitalWrite(pin,LOW);}
  else if (pack ==255) {digitalWrite(pin,HIGH);}
  else {analogWrite(pin,pack);}
  }
  
void doall(){
	lastpacketreceived = millis();

	process(leftfwpin   ,packet[leftfwpin   ]);
	process(leftbackpin   ,packet[leftbackpin   ]);
	process(rightfwpin    ,packet[rightfwpin    ]);
	process(rightbackpin  ,packet[rightbackpin  ]);
	process(turretleftpin ,packet[turretleftpin ]);
	process(turretrightpin  ,packet[turretrightpin  ]);
	process(turretuppin   ,packet[turretuppin   ]);
	process(turretdownpin ,packet[turretdownpin ]);
	process(lightpin    ,packet[lightpin    ]);
	process(smokepin    ,packet[1          ]);

	printtimers();
	if (packet[firepin] && lastshottime+shotdelay < millis()){
		digitalWrite(firepin,HIGH);
		lastshottime=millis();
	}
	
}


void loop(void){
  if (lastpacketreceived + packettimeout < millis() ){
    Serial.print('.');
    Serial.print("T=");
    Serial.println(temperature);
    stopmotion();
    analogWrite(lightpin,0);
    lastpacketreceived = millis();
  }
  if (lastshottime + 800 < millis()){ // wait a bit so that the switch can deactivate again
    if (digitalRead(shotfeedbackpin) == 0 ){
      digitalWrite(firepin,LOW);
     }
  }
  
  if (lastshottime+shotdelay < millis()){
    digitalWrite(firepin,LOW);
  }
  
  while (Serial.available() > 0){
    char inchar;
	inchar = Serial.read();
	serialpacket+= inchar;
	if (inchar == '\n'){
		//Time to parse the packet. 
		int checksum = -1;
		int n = sscanf(serialpacket, 'LF:%d;LB:%d;RF:%d;RB:%d;TL:%d;TR:%d;TL:%d;TU:%d;TD:%d;LI:%d;FP:%d;CS:%d\n',
			packet + leftfwpin,
			packet + leftbackpin,
			packet + rightfwpin,
			packet + rightbackpin,
			packet + turretleftpin,
			packet + turretrightpin,
			packet + turretuppin,
			packet + turretdownpin,
			packet + lightpin,
			packet + firepin,
			&checksum)
		int sum = 0;
		for (uint8_t i = 0; i<16;i++){
			sum += packet[i];
		}
		if (sum == checksum){
			Serial.print("OK;");
			Serial.print(serialpacket);
			serialpacket = "";
			doall();
		}else{
			Serial.print("BAD;");
			Serial.println(serialpacket);
			serialpacket = "";
		}
	}
  }
  
  if ( radio.available() ){
    Serial.print('k');
    while (radio.available()){
      radio.read( packet, sizeof(packet));
      if (debug){
        Serial.print("Got packet with delay of:\n\r");
        Serial.println(millis()-lastpacketreceived);
        printpacket();
      }
      doall();
      
    }
  }
  temperature = good_analog(thermistor,8);

  if (temperature  < thermistor_threshold)  digitalWrite(fanpin,HIGH);
  delay(1);
}

