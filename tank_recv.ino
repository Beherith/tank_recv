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
//TODO:
//Laser command
//
//example command: LF:1;LB:1;RF:1;RB:1;TL:1;TR:1;TL:1;TU:1;TD:1;LI:1;FP:0;LA:1;CS:0\n'
//low-voltage cutoff for motors!

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h" //for the NRF library
#include <MPU9255.h>

uint8_t address[] = "Tiger";
uint8_t packet[16]; //Very important, stores the state of the whole tank!
uint8_t packetRadio[16];
bool useRadio = 0;
RF24 radio(48, 49); //CE & CS

const int fanpin = A7;
const int thermistor_vcc = A6;
const int thermistor = A5;
const int thermistor_gnd = A4;
const int thermistor_threshold = 560;
int temperature = 0;

const int leftfwpin   = 10;//12;
const int leftbackpin = 9;//11;
const int rightfwpin  = 12;//10;
const int rightbackpin  = 11;//9;
const int turretleftpin = 7;//8;
const int turretrightpin= 8;//7;
const int turretuppin = 6;
const int turretdownpin = 5;
const int firepin   = 4;
const int shotfeedbackpin = 3; 
const int lightpin    = 2;
const int laserpos = 13;
const int smokepin    = 44; //unused

const int inputvoltagelow = A1;
const int inputvoltagesense = A0;

const int laserlow = 44;
const int laserpin = 45;

const int builtin_led_pin = 13;

//Inertia info:

const int inertia = 25;
//from packet to packet, we will only allow packet deltat*inertia 

bool debug = 1;
bool inmotion = 0;

const int VOLTAGE_MULTIPLIER  = 15; //mV per ADC unit //(5.0*/1023.0)*3.0; //a 2R/R divisor is on VIN to A1; using 10K resistors
const int VOLTAGE_CUTOFF = 6000;
int VCCmV = 0; 
char serialpacket[200];
uint8_t serialpacketptr = 0;


// MPU9255 Gyro+accel
const int MPU9255_VCC = 23;
const int MPU9255_GND = 25;
const int MPU9255_SCL = 27;
const int MPU9255_SDA = 29;
char gyrobuf[200];
MPU9255 mpu;

// Various update rates
unsigned long lastshottime = millis();
const unsigned long shotdelay = 3000;

unsigned long packetLastReceived = millis();
unsigned long packetTimeout = 250;

unsigned long gyroLastSent = millis();
unsigned long gyroTimeout = 1000;



void stopMotion(){
	packet[leftfwpin		] = 0;
	packet[leftbackpin		] = 0;
	packet[rightfwpin		] = 0;
	packet[rightbackpin		] = 0;
	packet[turretleftpin	] = 0;
	packet[turretrightpin	] = 0;
	packet[turretuppin		] = 0;
	packet[turretdownpin	] = 0;
	updatePhysicalState();
	inmotion = 0;
	if (debug){
			Serial.println("STOP All");		
	}
}

uint16_t analogReadMultisample(int pin, const uint8_t samples){
	uint16_t result = 0;
	for (uint8_t b=0; b<samples;b++){ result = result + analogRead(pin); }
	return (result/samples) ; //[0-1023]
}

void setup(){
	Serial.begin(115200);
	Serial.println("I AM TANK! https://github.com/Beherith/tank_recv");

	//init thermistor,fan, input voltage monitoring [NOT IMPLEMENTED!]
	pinMode(thermistor_vcc,OUTPUT);
	digitalWrite(thermistor_vcc,HIGH);
	pinMode(thermistor_gnd,OUTPUT);
	digitalWrite(thermistor_gnd,LOW);
	pinMode(thermistor,INPUT);
	pinMode(fanpin,OUTPUT);
	digitalWrite(fanpin,LOW);
	analogRead(thermistor);
	temperature = analogReadMultisample(thermistor,8);
	pinMode(inputvoltagelow, OUTPUT);
	digitalWrite(inputvoltagelow, LOW);

	//Init output pins
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
	pinMode(laserlow,OUTPUT);
	pinMode(laserpin,   OUTPUT);
	digitalWrite(laserlow,LOW);
	digitalWrite(laserpin, HIGH);
	digitalWrite(firepin    , LOW);
	digitalWrite(lightpin   , HIGH);
	digitalWrite(smokepin   , LOW);
	digitalWrite(builtin_led_pin,LOW);
	pinMode(builtin_led_pin,OUTPUT);

	stopMotion();


	//We need 120hz pwm, even though it might be far from ideal
	// note 120hz pwm is better with fast decay, but still judders like mad
	// better to use 30khz combo'd with slow decay.
	
	//use 30KHZ PWM with FAST DECAY!
	TCCR1B = TCCR1B & 0b11111000 | 0x01 ;  //
	//For timer 1,3,4,5:[0;7]:[X, 1, 8, 64, 256,1024, EXT_FALL,EXT_RISE]
	TCCR2B = TCCR2B & 0b11111000 | 0x01; //1
	// For timer2: [0;7]:[X,1,8,32,64,128,256,1024]
	// Note that phase-correctness doubles period

	TCCR3B = TCCR3B & 0b11111000 | 0x01;//4
	TCCR4B = TCCR4B & 0b11111000 | 0x01;//4
	TCCR5B = TCCR5B & 0b11111000 | 0x01;//4


	printf_begin(); //needed by NRF24
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
	if (debug) radio.printDetails();
	Serial.println("Radio ready");

	for (uint8_t i =0; i<200; i++) serialpacket[i] = 0; //init packet

	for (uint8_t i =0; i<sizeof(packet); i++) packet[i] = 0; //init packet

	//init gyro
	pinMode(MPU9255_GND,OUTPUT);
	digitalWrite(MPU9255_GND,LOW);
	pinMode(MPU9255_VCC,OUTPUT);
	digitalWrite(MPU9255_VCC,HIGH);
	delay(20); //wait for gyro to power up
	mpu.MPU9255_SCL = MPU9255_SCL;
	mpu.MPU9255_SDA = MPU9255_SDA;
	if(mpu.init()) Serial.println("Gyro initialization Success");
	else Serial.println("Gyro initialization failed");
	//https://github.com/Bill2462/MPU9255-Arduino-Library/blob/master/doc/settings.md
	mpu.set_acc_bandwidth(acc_41Hz);//set accelerometer bandwidth //seems ok, is lower than the motor driver 
	mpu.set_gyro_bandwidth(gyro_41Hz);//set gyroscope bandwidth
	mpu.set_acc_scale(scale_4g); //+- 4g
	mpu.set_gyro_scale(scale_500dps); //+- 500 degrees per second

	Serial.println('L'<<8+'A',HEX);


}

void printGyroPacket(){
	mpu.read_acc(); //[-32768; +32768]  4G (40 m/s^2) range
	mpu.read_gyro();// [-32768; +32768] 500 degrees per second range
	mpu.read_mag(); // [-600; 600 ] arbitrary magnetism units
	int gyrotemperature = mpu.read_temp(); //100x deg C
	sprintf(gyrobuf, "GYRO:\tDT=%lu\tAX=%i\tAY=%i\tAZ=%i\tGX=%i\tGY=%i\tGZ=%i\tMX=%i\tMY=%i\tMZ=%i\tTDC=%i",millis(),
	mpu.ax, mpu.ay, mpu.az, mpu.gx, mpu.gy, mpu.gz, mpu.mx, mpu.my, mpu.mz, gyrotemperature); // ~ max 130 chars in a sane fashion
	//at 115200 baud rate, we can send 10k chars per sec, if we send gyro every 50ms (gyroTimeout) thats still only 2.6k chars per sec
	Serial.println(gyrobuf);
}

void printPacket(){
  if (!debug) return;
	  sprintf(gyrobuf,"TANK:\tLF=%i\tLB=%i\tRF=%i\tRB=%i\tTU=%i\tTD=%i\tTL=%i\tTR=%i\tLA=%i\tLI=%i\tDB=%i",
	  packet[leftfwpin],
	  packet[leftbackpin],
	  packet[rightfwpin],
	  packet[rightbackpin],
	  packet[turretuppin],
	  packet[turretdownpin],
	  packet[turretleftpin],
	  packet[turretrightpin],
	  packet[laserpos],
	  packet[lightpin],
	  packet[debug]);
	  Serial.println(gyrobuf);
	  /*
      for (uint8_t i =0; i<sizeof(packet); i++){
        Serial.print(i);
        Serial.print(":");
        Serial.print(packet[i]);
        Serial.print("\t" );
      }
      Serial.println();
	*/
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
//
/*
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
}*/
/*
void movetrack(uint8_t fwpin, uint8_t fwvalue,uint8_t backpin, uint8_t backvalue){
	//Guess the decay mode?
	if (((fwvalue == 255) && (backvalue >0) ) || (backvalue == )
	
	
}
*/
void process(int pin, int pack){
	if (pack==0){ digitalWrite(pin,LOW);}
	else if (pack ==255) {digitalWrite(pin,HIGH);}
	else {analogWrite(pin,pack);}
}
  
void updatePhysicalState(){
	packetLastReceived = millis();
	process(leftfwpin   ,packet[leftfwpin   ]);
	process(leftbackpin   ,packet[leftbackpin   ]);
	process(rightfwpin    ,packet[rightfwpin    ]);
	process(rightbackpin  ,packet[rightbackpin  ]);
	process(turretleftpin ,packet[turretleftpin ]);
	process(turretrightpin  ,packet[turretrightpin  ]);
	process(turretuppin   ,packet[turretuppin   ]);
	process(turretdownpin ,packet[turretdownpin ]);
	digitalWrite(lightpin    ,(packet[lightpin]>0));
	digitalWrite(laserpin    ,(packet[laserpos]>0));
	if (packet[firepin] && lastshottime+shotdelay < millis()){
		digitalWrite(firepin,HIGH);
		lastshottime=millis();
		packet[firepin] = 0; //eat the shot command!
	}
	inmotion = 1;
}

uint16_t cmdkey(char * cmd){
	uint16_t result = cmd[0];
	result = result <<8;
	result = result + cmd[1];
	return result;
}

uint16_t cmdkey(char a, char b){
	uint16_t result = a;
	result = result <<8;
	result = result + b;
	return result;
}

void loop(void){
	if ( inmotion && (packetLastReceived + packetTimeout < millis()) ){

		stopMotion();
		analogWrite(lightpin,0);

		packetLastReceived = millis();
	}

	if (lastshottime + 500 < millis()){ // wait a bit so that the switch can deactivate again
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
		serialpacket[serialpacketptr] = inchar;
		serialpacketptr ++;
		if (inchar == '\n'){
			serialpacket[serialpacketptr] = 0; //null terminator
			
			//parsing tokens:
			if (debug) {Serial.print("Got message: ");Serial.print(serialpacket);}
			const char separator[3] = ";";
			char * command;
			
			command = strtok(serialpacket,separator); //could this leak the heap?
			while (command != NULL){
				if (debug) {Serial.print("Command: "); Serial.println(command);}
				char cmdid[2];
				cmdid[0] = 0;
				cmdid[1] = 0;
				int value = -1000;
				int n = sscanf(command,"%1c%1c:%d",cmdid,cmdid+1,&value);
				#define LF 0x4C46
				#define LB 0x4C62
				#define RF 0x5246
				#define RB 0x5242
				#define TL 0x544C
				#define TR 0x5452
				#define TU 0x5455
				#define TD 0x5444
				#define LI 0x4C49
				#define FP 0x4650
				#define LA 0x4C41
				
				#define GY 0x4759
				#define DB 0x4442
				#define TO 0x544F
				#define CH 0x4348
				#define RA 0x5241
				
				#define LL 0x4C4C
				#define RR 0x5252
				
				if ((n == 3) && (value >(-256)) && (value <256)){
					packetLastReceived = millis();
					switch (cmdkey(cmdid)){
						case LF:
							packet[leftfwpin] = value; break;
						case LB:
							packet[leftbackpin] = value; break;
						case RF:
							packet[rightfwpin] = value; break;
						case RB:
							packet[rightbackpin] = value; break;
						case TL:
							packet[turretleftpin] = value; break;
						case TR:
							packet[turretrightpin] = value; break;
						case TU:
							packet[turretuppin] = value; break;
						case TD:
							packet[turretdownpin] = value; break;
							
						case LI:
							packet[lightpin] = value; break;
						case FP:
							packet[firepin] = value; break;
						case LA:
							packet[laserpos] = value; break;
							
						case GY:
							if (value == 0)	printGyroPacket();
							gyroTimeout = value *10;
							break;
						case DB:
							if (value == 0) debug = 0;
							else debug = 1;
							break;
						case CH:
							radio.setChannel(value);
							break;					
						case RA:
							useRadio = (value > 0);
							break;
						case TO:
							if (value > 0) packetTimeout = value*10;
							break;
						case LL:
							value = min(255,max(-255,value));
							if (value == 0) {
								packet[leftfwpin] = 0;
								packet[leftbackpin] = 0;
							}else if( value >0){
								packet[leftbackpin] = 255-value;
								packet[leftfwpin] = 255;
							}else if(value <0){
								value = abs(value);
								packet[leftbackpin] = 255;
								packet[leftfwpin] = 255-value;
							}
							break;
							
						case RR:							
							value = min(255,max(-255,value));
							if (value == 0) {
								packet[rightbackpin] = 0;
								packet[rightfwpin] = 0;
							}else if( value >0){
								packet[rightbackpin] = 255-value;
								packet[rightfwpin] = 255;
							}else if(value <0){
								value = abs(value);
								packet[rightbackpin] = 255;
								packet[rightfwpin] = 255-value;
							}
							break;
						
						default: 
							Serial.print("Unknown command: ");Serial.print(command);Serial.println(cmdkey(cmdid),HEX);
							break;
					}
					updatePhysicalState();
			
				}else{
					if (debug) {Serial.print("Failed to parse command: "); Serial.println(command)};
				}
				
				
				command = strtok(NULL,separator);
			}
			
			if (debug) printPacket();
			for (; serialpacketptr>0;serialpacketptr--) serialpacket[serialpacketptr-1]=0;
			
		}
	}
	if (useRadio && radio.available() ){
		Serial.print("Radio OK");
		while (radio.available()){
			radio.read( packetRadio, sizeof(packetRadio));
			uint8_t checkSum = 0;
			for (uint8_t i = 0;i<15;i++){ checkSum = checkSum + packetRadio[i];	}
			if (packetRadio[15] != checkSum){
				Serial.println("Invalid packet received on the radio, consider turning off the radio with \"RA:0;\"");
				printPacket();
			}else{
			
				if (debug){
					Serial.print("Got packet with delay of:\n\r");
					Serial.println(millis()-packetLastReceived);
					printPacket();
				}
				updatePhysicalState();
			}
		}
	}

	temperature = analogReadMultisample(thermistor,8);
	if (temperature  < thermistor_threshold)  digitalWrite(fanpin,HIGH);
	if (gyroTimeout > 0){
		if ((millis() - gyroLastSent)>=gyroTimeout){
			gyroLastSent = millis();
			printGyroPacket();
		}
	}
	digitalWrite(builtin_led_pin,!digitalRead(builtin_led_pin));
}

