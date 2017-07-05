/*
This file is part of the Tinovi.io Ethernet communications library for Arduino

2016

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
https://github.com/BlueVia/Official-Arduino
*/

//needed for library
#include "ZiinodeArd.h"
#include "DallasTemperature.h" //http://www.pjrc.com/teensy/arduino_libraries/OneWire.zip   place under arduino libraries
#include "DHT.h" //https://github.com/adafruit/DHT-sensor-library    place under libraries and rename folder to DHT
#include <pcint/PinChangeInterrupt.h>
//#include <PZEM004T.h>
#include <SHT1x.h>
#include <utility/Time.h>

ZiinodeArd zn;

//analog 10k thermocouple
#define NTC10K 1
//dallas sensor
#define INPUT_DS 2
//digital themp & hummudity
#define INPUT_TH_11 3
#define INPUT_TH_22 4
//analog read
#define VOLTAGE 5
#define COUNTER 6
#define ENERGYPZ 7
#define SHT10  8

//total 4+1
//#define IN_start 15
//total 20*6+1
//#define TRIG_start 20



typedef struct{
	uint8_t input;
	uint8_t oper;
	uint8_t out;
	bool out_dir;
	int16_t val;
	int16_t precision; //equals historize
} trigger_t;//size 8

typedef struct{
	uint8_t output;
	uint8_t days;
	uint8_t hour;
	uint8_t minute;
	int16_t duration;
} schedule_t;//size 7

#define IN_COUNT 8
#define READ_COUNT 16
#define OUT_COUNT 5
#define TRIG_COUNT 16
#define SCHED_COUNT 16

uint8_t in_pin[IN_COUNT] = { A0, A1, A2, A3, A4, A5,A6, A7 };
uint8_t out_pins[OUT_COUNT] = { 21, 22, 23, 2, 3 };

//reading digital may have hummuduty as well

//#define DEBUG 1


#define OPER_EQUAL 1
#define OPER_LESS 2
#define OPER_GREATER 3

int readings[READ_COUNT];

byte in_type[IN_COUNT];
int in_res[IN_COUNT];
trigger_t trigger[TRIG_COUNT];
schedule_t schedule[SCHED_COUNT];
unsigned long trig_tim[TRIG_COUNT];
bool trig_dir[TRIG_COUNT];

unsigned long sch_tim[SCHED_COUNT];

byte out_s;

boolean hasAck=false;


#define EEP_S_ADR 29  //5
#define IN_TYP_ADR 34 //9
#define IN_RES_ADR 43 //17
#define TRIG_ADR 60 // TRIG_COUNT(16) * 8 = 128 +1
#define SCHED_ADR 189 // SCHED_COUNT(16) * 7 = 112+1

void  savePostingInt(){
	write_eeprom_byte(EEP_S_ADR, 55);
	EEPROM_writeAnything(EEP_S_ADR+1,zn.postingInterval);
}


void saveInType(){
	write_eeprom_byte(IN_TYP_ADR, 55);
	EEPROM_writeAnything(IN_TYP_ADR+1,in_type);
}

void saveRes(){
	write_eeprom_byte(IN_RES_ADR, 55);
	EEPROM_writeAnything(IN_RES_ADR+1,in_res);
}

void saveTrig(){
	write_eeprom_byte(TRIG_ADR, 55);
	EEPROM_writeAnything(TRIG_ADR+1,trigger);
}

void saveSched(){
	write_eeprom_byte(SCHED_ADR, 55);
	EEPROM_writeAnything(SCHED_ADR+1,schedule);
}

void readConf(){

  if(read_eeprom_byte(EEP_S_ADR)==55){
	  EEPROM_readAnything(EEP_S_ADR+1,zn.postingInterval);
  }

  if(read_eeprom_byte(IN_TYP_ADR)==55){
	  EEPROM_readAnything(IN_TYP_ADR+1,in_type);
  }
  if(read_eeprom_byte(IN_RES_ADR)==55){
	  EEPROM_readAnything(IN_RES_ADR+1,in_res);
  }
  //total 20*6+1
  if(read_eeprom_byte(TRIG_ADR)==55){
	  EEPROM_readAnything(TRIG_ADR+1,trigger);
  }
  if(read_eeprom_byte(SCHED_ADR)==55){
	  EEPROM_readAnything(SCHED_ADR+1,schedule);
  }
	for (int i = 0; i < IN_COUNT; i++) {
		if(in_type[i]==COUNTER){
			attachPinChangeInterrupt(i, FALLING);
		}
	}
}



void sendInputType(){
	zn.msg(CMD_INT_TYPE, IN_COUNT*3);
	zn.write((const byte *)in_type,IN_COUNT);
	zn.write((const byte *)in_res,IN_COUNT*2);
}

void sendTrigAll(){
	zn.writeBody(CMD_TRIG_ALL,(const byte *)trigger,TRIG_COUNT * 8);
}

void sendSchedAll(){
	zn.writeBody(CMD_SCHED_ALL,(const byte *)schedule,SCHED_COUNT * 6);
}

void sendNet(){
	//zn.write(CMD_NET);
	//zn.writeBody((const byte*)&zn.getNet(),16);
}

void sendBinTrap(){
	zn.msg(TRAP,(READ_COUNT * 2) + 1);
    for(int i=0;i<READ_COUNT;i++){
    	int in = i/2;
    	if(in_type[in]==COUNTER){
    		int rer = in_res[in];
    		if(rer==0){
    			rer=1;
    		}
        	zn.writeIntE(readings[i]*rer);
    		readings[i] = 0;
    	}else{
        	zn.writeIntE(readings[i]);
    	}
#if DEBUG
       	Serial.print(readings[i]);
        Serial.print(',');
#endif
    }
	zn.write(out_s);
#if DEBUG
	Serial.println();
#endif
}

void ack(int mid){
#if DEBUG
	Serial.print(">ack:");
	Serial.println(mid);
#endif
}

void cack(){
	hasAck = true;
	sendInputType();
	sendTrigAll();
	sendSchedAll();
	zn.sendNet();
	zn.sendInterval();
#if DEBUG
	Serial.println("conn ack.... ");
#endif
}

void dataReceived(byte cmd, ByteBuffer *buf){
#if DEBUG
	Serial.print("data:");
	Serial.println(cmd);
#endif
	if(cmd==CMD_POSTING_INTERVAL){
		zn.postingInterval = buf->getRUInt32();
		savePostingInt();
	}else if(cmd==CONN_ACK){
		cack();
	}else if(cmd==CMD_TRIG_OUT){
		byte idx = buf->get();
		uint8_t val = buf->get();
		if(idx<OUT_COUNT){
			uint8_t pin = out_pins[idx];
			uint8_t cur = digitalRead(pin);
			zn.writeLog(0,1,"setPin #:%i val:%i  idx:%i cur%i",pin,val, idx, cur);
			if(cur!=val){
				digitalWrite(pin,val);
				if(val){
					bitSet(out_s, idx);
				}else{
					bitClear(out_s, idx);
				}
			}
		}else{
			zn.writeLog(0,1,"ERR out idx:%i max:%i",idx,OUT_COUNT);
		}
	}else if(cmd==CMD_INT_TYPE){
		byte idx = buf->get();
		if(idx<IN_COUNT){
			in_type[idx]=buf->get();
			if(in_type[idx]==COUNTER){
				attachPinChangeInterrupt(idx, FALLING);
			}
//			else if(in_type[idx]==ENERGYPZ || in_type[idx]==SHT10){
//				if(idx % 2==1){
//					in_type[idx-1]=in_type[idx];
//				}else{
//					in_type[idx+1]=in_type[idx];
//				}
//
//			}
			saveInType();
			if(buf->getSize()>1){
				in_res[idx]=buf->getInt();
				saveRes();
			}
			zn.writeLog(0,1,"in type idx:%i val:%i res:%i",idx,in_type[idx],in_res[idx]);
		}else{
			zn.writeLog(0,1,"ERR in type idx:%i max:%i",idx,IN_COUNT);
		}
	}else if(cmd==CMD_TRIG){
		//typedef struct{
		//	uint8_t input;
		//	uint8_t oper;
		//	uint8_t out;
		//	uint8_t out_dir;
		//	int val;
		//} trigger_t;//size 6
		byte idx = buf->get();
		trigger[idx].input = buf->get();
		trigger[idx].oper = buf->get();
		trigger[idx].out = buf->get();
		trigger[idx].out_dir = buf->get();
		trigger[idx].val = buf->getInt();
		trigger[idx].precision = buf->getInt();
		if(idx<TRIG_COUNT && trigger[idx].input < READ_COUNT){
			saveTrig();
			zn.writeLog(0,1,"trig idx:%i input:%i oper:%i out:%i out_dir:%i val:%i prec:%i",idx,trigger[idx].input,trigger[idx].oper ,trigger[idx].out,trigger[idx].out_dir,trigger[idx].val,trigger[idx].precision);
		}else{
			zn.writeLog(0,1,"ERR ttrig idx:%i max:%i",idx,TRIG_COUNT);
		}
	}else if(cmd==CMD_SCHEDULE){
//		typedef struct{
//			uint8_t output;
//			uint8_t days;
//			uint8_t hour;
//			uint8_t minute;
//			uint8_t out_dir;
//			int16_t duration;
//		} schedule_t;//size 6
		byte idx = buf->get();
		schedule[idx].output = buf->get();
		schedule[idx].days = buf->get();
		schedule[idx].hour = buf->get();
		schedule[idx].minute = buf->get();
		schedule[idx].duration = buf->getInt();
		setTime(buf->getRUInt32());
		if(idx<TRIG_COUNT && trigger[idx].input < READ_COUNT){
			saveSched();
			zn.writeLog(0,1,"sched idx:%i output:%i days:%i hour:%i minute:%i out_dir:%i duration:%i time %i" ,idx,schedule[idx].output,schedule[idx].days ,schedule[idx].hour,schedule[idx].minute,bitRead(schedule[idx].days,7),schedule[idx].duration, now());
		}else{
			zn.writeLog(0,1,"ERR sched idx:%i max:%i",idx,SCHED_COUNT);
		}
	}else if(cmd==CMD_TRIG_ALL){
		for(int idx=0;idx<TRIG_COUNT;idx++){
			trigger[idx].input = buf->get();
			trigger[idx].oper = buf->get();
			trigger[idx].out = buf->get();
			trigger[idx].out_dir = buf->get();
			trigger[idx].val = buf->getInt();
			trigger[idx].precision = buf->getInt();
		}
		//EEPROM_writeAnything(e_trigger,trig);
	}else if(cmd==ACK){
		//ul_PreviousMillis = millis();
	}
	zn.msg(ACK,1);
	zn.write(cmd);
	//zn.flush();
}

void setSPin(uint8_t i, bool on_dir){
	if(schedule[i].output<OUT_COUNT){
		uint8_t pin = out_pins[schedule[i].output];
		uint8_t val = bitRead(schedule[i].days,7);
		if(!on_dir){
			val = !val;
		}
		if(digitalRead(pin)!=val){
			trig_dir[i] = on_dir;
			zn.writeLog(0,1,"setSPin:%i val:%i",pin,val);
			zn.sendEvent(0,i+TRIG_COUNT,"SOutput changed #:%i state:%i on duration:%i",schedule[i].output,val,schedule[i].duration);
			digitalWrite(pin,val);
			if(val){
				bitSet(out_s, trigger[i].out);
			}else{
				bitClear(out_s, trigger[i].out);
			}
		}
	}else{
		zn.sendEvent(0,i,"Sch #:%i",i);
	}
}


void checkSched(){
	time_t t = now();
	int h = hour(t);
	int m = minute(t);
	int d = weekday(t);
	if(timeStatus()==timeNotSet){
		return;
	}
	for (uint8_t i = 0; i < SCHED_COUNT; i++) {
		if(sch_tim[i]==0 & (schedule[i].days==0 && schedule[i].hour==h && schedule[i].minute==m)
				||(schedule[i].hour==h && schedule[i].minute==m && bitRead(schedule[i].days,d-1))){
			sch_tim[i] = millis();
			setSPin(i, true);
		}else if(sch_tim[i]>0 && (millis() - trig_tim[i]) > (schedule[i].duration * 100)){
			sch_tim[i] = 0;
			setSPin(i, false);
		}
	}
}


char oper[3] = {'=','<','>'};

void setPin(uint8_t i, bool on_dir){
	if(trigger[i].out<OUT_COUNT){
		uint8_t pin = out_pins[trigger[i].out];
		uint8_t val = trigger[i].out_dir;
		if(!on_dir){
			val = !val;
		}
		if(digitalRead(pin)!=val){
			if(!trig_dir[i] && on_dir && trig_tim[i]>0 && (millis() - trig_tim[i]) < (trigger[i].precision * i * 100)){ //block swittch on if it was switched out by timeout
				return;
			}
			trig_dir[i] = on_dir;
			if(trigger[i].precision>0 && trigger[i].oper>1){
				trig_tim[i] = millis();
			}
			zn.writeLog(0,1,"setPin:%i val:%i",pin,val);
			zn.sendEvent(0,i,"Output changed #:%i state:%i on value:%i",trigger[i].out,val,trigger[i].val);
			digitalWrite(pin,val);
			if(val){
				bitSet(out_s, trigger[i].out);
			}else{
				bitClear(out_s, trigger[i].out);
			}
		}
	}else{
		zn.sendEvent(0,i,"Trigger #:%i on %i %c %i",i,trigger[i].val,oper[trigger[i].oper], readings[trigger[i].input]);
	}
}

void trigger_out() {
	for (uint8_t i = 0; i < TRIG_COUNT; i++) {
		if( trigger[i].oper>0 && trigger[i].oper<255 && trigger[i].input < READ_COUNT){
			unsigned long ms = millis();
			if(trig_dir[i] && trig_tim[i]>0 && (ms - trig_tim[i]) > (trigger[i].precision * 100)){ // time out - switch off
				setPin(i, false);
			}else if(trigger[i].oper==OPER_EQUAL && trigger[i].val<=(readings[trigger[i].input]- trigger[i].precision)){ // on
				setPin(i, true);
			}else if(trigger[i].oper==OPER_EQUAL && trigger[i].val>=(readings[trigger[i].input]+ trigger[i].precision)){ //off
				setPin(i, false);
			}else if(trigger[i].oper==OPER_GREATER && trigger[i].val<readings[trigger[i].input] ){
				setPin(i, true);
			}else if(trigger[i].oper==OPER_LESS && trigger[i].val>readings[trigger[i].input] ){
				setPin(i, true);
			}else if(!trig_dir[i] && trig_tim[i]>0 && (ms - trig_tim[i]) > (trigger[i].precision * i * 100)){ // clear flag if block expired
				trig_tim[i] = 0;
			}
		}

	}
}


// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3435

int isPUp(uint8_t pin){
	pinMode(pin, INPUT);
	return digitalRead(pin);
}

void read_inputs() {

	for (int i = 0; i < IN_COUNT; i++) {
		///=====NTC10K=======================================================
		if(in_type[i]==NTC10K){
			//analogReference(DEFAULT);
			int res = in_res[i];
			if(res==0){
				res=BCOEFFICIENT;
			}
			readings[i * 2] = zn.readThermistor(in_pin[i],res);
//#if DEBUG
//	zn.writeLog(1,"NTC10k :%i  temp:%i",i,readings[i * 2]);
//#endif
		}else if(in_type[i]==INPUT_DS && isPUp(in_pin[i])){

			OneWire oneWire(in_pin[i]);
			DallasTemperature sensor_inhouse(&oneWire);
			sensor_inhouse.begin();
			if(sensor_inhouse.getDeviceCount()){
				//sensor_inhouse.setResolution(10);
				sensor_inhouse.requestTemperatures();
				int rez = sensor_inhouse.getTempCByIndex(0)*10;
				if(rez !=-1270){
					readings[i * 2] = rez;
				}
			}
//			}
//#if DEBUG
//	zn.writeLog(1,"DS:%i val:%i",in_pin[i],readings[i * 2]);
//#endif
		}else if(in_type[i]==INPUT_TH_11 && isPUp(in_pin[i])){
			DHT dht(in_pin[i], DHT11);
			dht.begin();
			int t = dht.readTemperature() * 10;
			int h = dht.readHumidity() * 10;
			//if(t>0){
				readings[i * 2] = t;
				readings[i * 2+1] = h;
			//}
//#if DEBUG
//	zn.writeLog(0, 1,"DS:%i humy:%i temp:%i",in_pin[i],readings[i * 2], readings[i * 2+1]);
//#endif
		}else if(in_type[i]==INPUT_TH_22 && isPUp(in_pin[i])){
			DHT dht(in_pin[i], DHT22);
			dht.begin();
			int t = dht.readTemperature() * 10;
			int h = dht.readHumidity() * 10;
			//if(t>0){
				readings[i * 2] = t;
				readings[i * 2+1] = h;
			//}
//#if DEBUG
//	zn.debugL("DS:%i humy:%i temp:%i ",in_pin[i],readings[i * 2], readings[i * 2+1]);
//#endif
//		}else if(in_type[i]==ENERGYPZ && i%2 == 0){
//			PZEM004T pzem(in_pin[i],in_pin[i+1]);
//			pzem.setAddress(ip);
//			float v = pzem.voltage(ip);
//			float i = pzem.current(ip);
//			float p = pzem.power(ip);
//			float e = pzem.energy(ip);
//			readings[i] = v*10;
//			readings[i+1] = i*10;
//			readings[i+2] = p*10;
//			readings[i+3] = e*10;
//			pzem.close();
		}else if(in_type[i]==SHT10 && i%2 == 0 && isPUp(in_pin[i])){
			SHT1x  sht1x(in_pin[i],in_pin[i+1]);
			//pzem.setAddress(ip);
			float h = sht1x.readHumidity() * 10;
			float t = sht1x.getTemperature() * 10;
			readings[i * 2] = (int)t;
			readings[i * 2+1] = (int)h;
		//#if DEBUG
		//	zn.writeLog(0, 1,"SH:%i humy:%i temp:%i",in_pin[i],readings[i * 2], readings[i * 2+1]);
		//#endif
		}else if(in_type[i]==VOLTAGE){
			//analogReference(INTERNAL2V56);
			//10v - 30k-10k = 2,5v
			//5v - 10k - 10k = 2,5v
			int res = in_res[i];
			if(res==0){
				res=50;
			}
			readings[i * 2]  = zn.readAnalogVolts(in_pin[i],res);
//#if DEBUG
//	zn.writeLog(0,1,"voltage:%i voltage:%i ",in_pin[i], readings[i * 2]);
//#endif
		}
	}
}



uint32_t lastConnectionTime = 0;

void loop() {
//	if(!reset){
//		wdt_reset();
//	}
	zn.ether_loop();
	if((millis() - lastConnectionTime > zn.postingInterval)) {
		read_inputs();
		checkSched();
		trigger_out();
		if(zn.checkConn()){
			if(hasAck){
				sendBinTrap();
			}
		}else{
			hasAck = false;
		}
		lastConnectionTime = millis();

		//enable this if you want to reset device id
		//if(digitalRead(2)){
		//	zn.resetDevId();
		//}
	}

}

void PinChangeInterruptEvent(0)(void) {
	readings[0]++;
}
void PinChangeInterruptEvent(1)(void) {
	readings[2]++;
}
void PinChangeInterruptEvent(2)(void) {
	readings[4]++;
}
void PinChangeInterruptEvent(3)(void) {
	readings[6]++;
}
void PinChangeInterruptEvent(4)(void) {
	readings[8]++;
}
void PinChangeInterruptEvent(5)(void) {
	readings[10]++;
}
void PinChangeInterruptEvent(6)(void) {
	readings[12]++;
}
void PinChangeInterruptEvent(7)(void) {
	readings[14]++;
}

void setup() {
  for(int i=0;i<OUT_COUNT;i++){
	  pinMode(out_pins[i], OUTPUT);
	  digitalWrite(out_pins[i],LOW);
  }
  // put your setup code here, to run once:
  Serial.begin(115200);
  //zn.resetDevId();
  byte emac[] = MAC;
  zn.begin(emac, dataReceived, cack, ack);
  //in seconds
  pinMode(27, INPUT_PULLUP);
  if(digitalRead(27)){
	  readConf();
	  read_inputs();
	  trigger_out();
  }
}

int main(void) {

	init();
	setup();

	while (true) {
		loop();
	}
}
