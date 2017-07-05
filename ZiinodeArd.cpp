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
https://github.com/tinovi
*/

#include "ZiinodeArd.h"
#include "HttpClient.h"
#include <inttypes.h>
#include <avr/io.h>

byte cmd=0;
int zsize=-1;

ZiinodeArd::ZiinodeArd(){
	_clientBuffer = (ByteBuffer*)malloc(sizeof(ByteBuffer));
	_clientBuffer->init(156);
	postingInterval = 1000;
	hasAddr = false;
	_hasDevId = true;
	strncpy(_devid,APIKEY,7);
}

void ZiinodeArd::sendNet(){
//	IPAddress ip =_client.localIP();
//	IPAddress gw =_client.gatewayIP();
//	IPAddress sm =_client.subnetMask();
//	IPAddress dns =_client.dnsIP();
//	net_t net = {{ip[0],ip[1],ip[2],ip[3]},{gw[0],gw[1],gw[2],gw[3]},{sm[0],sm[1],sm[2],sm[3]},{dns[0],dns[1],dns[2],dns[3]}};
//	writeBody(CMD_NET,(const byte*)&net,16);
}


size_t ZiinodeArd::write(uint8_t b)
{
    return _client.write(b);
}

size_t ZiinodeArd::write(const uint8_t *buf, size_t size){
	return _client.write(buf,size);
}

void  ZiinodeArd::writeInt(int in){
    byte *pointer = (byte *)&in;
    _client.write(pointer[1]);
    _client.write(pointer[0]);
}


void  ZiinodeArd::writeIntE(int in){
    byte *pointer = (byte *)&in;
    _client.write(pointer[0]);
    _client.write(pointer[1]);
}

void ZiinodeArd::writeUint32(uint32_t in){
    byte *pointer = (byte *)&in;
    _client.write(pointer[3]);
    _client.write(pointer[2]);
    _client.write(pointer[1]);
    _client.write(pointer[0]);
}

void ZiinodeArd::writeInt64(int64_t in){
    byte *pointer = (byte *)&in;
    _client.write(pointer[7]);
    _client.write(pointer[6]);
    _client.write(pointer[5]);
    _client.write(pointer[4]);
    _client.write(pointer[3]);
    _client.write(pointer[2]);
    _client.write(pointer[1]);
    _client.write(pointer[0]);
}

/*
 typedef struct{
	uint8_t ip[4];
	uint8_t gw[4];
	uint8_t sm[4];
	uint8_t dns[4];
} net_t; //16

 * */

//net_t  ZiinodeArd::getNet(){
	//_client.
//	net_t net;
//	return net;
//}

char* log_buf;
char lb[255];

//ANNOTATION
void ZiinodeArd::sendEvent(int64_t time, int code, const char *fmt, ...){
	if(!log_buf){
		log_buf = lb;
	}
	va_list va;
	va_start (va, fmt);
	int size = vsprintf (log_buf, fmt, va);
	va_end (va);
	if(_client.connected()){
		msg(ANNOTATION,size+2+8);
		writeInt64(time);
		writeInt(code);
		//writeInt(size);
		_client.write((const uint8_t *)log_buf,size);
	}
}

void ZiinodeArd::writeLog(int64_t time, int code, const char *fmt, ...){
	if(!log_buf){
		log_buf = lb;
	}
	va_list va;
	va_start (va, fmt);
	int size = vsprintf (log_buf, fmt, va);
	va_end (va);
	if(_client.connected()){
		msg(LOG,size+2+8);
		writeInt64(time);
		writeInt(code);
		//writeInt(size);
		_client.write((const uint8_t *)log_buf,size);
	}
}

void ZiinodeArd::msg(uint8_t cmd, int size){
	_client.write(cmd);
	writeInt(ncnt());
	writeInt(size);

}
int ZiinodeArd::ncnt(){
	cnt++;
	return cnt;
}


void  ZiinodeArd::writeBody(uint8_t cmd, const uint8_t *buf, int size) {
	msg(cmd,size);
	_client.write(buf,size);
}

unsigned long ul_LastComm = 0UL;
boolean ZiinodeArd::checkConn(){
	if(!hasAddr){
		return false;
	}
	if(!_client.connected() || (ul_LastComm!=0 && (millis() - ul_LastComm) > ((postingInterval*3)+5000))){
	 stop();
	}
	return hasAddr;

}
void lastComm(){
	ul_LastComm = millis();
}


void  ZiinodeArd::sendInterval(){
	msg(CMD_POSTING_INTERVAL,4);
	writeUint32(postingInterval);
}

uint8_t ZiinodeArd::connected(){
	if(!hasAddr){
		return 0;
	}
	return _client && (_client.connected() || _client.available());
}


// Number of milliseconds to wait without receiving any data before we give up
const uint16_t kNetworkTimeout = 30*1000;
const uint16_t udpLocalTimeout = 5*1000;
// Number of milliseconds to wait if no data is available before trying again
const uint16_t kNetworkDelay = 100;

//const char *apitype = APITYPE;
//const char *apikey = APIKEY;

void ZiinodeArd::stop(){
#if DEBUG
	 Serial.println("stop..");
#endif
	hasAddr = false;
	_client.stop();
	cmd=0;
	zsize=-1;
	_clientBuffer->clear();
}

//"239.193.110.110"
IPAddress mip(239, 193, 110, 110);
void ZiinodeArd::begin(byte *emac, TOnDataHandler handler, TOnConnAckHandler cack, TOnAckHandler ack){
  _onDataHandler = handler;
  _cack = cack;
  _ack = ack;
  if(!_hasDevId){
	if(read_eeprom_byte(0)==55){
		read_eeprom_array(1,_devid,7);
	    _hasDevId = true;
	}else{
		strncpy(_devid,APITYPE,7);
	}
  }
	 while (Ethernet.begin(emac) == 0) {
	#if DEBUG
		 Serial.println("DHCP failed");
	#endif
	  }
  Udp.beginMulticast(mip, UDP_MULTI_PORT);
  timeoutBegin = millis();
}

void ZiinodeArd::setDevId(char* did) {
  _hasDevId=true;
  strncpy(_devid,did,7);
#if DEBUG
	Serial.print("set devid:");
	Serial.println(_devid);
#endif
  write_eeprom_byte(0, 55);
  write_eeprom_array(1,_devid,7);
}


void ZiinodeArd::resetDevId() {
#if DEBUG
	 Serial.println("reset devid");
#endif
  hasAddr = false;
  _hasDevId=false;
  strncpy(_devid,APITYPE,7);
  write_eeprom_byte(0, 255);
  //write_eeprom_array(1,_devid.c_str(),7);
}

unsigned long timeoutStart;

int16_t ZiinodeArd::getInt(){
	int ret;
    byte *pointer = (byte *)&ret;
	pointer[0] =  _client.read();
	pointer[1] =  _client.read();
	return ret;
}

void ZiinodeArd::ether_loop() {

	int packetSize = Udp.parsePacket();
	if(packetSize){
		IPAddress remote = Udp.remoteIP();
		Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
//			Serial.print("Udp ");
//			Serial.print(remote);
//			Serial.print(":");
//			Serial.print(Udp.remotePort());
//			Serial.print(" d:");
//			Serial.println(packetBuffer);

		if(packetBuffer[0]=='S' && packetBuffer[1]=='E' && packetBuffer[2]=='E' && packetBuffer[3]=='K'){
			//reconnect to new agent
			if(rem[0]!=remote[0] || rem[1]!=remote[1] || rem[2]!=remote[2] || rem[3]!=remote[3]){
				rem = remote;
				memset(address, 0, sizeof(address));
				sprintf(address, "%d.%d.%d.%d", remote[0], remote[1], remote[2], remote[3]);
#if DEBUG
				Serial.print(" conn to:");Serial.println(address);
#endif
				hasAddr = true;
				if(_client.connected()){
					_client.stop();
				}
				timeoutStart = millis();
				while(((millis() - timeoutStart) < kNetworkTimeout) ){
#if DEBUG
Serial.println("connecting ..");
#endif
					if(_client.connect(address,8787)){
						ul_LastComm = 0UL;
						write(ENQ);
						write((const byte*)APITYPE,7);
						write((const byte*)_devid,7);
						write((const byte*)PIN,4);
						writeInt(VERSION);
#if DEBUG
Serial.println("enq sent");
#endif
						return;
					}
				}
#if DEBUG
Serial.println("conn timeout");
#endif
			    hasAddr = false;
			}
		}

	}else if(((millis() - timeoutBegin) < udpLocalTimeout)){ //check local server first - wait 5sec
		return;
	}
	while(!hasAddr) {
		char path[40];
	    sprintf(path,"/api/v1/node/host/%s%s%s",APITYPE,_devid,PIN);

#if DEBUG
		Serial.println("[HTTP] begin...");
		Serial.println(path);
		Serial.println(APITYPE);
		Serial.println(_devid);
		Serial.println(PIN);
#endif
		// start connection and send HTTP header
		int err =0;
		_client.stop();
		HttpClient http(_client);
		err = http.get("www.tinovi.io", path);
		  if (err == 0)
		  {

			err = http.responseStatusCode();
			if (err == 200)
			{
				#if DEBUG
					Serial.print("Got status code:");
					Serial.println(err);
				#endif

			  // Usually you'd check that the response code is 200 or a
			  // similar "success" code (200-299) before carrying on,
			  // but we'll print out whatever response we get

			  err = http.skipResponseHeaders();
			  if (err >= 0)
			  {
		        int bodyLen = http.contentLength();
				#if DEBUG
					Serial.print("Content length is:");
					Serial.println(bodyLen);
					Serial.print("available:");
					Serial.println(http.available());
				#endif

		        // Now we've got to the body, so we can print it out
		        timeoutStart = millis();
		        // Whilst we haven't timed out & haven't reached the end of the body
		        int rec=0;

		        char resp[bodyLen];
		        while ( (http.connected() || http.available()) &&
		               ((millis() - timeoutStart) < kNetworkTimeout) )
		        {
		        	if (http.available() && rec<bodyLen) {
		        		while(http.available() && rec<bodyLen){
		        			resp[rec]=(char)http.read();
		        			rec++;
		        		}
		        		if(rec == 4 && bodyLen == 4 && resp[0]==resp[1] && resp[1]==resp[2] && resp[2]==resp[3] && resp[3]==0){
#if DEBUG
	Serial.println("wait for reg, got 0.0.0.0 ");
#endif
							http.stop();
							hasAddr = false;
							delay(5000);
							return;
		        		}else if(rec == bodyLen){
#if DEBUG
   Serial.print("hd:");
   Serial.print(_hasDevId);
   Serial.print(" resp:");
   Serial.println(resp);
#endif
							http.stop();
							if(!_hasDevId){
								setDevId(resp);
								return;
							}
							hasAddr = true;
							strncpy(address,resp,bodyLen);
							timeoutStart = millis();
							while(((millis() - timeoutStart) < kNetworkTimeout) ){
#if DEBUG
	Serial.println("connecting ..");
#endif
								if(_client.connect(address,8787)){
									ul_LastComm = 0UL;
									write(ENQ);
									write((const byte*)APITYPE,7);
									write((const byte*)_devid,7);
									write((const byte*)PIN,4);
									writeInt(VERSION);
#if DEBUG
	Serial.println("enq sent");
#endif
									return;
								}
							}
#if DEBUG
	Serial.println("conn timeout");
#endif
						    hasAddr = false;
		        		}
		        	}
		            else
		            {
#if DEBUG
	Serial.println("delay...");
#endif
		                delay(kNetworkDelay);
		            }
		        }
#if DEBUG
	Serial.println("http read timeout");
#endif
			  }else{
				#if DEBUG
					Serial.print("Failed to skip response headers");
					Serial.println(err);
				#endif
			  }
			}else {
			#if DEBUG
				Serial.print("Getting response failed:");
				Serial.println(err);
			#endif
			}
		  }
		  else
		  {
			#if DEBUG
				Serial.print("Connect failed:::");
				Serial.println(err);
			#endif
		  }
		  http.stop();
	}
	if (connected()){

		if(_client.available() > 0){
			if(cmd==0){
				lastComm();
				cmd = _client.read();
				zsize = -1;
				if(cmd==CONN_ACK){
					if( _cack != 0 ){
						_cack();
					}
					cmd = 0;
					return;
				}
			}
			if(cmd==ACK && _client.available() > 1){
				int aa = getInt();
				if( _ack != 0 ){
					_ack(aa);
				}
				cmd = 0;
				return;
			}else if(cmd!=0 && _client.available() > 1 && zsize==-1 && _clientBuffer->getSize()==0){
				zsize = getInt();
				timeoutStart = millis();
#if DEBUG
			Serial.print("cmd:");
			Serial.print(cmd);
			Serial.print(" av:");
			Serial.print(_client.available());
			Serial.print(" size:");
			Serial.println(zsize);
#endif
			}
			while(zsize>0 && _client.available() > 0  && _clientBuffer->getCapacity()>0) {
				_clientBuffer->put(_client.read());
				zsize--;
			}

			if(zsize>0 && ((millis() - timeoutStart) > 2000)){
#if DEBUG
			Serial.print("tiemout:");
			Serial.println(cmd);
#endif
				cmd=0;
				zsize=-1;
				_clientBuffer->clear();
			}
			if(cmd!=0 && zsize==0){
				//packet ready
#if DEBUG
			Serial.print("packet:");
			Serial.println(_clientBuffer->getSize());
#endif
				if( _onDataHandler != 0 ){
					_onDataHandler(cmd,_clientBuffer);
				}else{
					//
				}
				_clientBuffer->clear();
				cmd=0;
				zsize=-1;
			}
		}
	}
}


// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// the value of the 'other' resistor
#define SERIESRESISTOR 10000
//========================================

int samples[NUMSAMPLES];

float readAnalog(int adcpin){
	uint8_t i;
	float average;
// take N samples in a row, with a slight delay
	for (i = 0; i < NUMSAMPLES; i++) {
		samples[i] = analogRead(adcpin);
		delay(10);
	}
// average all the samples out
	average = 0;
	for (i = 0; i < NUMSAMPLES; i++) {
		average += samples[i];
	}
	return average /= NUMSAMPLES;
}

int ZiinodeArd::readThermistor(int adcpin, int bconf) {
	float average = readAnalog(adcpin);
//#if DEBUG
//	Serial.print("Average analog reading ");
//	Serial.println(average);
//#endif
	if (average == 0)
		return 0;
// convert the value to resistance
	average = 1023 / average - 1;
	average = SERIESRESISTOR / average;
//#if DEBUG
//	Serial.print("Thermistor resistance ");
//	Serial.println(average);
//#endif
	float steinhart;
	steinhart = average / THERMISTORNOMINAL; // (R/Ro)
	steinhart = log(steinhart); // ln(R/Ro)
	steinhart /= bconf; // 1/B * ln(R/Ro)
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart; // Invert
	steinhart -= 273.15; // convert to C
//#if DEBUG
//	//Serial.print("Temperature ");
//	Serial.print(steinhart);
//	//Serial.println(" *C");
//#endif
//delay(1000);
	return steinhart * 10;
}


//analogReference(INTERNAL2V56);
//10v - 30k-10k = 2,5v
//5v - 10k - 10k = 2,5v
#define NUM_READS 40
float ZiinodeArd::readAnalogVolts(int sensorpin, int voltage){
   // read multiple values and sort them to take the mode
	int sortedValues[NUM_READS];
   for(int i=0;i<NUM_READS;i++){
     int value = analogRead(sensorpin);
     int j;
     if(value<sortedValues[0] || i==0){
        j=0; //insert at first position
     }
     else{
       for(j=1;j<i;j++){
          if(sortedValues[j-1]<=value && sortedValues[j]>=value){
            // j is insert position
            break;
          }
       }
     }
     for(int k=i;k>j;k--){
       // move all values higher than current reading up one position
       sortedValues[k]=sortedValues[k-1];
     }
     sortedValues[j]=value; //insert current reading
   }
   //return scaled mode of 10 values
   float returnval = 0;
   for(int i=NUM_READS/2-3;i<(NUM_READS/2+3);i++){
     returnval +=sortedValues[i];
   }
   returnval = returnval/6;
   return returnval*voltage/1023;
}
