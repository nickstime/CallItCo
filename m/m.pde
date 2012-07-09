#include <WProgram.h>
#include <MaestroAPI.h>
#include <NewSoftSerial.h>  
#include <SoftwareSerial.h>
#include <SD.h>

#define resetPin 5
#define rxMaestro 3
#define txMaestro 4
#define rxBT A5
#define txBT A4
#define sdPin 10
#define baudRateBT 38400
#define baudRate 57600
#define minCommand 1
#define maxCommand 15
#define headB 2
#define minPosition 64
#define maxPosition 4080
#define SERVO_COUNT 18

//first day in steelMountain, part1
#define danceFile "dance1.txt"
//first day in steelMountain, part2
#define danceFile2 "dance2.txt"
//second day in steelMountain
#define danceFile3 "dance3.txt"
//free
#define testFile "test1.txt"
//free
#define testFile2 "test2.txt"
//walk
#define testFile3 "test3.txt"

#define DEBUG 0

unsigned char fileSwitch;
unsigned char bt;//bluetooth lock flag (should be 00000000 to continue): 1bit=~got hello, 2bit=need more
unsigned char servoSpeed,cmd,bytes,start;
unsigned char BT[128],BTi;//BlueTooth Bytes
unsigned char i,ii,iii;
unsigned long fileCur;
unsigned long timeStart;

unsigned char servoPosSize,servoPosKeys[SERVO_COUNT],servoPosSizeOld,servoPosKeysOld[SERVO_COUNT];
unsigned short servoPos[SERVO_COUNT],servoPosOld[SERVO_COUNT],a[SERVO_COUNT];

NewSoftSerial blueToothSerial(rxBT,txBT);
Maestro maestroPololu(rxMaestro,txMaestro);
File myFile;

void sdWrite(char file[]=testFile);
void sdRemove(char file[]=testFile);
void sdRead(char file[]=testFile);
bool loadServoPos(unsigned char source=0,char file[]=testFile);
void sdMoveServos(char file[]=testFile);

void setup(){
	pinMode(resetPin, OUTPUT);
	reset();
	maestroPololu.set();
	#if DEBUG
	Serial.begin(baudRate);
	Serial.flush();
	Serial.println("\r\nCallitco initialization");
	#endif
	pinMode(sdPin, OUTPUT);

	if(!SD.begin(sdPin)){
		#if DEBUG
		Serial.println("SD initialization failed!");
		#endif
	}else{
		#if DEBUG
		Serial.println("SD initialization done.");
		#endif
	}

	pinMode(rxBT, INPUT);
	pinMode(txBT, OUTPUT);
	pinMode(rxMaestro, INPUT);
	pinMode(txMaestro, OUTPUT);
	setupBlueToothConnection();
	
	bt=0;
	bitWrite(bt,1,1);
	bitWrite(bt,2,1);
	fileSwitch=0;
	initPosition();
	// initLeds();
}

void initLeds(){
	for(int j=6;j<=10;++j){
		pinMode(j, OUTPUT);
	}
}

void ledBlink(int trigger){
	for(int j=6;j<=10;++j){
		digitalWrite(j, trigger);
		delay(100);
	}
}

int ledTrigger=0;

void loop(){
	parse();
}

void setupBlueToothConnection(){
	blueToothSerial.begin(baudRateBT); //Set BluetoothBee BaudRate to default baud rate 38400
	delay(1000);
	blueToothSerial.print("\r\n+STWMOD=0\r\n"); //set the bluetooth work in slave mode
	blueToothSerial.print("\r\n+STNA=Callitco\r\n"); //set the bluetooth name as "SeeedBTSlave"
	//blueToothSerial.print("\r\n+STPIN=4321\r\n"); //pair code
	blueToothSerial.print("\r\n+STOAUT=1\r\n"); // Permit Paired device to connect me
	blueToothSerial.print("\r\n+STAUTO=0\r\n"); // Auto-connection should be forbidden here
	delay(2000); // This delay is required.
	blueToothSerial.print("\r\n+INQ=1\r\n"); //make the slave bluetooth inquirable 
	#if DEBUG
	Serial.println("The slave bluetooth is ready!");
	#endif
	delay(2000); // This delay is required.
	blueToothSerial.flush();
}

void reset(){
	delay(200);
	digitalWrite(resetPin, LOW);
	delay(200);
	digitalWrite(resetPin, HIGH);
	delay(200);
	
	fileCur=0;
	BTi=0;
	#if DEBUG
	Serial.print("BTi=0\r\n");
	#endif
	servoSpeed=0;
}

void readPositionSD(char file[]=testFile){
	unsigned char B;
	myFile=SD.open(file,FILE_WRITE);
	for(i=0;i<SERVO_COUNT;++i){
		B=BBtoB(maestroPololu.getPositionCompact(i));
		myFile.print(B,BYTE);
		
		#if DEBUG
		if(i==0)Serial.print("--readPositionSD--\r\n");
		Serial.print(B,DEC);
		Serial.print(" ");
		if(i==SERVO_COUNT-1)Serial.print("\r\n");
		#endif
	}
	myFile.close();
}

void parse(){
	unsigned char B2,BTsize;
	
	//--serial{
	#if DEBUG
	if(Serial.available()){
		B2=Serial.read();
		
		#if DEBUG
		Serial.print("---serial---\r\n");
		Serial.print(B2,DEC);
		Serial.print(" ");
		Serial.print(B2,BYTE);
		Serial.print("\r\n\r\n");
		#endif

		if(B2=='1'){reset();}
		else if(B2=='2'){initPosition();}
		else if(B2=='3'){sdMoveServos(danceFile);}
		else if(B2=='4'){sdMoveServos(danceFile2);}
		else if(B2=='5'){sdRead(danceFile);}
		else if(B2=='6'){fileCur=0;while(loadServoPos(3,danceFile)){moveServos();}}
		else if(B2=='7'){}
		else if(B2=='8'){}
		else if(B2=='9'){}
		else if(B2=='0'){readPositionSD();}
		else if(B2=='q'){}
		else if(B2=='w'){}
		Serial.flush();
	}
	#endif
	//--serial}

	//--bluetooth{
	BTsize=0;
	BTsize=blueToothSerial.available();
	if(BTsize){
		#if DEBUG
		Serial.print("\r\n---available(");
		Serial.print(BTsize,DEC);
		Serial.print(")---\r\n");
		#endif
		
		for(i=0;i<BTsize;++i){
			BT[BTi+i]=blueToothSerial.read() & 0xff;
			
			#if DEBUG
			Serial.print(BTi+i,DEC);
			Serial.print(":");
			Serial.print(BT[BTi+i],DEC);
			Serial.print(" ");
			Serial.print(BT[BTi+i],BYTE);
			Serial.print("\r\n");
			#endif
		}
		BTi+=BTsize;//BTi is bluetooth size now, !!carefully, you have to reset it
		
		if(bitRead(bt,1)==0){
			if(BTi<headB){bitWrite(bt,2,1);}
			else if(BT[0]==13 && BT[1]==10){//bluetooth end command sequence started
				#if DEBUG
				Serial.print("BTi=0\r\n");
				#endif
				BTi=0;
				bitWrite(bt,1,1);
			}else if(minCommand<=BT[0] && BT[0]<=maxCommand){//valid command
				if(BT[1]>BTi-headB){
					bitWrite(bt,2,1);
				}
				else{bitWrite(bt,2,0);}
			}else{
				BTi=0;bitWrite(bt,2,1);
				
				#if DEBUG
				Serial.print("BTi=0\r\n");
				Serial.print("---error(1)---\r\n");
				#endif
			}
			
			if(bt==0){//all bits 0
				cmd=0;bytes=0;start=headB;
				
				i=0;//global BT iterator
				ii=0;//inside Command iterator
				iii=0;//dataBytes iterator
				for(i=0;i<BTi;++i){
					if(ii==0){
						cmd=BT[i];++ii;
					}else if(ii==1){
						bytes=BT[i];++ii;
					}else{
						start=i;
						ii=0;i+=bytes-1;
						
						if(start+bytes>BTi){
							#if DEBUG
							Serial.print("---error(2)---\r\n");
							Serial.print("BTi=0\r\n");
							#endif
							BTi=0;
						}
					}
				}
				
				#if DEBUG
				Serial.print("start: ");
				Serial.print(start,DEC);
				Serial.print(" bytes: ");
				Serial.print(bytes,DEC);
				Serial.print("\r\n");
				#endif
				
				if(BTi>=headB){
					#if DEBUG
					Serial.print("executing\r\nBTi=0\r\n");
					#endif
					BTi=0;
					exec();
				}
			}
		}else{
			char H[]="hello";
			
			for(i=0;i<BTi;++i){
				if(BT[i]==H[i]){
					if(H[i]=='o'){
						bitWrite(bt,1,0);
						BTi=0;
						blueToothSerial.print(2,BYTE);
						#if DEBUG
						Serial.println("---got hello---");
						#endif
						break;
					}
				}
				else{BTi=0;break;}
			}
			//can continue receive here
		}
	}
	//--bluetooth}	
}

bool loadServoPos(unsigned char source,char file[]){//source: 0=from console, 1=from maestro, 2=from BT with servoNum and servoPos, 3=from SD, 4=from BT all
	//servoPos is old now{
	servoPosSizeOld=servoPosSize;
	for(i=0;i<servoPosSize;++i){
		servoPosKeysOld[i]=servoPosKeys[i];
		servoPosOld[servoPosKeysOld[i]]=servoPos[servoPosKeys[i]];
	}
	//}
	
	if(source==2){
		ii=0;
		for(i=0;i<bytes;++i){
			if(i%2==0){
				servoPosKeys[ii]=BT[start+i];
			}
			else{
				servoPos[servoPosKeys[ii]]=BtoBB(BT[start+i]);
				++ii;
			}
		}
		servoPosSize=ii;
	}else if(source==4){
		for(i=0;i<bytes;++i){
			if(i<SERVO_COUNT){
				servoPosKeys[i]=i;
				servoPos[i]=BtoBB(BT[start+i]);
			}else{servoSpeed=BT[start+i];}
		}
		servoPosSize=SERVO_COUNT;
	}else if(source==3){
		myFile = SD.open(file);
		if(!myFile){
			#if DEBUG
			Serial.print("error opening ");
			Serial.println(file);
			#endif
			
			return false;
		}
		
		if(fileCur>=myFile.available()){
			#if DEBUG
			Serial.println("---EOF---");
			#endif

			return false;
		}
		else{
			myFile.seek(fileCur);
		}
		
		for(i=0;i<SERVO_COUNT;++i){
			servoPos[i]=BtoBB(myFile.read());
		}
		servoPosSize=i;
		servoSpeed=myFile.read();
		++i;
		fileCur+=i;
		
		myFile.close();
	}else{
		for(i=0;i<SERVO_COUNT;++i){
			if(source==1){servoPos[i]=a[i]*4;}
			servoPosKeys[i]=i;
		}
		servoPosSize=SERVO_COUNT;
	}
	
	#if DEBUG
	for(i=0;i<SERVO_COUNT;++i){
		if(servoPosKeys[0]<=i && i<=servoPosKeys[0]+servoPosSize-1){Serial.print("\t");}
		Serial.print("servoPos[");
		Serial.print(i,DEC);
		Serial.print("]=");
		Serial.print(servoPos[i],DEC);
		Serial.print(";\r\n");
		
		if(servoPosKeysOld[0]<=i && i<=servoPosKeysOld[0]+servoPosSizeOld-1){Serial.print("\t");}
		Serial.print("servoPosOld[");
		Serial.print(i,DEC);
		Serial.print("]=");
		Serial.print(servoPosOld[i],DEC);
		Serial.print(";\r\n");
	}
	#endif
	
	return true;
}

//--exec{
void exec(){
	if(cmd==1){//singleMove
		loadServoPos(2);
		
		for(i=0;i<SERVO_COUNT;++i){
			maestroPololu.setSpeedCompact(i,0);
		}

		moveServosWorks();
		blueToothSerial.print(1,BYTE);
	}else if(cmd==2){//blueTooth moveServos
		loadServoPos(4);
		//moveServosWorks();
		moveServos();
		/*
		if(fileSwitch==0){
			sdWrite(testFile);
		}
		else if(fileSwitch==1){
			sdWrite(testFile2);
		}
		else if(fileSwitch==2){
			sdWrite(danceFile3);
		}
		*/
		blueToothSerial.print(1,BYTE);
	}else if(cmd==3){//blueTooth sdWrite
		loadServoPos(4);
		if(fileSwitch==0){
			sdWrite(testFile);
		}
		else if(fileSwitch==1){
			sdWrite(testFile2);
		}
		else if(fileSwitch==2){
			sdWrite(danceFile3);
		}
		blueToothSerial.print(1,BYTE);
	}else if(cmd==10){//play SD main
		/*
		fileCur=0;
		while(loadServoPos(3,danceFile)){moveServos();}
		delay(3000);
		fileCur=0;
		while(loadServoPos(3,danceFile2)){moveServos();}
		delay(3000);
		*/
		if(fileSwitch==0){
			// sdMoveServos(testFile);
			fileCur=0;moveServos();
			while(loadServoPos(3,testFile)){moveServos();}
		}
		else if(fileSwitch==1){
			// sdMoveServos(testFile2);
			fileCur=0;moveServos();
			while(loadServoPos(3,testFile2)){moveServos();}
		}
		else if(fileSwitch==2){
			// sdMoveServos(danceFile3);
			fileCur=0;moveServos();
			while(loadServoPos(3,danceFile3)){moveServos();}
		}
	}else if(cmd==11){//SD remove
		fileCur=0;
		if(fileSwitch==0){
			sdRemove(testFile);
		}
		else if(fileSwitch==1){
			sdRemove(testFile2);
		}
		else if(fileSwitch==2){
			sdRemove(danceFile3);
		}
	}else if(cmd==12){//bluetooth read fileSwitch
		fileSwitch=BT[start];
		#if DEBUG
		Serial.print("fileSwitch=");
		Serial.print(BT[start],DEC);
		Serial.print("\r\n");
		#endif
	}else if(cmd==15){//reset
		reset();
		#if DEBUG
		Serial.print("reset=1");
		Serial.print("\r\n");
		#endif
	}else{}
}
//--exec}
/*
void sdCopy(char what[],char where[]){
	myFile=SD.open(what);
	File myFile2;
	myFile2=SD.open(where);
	int B=0;
	while(B!=-1){
		B=myFile.read();
		myFile2.print(B,BYTE);
	}
	myFile.close();
	myFile2.close();
}
*/

void sdMoveServos(char file[]){
	myFile = SD.open(file);
	if(!myFile){
		#if DEBUG
		Serial.print("error opening ");
		Serial.println(file);
		#endif
		
		return;
	}
	
	unsigned long available=myFile.available();
	available=available/(SERVO_COUNT+1);
	#if DEBUG
	Serial.print("available=");
	Serial.print(available,DEC);
	Serial.print("\r\n");
	#endif
	for(unsigned char j=0;j<available;++j){
		loadServoPos();
		for(i=0;i<SERVO_COUNT;++i){
			servoPos[i]=BtoBB(myFile.read());
		}
		servoSpeed=myFile.read();
		servoPosSize=SERVO_COUNT;
		moveServos();
	}
	myFile.close();
}

void sdWrite(char file[]){
	//File myFile;
	myFile = SD.open(file, FILE_WRITE);
	if(myFile){
		for(i=0;i<SERVO_COUNT;++i){
			myFile.print(BBtoB(servoPos[i]),BYTE);
		}
		myFile.print(servoSpeed,BYTE);
		myFile.close();
	}else{
		#if DEBUG
		Serial.print("error opening ");
		Serial.println(file);
		#endif
	}
}

void sdRemove(char file[]){
	SD.remove(file);
}

void sdRead(char file[]){
	myFile = SD.open(file);
	if(myFile){
		unsigned char B;
		unsigned int fileSize;
		fileSize=myFile.available();
		
		#if DEBUG
		Serial.print("\r\n---");
		Serial.print(file);
		Serial.print("(");
		Serial.print(fileSize,DEC);
		Serial.print(")---\r\n");
		i=0;
		for(i=0;i<fileSize;++i){
			B=myFile.read();
			Serial.print(i,DEC);
			Serial.print(":");
			Serial.print(B,DEC);
			Serial.print(" ");
			Serial.print(B,BYTE);
			Serial.print("\r\n");
		}
		#endif
		
		myFile.close();
	}else{
		#if DEBUG
		Serial.print("error opening ");
		Serial.println(file);
		#endif
	}
}

void moveServos(){
	timeStart=millis();
	unsigned short diff[SERVO_COUNT];
	//char sign[SERVO_COUNT];
	ii=0;
	for(i=0;i<SERVO_COUNT;++i){
		if(servoPos[i]>=servoPosOld[i]){
			//sign[servoPosKeys[i]]=1;
			diff[i]=servoPos[i]-servoPosOld[i];
		}
		else{
			//sign[servoPosKeys[i]]=-1;
			diff[i]=servoPosOld[i]-servoPos[i];
		}
		if(diff[i]==0){++ii;}
	}

	#if DEBUG
	Serial.print("servoSpeed=");
	Serial.print(servoSpeed,DEC);
	Serial.print("\r\n");
	#endif
	if(ii!=SERVO_COUNT){
		unsigned long spd=0;
		for(i=0;i<SERVO_COUNT;++i){
			if(servoSpeed==0){spd=0;}
			else{
				spd=(unsigned long)(5*diff[i])/(servoSpeed<<4);
				if(spd==0){spd=1;}
			}
			
			#if DEBUG
			Serial.print(i,DEC);
			Serial.print(": ");
			Serial.print("diff=");
			Serial.print(diff[i],DEC);
			Serial.print(" spd=");
			Serial.print(spd,DEC);
			Serial.print("\r\n");
			#endif
			
			maestroPololu.setSpeedCompact(i,spd);
			//maestroPololu.setSpeedCompact(servoPosKeys[i],0);
		}
		maestroPololu.setMultipleTargets(servoPosSize,servoPosKeys,servoPos);
		
		#if DEBUG
		Serial.print("timeStart=");
		Serial.print(timeStart,DEC);
		Serial.print("\r\n");
		#endif
		while(maestroPololu.getMovingStateCompact()){
			#if DEBUG
			Serial.print("millis()-timeStart=");
			Serial.print(millis()-timeStart,DEC);
			Serial.print(" servoSpeed<<5=");
			Serial.print(servoSpeed<<5,DEC);
			Serial.print("\r\n");
			#endif
			
			if((millis()-timeStart)>=64+(servoSpeed<<5)){break;}
		}
	}
	else if(ii==SERVO_COUNT){
		delay(servoSpeed<<5);
		
		#if DEBUG
		Serial.print("allServos stands still for: ");
		Serial.print(servoSpeed<<5,DEC);
		Serial.print("ms\r\n");
		#endif
	}
	
	while((millis()-timeStart)<(servoSpeed<<5)){}
	/*
	for(i=0;i<SERVO_COUNT;++i){
		//maestroPololu.setSpeedCompact(servoPosKeys[i],diff[servoPosKeys[i]]);
		maestroPololu.setSpeedCompact(i,0);
	}
	maestroPololu.setMultipleTargets(servoPosSize,servoPosKeys,servoPos);
	*/
}

void moveServosWorks(){
	maestroPololu.setMultipleTargets(servoPosSize,servoPosKeys,servoPos);
	//while(!maestroPololu.getMovingStateCompact());
}

unsigned short BtoBB(unsigned char B){
	//return  (B*(maxPosition-minPosition)/254+minPosition)*4;
	return map(B,0,254,minPosition,maxPosition)<<2;
}

unsigned char BBtoB(unsigned short BB){
	//return (BB/4-minPosition)*254/(maxPosition-minPosition);
	return map(BB>>2,minPosition,maxPosition,0,254);
}

void initPosition(){
	loadServoPos();
	
	servoPos[0]=5884;
	servoPos[1]=8920; 
	servoPos[2]=7780; 
	servoPos[3]=6200;
	servoPos[4]=5124; 
	servoPos[5]=5884; 
	servoPos[6]=4112;
	servoPos[7]=4680; 
	servoPos[8]=5628; 
	servoPos[9]=5756; 
	servoPos[10]=2656; 
	servoPos[11]=8412;
	servoPos[12]=3796; 
	servoPos[13]=3100; 
	servoPos[14]=9296;
	servoPos[15]=3100; 
	servoPos[16]=7084;
	servoPos[17]=8856;
	
	moveServosWorks();
}

void moveServosPasha(){
	short delta = 64;
	short max = 0;
	short dist[SERVO_COUNT];
	bool sign[SERVO_COUNT];

	for(short i = 0; i < SERVO_COUNT; i ++){
		short diff;
		if(servoPos[i]>=servoPosOld[i]){
			diff=(servoPos[i]-servoPosOld[i])/delta;
			sign[i] = 1;
		}else{
			diff=(servoPosOld[i]-servoPos[i])/delta;
			sign[i] = 0;
		}

		dist[i] = abs(diff);

		if(dist[i] > max)
		max = dist[i];
	}

	short shift[SERVO_COUNT];
	for(short i = 0; i < SERVO_COUNT; i ++)
		shift[i] = 0;

	unsigned short sv[SERVO_COUNT];
	for(int i = 0; i < SERVO_COUNT; i ++){
		sv[i] = servoPosOld[i];
	}

	for(short i = 0; i < max; i ++) {
		for(short j = 0; j < SERVO_COUNT; j ++) {
			shift[j] += dist[j];
			if(2 * shift[j] > max) {
				shift[j] -= max;
				sv[j] += (sign[j] ? delta : -delta);
				maestroPololu.setTargetCompact(j, sv[j]);
			}
		}
	}

	while(maestroPololu.getMovingStateCompact());
	maestroPololu.setMultipleTargets(servoPosSize,servoPosKeys,servoPos);
}