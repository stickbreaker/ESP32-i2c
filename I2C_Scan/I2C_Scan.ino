/* I2C slave Address Scanner
for 5V bus
 * Connect a 4.7k resistor between SDA and Vcc
 * Connect a 4.7k resistor between SCL and Vcc
for 3.3V bus
 * Connect a 2.4k resistor between SDA and Vcc
 * Connect a 2.4k resistor between SCL and Vcc

 */

#include <Wire.h>
/* Declarations to support Slave Mode */
#include "esp_attr.h"
#include "soc/i2c_reg.h"
#include "soc/i2c_struct.h"
#include "I2C_Scan.h"

#define DR_REG_I2C_EXT_BASE_FIXED               0x60013000
#define DR_REG_I2C1_EXT_BASE_FIXED              0x60027000

i2c_dev_t * dev=NULL;
volatile uint32_t* port=NULL;
/*forwards */
uint16_t dispField(STATUS_vt* vt, uint16_t inlen,bool force, bool title);
void status_createTitle(char* buf,uint16_t len, uint16_t index);

uint32_t timeOut =0; // current timeout mark
uint32_t RepeatPeriod = 1000; // default 1 second
uint16_t BlockLen = 10; // default to 10 byte transfer
uint16_t ID = 0x50; // default to EEPROM
bool TenBit = false; // 7bit mode
uint16_t addr=0;

#define BUFLEN 100
char keybuf[BUFLEN+1];
uint16_t keyLen=0;
SERIAL_COMMANDS_t nextCommand=NO_COMMAND,currentCommand=NO_COMMAND,priorCommand=NO_COMMAND;
bool status_on=false; // continous I2C status reporting

char *command_name(SERIAL_COMMANDS_t cmd){
uint16_t a=0;
char* name=(char*)&COMMAND_NAMES;
while((a<cmd)&&(name[0])){
	name= name + strlen(name)+1;
	a++;
	}
return name;
}

void setI2cDev(uint8_t num){
if(num==0) {
	dev=(volatile i2c_dev_t*)DR_REG_I2C_EXT_BASE_FIXED;
	port=(volatile uint32_t*)DR_REG_I2C_EXT_BASE_FIXED;
	}
else {
	dev=(volatile i2c_dev_t*)DR_REG_I2C1_EXT_BASE_FIXED;
	port=(volatile uint32_t*)DR_REG_I2C1_EXT_BASE_FIXED;
	}
}
	
void setSlave(uint16_t slaveId,bool tenBit){
dev->ctr.trans_start=0;
dev->slave_addr.en_10bit=(tenBit?1:0);
if(tenBit){// got to really fung the address
	dev->slave_addr.addr=((slaveId&0xff)<<7)|(((slaveId>>8)&0x3)|0x78);
	}
else dev->slave_addr.addr=slaveId;
Serial.printf("address 0x%02X mode %s\n",slaveId,tenBit?"10Bit":"7Bit");
Serial.printf("act slaveAddress =0x%04X\n",dev->slave_addr.addr);
Serial.printf("Fifo_Conf=0x%08lx\n",dev->fifo_conf.val);
dev->fifo_conf.fifo_addr_cfg_en=0;
dev->fifo_conf.tx_fifo_empty_thrhd =1;
dev->fifo_conf.nonfifo_tx_thres = 1;
dev->ctr.ms_mode = 0; // set as slave
dev->ctr.trans_start=1;
}

void emptyFifo(){
while(dev->status_reg.rx_fifo_cnt >0){
	uint8_t b= dev->fifo_data.data;
	Serial.printf("0x%02x ",b);
	}
}

uint16_t xtoi(char * buf){
uint16_t len=0;
uint16_t out=0;
if(buf){
	while(buf[len]){
		if((buf[len]<'0')
			||((buf[len]>'9')&&(buf[len]<'A'))
			||((buf[len]>'F')&&(buf[len]<'a'))
			||((buf[len]>'f'))) return out;
		out *=16;
		uint8_t b=buf[len]-48;
		if(b>9) b -=7;
		if(b>15) b -= 32;
		out += b;
		len++;
		}
	}
return out;
}

uint32_t xtol(char *buf){
uint16_t len=0;
uint32_t out=0;
if(buf){
	while(buf[len]){
		if((buf[len]<'0')
			||((buf[len]>'9')&&(buf[len]<'A'))
			||((buf[len]>'F')&&(buf[len]<'a'))
			||((buf[len]>'f'))) return out;
		out *=16;
		uint8_t b=buf[len]-48;
		if(b>9) b -=7;
		if(b>15) b -= 32;
		out += b;
		len++;
		}
	}
return out;
}

char * proc_slave(uint8_t bits,char** savePtr){
char * param = strtok_r(NULL," ,",savePtr);
uint16_t i=xtoi(param);
if(i!=0){
	setSlave(i,(bits==10));
	nextCommand = COMMAND_SLAVE_MODE;
	}
return NULL;
}

char * proc_off(char** savePtr){
char * param = strtok_r(NULL," ,",savePtr);
Serial.printf("OFF with: [%s]\n",(param?param:""));
nextCommand = COMMAND_STOP;
return NULL;
}

char * proc_newRead(char** savePtr){
char * param = strtok_r(NULL," ,",savePtr);
if(param){ // get ID
	uint16_t t = xtoi(param);
	if(t!=0) ID=t;
	param = strtok_r(NULL," ,",savePtr);
	if(param){ // get repeat
		t = atoi(param);
		RepeatPeriod = t * 1000; // zero means oneshot
		}
	}
Serial.printf("NewRead 0x%02x len=%d repeat=%d\n",ID,BlockLen,(RepeatPeriod/1000));
nextCommand = COMMAND_NEW_READ;
timeOut = millis()-RepeatPeriod;
return NULL;
}

void printID(){
Serial.printf("I2C Addr=0x%03x\n",ID);
}

void printRepeat(){
Serial.printf("repeat=%d\n",(RepeatPeriod/1000));
}

void printBlock(){
Serial.printf("blockSize=%d\n",BlockLen);
}

void status_init_rec(STATUS_vt * vt){
	vt->length = 0;
	vt->position = 0;
	vt->mask = 0xFFFFFFFF;
	vt->oldVal = 0;
	vt->options = DISP_DEFINED;
	vt->portIndex=0;
	vt->fields = NULL;
	vt->fldCnt = 0;
	}

void status_init(){
sc.count=0;
for(uint8_t a=0; a<10;a++){
	status_init_rec(&sc.vt[a]);
	}
}

void status_help(){
Serial.print("\nStatus help\n");
Serial.println("  stat [[+|-|<|>]reg_name[:raw|:r|:d|:nraw|:nr|:nd|:m###]\n"
  "				+  = add\n"
	"				-  = remove\n"
	"				>  = Shift right\n"
	"				<  = shift Left\n"
  "				reg_name	= I2C register name\n"
	"				:raw	= display register as 0x08lx\n"
	"				:r		= display reserved fields\n"
	"				:d		= display defined fields\n"
	"				:m##	= set mask value to (##)hex digits\n\n");
Serial.println("  stat [on|off|init|title|names|?]\n"
	"				on	= continous updates\n"
	"				off	= end updates\n"
	"				init	= delete all status selections\n"
	"				title	= Display column headings\n"
	"				names	= list valid register names\n");
}

uint16_t status_find_name(char *nextParam){ // returns 1..n or 0 for not found
/*
needs to handle:
 command[nn]   // this would return 23+nn 
 command[nn]:raw:mff // this would return 23+nn
 int_status:mf
 
*/

char* catalog=(char*)&STATUS_NAMES;
bool found=false;
uint16_t cnt=1;
char* savePtr, *testParam, *testBuf, *arryTest;
testBuf =(char*)malloc(strlen(nextParam)+1); // remember the Terminating Null!
strcpy(testBuf,nextParam); // build temp buf for strtok_r
testParam =strtok_r(testBuf,"[:",&savePtr);
char buf[32]; // buffer to handle '[]'

while(!found&&catalog[0]){
	strcpy(buf,catalog);
	arryTest = strtok(buf,"[");
	found=(strcmp(testParam,arryTest)==0);
//	Serial.printf("comp(\"%s\",\"%s\")==%d\n",testParam,arryTest,found);
	if(!found){
		catalog = catalog + strlen(catalog)+1;
		cnt++;
		}
	}
if(found){ // now we have to handle the [] if it exists
	strcpy(testBuf,nextParam);// build temp buf for strtok_r
	testParam = strchr(testBuf,'[');
	int16_t i=0;
	if(testParam) {
//		Serial.print("found [");
		i= atoi(testParam+1);
		if (cnt==23) { // command!
		  if((i>=0)&&(i<=15)) cnt += i;
			}
//		Serial.printf("%d] cnt=%d",i,cnt);
		}
	}

free(testBuf);

if(found) return cnt;
else return 0;
}

char* status_name_byIndex(uint16_t index){
	// returns pointer to name text;
char* catalog=(char*)&STATUS_NAMES;
if(index>=(STATUS_NAMES_COUNT-1)){
 index = STATUS_NAMES_COUNT-1;
	}
	
while(index&&catalog[0]){
	catalog = catalog + strlen(catalog)+1;
  index--;
	}
return catalog;
}

uint16_t status_find_inList(char *nextParam){
bool found =false;
uint16_t b=atoi(nextParam),cmd=status_find_name(nextParam);
if(b&&(b<=sc.count)){
//	Serial.printf("inList directed =%d\n",b);
	return b; // directed?
	}
if(cmd){
	cmd--; // 1 based
	while(!found&&(b<sc.count)){
		found = (sc.vt[b].portIndex==cmd);
		if(!found) b++;
		}
	}
if(found)
	return b+1;
else
	return 0;
}

void status_update(char *nextParam, uint16_t directed){ // nextParam points to the next single word with suffixs
uint16_t b;
if(directed) b = directed;
else b = status_find_inList(nextParam);
if(b){ // exists valid to modify
//	Serial.printf("update: %s as %d\n",nextParam,b);
//	status_mask_disp(nextParam,b);
	char* testParam=strchr(nextParam,':');
	char* saveptr;
	if(testParam) { // has suffixes
		testParam=strtok_r(nextParam,":",&saveptr); // throw away word
		testParam=strtok_r(NULL,":",&saveptr); // now first suffix
		}
	while(testParam){ // has suffixes
//		Serial.printf("parsing %s\n",testParam);
		if(strcmp(testParam,"raw")==0){
			sc.vt[b-1].options |= DISP_RAW;
			}
		else if(strcmp(testParam,"nraw")==0){
			sc.vt[b-1].options &= ~DISP_RAW;
			}
		else if(strcmp(testParam,"d")==0){
			sc.vt[b-1].options |= DISP_DEFINED;
			}
		else if(strcmp(testParam,"nd")==0){
			sc.vt[b-1].options &= ~DISP_DEFINED;
			}
		else if(strcmp(testParam,"r")==0){
			sc.vt[b-1].options |= DISP_RESERVED;
			}
		else if(strcmp(testParam,"nr")==0){
			sc.vt[b-1].options &= ~DISP_RESERVED;
			}
		else if(testParam[0]=='m'){//mask
			sc.vt[b-1].mask = xtol(&testParam[1]);
			}
		else{
			Serial.printf("unknown suffix %s\n",testParam);
			}
		testParam = strtok_r(NULL,":",&saveptr);
		}
//	status_mask_disp(nextParam,b);
	}
}

void status_add(char *nextParam){
STATUS_vt vtx;
status_init_rec(&vtx);
nextParam++; // skip past '+'
uint16_t cmd=status_find_name(nextParam);
//Serial.printf("status_add: %s=%d\n",nextParam,cmd);

if(cmd&&(sc.count<10)){// ok, now have to parse the STATUS_FIELDS to create MASK and fields[]
  cmd--; // status_find_name is 1 based
	vtx.portIndex = cmd; //22..37 are the Command fields 
  bool isCommand = (cmd>=(STATUS_NAMES_COUNT-1));
	uint16_t tempCmd = (isCommand)?STATUS_NAMES_COUNT-1:cmd;
  int8_t fx[32];
	uint8_t count=0;
	char* fdesc=(char*)&STATUS_FIELDS;
	char* savePtr;
	char buf[100]; //temp buffer
	while(tempCmd&&fdesc[0]){// inc through STATUS_FIELDS for each line
		fdesc = fdesc + strlen(fdesc)+1;
		tempCmd--;
		}
	// now should have pointer to correct STATUS_FIELDS line.
	strcpy(buf,fdesc);
	fdesc=strtok_r(buf,",",&savePtr);
	int8_t tf=0;
	char* endptr;
	while(fdesc&&(count<32)){ // convert from text to int
	  tf = strtol(fdesc,&endptr,10);
		fx[count] = abs(tf);
		if(tf<0) { // neg, reserved field
			fx[count] |= DISP_RESERVED;
			}
		if(endptr){
			if(endptr[0]=='H')
				fx[count] |= DISP_HEX;
			else if(endptr[0]=='B')
				fx[count] |= DISP_BOOL;
			}
		count++;
		fdesc=strtok_r(NULL,",",&savePtr);
		}
	vtx.fields=(uint8_t*)malloc(count);
	memmove(vtx.fields,&fx,count);
	vtx.fldCnt = count;
	
// build default defined field mask
	uint16_t i=0,j=32,k=0;
	uint32_t m=0;
	while(i<vtx.fldCnt){
		k=vtx.fields[i++];
		j=j - (k&0x1f);
		m=m<<(k&0x1f);// make room
		if(!(k&DISP_RESERVED)){ // defined field
			m=m | (((uint32_t)1<<(k&0x1f))-1); // create mask, merge it
			}
		}
	vtx.mask = m;		
	
	memmove(&sc.vt[sc.count++],&vtx,sizeof(vtx));
//	Serial.printf("added vt[%d] portIndex=%d",sc.count-1,sc.vt[sc.count-1].portIndex);
	status_update(nextParam,sc.count); // apply suffix 
	}
else { // unknown parameter
  if(sc.count<10)
		Serial.printf("Unknown statusWord =%s\n",nextParam);
	else
		Serial.print("too many monitored Stat");
	}
}

void status_remove(char *nextParam){
//Serial.printf("remove %s\n",nextParam);
nextParam++; // skip '-'
uint16_t b=status_find_inList(nextParam);
if(b){
	free(sc.vt[b-1].fields); // release field data array
	memmove(&sc.vt[b-1],&sc.vt[b],sizeof(STATUS_vt)*((sc.count-b)));
	sc.count--;
	}
}
	
void status_display_word(char *nextParam,bool title){
//Serial.printf("display word %s\n",nextParam);
uint16_t b = status_find_inList(nextParam);
if(b){// exists in list
	uint16_t len = 0;
	dispField(&sc.vt[b-1],len,true,title);
	}
}

void status_display_list(bool title){
uint16_t len=0,a=0;
while(a<sc.count){
	len=dispField(&sc.vt[a],len,true,title);
	a++;
	}
Serial.println();
}

void status_mask_disp(char *nextParam, uint16_t directed){
nextParam++; // skip '?'
uint16_t b;
if(directed) b= directed;
else b= status_find_inList(nextParam);
char buf[100];
//Serial.printf("\nmask_disp: %s as %d\n",nextParam,b);
if(b){ // exists in list
	status_createName(buf,100,sc.vt[b-1].portIndex);
  Serial.printf("\n%s%s%s%s mask=0x%08lx\n",buf,
	  (sc.vt[b-1].options&DISP_RAW)?" raw":"",
		(sc.vt[b-1].options&DISP_RESERVED)?" r":"",
		(sc.vt[b-1].options&DISP_DEFINED)?" d":"",
		sc.vt[b-1].mask);
	}
}

void status_shift_left(char *nextParam){
nextParam++; //' skip '<'
uint16_t b= status_find_inList(nextParam);
if(b){
  if(b>1){ // exchange with left
		STATUS_vt vtx;
		memmove(&vtx,&sc.vt[b-2],sizeof(STATUS_vt));
		memmove(&sc.vt[b-2],&sc.vt[b-1],sizeof(STATUS_vt));
		sc.vt[b-2].position = vtx.position;
		vtx.position += sc.vt[b-2].length;
		memmove(&sc.vt[b-1],&vtx,sizeof(STATUS_vt));
		}
	else {
		// already at left edge!
		}
	}
}

void status_shift_right(char *nextParam){
nextParam++; //' skip '>'
uint16_t b= status_find_inList(nextParam);
if(b){
  if(b<sc.count){ // exchange with right
		STATUS_vt vtx;
		memmove(&vtx,&sc.vt[b-1],sizeof(STATUS_vt));
		memmove(&sc.vt[b-1],&sc.vt[b],sizeof(STATUS_vt));
		sc.vt[b-1].position = vtx.position;
		vtx.position += sc.vt[b-1].length;
		memmove(&sc.vt[b],&vtx,sizeof(STATUS_vt));
		}
	else {
		// already at right edge!
		}
	}
}

void status_process_list(char *nextParam, char** savePtr){
bool titleChanged=false;
while(nextParam){
//Serial.printf("status Process List=%s\n",nextParam);
	if(nextParam[0]=='+'){ // add next status word to list
		status_add(nextParam);
		titleChanged = true;
		}
	else if(nextParam[0]=='-'){ // remove next status word from list
	  status_remove(nextParam);
		titleChanged = true;
		}
	else if(nextParam[0]=='?'){// display mask value
		status_mask_disp(nextParam,0);
		}
	else if(nextParam[0]=='>'){// shift this word right
	  status_shift_right(nextParam);
		titleChanged = true;
		}
	else if(nextParam[0]=='<'){// shift this word left
	  status_shift_left(nextParam);
		titleChanged = true;
		}
	else { // display info for this word
		char* testParam=strchr(nextParam,':');
		if(testParam) status_update(nextParam,0);
		else status_display_word(nextParam,false);
		}	
	
	nextParam=strtok_r(NULL," ,",savePtr);
	}
if(titleChanged) status_display_list(true);
}

void status_display_register_names(void){
uint16_t a=0;
char buf[100];
while(a<STATUS_NAMES_COUNT_MAX){
	status_createName(buf,100,a++);
	Serial.println(buf);
	}
}

void dispBuff(uint8_t *buf, uint16_t len,uint16_t offset){
char asciibuf[100];
uint8_t bufPos=0;
uint16_t adr=0;
asciibuf[0] ='\0';
while(adr<len){
	if(((offset+adr)&0x1F)==0){
		Serial.printf(" %s\n0x%04x:",asciibuf,offset+adr);
		bufPos=0;
		}
	Serial.printf(" %02x",buf[adr]);
	char ch=buf[adr];
	if((ch<32)||(ch>127)) ch ='.';
	bufPos+=sprintf(&asciibuf[bufPos],"%c",ch);
	adr++;
	}

while(bufPos<32){
	Serial.print("   ");
	bufPos++;
	}
Serial.printf(" %s\n",asciibuf);
}

void bigBlock(bool display){
uint32_t start=0;
Serial.printf("starting at: %ld ",start=millis());
Wire.beginTransmission(ID);
Wire.write(highByte(addr));
Wire.write(lowByte(addr));
uint16_t err=Wire.endTransmission();
if(err){
	Serial.printf(" setting address failed =%d",err);
	nextCommand=NO_COMMAND;
	return;
	}
uint8_t* bigBlock=(uint8_t*)calloc(BlockLen,sizeof(uint8_t));
uint16_t cnt=Wire.newRequestFrom(ID,bigBlock,BlockLen,true);
Serial.printf(" completed at: %ld, %ld cnt=%d\n",millis(),millis()-start,cnt);
if(cnt!=BlockLen){
	Serial.printf("ERROR newRequestFrom \n");
	nextCommand=NO_COMMAND;
	}
if(display||(cnt==0)) dispBuff(bigBlock,BlockLen,addr);
free(bigBlock);
}

void setSpeed(uint32_t speed){
Wire.setClock(speed);
}

void processCommand(){
//Serial.println("processing:");
//Serial.println(keybuf);

char *nextParam,*saveptr,*line,*saveptr1;

uint16_t l=0;
uint32_t ll=0;
line = strtok_r(keybuf,"\n",&saveptr);
if(line==NULL){
	if(currentCommand==NO_COMMAND) nextCommand=priorCommand;
	else nextCommand=currentCommand; // repeat last command on enter
	}
else{
	nextParam =strtok_r(line," ,",&saveptr1);
	}
while(line){ // have next command line, parse it
  if(strcmp(nextParam,"big")==0){
		bigBlock(true);
		}
	else if(strcmp(nextParam,"explode")==0){
		nextCommand=COMMAND_EXPLODE;
		}
	else if(strcmp(nextParam,"speed")==0){
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam) {
			ll=atol(nextParam);
			setSpeed(ll);
			}
		}
	else if(strcmp(nextParam,"addr")==0){ // set next read address
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam) l=xtoi(nextParam);
		else l=0;
		addr = l;
		}
	else if(strcmp(nextParam,"slave")==0){// "slave, id(7bit hex)=0x83"
		nextParam = proc_slave(7,&saveptr1);
		}
	else if(strcmp(nextParam,"slave10")==0){// "slave, id(10bit hex)=0x113"
		nextParam = proc_slave(10,&saveptr1);
		}
	else if(strcmp(nextParam,"off")==0){ // shutdown I2C, 
		nextParam = proc_off(&saveptr1);
		}
	else if(strcmp(nextParam,"nr")==0){ // new read "nr id(hex)=0x50 len(dec)=10 repeat(dec seconds)=5"
		nextParam = proc_newRead(&saveptr1);
		}
	else if(strcmp(nextParam,"scan")==0){ // run I2C device NAK scan
		nextCommand = COMMAND_SCAN;
		}
	else if(strcmp(nextParam,"size")==0){ // run I2C EEPROM size probe
		nextCommand = COMMAND_SIZE;
		}
	else if(strcmp(nextParam,"pins")==0){ // read scl,sda value
	  Serial.printf("\nscl=%d, sda=%d\n",digitalRead(22),digitalRead(21));
		}
	else if(strcmp(nextParam,"pinsscl")==0){ // try to manhandle them
	  Serial.printf("before scl=%d, sda=%d\n",digitalRead(22),digitalRead(21));
//	  digitalWrite(21,HIGH);
		for(uint8_t i=0;i<9;i++){
			digitalWrite(22,HIGH);
			digitalWrite(22,LOW);
			}
	  Serial.printf("after  scl=%d, sda=%d\n",digitalRead(22),digitalRead(21));
		}
	else if(strcmp(nextParam,"pinssda")==0){ // try to manhandle them
	  Serial.printf("before scl=%d, sda=%d\n",digitalRead(22),digitalRead(21));
//	  digitalWrite(21,HIGH);
		for(uint8_t i=0;i<9;i++){
			digitalWrite(21,HIGH);
			digitalWrite(21,LOW);
			}
	  Serial.printf("after  scl=%d, sda=%d\n",digitalRead(22),digitalRead(21));
		}
	else if(strcmp(nextParam,"?")==0){ // run I2C EEPROM size probe
	  displayHelp();
//		nextCommand = COMMAND_HELP;
		}
	else if(strcmp(nextParam,"help")==0){ // run I2C EEPROM size probe
		nextCommand = COMMAND_HELP;
		}
	else if(strcmp(nextParam,"repeat")==0){
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam) l=atoi(nextParam);
		else l=0;
		
		RepeatPeriod = l*1000;
		printRepeat();
		}
	else if(strcmp(nextParam,"block")==0){
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam) l=atoi(nextParam);
		else l=0;
		BlockLen = l;
		printBlock();
		}
	else if(strcmp(nextParam,"id")==0){
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam) l=xtoi(nextParam);
		else l=0;
		ID = l;
		printID();
		}
	else if(strcmp(nextParam,"stat")==0){
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam){
			if(strcmp(nextParam,"on")==0) { // enable continous Status output
			  status_on = true;
				}
			else if(strcmp(nextParam,"off")==0) { // Stop status Display, retain prior values
				status_on = false;
				}
			else if(strcmp(nextParam,"init")==0){ // Stop display, set all position to zero
				status_init();
				}
			else if((strcmp(nextParam,"title")==0)||
				(strcmp(nextParam,"t")==0)){
				status_display_list(true);
				}
			else if(strcmp(nextParam,"names")==0){ // display list of register names
				status_display_register_names();
				}
			else if(strcmp(nextParam,"?")==0){
				status_help();
				}	
			else status_process_list(nextParam,&saveptr1);
			}
		else status_display_list(false);
//		nextCommand=COMMAND_STATUS;
		}
	
	line = strtok_r(NULL,"\n",&saveptr);
	if(line){
		nextParam = strtok_r(line," ,",&saveptr1);
		}
	}
keyLen=0;
}

void processSerial(){
char ch=Serial.read();
if(ch=='\n'){
	keybuf[keyLen]='\0';
	processCommand();
	}
else {
	keybuf[keyLen]=ch;
	if(keyLen<BUFLEN) keyLen++;
	}
}

void NewRead(){
uint8_t err=0;
if((millis()-timeOut)>RepeatPeriod){
	if(RepeatPeriod==0){ // one shot
		priorCommand = currentCommand;
		currentCommand = NO_COMMAND;
		}
	timeOut = millis(); // reset for next
	Serial.print("before call");
	Wire.beginTransmission(ID);
	Wire.write(highByte(addr));
	Wire.write(lowByte(addr));
	if((err=Wire.endTransmission())!=0)	{
		Serial.printf("EndTransmission=%d, resetting\n",err);
		Wire.reset();
		}
	err = Wire.newRequestFrom(ID,BlockLen,true);
	Serial.printf("@0x%02x(0x%04x)=%d ->",ID,addr,err);
	addr += BlockLen;
	uint8_t b=0;
	char buf[100];
	uint16_t blen=0;
	
	while(Wire.available()){
		char a=Wire.read();
		if((a<32)||(a>127)) Serial.print('.');
		else Serial.print(a);
		blen+=sprintf(&buf[blen]," %02x",a);
		b++;
		if((b%32)==0){
			Serial.println(buf);
			blen=0;
			}
		}
	if(!((b%32)==0)) Serial.println(buf);
	}
}

void pollI2cStatus(){
uint32_t stat=0;
uint32_t statusMask = 0x00FFFF6E;
uint32_t fifo=0;
uint32_t intRaw=0;
bool change=true;
uint8_t eeProm=0x50;
pinMode(18,INPUT_PULLUP);
pinMode(14,INPUT_PULLUP); // new i2cRead()
char buf[256];
uint8_t tick=0;
uint32_t timeout=millis();

dev->int_ena.val = 0xFFFFFFFF;
dev->int_clr.val = 0xFFFFFFFF;
while(nextCommand==NO_COMMAND){
	if(Serial.available()) processSerial();
	
	memset(buf,32,100);
	sprintf(buf,"% 8ld ",millis());
	buf[9]=' ';
	uint32_t L = dev->status_reg.val;
	if((L&statusMask) != stat){
		change=true;
		stat=(L&statusMask); // don't want I2C bus phases
		sprintf((char*)&buf[9],"0x%08lx ",stat);
		buf[20]=' ';
		}
	L = dev->fifo_st.val;
	if(L != fifo){
		change = true;
		fifo=L;
		sprintf((char*)&buf[20],"0x%08lx ",fifo);
		buf[31]=' ';
		}
	L = dev->int_status.val;
	if(L != intRaw){
		change = true;
		intRaw=L;
		sprintf((char*)&buf[31],"0x%08lx ",intRaw);
		buf[42]=' ';
//		if(intRaw!=2){ // clear interrupt flags, ignore Tx_Fifo_Empty
		dev->int_clr.val=intRaw;

		}
	if(change){
		buf[42]='\0';
		buf[43]='\0';
  	Serial.print(buf);
		}
	if((intRaw & I2C_TRANS_COMPLETE_INT_RAW_M)==I2C_TRANS_COMPLETE_INT_RAW_M){
		emptyFifo();
		change=true;
		}
	if((dev->status_reg.tx_fifo_cnt<8)&&(!digitalRead(18))){
		dev->fifo_data.data=tick;
		dev->int_clr.val=I2C_TXFIFO_EMPTY_INT_RAW; // try to clear int after fill
		tick++;
		Serial.print('+');
		}
	if(change){
		Serial.println();
		change=false;
		}
	}
}

/*	
void IRAM_ATTR slave_isr_handler(void* arg){
    // ...
uint32_t num = (uint32_t)arg;
if(num==0) dev=(volatile i2c_dev_t*)DR_REG_I2C_EXT_BASE_FIXED;
else dev=(volatile i2c_dev_t*)DR_REG_I2C1_EXT_BASE_FIXED;

uint32_t stat = dev->int_status.val;		
		
}
*/

void scan(){
currentCommand=NO_COMMAND;
priorCommand=COMMAND_SCAN;
Serial.println("\n Scanning I2C Addresses");
uint8_t cnt=0;
for(uint8_t i=0;i<128;i++){
  Wire.beginTransmission(i);
  uint8_t ec=Wire.endTransmission(true);
  if(ec==0){
    if(i<16)Serial.print('0');
    Serial.print(i,HEX);
    cnt++;
  }
  else Serial.print("..");
  Serial.print(' ');
  if ((i&0x0f)==0x0f)Serial.println();
  }
Serial.print("Scan Completed, ");
Serial.print(cnt);
Serial.println(" I2C Devices found.");
}
void status_createName(char* buf, uint16_t maxLen, uint16_t index){
bool isCommand=(index>=(STATUS_NAMES_COUNT-1));
uint16_t tempIndex= (isCommand)?STATUS_NAMES_COUNT-1:index;
char* title=status_name_byIndex(tempIndex);
if(!title) {
	buf[0]='\0';
	return;
	}
if(maxLen>strlen(title)){
	sprintf(buf,title,(index-(STATUS_NAMES_COUNT-1)));
	}
else {
	memmove(buf,title,maxLen-1);
	buf[maxLen-1] ='\0';
	}
}
	
void status_createTitle(char* buf,uint16_t len, uint16_t index){
char titleBuf[100];
status_createName(titleBuf,100,index);

memset(buf,'-',len);
buf[0]= ' ';
buf[1]= '|';
buf[len-1]= '|';
buf[len] = '\0';

if(len>strlen(titleBuf)){
	len-=strlen(titleBuf);
	len=(len/2)+1;
	memmove(&buf[len],titleBuf,strlen(titleBuf));
	}
	
else {
	memmove(buf,titleBuf,len);
	buf[len]='\0';
	}
//Serial.printf("titleBuf=%s\n",titleBuf);
}

uint16_t dispField(STATUS_vt* vt, uint16_t inlen,bool force, bool title){
//uint32_t a = READ_PERI_REG(REG_I2C_BASEO(0,vt->portIndex));
uint32_t a = port[vt->portIndex];
/*uint32_t b = READ_PERI_REG(REG_I2C_BASEO(0,vt->portIndex));
if(a!=b)
	Serial.printf("different: port=0x%08lx PERI=0x%08x\n",a,b);
//Serial.printf("dispField: port(%d):%p=%d\n",vt->portIndex,port+vt->portIndex,port[vt->portIndex]);
*/
if(((a&vt->mask)!=(vt->oldVal&vt->mask))||force){// changed
  vt->oldVal = a;
	if(vt->position==0) vt->position=inlen;
	if(inlen==0) Serial.print("\n+");
	if(vt->position>0){
		while(inlen<vt->position) {
			Serial.print(' ');
			inlen++;
			}
	  }
	vt->length = 0;
	char buf[100];
	if(vt->options&DISP_RAW){
		vt->length += sprintf(buf," %08lx",a);
		}
	uint32_t m=0;
	uint8_t i=0,j=32;
	uint8_t k=0;
	if(vt->options&DISP_RESERVED){
		while(i<vt->fldCnt){
			k = vt->fields[i];
			j=j-(k&0x1f);
			if(k&DISP_RESERVED){ //found reserved
			//	k=k&0x1f;
				m=((uint32_t)1<<(k&0x1F))-1;
				m=m<<j;
				m=a&m;
				m=m>>j;
				if(k&DISP_HEX){
					uint16_t fw=((k&0x1f)/4)+(k&0x3)?1:0;
					vt->length += sprintf(&buf[vt->length]," %0*lx",fw,m);
					}
				else if(k&DISP_BOOL){
					vt->length += sprintf(&buf[vt->length]," %c",m?'Y':'N');
					}
				else 
					vt->length += sprintf(&buf[vt->length]," %ld",m);
				}
			i++;
			}
		}
	j=32; // reset bit counter
	i=0; // start over
	if(vt->options&DISP_DEFINED){
		while(i<vt->fldCnt){
			k = vt->fields[i];
			j=j-(k&0x1f);
			if(!(k&DISP_RESERVED)){ //found normal
				m=((uint32_t)1<<(k&0x1f))-1; // create bit mask
				m=m<<j;
				m=a&m;
				m=m>>j;
				if(k&DISP_HEX){
					uint16_t fw=((k&0x1f)/4)+((k&0x3)?1:0);
					vt->length += sprintf(&buf[vt->length]," %0*lx",fw,m);
					}
				else if(k&DISP_BOOL){
					vt->length += sprintf(&buf[vt->length]," %c",m?'Y':'N');
					}
				else 
					vt->length += sprintf(&buf[vt->length]," %ld",m);
				}
			i++;
			}
		}
		
	if(title){// only print title
	  status_createTitle(buf,vt->length,vt->portIndex);
		}
	Serial.print(buf);
	return inlen+vt->length;
	}
return inlen;
}

void I2Cstat(){
/*if(millis()-timeOut<RepeatPeriod) return;
timeOut = millis();
*/
uint16_t len=0;
uint16_t a=0;
while(a<sc.count){
	len = dispField(&sc.vt[a],len,false,false);
  a++;
	}
}

bool i2cReady(uint8_t adr){
uint32_t timeout=millis();
bool ready=false;
while((millis()-timeout<100)&&(!ready)){
	Wire.beginTransmission(adr);
	int err=Wire.endTransmission();
	ready=(err==0);
	if(!ready){
		if(err!=2)Serial.printf("{%d}",err);
		}
	}
return ready;
}

void eepromSize(){
currentCommand=NO_COMMAND;
priorCommand=COMMAND_SIZE;
Serial.println("Discovering eeprom sizes 0x50..0x57");
uint8_t adr=0x50,i;
uint16_t size;
char buf[256];
while(adr<0x58){
	i=0;
	size = 0x1000; // Start at 4k
	i += sprintf_P(&buf[i],PSTR("0x%02X: "),adr);
	if(i2cReady(adr)) { // EEPROM answered
		uint8_t zeroByte;
		Wire.beginTransmission(adr);
		Wire.write((uint8_t)0); // set address ptr to 0, two bytes High
		Wire.write((uint8_t)0); // set address ptr to 0, two bytes Low
		uint8_t err=Wire.endTransmission();
		if(err==0){// worked
		  err=Wire.requestFrom(adr,(uint8_t)1);
			if(err==1){// got the value of the byte at address 0
				zeroByte=Wire.read();
				uint8_t saveByte,testByte;
				do{
					if(i2cReady(adr)){
						Wire.beginTransmission(adr);
						Wire.write(highByte(size)); // set next test address
						Wire.write(lowByte(size));
						Wire.endTransmission();
						err=Wire.requestFrom(adr,(uint8_t)1);
						if(err==1){
							saveByte=Wire.read();
							Wire.beginTransmission(adr);
							Wire.write(highByte(size)); // set next test address
							Wire.write(lowByte(size));
							Wire.write((uint8_t)~zeroByte); // change it
							err=Wire.endTransmission();
							if(err==0){ // changed it
								if(!i2cReady(adr)){
									i+=sprintf_P(&buf[i],PSTR(" notReady2.\n"));
									Serial.print(buf);
									adr++;
									break;
									}
								Wire.beginTransmission(adr);
								Wire.write((uint8_t)0); // address 0 byte High
								Wire.write((uint8_t)0); // address 0 byte Low
								err=Wire.endTransmission();
								if(err==0){
									err=Wire.requestFrom(adr,(uint8_t)1);
									if(err==1){ // now compare it
									  testByte=Wire.read();
										}
									else {
										testByte=~zeroByte; // error out
										}
									}
								else {
									testByte=~zeroByte;
									}
								}
							else {
								testByte = ~zeroByte;
								}
							//restore byte
							if(!i2cReady(adr)){
								i+=sprintf_P(&buf[i],PSTR(" notReady4.\n"));
								Serial.print(buf);
								adr++;
								break;
								}
							Wire.beginTransmission(adr);
							Wire.write(highByte(size)); // set next test address
							Wire.write(lowByte(size));
							Wire.write((uint8_t)saveByte); // restore it
							Wire.endTransmission();
							}
						else testByte=~zeroByte;
						}
					else testByte=~zeroByte;
					if(testByte==zeroByte){
						size = size <<1;
						}
					}while((testByte==zeroByte)&&(size>0));
				if(size==0) i += sprintf_P(&buf[i],PSTR("64k Bytes"));
				else i+=sprintf_P(&buf[i],PSTR("%dk Bytes"),size/1024);
				if(!i2cReady(adr)){
					i+=sprintf_P(&buf[i],PSTR(" notReady3.\n"));
					Serial.print(buf);
					adr++;
					continue;
					}
				Wire.beginTransmission(adr);
				Wire.write((uint8_t)0); // set address ptr to 0, two bytes High
				Wire.write((uint8_t)0); // set address ptr to 0, two bytes Low
				Wire.write(zeroByte);  //Restore
				err=Wire.endTransmission();
				}
			else i+=sprintf_P(&buf[i],PSTR("Read 0 Failure"));
			}
		else i+=sprintf_P(&buf[i],PSTR("Write Adr 0 Failure"));
			
	  }
	else i+=sprintf_P(&buf[i],PSTR("Not Present."));
	Serial.println(buf);
	adr++;
	}
}
void displayHelp(){
Serial.println();
Serial.println("Commands:");
Serial.println("  ?,help		: Help, this Screen");
Serial.println("  scan			: Run I2C Device Scan");
Serial.println("  size			: Run I2C eeprom size Probe 0x50:0x57, 16bit addressing");
Serial.println("  nr [id [blk [repeat]]]		: NewRead \n"
	"						id = (hex) I2C address\n"
	"						blk = (dec) block length in bytes\n"
	"						repeat = (dec) period in seconds to repeat, 0 = onshot.");
Serial.println("	stat			:	control of background process to display I2C register values\n			for more info 'stat ?'\n");
Serial.printf("Current Values:\n"
"		repeat = %d\n"
"		blk = %d\n"
"		id = 0x%02x\n"
"		addr= 0x%04x\n",(RepeatPeriod/1000),BlockLen,ID,addr);
Serial.printf("prior =%s, next=%s, current=%s STAT=%s\n",
	command_name(priorCommand),command_name(nextCommand),command_name(currentCommand),
	(status_on)?"ON":"OFF");
uint32_t tick=millis();	
Serial.printf("millis()=%ld, timeout=%ld delta=%ld\n",tick,timeOut,tick-timeOut);
Serial.printf("Free Heap=%d",system_get_free_heap_size());
//currentCommand=NO_COMMAND;
}

void testExplosion(){
if(millis()-timeOut<RepeatPeriod) return;
timeOut = millis();
BlockLen++;
addr--;
if(!BlockLen) BlockLen=1; // can't have zero length block!
bigBlock(false); // don't display buf
}


void setup(){
Serial.begin(115200);
Serial.print(" debug enabled \n");
Serial.setDebugOutput(true);
Wire.begin();
setI2cDev(0);
strcpy(keybuf,"stat +ctr +status_reg\nstat title\nstat");
processCommand();
}

void loop(){
if(Serial.available()) processSerial();
if(nextCommand!=NO_COMMAND){
	priorCommand=currentCommand;
// terminate current command
	if(currentCommand==COMMAND_SLAVE_MODE){ // Shutdown Slave Mode
		}
	if(currentCommand==COMMAND_NEW_READ){ // shutdown new read
		// do nothing,
		}
// any special handling to start new command

//
	if(nextCommand==COMMAND_STOP){
		currentCommand=NO_COMMAND;
		}
	else currentCommand=nextCommand;
	nextCommand = NO_COMMAND;
	}
else{
	if(status_on) I2Cstat();
	
	switch(currentCommand){
		case NO_COMMAND :
			break; // idle
		case COMMAND_SLAVE_MODE:
			pollI2cStatus();
			break; 
		case COMMAND_NEW_READ:
			NewRead();
			break;
		case COMMAND_SCAN:
		  scan();
			break;
		case COMMAND_SIZE:
			eepromSize();
			break;
		case COMMAND_HELP:
			displayHelp();
			break;
		case COMMAND_EXPLODE:
			testExplosion();
			break;
		default :;
		}
	}
}
/*
x 11/02/17 add :raw:d:r:m05fca to all Variables



*/
