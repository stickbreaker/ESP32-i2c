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
#include "driver/periph_ctrl.h"


#define DR_REG_I2C_EXT_BASE_FIXED               0x60013000
#define DR_REG_I2C1_EXT_BASE_FIXED              0x60027000

i2c_dev_t * dev=NULL;
volatile uint32_t* port=NULL;
/*forwards */
uint16_t dispField(STATUS_vt* vt, uint16_t inlen,bool force, bool title, bool* good,uint16_t line);
//bool status_createTitle(char* buf,uint16_t len, uint16_t index,uint16_t line);

uint32_t timeOut =0; // current timeout mark
uint32_t RepeatPeriod = 1000; // default 1 second
uint16_t BlockLen = 10; // default to 10 byte transfer
uint16_t ID = 0x50; // default to EEPROM
bool TenBit = false; // 7bit mode
uint16_t addr=0;
uint8_t* compBuff=NULL;
uint16_t compLen=0;
bool sendStop=true;
bool ackBit=true;

#define BUFLEN 100
char keybuf[BUFLEN+1];
uint16_t keyLen=0;
SERIAL_COMMANDS_t nextCommand=NO_COMMAND,currentCommand=NO_COMMAND,priorCommand=NO_COMMAND;
bool status_on=false; // continous I2C status reporting

/* char* command_name(SERIAL_COMMANDS_t cmd)
*		Convert from ENUM value to cString 
*/
char *command_name(SERIAL_COMMANDS_t cmd){
uint16_t a=0;
char* name=(char*)&COMMAND_NAMES;
while((a<cmd)&&(name[0])){
	name= name + strlen(name)+1;
	a++;
	}
return name;
}

/* void setI2cDev(uint8_t num)
*		initialize base pointer to I2C hardware registers
*/
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
uint16_t i=xtoi(param); // read slave addrs if present
if(param)	setSlave(i,(bits==10));
else setSlave(ID,(bits==10));

nextCommand = COMMAND_SLAVE_MODE;

return NULL;
}

char * proc_off(char** savePtr){
//char * param = strtok_r(NULL," ,",savePtr);
Serial.print("** OFF **\n");
nextCommand = COMMAND_STOP;
return NULL;
}

char * proc_newRead(char** savePtr){
char * param = strtok_r(NULL," ,",savePtr);
if(param){ // get ID
  if(strcmp(param,"?")==0){
		displayNRHelp();
		return NULL;
		}
	uint16_t t = xtoi(param);
	if(t!=0) ID=t;
	param = strtok_r(NULL," ,",savePtr);
	if(param){ // get BlockLen
		t = atoi(param);
		BlockLen = t; 
		param = strtok_r(NULL," ,",savePtr);
		if(param){ // get repeat
			t = atoi(param);
			if(t<50) t = t*1000; // seconds
			RepeatPeriod = t; // zero means oneshot
			}
		}
	}
Serial.printf("\nNewRead 0x%02x len=%d repeat=%d\n",ID,BlockLen,(RepeatPeriod/1000));
nextCommand = COMMAND_NEW_READ;
timeOut = millis()-RepeatPeriod;
return NULL;
}

void printID(){
Serial.printf("I2C Addr=0x%03x\n",ID);
}

void printRepeat(){
if((RepeatPeriod%1000)==0)
	Serial.printf("\nrepeat=%d seconds",(RepeatPeriod/1000));
else
	Serial.printf("\nrepeat=%d ms",RepeatPeriod);
}

void printBlock(){
Serial.printf("\nblockSize=%d",BlockLen);
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
for(uint8_t a=0; a<MAX_VT;a++){
	status_init_rec(&sc.vt[a]);
	}
}

void status_help(){
Serial.print("\nStatus help\n");
Serial.println("  stat [+|-|<|>]*|reg_name[:raw][:r][:d][:nraw][:nr][:nd][:m###]\n"
  "				+  = add\n"
	"				-  = remove\n"
	"				>  = Shift Right\n"
	"				<  = shift Left\n"
  "				reg_name	= I2C register name\n"
	"				:raw	= display register as 0x08lx\n"
	"				:r		= display reserved fields\n"
	"				:d		= display defined fields\n"
	"				:m##	= set mask value to (##)hex digits\n\n");
Serial.println("  stat [on|off|init|title|names|?|shrink]\n"
	"				on	= continous updates\n"
	"				off	= end updates\n"
	"				init	= delete all status selections\n"
	"				title	= Display column headings\n"
	"				names	= list valid register names\n"
	"				shrink = recover removed columns\n");
}

uint16_t status_find_name(char *nextParam){ // returns 1..n or 0 for not found
if(nextParam){
  if(strlen(nextParam)==0) return 0;
  }
else return 0;

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
//Serial.printf("byIndex-> [%d]=",index);
char* catalog=(char*)&STATUS_NAMES;
if(index>=(STATUS_NAMES_COUNT-1)){
 index = STATUS_NAMES_COUNT-1;
	}
	
while(index&&catalog[0]){
	catalog = catalog + strlen(catalog)+1;
  index--;
	}
//Serial.printf("%s|\n",catalog);
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
void sendit(uint8_t it){
  // assumes SCL is low on entry and Low on output
for(uint8_t a =0;a<8;a++){
  if(it&0x80) digitalWrite(19,HIGH);
  else digitalWrite(19,LOW);
  delayMicroseconds(10);
  digitalWrite(23,HIGH);
  delayMicroseconds(10);
  digitalWrite(23,LOW);
  it= it <<1;
  }
digitalWrite(19,HIGH); // release SDA
}

bool testAck(bool ack,uint16_t bytenum){
 // expects scl_drive Low, sda_drive high
delayMicroseconds(5);
digitalWrite(23,HIGH);
bool good = true;
if(digitalRead(21)&&ack) {
  Serial.printf(" byte(%d) Expected Ack bit ",bytenum);
  pins("");
  Serial.print(" NAK\n");
  good =false;
  }
else if(!ack && !digitalRead(21)){
  Serial.printf(" byte(%d) Unexpect Ack bit ",bytenum);
  pins("");
  Serial.print(" ack\n");
  good=false;
  }
delayMicroseconds(5);
digitalWrite(23,LOW);
return good;
}
void emptyRxFifo(){
Serial.printf("data[%02d]=",dev->status_reg.rx_fifo_cnt);
while(dev->status_reg.rx_fifo_cnt){
  uint32_t a = dev->fifo_data.val;
  Serial.printf(" 0x%02x",a);
  }
Serial.println();
dev->int_clr.rx_fifo_full = 1;
}

void toggle( bool ack, bool sendStop, char* buff){
// toggle wired 19->21, 23->22
// diode cathode (bar'd) at 19,23
dev->int_clr.val =0x1FFFF;
I2Cstat();//
Serial.printf("Toggle ack=%s STOP=%s",(ack)?"YES":"NO",(sendStop)?"YES":"NO");
if(buff){
  uint16_t a=0;
  if(buff[a]){
    Serial.printf(" data[%d]=",strlen(buff));
    while(buff[a]){
      Serial.printf(" 0x%02x",buff[a++]);
      }
    }
  }
Serial.println();
if(!(digitalRead(21) && digitalRead(22))){
  pins("\n");
  }
digitalWrite(19,HIGH);
digitalWrite(23,HIGH);
pinMode(19,OUTPUT); //assume SDA
pinMode(23,OUTPUT); //assume SCL
digitalWrite(23,HIGH); // scl High
digitalWrite(19,LOW); // drop SDA for a start
delayMicroseconds(5);
digitalWrite(23,LOW);
sendit((ID)<<1); // should be write of 0x51
// ack Pulse
testAck(ack,0);
uint16_t i=0;
while(i<strlen(buff)){
  sendit(buff[i++]);
  testAck(ack,i);
  }

if(sendStop){
  digitalWrite(19,LOW); //setup for stop bit
  }
delayMicroseconds(5);
digitalWrite(23,HIGH); // release SCL bit
delayMicroseconds(2);
if(sendStop){
  digitalWrite(19,HIGH); // Stop bit done
  }
if(!sendStop){
  Serial.println("Waiting for ReStart");
  }
Serial.println();
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

if(cmd&&(sc.count<MAX_VT)){// ok, now have to parse the STATUS_FIELDS to create MASK and fields[]
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
  if(sc.count<MAX_VT){
		if(nextParam[0]=='*'){
			uint8_t a =0;
			char ch[32];
			char *byIndex;
			while(a<(STATUS_NAMES_COUNT-1)){
				byIndex = status_name_byIndex(a++);
				if(byIndex) {
					ch[0]='+';
					strcpy(&ch[1],byIndex);
					if(strlen(nextParam)>1)
						strcat(ch,&nextParam[1]); // append any suffixes if they exist
					status_add(&ch[0]);
					}
				else a=STATUS_NAMES_COUNT;
				}
			}
		else Serial.printf("Unknown statusWord =%s\n",nextParam);
		}
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
	
void status_display_word(char *nextParam,bool title, uint16_t * lenin){
//Serial.printf("display word %s\n",nextParam);
uint16_t b = status_find_inList(nextParam);
if(b){// exists in list
	uint16_t line=0,len;
	bool success;
  do{
		success=true;
		len = *lenin;
		len=dispField(&sc.vt[b-1],len,true,title,&success,line);
		line++;
		}while(!success);
  *lenin = len;
	}
else if(strcmp(nextParam,"*")==0){
  // rethink this so that is displays on one line?
  uint16_t line=0,len;
	bool success;
  do{
    b=0;
		success=true;
    len = 0;
    while(b<sc.count){
      len=dispField(&sc.vt[b],len,true,title,&success,line);
      b++;
      }
		line++;
		}while(!success);
  }
}

void status_display_list(bool title){
uint16_t len,a,line=0;
bool success;
do{
	len=0;
	a=0;
	success=true;
	while(a<sc.count){
		len=dispField(&sc.vt[a],len,true,title,&success,line);
		a++;
		}
	line++;
	}while(!success);
//Serial.println();
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

bool nextWildCard(char *nextParam, uint16_t *wildOne, char *wildName){
// will support trailing wildcards only, no mixed or leading.
// +scl* -scl* scl*:raw sda*
uint16_t wildPos=strcspn(nextParam,"*");
bool wildCard=(wildPos != strlen(nextParam));
bool found =false;
if(wildCard){ // start looking
  uint16_t startPos = strcspn(nextParam,"-+><?");
  if(startPos==strlen(nextParam)) startPos=0; // not found so start at first char
  else startPos=1; // skip over modifier
  uint16_t len =wildPos;
  if(len>startPos) len = len -startPos;
  else len=0;
  if(len>0) memmove(wildName,&nextParam[startPos],len);
  wildName[len]='\0';
  // now should have leading characters of name.
  // now search through catalog for matching, using *wildOne as starting
  char* catalog=(char*)&STATUS_NAMES;
  uint16_t cnt=0;
  while((cnt < *wildOne)&&(catalog[0])){ // index to next possible match
    catalog = catalog + strlen(catalog)+1;
    cnt++;
    }
 
  while((!found)&&(catalog[0])){
    found = strncmp(catalog,wildName,strlen(wildName))==0;
    if(!found){
      catalog = catalog + strlen(catalog)+1;
      cnt++;
      }
    }
  if(found){
    *wildOne = cnt+1;
    memmove(wildName,nextParam,startPos);
    memmove(&wildName[startPos],catalog,strlen(catalog)+1); //include null 
    if((len+startPos+1) < strlen(nextParam)){ // add suffix
      memmove(&wildName[strlen(wildName)],&nextParam[len+startPos+1],(strlen(nextParam)-(len+startPos+1))+1);// include Null
      }
    }
  }    
else {// no wild card just copy from nextParam to wildName
  found = false;
  *wildOne = 0;
  uint16_t len=strlen(nextParam);
  if(len>31) len=31; // wildName is a 32 char buffer
  memmove(wildName,nextParam,len);
  wildName[len]='\0';
  }
return found;
}

void status_process_list(char *nextParam, char** savePtr){
bool titleChanged=false;
uint16_t dispLen = 0;
while(nextParam){
  uint16_t wildOne = 0;
  char wildName[32];
  bool wildCard = nextWildCard(nextParam,&wildOne,(char*)&wildName); 
  do{
  //Serial.printf("status Process List=%s\n",nextParam);
    if(wildName[0]=='+'){ // add next status word to list
      status_add(wildName);
      titleChanged = true;
      }
    else if(wildName[0]=='-'){ // remove next status word from list
      status_remove(wildName);
      titleChanged = true;
      }
    else if(wildName[0]=='?'){// display mask value
      status_mask_disp(wildName,0);
      }
    else if(wildName[0]=='>'){// shift this word right
      status_shift_right(wildName);
      titleChanged = true;
      }
    else if(wildName[0]=='<'){// shift this word left
      status_shift_left(wildName);
      titleChanged = true;
      }
    else if(strcmp(wildName,"shrink")==0){ // recover deleted columns
      for(uint8_t a=0; a<MAX_VT; a++){
        sc.vt[a].position=0;
        }
      titleChanged = true;
      }
    else if(strcmp(wildName,"init")==0){ // Stop display, delete all positions
      status_init();
      }
    else { // display info for this word
      char* testParam=strchr(wildName,':');
      if(testParam) status_update(wildName,0);
      else status_display_word(wildName,false,&dispLen);
      }	
    }while(wildCard=nextWildCard(nextParam,&wildOne,(char*)&wildName));
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
    if(asciibuf[0]) Serial.printf(" %s\n",asciibuf);
		Serial.printf("0x%04x:",(uint16_t)(offset+adr));
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

bool compBuffer(uint8_t *buf, uint16_t len, uint16_t offset){
bool good = true;
if(!compBuff) return true;
uint16_t a=0,b=offset%compLen;
while(good&&(a<len)){
	good=(buf[a++]==compBuff[b++]);
	b=b%compLen;
	}
if(!good){
Serial.printf("comp failed at 0x%04x %d!=%d\n",a-1,buf[a-1],compBuff[(b-1)%compLen]);
	}
return good;	
}

void fillCompBuffer(uint16_t len){
if(compBuff){
	free(compBuff);
	}
compBuff =(uint8_t*)calloc(len,sizeof(uint8_t));
if(!compBuff){
	Serial.printf("compBuff calloc Fail\n");
	return;
	}
compLen = len;
Wire.beginTransmission(ID);
Wire.write((uint8_t)0);
Wire.write((uint8_t)0);
uint8_t err=Wire.endTransmission();
if(err!=0){
	Serial.printf("FillBuff set Address failed=%d\n",err);
	free(compBuff);
	compBuff=NULL;
	compLen=0;
	return;
	}
uint16_t a=0;
uint8_t size;
uint32_t timeout=millis();
while((a<len)&&((millis()-timeout)<5000)){
	size=(len-a)>16?16:(len-a);
	err=Wire.requestFrom((uint8_t)ID,size);
	if(err==0){
		Serial.printf("request from failed @0x%04x = %d",a,Wire.lastError());
		}
	else{
		timeout=millis();
		while(Wire.available()){
			compBuff[a++]=Wire.read();
			}
		}
	}
compLen = a;
}

void bigBlock(bool display){
uint32_t start=0;
Serial.printf("starting at: %ld ",start=millis());
Wire.beginTransmission(ID);
if((ID>=0x50)&&(ID<=0x50)) Wire.write(highByte(addr));
Wire.write(lowByte(addr));
uint16_t err=Wire.endTransmission();
if(err){
	Serial.printf("device(0x%02x) setting address failed =%d",ID,err);
	nextCommand=NO_COMMAND;
	return;
	}
uint8_t* bigBlock=(uint8_t*)calloc(BlockLen,sizeof(uint8_t));
uint16_t cnt=Wire.polledRequestFrom(ID,bigBlock,BlockLen,true);
Serial.printf(" completed at: %ld, %ld cnt=%d\n",millis(),millis()-start,cnt);
if(cnt!=BlockLen){
	Serial.printf("ERROR newRequestFrom \n");
	nextCommand=COMMAND_STOP;
	}
if(display||(cnt==0)) dispBuff(bigBlock,BlockLen,addr);
if(!compBuffer(bigBlock,BlockLen,addr)){
	nextCommand=COMMAND_STOP;
	}	
free(bigBlock);
}

void setSpeed(uint32_t speed){
Wire.setClock(speed);
}

void cmd(){// dump I2C command register
uint8_t i=0;
I2C_COMMAND_t cmd;
char op[][8]={
	{"RSTART"},
	{"WRITE "},
	{"READ  "},
	{"STOP  "},
	{"END   "}};
Serial.print("\nI2C command List\n");
while(i<16){
	cmd.val=dev->command[i].val;
	
	Serial.printf("command[%02d]: %c %s %d %d %d %d\n",i,cmd.done?'Y':'N',
		op[cmd.op_code],cmd.ack_val,cmd.ack_exp,cmd.ack_en,cmd.byte_num);
	i++;
	}
}
/* 11/07/2017
 tested with induced bit timeout(ground SCL) Produced 0x100 Int
	No effect, Statemachine Hung 4,3
cmd 'force [state]'
*/	
void forceState( uint8_t state){
I2C_STATUS_REG_t stat;
stat.val = dev->status_reg.val;
Serial.printf("\nold Values Main=%d, State=%d\n",stat.scl_main_state_last,
stat.scl_state_last);
stat.scl_main_state_last = state&0x7;
stat.scl_state_last = state&0x7;
stat.reserved27 = (state&0x8)?1:0;
stat.reserved31 = (state&0x8)?1:0;
dev->status_reg.val = stat.val;
stat.val = dev->status_reg.val;
Serial.printf("new Values Main=%d, State=%d\n",stat.scl_main_state_last,
stat.scl_state_last);
}


static void i2c_hw_enable(){

periph_module_enable(PERIPH_I2C0_MODULE);
}

static void i2c_hw_disable(){

periph_module_disable(PERIPH_I2C0_MODULE);
}

static void i2c_restoreState(uint32_t *saveBuff){
port[0] = saveBuff[0];
port[1] = saveBuff[1]; // trans_start
port[3] = saveBuff[2] ;
port[4] = saveBuff[3];
port[6] = saveBuff[4];
port[10] = saveBuff[5];
port[12] = saveBuff[6];
port[13] = saveBuff[7];
port[14] = saveBuff[8];
port[16] = saveBuff[9];
port[17] = saveBuff[10];
port[18] = saveBuff[11];
port[19] = saveBuff[12];
port[20] = saveBuff[13];
port[21] = saveBuff[14];

}	

static void i2c_saveState(uint32_t *saveBuff){
saveBuff[0] = port[0];
saveBuff[1] = port[1] &~(BIT(5)); // trans_start
saveBuff[2] = port[3];
saveBuff[3] = port[4];
saveBuff[4] = port[6];
saveBuff[5] = port[10];
saveBuff[6] = port[12];
saveBuff[7] = port[13];
saveBuff[8] = port[14];
saveBuff[9] = port[16];
saveBuff[10] = port[17];
saveBuff[11] = port[18];
saveBuff[12] = port[19];
saveBuff[13] = port[20];
saveBuff[14] = port[21];
}



/* 11/07/2017
recover

cmd 'recover'
*/

void recover(){ // did not work
dev->ctr.trans_start=0;
uint8_t i = 0;
/*while(i<16){
	forceState(i++);
	}	
*/

I2C_COMMAND_t cmd;
I2C_STATUS_REG_t st;
I2C_INTERRUPT_t ic;
st.val =dev->status_reg.val;
ic.val =dev->int_raw.val;
// set cmdlist to known state
i =0;
cmd.val = 0;
cmd.done = 1;
for(i=0;i<16;i++){
	dev->command[i].val = cmd.val; // set all commands to done and start
	}

if(((st.scl_main_state_last==3)||(st.scl_main_state_last==5))
	&&(st.scl_state_last==4)
	&&(ic.time_out==1)
	&&(st.bus_busy==1)){
	Serial.print("\n 4 [5,3] time out, busy repair");
	uint32_t *saveBuff=(uint32_t*)calloc(16,sizeof(uint32_t));
	i2c_saveState(saveBuff);
	i2c_hw_disable();
	i2c_hw_enable();
	i2c_restoreState(saveBuff);
	free(saveBuff);
	ic.time_out =1;
	ic.master_tran_comp = 1;
	dev->int_clr.val = ic.val; // try to clear timeout
/*  cmd.op_code = 3; // stop
	cmd.done = 0; 
	dev->command[0].val = cmd.val;
	dev->ctr.trans_start=1;
*/
	}
else {
	Serial.print("\n repair");
	cmd.done = 0; // not done
	dev->command[0].val = cmd.val;
	cmd.op_code = 1; // Write
	cmd.byte_num = 1; // one byte
	dev->command[1].val = cmd.val;
	cmd.byte_num = 0; // one byte
	cmd.op_code = 3; // stop
	dev->command[2].val = cmd.val;
	// clear fifos to known stat
	I2C_FIFO_CONF_t fifo_conf;
	fifo_conf.val = dev->fifo_conf.val;
	fifo_conf.rx_fifo_rst =1;
	fifo_conf.tx_fifo_rst = 1;
	dev->fifo_conf.val= fifo_conf.val;
	fifo_conf.rx_fifo_rst = 0;
	fifo_conf.tx_fifo_rst = 0;
	dev->fifo_conf.val= fifo_conf.val;
	
	dev->fifo_data.val = 0; // send null
	dev->ctr.trans_start=1; // do it!
	}
}

void tran( bool title){// 11/13/17 test for transaction (sendStop=false);
Wire.beginTransmission(ID);
if((ID>=0x50)&&(ID<=0x57))Wire.write(highByte(addr)); // 16bit address
Wire.write(lowByte(addr));
//Wire.newEndTransmission(); // worked Interrupt driven Transmit 
uint16_t index=0;
uint8_t *buf=(uint8_t*)calloc(BlockLen,sizeof(uint8_t));

if(BlockLen <= 128) { // use internal Wire.h buffer
  if(title)Serial.print("Transact using Wire()'s Buffer\n");
  uint8_t err =Wire.transact(BlockLen);
  if(err!=BlockLen){
    Serial.printf("trans=%d\n",err);
    Serial.printf("last Error =%d\n",Wire.lastError());
    }
  while(Wire.available()){
    if(index<BlockLen)
      buf[index]=Wire.read();
    index++;
    }
  }
else { // use local buffer, no Wire.read()
  if(title) Serial.print("Transact using local Buffer\n");
  uint16_t err = Wire.transact(buf,BlockLen);
  if(err != BlockLen){
    Serial.printf("trans=%d\n",err);
    Serial.printf("last Error =%d\n",Wire.lastError());
    }
  }
dispBuff(buf,BlockLen,addr);
free(buf);
}

void unknownCmdParam(const char* cmd,char* nextParam){
	Serial.printf("%s: unknown Parameter= %s\n",cmd,nextParam);
}

void pins(const char* term){
Serial.printf("scl=%d, sda=%d%s",digitalRead(22),digitalRead(21),term);
}

bool parseAsQuotedEscapedString(char ** nextParam,char **savePtr,char *buff,uint16_t maxBuff){
  // parse the string nextParam for a quoted string "string\x3f\x3f\x0a\x0d"
  // -> string\\<cr><lf>
//Serial.printf("*nextParam(%p)=%p, [0]=%c\n",nextParam,*nextParam,*nextParam[0]);
uint16_t foundAt = strcspn(*nextParam,"\"");
//Serial.printf("afterFound =%d\n",foundAt);
if((foundAt==0)&&(strlen(*nextParam)>0)){
  char *local=*nextParam; // without this the compiler is confused of &(*nextParam[offset])
  uint16_t endsAt = strcspn(&(local[foundAt+1]),"\"");
  uint16_t len = (endsAt>(maxBuff-1))?maxBuff-1:endsAt;
  memmove(buff,&(local[foundAt+1]),len); // grab quoted string,
  *nextParam=strtok_r(&(local[endsAt+foundAt+1])," ,",savePtr); //skip quoted
  if(*nextParam){
    if(*nextParam[0] == '"') (*nextParam)++; 
    if(strlen(*nextParam)==0) *nextParam=NULL; // ack like strtok_r found the end.
//    Serial.printf("*nextParam(%p)=%c\n",*nextParam,*nextParam[0]);
    }
  else {
//    Serial.printf("end of line\n");
    }
  buff[len]='\0';
  uint16_t outpos=0,inpos=0;  
  char c,c1;
  bool copy;
  while(inpos<len){
    copy = true;
    if(buff[inpos]=='\\') {
      if((inpos+1)<len){
        inpos++;
        if(buff[inpos]=='x'){
          if((inpos+1)<len){//first char
            inpos++;
            c=buff[inpos];
            if((c>='0')&&(c<='9')) c = c -48;
            else if((c>='A')&&(c<='F')) c = c -55;
            else if((c>='a')&&(c<='f')) c = c - 87;
            else {
              memmove(&buff[outpos],&buff[inpos-2],3);
              inpos++;
              outpos += 3; // was not valid escape
              continue;
              }
            if((inpos+1)<len){ // second char
              inpos++;
              c1=buff[inpos];
              if((c1>='0')&&(c1<='9')) c1 = c1 -48;
              else if((c1>='A')&&(c1<='F')) c1 = c1 -55;
              else if((c1>='a')&&(c1<='f')) c1 = c1 - 87;
              else {
                memmove(&buff[outpos],&buff[inpos-3],4);
                inpos++;
                outpos += 4; // no valid escape
                continue;
                }
              buff[outpos]=(c<<4)+c1;
              copy=false;
              }
            else {
              memmove(&buff[outpos],&buff[inpos-2],3);
              inpos++;
              outpos += 3;
              continue;
              }
            }
          else {
            memmove(&buff[outpos],&buff[inpos-1],2);
            inpos++;
            outpos += 2;
            continue;
            }
          }
        else {
          memmove(&buff[outpos],&buff[inpos-1],1);
          outpos += 1;
          continue;
          }
        }
      }
    if(copy) buff[outpos]=buff[inpos];
    inpos++;
    outpos++;
    }
  buff[outpos]='\0';   
  return true; // moved quoted string into buff, updated nextParam
  }
else {
  if(maxBuff>0) buff[0]=='\0';
  return false; // buff is empty nextParam still contains the current value
  }
}  

void processCommand(){

char *nextParam,*saveptr,*line,*saveptr1;

uint16_t l=0;
uint32_t ll=0;
char buff[100];
line = strtok_r(keybuf,"\n",&saveptr);//break input into lines, to process one at a time.
if(line==NULL){
	if(currentCommand==NO_COMMAND) nextCommand=priorCommand;
	else nextCommand=currentCommand; // repeat last command on enter
	}
else{
	nextParam =strtok_r(line," ,",&saveptr1); // get the first word(command) 
	}

while(line){ // have next command line, parse it
  if (strcmp(nextParam,"test")==0){
    nextParam = strtok_r(NULL,"",&saveptr1); // grab rest of line
    if(nextParam){
      Serial.printf("test =%s|",nextParam);
      Serial.printf(" %s is a quoted string=%s, parsedAs=%s|",nextParam,(parseAsQuotedEscapedString(&nextParam,&saveptr1,buff,100))?"YES":"NO",buff);
      }
    }
  else if(strcmp(nextParam,"fifo")==0){
    emptyRxFifo();
    }
  else if(strcmp(nextParam,"clr")==0){// clear pending interrupts
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam) {
			if(strcmp(nextParam,"?")!=0){
        if(strstr(nextParam,"0x")==nextParam){// hex
          ID=strtol(&nextParam[2],NULL,16);
          }
        else ID=atoi(nextParam);
				}
			}
    else status_process_list("int_raw",&saveptr1); // 
    }
  else if(strcmp(nextParam,"toggle")==0){
    uint8_t cnt=0;
    buff[0] ='\0';
    char testbuff[100];
    nextParam = strtok_r(NULL,"",&saveptr1);//grab rest of line
    while(nextParam){
      bool val=false;
      if(parseAsQuotedEscapedString(&nextParam,&saveptr1,testbuff,100)){ // already in buff
        strcpy(buff,testbuff);
        }
      else nextParam = strtok_r(nextParam," ,",&saveptr1);
      if(nextParam){
        if(strcmp(nextParam,"1")==0) val=true;
        if(strcmp(nextParam,"yes")==0) val=true;
        if(strcmp(nextParam,"t")==0) val = true;
        if((cnt%2)==0) {
          ackBit=val;
          }
        if((cnt%2)==1){
          sendStop=val;
          }
          
        cnt++;
        nextParam = strtok_r(NULL,"",&saveptr1); // whole string 
        }
      }
    toggle(ackBit,sendStop,buff);
    }
  else if(strcmp(nextParam,"ints")==0){ // Dump Interrupt Capture Buffer
    Wire.dumpInts();
    }
  else if(strcmp(nextParam,"dev")==0){// Try I2C Device Id
    i2cDeviceId();
    }
  else if(strcmp(nextParam,"tran")==0){ // test for transaction queued data
		nextParam = strtok_r(NULL," ,",&saveptr1);
    if(nextParam){
      uint16_t a = atoi(nextParam);
      tran(true);
      while(a>0){
        tran(false);
        a--;
        }
      }
    else tran(true);
    }
  else if(strcmp(nextParam,"big")==0){ // run bigBlock with current addr,ID,BlockLen
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam){
			if(strcmp(nextParam,"?")==0) displayBigHelp();
			else unknownCmdParam("big",nextParam);
			}
		else bigBlock(true); // disp buff on Serial
		}
	else if(strcmp(nextParam,"explode")==0){ // run bigBlock continuous while inc size, dec addr
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam){
			if(strcmp(nextParam,"?")==0) displayExplodeHelp();
			else unknownCmdParam("explode",nextParam);
			}
		else
			nextCommand=COMMAND_EXPLODE;
		}
	else if(strcmp(nextParam,"cmd")==0){ // display I2C command list
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam){
			if(strcmp(nextParam,"?")==0) displayCmdHelp();
			else unknownCmdParam("CMD",nextParam);
			}
		else cmd();
		}
	else if(strcmp(nextParam,"recover")==0){
		recover();
		}
	else if(strcmp(nextParam,"speed")==0){ // set the Wire clock to !DANGER !DANGER no rails
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam) {
			ll=atol(nextParam);
			setSpeed(ll);
			}
    else {// display current Speed
      Serial.printf("Current i2c Bus Speed = %d\n",Wire.getClock());
      }
		}
	else if(strcmp(nextParam,"slave")==0){// "slave, id(7bit hex)=0x83"
		// configure for 7bit slave access
		nextParam = proc_slave(7,&saveptr1);
		}
	else if(strcmp(nextParam,"slave10")==0){// "slave, id(10bit hex)=0x113"
	  // configure for 10bit slave access
		nextParam = proc_slave(10,&saveptr1);
		}
	else if(strcmp(nextParam,"off")==0){ // shutdown repetive commands 
		nextParam = proc_off(&saveptr1);
		}
	else if(strcmp(nextParam,"nr")==0){ // polled block read "nr id(hex)=0x50 len(dec)=10 repeat(dec seconds)=5"
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
	else if(strcmp(nextParam,"?")==0){ // Show cmd help
	  displayHelp();
		}
	else if(strcmp(nextParam,"help")==0){ // run I2C EEPROM size probe
		nextCommand = COMMAND_HELP;
		}
	else if(strcmp(nextParam,"repeat")==0){ // change repetive command delay in seconds
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam){
			if(strcmp(nextParam,"?")!=0){
				l=atoi(nextParam);
				if(l<50){ // treat input as seconds
					l=l*1000;
					}
				RepeatPeriod = l;
				}
			}
		dispVars();
		}
	else if(strcmp(nextParam,"block")==0){ // data block size
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam) {
			if(strcmp(nextParam,"?")!=0){
				BlockLen=atoi(nextParam);
				}
			}
		dispVars();
		}
	else if(strcmp(nextParam,"id")==0){ // I2C device address
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam) {
			if(strcmp(nextParam,"?")!=0){
        if(strstr(nextParam,"0x")==nextParam){// hex
          ID=strtol(&nextParam[2],NULL,16);
          }
        else ID=atoi(nextParam);
				}
			}
		dispVars();
		}
	else if(strcmp(nextParam,"addr")==0){ // set next read address
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam){
			if(strcmp(nextParam,"?")!=0){
        if(strstr(nextParam,"0x")==nextParam){// hex
          addr=strtol(&nextParam[2],NULL,16);
          }
        else addr=atoi(nextParam);
				}
			}
		dispVars();
		}
	else if(strcmp(nextParam,"stat")==0){ // subCommand for I2C register display
		nextParam = strtok_r(NULL," ,",&saveptr1);
		if(nextParam){
			if(strcmp(nextParam,"on")==0) { // enable continous Status update output
			  status_on = true;
				}
			else if(strcmp(nextParam,"off")==0) { // Stop status Display, retain prior values
				status_on = false;
				}
			else if((strcmp(nextParam,"title")==0)|| // show column headings
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
//	Serial.print("before call");
	Wire.beginTransmission(ID);
  if((ID >= 0x50)&&(ID <= 0x57)) Wire.write(highByte(addr));
	Wire.write(lowByte(addr));
	if((err=Wire.endTransmission(false))!=7)	{
		Serial.printf(" EndTransmission=%d",err);
    if(err!=2) {
      Serial.printf(", resetting\n");
      Wire.reset();
      return;
      }
    else {
      Serial.printf(", No Device presend, aborting\n");
      currentCommand= NO_COMMAND;
      return;
      }
		}
	err = Wire.requestFrom(static_cast<uint8_t>(ID),static_cast<size_t>(BlockLen),static_cast<bool>(true));
	Serial.printf("@0x%02x(0x%04x)=%d ->",ID,addr,err);
	addr += BlockLen;
  if(!((ID >= 0x50)&&(ID <= 0x57))) addr %= 256; // byte addressed devices
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
	
bool status_createTitle(char* buf,uint16_t len, uint16_t index,uint16_t line){
	// buf must be len+1 characters Long!
	
buf[len]= '\0';
if(len==0)return true; // nowhere to print it
	
char titleBuf[100];

status_createName(titleBuf,100,index);
bool good = true;
//Serial.printf(" (%d)|%s|,|%s|\n",index,titleBuf,buf);
if(len>strlen(titleBuf)){
	memset(buf,' ',len);
	buf[1]= '|';
	buf[len-1]= '|';
//	buf[len] = '\0';
	if(line==0){
		memset(&buf[2],'-',len-3);
		len-=strlen(titleBuf);
		len=(len/2)+1;
		memmove(&buf[len],titleBuf,strlen(titleBuf));
		}
	}
else {
//	Serial.printf("Title too Big line[%d] (%d)>(%d)%s\n",line,len,strlen(titleBuf),titleBuf);
	if(len>1){// need some space to actually display the title
		memset(buf,' ',len);
		buf[len-1]= '|';
//		buf[len] = '\0';
		uint16_t outlen=0;
		bool printIt =((line*(len-1))<strlen(titleBuf));
		good = !printIt;
		if(printIt) {// partial title needs to be output
			outlen =(strlen(titleBuf)-(line*(len-1)));
			if(outlen>=len) { // all won't fit 
				outlen=len-1;
				}
			else { // all will fit on this line, so, actually good should be true!
				good=true;
				}
			memmove(buf,&titleBuf[line*(len-1)],outlen);
			buf[len]='\0';
			}
//		Serial.printf("buf=%s, good=%d, outlen=%d\n",buf,good,outlen);
		}
//	Serial.printf("titleBuf=%s\n",titleBuf);
	}
return good;// good means title completed 
}

uint16_t dispField(STATUS_vt* vt, uint16_t inlen,bool force, bool title,bool *success, uint16_t line){
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
	buf[0]='\0';  // incase nothing is added
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
//		Serial.printf(" success=%d, length=%d, index=%d, line=%d\n",
//		*success,vt->length,vt->portIndex,line);
    bool thisSuccess =status_createTitle(buf,vt->length,vt->portIndex,line);
		*success = *success && thisSuccess;
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
uint16_t len,a,line=0;
bool success;
do{
	len=0;
	a=0;
	success=true;
	while(a<sc.count){
		len = dispField(&sc.vt[a],len,false,false,&success,line);
		a++;
		}
	line++;
	}while(!success);
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

void i2cDeviceId(){ //per Philips UN10204
uint8_t a=8;
Wire.beginTransmission(0x7c); // device Id request
if(Wire.endTransmission()==0){ // Some Device on the buss has Id Capabilities
  while (a<0x78){
    Wire.beginTransmission(0x7C);
    Wire.write((uint8_t)a<1);
    uint8_t err=Wire.transact(3);
    if(err!=3){
      if(!((err==0)&&(Wire.lastError()==3)))
        Serial.printf("\nDevice ID(0x%02x)= ERROR transact=%d, lastError=%d",a,err,Wire.lastError());
      }
    if(Wire.available()){
      Serial.printf("\nDevice ID(0x%02x)= ",a);
      while(Wire.available()){
        Serial.printf("0x%02x ",Wire.read());
        }
      }
    a++;
    }
  Serial.println();
  }
else {
  Serial.println("\nNo Devices Answered Device_ID Request.\n");
  }
}
void dispVars(){
Serial.printf("\n	ID=0x%02x, BlockLen=%d, Addr=0x%04x, Delay=%d %s\n",ID,BlockLen,addr,((RepeatPeriod%1000)==0)?(RepeatPeriod /1000):RepeatPeriod,((RepeatPeriod%1000)==0)?"sec":"ms");
}
void displayNRHelp(){
Serial.println("\n  nr [id [blk [repeat]]]		: NewRead \n"
	"	id = (hex) I2C address\n"
	"	blk = (dec) block length in bytes\n"
	"	repeat = (dec) period in seconds to repeat, 0 = one shot.");
Serial.print("Current Values:\n");
dispVars();
}

void displayExplodeHelp(){
Serial.println("\n  explode	: execute 'big' continuously while BlockLen++ and addr--, minimal display");
dispVars();
}

void displayBigHelp(){
Serial.println("\n  big	: execute newRequestFrom() using provided buffer, disp content");
dispVars();
}

void displayCmdHelp(){
Serial.println("	cmd				: Display I2C command list in a formatted array");
}	

void displayHelp(){
Serial.println();
Serial.println("Commands:");
Serial.println("  ?,help  : Help, this Screen");
Serial.println("  off     : Shutdown continuous display");
Serial.println("  scan    : Run I2C Device Scan");
Serial.println("  size    : Run I2C eeprom size Probe 0x50:0x57, 16bit addressing");
Serial.println("  nr      : (polled) newRequestFrom() with Internal Wire Buffers");
Serial.println("  stat    : control of background process to display I2C register values, help 'stat ?'");
Serial.println("  big     : (polled) newRequestFrom() using provided buffer, disp content");
Serial.println("  explode : (polled) execute 'big' continuously while BlockLen++ and addr--, minimal display");
Serial.println("  dev     : (intr) Request Manufactures Device ID for all Devices on I2C bus");
Serial.println("  tran    : (intr) issue beginTransmision(id),write(addr),transact(BlockLen)");
Serial.println("  ints    : (intr) Display Interrupt Capture Buffer from esp32-hal-i2c.c");
Serial.println("  speed   : Set i2c bus speed in Hz, default=100000");
Serial.println("  cmd     : display current I2C Command[] buffer"); 
Serial.println("  pins    : DigitalRead of SCL and SDA");
Serial.println("  toggle ack stop \"string\" : manual Stimulate SCL,SDA via Diodes from 19,23\n"
  "     \"string\" : will be send out as binary data, so, chr1 is High Addr chr2 is lowByte\n"
  "         to facilitate address bytes standard \\xnn escapes are processed");

Serial.println("  clr 0xnn : clear Interrupts int_clr.val=nn");
Serial.println("  fifo    : empty rx_fifo (status_reg.rx_fifo_cnt !=0, fifo_data.data)");
Serial.println("  block   : set BlockLen(dec)\n"
  "  id      : set I2C device address(hex)\n"
  "  repeat  : set repeat(dec) 1..49(sec) 50>(milliseconds)\n"
  "  addr    : set address pointer for read");
Serial.println("\nCurrent values:");
Serial.printf("COMMAND: prior =%s, next=%s, current=%s STAT=%s\n",
	command_name(priorCommand),command_name(nextCommand),command_name(currentCommand),
	(status_on)?"ON":"OFF");
uint32_t tick=millis();	
Serial.printf("millis()=%ld, timeout=%ld delta=%ld\n",tick,timeOut,tick-timeOut);
Serial.printf("Free Heap=%d\n",system_get_free_heap_size());
dispVars();
Serial.printf("sendStop=%s expectAct=%s\n",(sendStop)?"YES":"NO",(ackBit)?"YES":"NO");
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
//dev->ctr.ms_mode = 1; // set as Master
//strcpy(keybuf,"stat init +* -int_clr -scl_low_period -timeout -sda_hold -sda_sample -scl_high_period -scl_start_hold -scl_rstart_setup -scl_stop_hold -scl_stop_setup\nstat");
processCommand();
}

void loop(){
if(Serial.available()) processSerial();
if(nextCommand!=NO_COMMAND){
	priorCommand=currentCommand;
// terminate current command
	if(currentCommand==COMMAND_EXPLODE){ // free comp buffer
		if(compBuff) { 
			free(compBuff);
			compBuff = NULL;
			compLen = 0;
			}
		}
	if(currentCommand==COMMAND_NEW_READ){ // shutdown new read
		// do nothing,
		}
// any special handling to start new command

//
	if(nextCommand==COMMAND_STOP){
		currentCommand=NO_COMMAND;
		}
	else if(nextCommand==COMMAND_EXPLODE){
		fillCompBuffer(4096);
		currentCommand=COMMAND_EXPLODE;
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


