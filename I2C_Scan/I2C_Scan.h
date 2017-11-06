#ifndef I2C_Scan_h
#define I2C_Scan_h
typedef enum{
	NO_COMMAND=0,
	COMMAND_SLAVE_MODE,
	COMMAND_NEW_READ,
	COMMAND_SCAN,
	COMMAND_SIZE,
	COMMAND_HELP,
	COMMAND_STOP,
	COMMAND_EXPLODE
	} SERIAL_COMMANDS_t;

const char COMMAND_NAMES[]={
	"NO_COMMAND\0"
	"SLAVE_MODE\0"
	"NEW_READ\0"
	"SCAN\0"
	"SIZE\0"
	"HELP\0"
	"STOP\0"
	"EXPLODE\0"
	"\0"};
	
typedef struct {
	uint32_t oldVal;
	uint32_t mask;
	uint16_t position;  // horizontal position on Serial
	uint16_t length;		// space taken up on serial
	uint16_t options;		// display options
	uint16_t portIndex; // offset from port base (uint32) 
	uint16_t fldCnt;    // how may sub fields
	uint8_t* fields;     // sub field desc, neg means reserved
	}	STATUS_vt;

typedef struct {
	uint16_t count;
	STATUS_vt vt[10];
} STATUS_CONTROL_t;
	
STATUS_CONTROL_t sc;	

#define DISP_RAW		(BIT(9))
#define DISP_DEFINED	(BIT(8))
#define DISP_RESERVED	(BIT(7))
#define DISP_HEX	(BIT(6))
#define	DISP_BOOL	(BIT(5))
	
/* forwards unused!

uint16_t I2C_STATUS_REG_disp(STATUS_vt *vt,uint16_t inlen,bool force,bool title);
uint16_t I2C_CTR_disp(STATUS_vt *vt,uint16_t inlen, bool force, bool title);
uint16_t I2C_SCL_LOW_PERIOD_disp(STATUS_vt *vt,uint16_t inlen, bool force, bool title);
*/

const char STATUS_FIELDS[]={ // negative markes reserved, 'H' says disp in HEX
	"-18,14\0"				// 'B' says Y or N
	"-23,1,1,1,1,1,-1,1,1,1\0"
	"-1,3,-1,3,6H,-4,6H,-1,1,1,1,1,1,1,1\0"
	"-12,20\0"
	"1,-15,16H\0"
	"-12,5H,5H,5H,5H\0"
	"-6,6,6,1,1,1,1,5,5\0"
	"-24,8\0"
	"-19,1,1,1,1,1,1,1,1,1,1,1,1,1\0"
	"-19,1,1,1,1,1,1,1,1,1,1,1,1,1\0"
	"-19,1,1,1,1,1,1,1,1,1,1,1,1,1\0"
	"-19,1,1,1,1,1,1,1,1,1,1,1,1,1\0"
	"-22,10\0"
	"-22,10\0"
	"-18,14\0"
	"-16,-16\0"
	"-22,10\0"
	"-22,10\0"
	"-18,14\0"
	"-22,10\0"
	"-28,1,3\0"
	"-28,1,3\0"
	"1,-17,3,1,1,1,8H\0"
	"\0"};

	const char STATUS_NAMES[]={
		"scl_low_period\0"
		"ctr\0"
		"status_reg\0"
		"timeout\0"
		"slave_addr\0"
		"fifo_st\0"
		"fifo_conf\0"
		"fifo_data\0"
		"int_raw\0"
		"int_clr\0"
		"int_ena\0"
		"int_status\0"
		"sda_hold\0"
		"sda_sample\0"
		"scl_high_period\0"
		"reserved_3c\0"
		"scl_start_hold\0"
		"scl_rstart_setup\0"
		"scl_stop_hold\0"
		"scl_stop_setup\0"
		"scl_filter_cfg\0"
		"sda_filter_cfg\0"
		"command[%02d]\0"
		"\0"};
		
#define STATUS_NAMES_COUNT 23
#define STATUS_NAMES_COUNT_MAX 38

typedef volatile uint32_t PORT_t;
	
#endif