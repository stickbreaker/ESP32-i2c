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

#define MAX_VT 32
typedef struct {
	uint16_t count;
	STATUS_vt vt[MAX_VT];
} STATUS_CONTROL_t;
	
STATUS_CONTROL_t sc;	

#define DISP_RAW		(BIT(9))
#define DISP_DEFINED	(BIT(8))
#define DISP_RESERVED	(BIT(7))
#define DISP_HEX	(BIT(6))
#define	DISP_BOOL	(BIT(5))
	
const char STATUS_FIELDS[]={ // negative marks reserved, 'H' says disp in HEX
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

#define DR_REG_I2C_EXT_BASE_FIXED               0x60013000
#define DR_REG_I2C1_EXT_BASE_FIXED              0x60027000
#define REG_I2C_BASEO(i,o)		(DR_REG_I2C_EXT_BASE +(i) *0x14000+(o)*4)

// from i2c_struct.h

typedef union {
        struct {
            uint32_t ack_rec:             1;        /*This register stores the value of ACK bit.*/
            uint32_t slave_rw:            1;        /*when in slave mode  1：master read slave  0: master write slave.*/
						/*chuck SLAVE Mode: this bit is UPDATED to match the LAST Master->Slave direction on the bus. IT IS NOT limited to transactions referencing THIS SLAVE*/
            uint32_t time_out:            1;        /*when I2C takes more than time_out_reg clocks to receive a data then this register changes to high level.*/
            uint32_t arb_lost:            1;        /*when I2C lost control of SDA line  this register changes to high level.*/
            uint32_t bus_busy:            1;        /*1:I2C bus is busy transferring data. 0:I2C bus is in idle state.*/
            uint32_t slave_addressed:     1;        /*when configured as i2c slave  and the address send by master is equal to slave's address  then this bit will be high level.*/
						/*chuck Slave Mode receive: non persistant, set when last byte received Matched Slave Id.  As soon as first byte of data received, this bit is cleared. No Interrupt generated*/
            uint32_t byte_trans:          1;        /*This register changes to high level when one byte is transferred.*/
            uint32_t reserved7:           1;
            uint32_t rx_fifo_cnt:         6;        /*This register represent the amount of data need to send.*/
            uint32_t reserved14:          4;
            uint32_t tx_fifo_cnt:         6;        /*This register stores the amount of received data  in ram.*/
            uint32_t scl_main_state_last: 3;        /*This register stores the value of state machine for i2c module.  3'h0: SCL_MAIN_IDLE  3'h1: SCL_ADDRESS_SHIFT 3'h2: SCL_ACK_ADDRESS  3'h3: SCL_RX_DATA  3'h4 SCL_TX_DATA  3'h5:SCL_SEND_ACK 3'h6:SCL_WAIT_ACK*/
            uint32_t reserved27:          1;
            uint32_t scl_state_last:      3;        /*This register stores the value of state machine to produce SCL. 3'h0: SCL_IDLE  3'h1:SCL_START   3'h2:SCL_LOW_EDGE  3'h3: SCL_LOW   3'h4:SCL_HIGH_EDGE   3'h5:SCL_HIGH  3'h6:SCL_STOP*/
            uint32_t reserved31:          1;
        };
        uint32_t val;
    } I2C_STATUS_REG_t;
    
typedef union {
        struct {
            uint32_t rx_fifo_full:     1;           /*The raw interrupt status bit for rx_fifo full when use apb fifo access.*/
            uint32_t tx_fifo_empty:    1;           /*The raw interrupt status bit for tx_fifo empty when use apb fifo access.*/
					/*chuck will only clear if tx_fifo has more than fifo_conf.tx_fifo_empty_thrhd bytes in it.*/
            uint32_t rx_fifo_ovf:      1;           /*The raw interrupt status bit for receiving data overflow when use apb fifo access.*/
            uint32_t end_detect:       1;           /*The raw interrupt status bit for end_detect_int interrupt. when I2C deals with  the END command  it will produce end_detect_int interrupt.*/
            uint32_t slave_tran_comp:  1;           /*The raw interrupt status bit for slave_tran_comp_int interrupt. when I2C Slave detects the STOP bit  it will produce slave_tran_comp_int interrupt.*/
						/*chuck Slave Mode: actually triggered after receipt of Slave Address. */
            uint32_t arbitration_lost: 1;           /*The raw interrupt status bit for arbitration_lost_int interrupt.when I2C lost the usage right of I2C BUS it will produce arbitration_lost_int interrupt.*/
            uint32_t master_tran_comp: 1;           /*The raw interrupt status bit for master_tra_comp_int interrupt. when I2C Master sends or receives a byte it will produce master_tran_comp_int interrupt.*/
            uint32_t trans_complete:   1;           /*The raw interrupt status bit for trans_complete_int interrupt. when I2C Master finished STOP command  it will produce trans_complete_int interrupt.*/
						/*chuck Slave Mode: triggerd when STOP is seen on the Bus. ANY STOP including those generated by OTHER MASTERS TALKING with OTHER SLAVES */
            uint32_t time_out:         1;           /*The raw interrupt status bit for time_out_int interrupt. when I2C takes a lot of time to receive a data  it will produce  time_out_int interrupt.*/
            uint32_t trans_start:      1;           /*The raw interrupt status bit for trans_start_int interrupt. when I2C sends the START bit it will produce trans_start_int interrupt.*/
            /*chuck Only issued after ctr.trans_start=1 and a START has been sent.  It does not fire on a ReSTART */
            uint32_t ack_err:          1;           /*The raw interrupt status bit for ack_err_int interrupt. when I2C receives a wrong ACK bit  it will produce ack_err_int interrupt..*/
						/*chuck SLAVE MODE: triggered WHENEVER a NAK is seen on the BUS.  If another master does an presense detect, this interrupt will be triggered on every failed acknowledgement*/
            uint32_t rx_rec_full:      1;           /*The raw interrupt status bit for rx_rec_full_int interrupt. when I2C receives more data  than nonfifo_rx_thres  it will produce rx_rec_full_int interrupt.*/
            uint32_t tx_send_empty:    1;           /*The raw interrupt status bit for tx_send_empty_int interrupt.when I2C sends more data than nonfifo_tx_thres  it will produce tx_send_empty_int interrupt..*/
            uint32_t reserved13:      19;
        };
        uint32_t val;
    } I2C_INTERRUPT_t;
    
	
#endif