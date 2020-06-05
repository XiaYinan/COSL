/*
*********************************************************************************************************
*                                                VSTOS-I
*                                          VSN: VST_2.18.6
*                              Used for DSM measure the vibration of downhole.
*                              (c) Copyright 2016-2026, COSL, WELL-TECH INSTITUTE, LWD TEAM
*                                           All Rights Reserved
*
*                                           MASTER INCLUDE FILE
*********************************************************************************************************
*/
/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#include "math.h"
#include "spi.h"
#include "sys_core.h"
#include "sci.h"
#include "gio.h"
#include "i2c.h"
#include "adc.h"
#include "het.h"
#include "reg_rti.h"
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
uint8 BootCount = 0;
uint16 status_code = 0;
uint16 raw_data_storage_cycle=2;
uint16 raw_data_storage_cnt=0;
int bF[10] = {0};  /*flash existence */
uint16 temperature = 0;				/* temperature value */
float temperature_float = 0;
uint16 rtemperature = 0;
uint16 temp_write0 = 0;
uint32 TimeSec=0;      /* time mesured by seconds */
uint32 acc2k_time = 0;		/* the low 32bits time of each frame of 2k saving data, this time is the last sampling data time */
uint8 tem_t=0;
uint8 second=0;
uint8 minute=0;
uint8 hour=0;
uint8 day=0;
uint8 month=0;
uint8 year=0;
float accr_float=0;
int realsecond=0;
int realminute=0;
int realhour=0;
int realday=0;
int realmonth=0;
int realyear=0;
unsigned int brxstat = 0;			/* UART receive data processing status */
uint16 accx_send, accy_send, accz_send, accr_send, temp_send = 0;	/* used for UART data transmition */
float accrfloatsend = 0;
uint8 rcvdata[64] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};			/* UART receive data buffer */
uint16  ReadFlagStatusData[2] = {0x70, 0x00};
uint16	ReceiveFlagStatusData[2]	= { 0 };
uint16	ReadStatusData[2]	= { 0x05, 0x00 };
uint16	ReceiveStatusData[2]	= { 0 };
unsigned char i_rcv = 0;												/* UART receive data pointer */
uint16 accx_2k_buf0[10000], accy_2k_buf0[10000],	accz_2k_buf0[10000], accr_2k_buf0[10000];	/* 1kHz sampling data buffers for accx, accy, accz (8x accumulation results)*/
uint16 accx_2k_buf1[10000], accy_2k_buf1[10000],	accz_2k_buf1[10000], accr_2k_buf1[10000];	/* 1kHz sampling data buffers for accx, accy, accz (8x accumulation results)*/
float rms_buf_100ms_x[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float rms_buf_100ms_y[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float rms_buf_100ms_z[100] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int shock_xy_num_thresh[4] = {0, 0, 0, 0};
int shock_z_num_thresh[4] = {0, 0, 0, 0};
bool adcflag=0;
// high pass filter for accx,  8kHz
int accx_f0 = 0;
// high pass filter for accy,  8kHz
int accy_f0 = 0;
// high pass filter for accz,  8kHz
int accz_f0 = 0;
// high pass filter for accr,  8kHz
int accr_f0 = 0;
int	rtemperature_sum=0;
int rtemperature_f0=0;
unsigned char bComHighSpeed = 0;	/* indicate the COM speed, 0__9600bps, 1__115200bps*/
unsigned int sav_addr0 = 0xf6dce00;			/* Global address of character data storage*/
unsigned int sav_addr1 = 0x0000000;         /* Global address of raw data storage, the range is 0x10000000~0x14000000 */
uint16 fm_addr = 0;
unsigned int sav_cnt = 0;
unsigned int ia2k = 0;				/* index for 2k buffer */
unsigned int ia_rms = 0;
int ia2k_sav_cnt = 0;
unsigned char bsav_level = 0,bsav_data = 0;			/* flag for saving 2k data */
int accx_sum = 0, accy_sum = 0, accz_sum = 0, accr_sum = 0;	/* used for acculumating accx, accy and accz data for 2kHz sampling */
long long rms_sum_sec_x = 0, rms_sum_sec_y = 0, rms_sum_sec_z = 0;		/* calculate the rms value of 10 seconds for three axises */
float rms_sec_x = 0, rms_sec_y = 0,  rms_sec_z = 0;
long long rms_sec_cur_x = 0, rms_sec_cur_y = 0, rms_sec_cur_z = 0;		/* storing the rms value of 10 seconds for three axises */
int peak_sec_x = 0, peak_sec_y = 0, peak_sec_z = 0;					/* calculate the peak value of 10 seconds for three axises */
float peak_sec_x_float=0, peak_sec_y_float=0, peak_sec_z_float=0;
float peak_rms_xy=0, peak_rms_z=0;
int peak_sec_cur_x = 0, peak_sec_cur_y = 0, peak_sec_cur_z = 0;
short int peak_sec_cur_r, peak_sec_r= 0;		/* storing the peak value of 10 seconds for three axises */
short int bottom_sec_cur_r=0, bottom_sec_r=0;
float peak_sec_r_float=0, peak_sec_cur_r_float=0, bottom_sec_cur_r_float=10000,bottom_sec_r_float=10000;
int avg_sec_x = 0, avg_sec_y = 0, avg_sec_z = 0;					/* calculate the average value of 10 seconds for three axises */
float avg_sec_x_float=0, avg_sec_y_float=0, avg_sec_z_float=0,avg_sec_r_float=0;
uint16 avg_sec_cur_x = 0, avg_sec_cur_y = 0, avg_sec_cur_z = 0;		/* storing the average value of 10 seconds for three axises */
short int avg_sec_cur_r, avg_sec_r = 0;
float avg_sec_cur_r_float = 0;
float avg_sec_x_float_100ms=0, avg_sec_y_float_100ms=0, avg_sec_z_float_100ms=0;
float square_sec_x_float_100ms=0, square_sec_y_float_100ms=0, square_sec_z_float_100ms=0;


long long rms_cur_bulk=0;
float rms_cur_bulk_float=0;
unsigned int gioA_tmp1 = 0xef;			/* GPIO value for current writing bulk */
unsigned int gioB_tmp1 = 0xff;			/* GPIO value for current writing bulk */
unsigned int gioA_tmp0 = 0xef;			/* GPIO value for current writing bulk */
unsigned int gioB_tmp0 = 0xff;			/* GPIO value for current writing bulk */
unsigned char bstart_tx = 0;   /* Control data sending */
unsigned char sendmode = 0;   /* Control data sending */
unsigned char bSend = 0;	/* Control data sending */
unsigned char bSave = 0;	/* Control data saving */
unsigned char bAdc=0;
unsigned char sav_stat = 0;
unsigned int tmp_spi;     /* tmperate data for spi control */
spiDAT1_t dataconfig1_t;  /* spi data format */
unsigned char btxstat = 0;
unsigned char bi2cstat = 0;
unsigned int sav_fm_stat = 0;
unsigned char bSaveFM = 0;
uint16 shock_g_thresh=50;
float rms_xy_thresh[15]={0.5,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
float rms_z_thresh[15]={0.5,1,2,3,4,5,6,7,8,9,10,11,12,13,14};
uint16 minusR=0;
uint8 xylevel=0,zlevel=0,rlevel=0,level=0;
uint8 shock_level_xy,shock_level_z=0;
uint8 rms_level_xy,rms_level_z=0;
uint16 max_rms_xy=0, avg_rms_xy=0, max_rms_z=0, avg_rms_z=0;
float s1=0.00,s2=0.00;
uint16 shock_cnt_xy=0,shock_cnt_z=0;
int iTick = 0;					/* I2C to sampling temperature interval count, default 0.5sps*/
unsigned char bStartI2c = 0;	/* flag for start temperature convert and time reading */
uint32 ar,br,cr,dr = 0;                   //parameters to calibrate the primitive data
uint32 Bx,Ax,By,Ay,Bz,Az=0;        
float Bx_float,Ax_float,By_float,Ay_float,Bz_float,Az_float=0;
float  arfloat,brfloat,crfloat,drfloat=0;
int i=0;
int tmp;
int ovtime = 0;
int ovtimeX = 0;
int i2c_datah = 0, i2c_datal = 0;          // received data from i2c
uint16 data_send = 0;
unsigned int rxdata = 0;
unsigned int rxdata1 = 0;
unsigned int rxdata2 = 0;
unsigned int rxdata3 = 0;
unsigned int rxdata4 = 0;
uint32 parameter = 0;
/* UART send data  */
uint16 data[304];
uint16 datalength;
uint16 check=0;
uint32 cksmspi1=0;
uint32 cksmrecover=0;
uint32 rcvspicksm=0;
uint16 txdata=0;
uint16 txdata1=0;
uint16 txdata2=0;
uint16 bulk_select=0;
uint16 baud=0;   // baud rate dlag
uint32 LB=0;     // LB address 1LB=256Bytes
uint16 LBN =0;   // LB amount
uint64 tool_ID=0;
uint8 tool_ID0=0,tool_ID1=0,tool_ID2=0,tool_ID3=0,tool_ID4=0,tool_ID5=0,tool_ID6=0,tool_ID7=0;
uint16 CHKS=0;           //checksum value
uint8 mode=0x00;

uint16 *ipWord;
uint8 itemp;
//const char FW_Version[]="VST2.18.61";       //unified the version management 2018.6�µ�һ�棬2018.6.5
const char FW_Version[]="VST2.18.62";    //modify rtc driver --> UTC to BJT
/* for int-float transfer */
uint32 *pword;
char *p ;
void wait(uint64 x)
{
	while(x--);   /* create delay */
}
int abs(int x);
float absf(float x);
void ADC_init(void);
void recovery(void);
void MyGioSetPortA(uint8 value);
/*  UART operation  */
void UART_init(void);
void MysciSendByte(uint8 byte);
void MysciSend2Bytes(uint16 byte);
void MysciSend(uint16 * data);
uint32 CacuSecondTime(int t1,int t2,int t3,int t4,int t5,int t6);
void CacuClockTime(uint32 TimeSec);
/*  flash operation */
uint32 spiReadAddr(void);
unsigned char ReadByte(uint32 addr);
void WriteByte(uint32 addr, int data);
void newWriteByte(uint32 addr, int data);
void SPI_init(void);
void WREN (int x);        /* flash write enable */
int ReadFlashID(int i);
void StartAdd(uint16 addr);
void spi1transmitByte(uint8 data);
void spi1sendByte(uint8 x);
void spi2sendByte(uint8 x);
void bulk_erase_character(void);
void bulk_erase_primitive(void);
void bulk_erase_test(void);
void bulk_erase(void);
void read_LB(uint32 addr_read);
/*  FM operation  */
void FMWREN (void);
uint8 ReadFMByte(uint16 addr);
void WriteFMByte(uint16 addr, uint8 data);
void WriteFM4Bytes(uint32 x,uint32 firstaddr);
//#define debug
#define watchdog

/* USER CODE END */
#pragma diag_suppress=951

void main(void)
{
/* USER CODE BEGIN (3) */
#ifdef watchdog
	rtiREG1->DWDPRLD=0xFFF;      //set watchdog preload counter
	rtiREG1->DWDCTRL=0xA98559DA; //enable watchdog
#endif	
	gioInit();
	gioSetDirection( gioPORTA, 0xFF );
	gioSetDirection( gioPORTB, 0xFF );
	hetInit();
	SPI_init();
	_enable_interrupt_();
	UART_init();
	ADC_init();
	dataconfig1_t.CS_HOLD	= 0;
	dataconfig1_t.WDEL	= 1;
	dataconfig1_t.DFSEL	= SPI_FMT_0;
	dataconfig1_t.CSNR	= 0;
	i2cInit();
	recovery();
#ifdef debug
	sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
	gioSetBit(gioPORTA,2,1);
	MysciSendByte(0x67);  //for debug
	gioSetBit(gioPORTA,2,0);
	sciEnableNotification(scilinREG,SCI_RX_INT);
#endif
	while(1)
	{
#ifdef watchdog
		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
#endif
		/*********** I2C temperature sampling and time recording process start ************/
		switch ( bi2cstat )
		{
			/*********** I2C temperature sampling process start ************/
		case 0:
			if ( bStartI2c )                        /* 0.5s periodically sampling Temperature, */
			{
				bStartI2c = 0;
				i2cInit();
				i2cSetOwnAdd( i2cREG1, 0x20 );
				i2cSetStart( i2cREG1 );
				i2cSetSlaveAdd( i2cREG1, 0x48 );
				i2cSendByte( i2cREG1, 0x90 );   /* ?????????????? */
				ovtime		= 0;            /* used for control overtime error and exit loop */
				bi2cstat	= 1;
			}
			break;
		case 1:
			ovtime++;                               /* wait for transimition done */
			if ( (i2cREG1->STR & (uint32) I2C_TX_INT) != 0U )
			{
				i2cREG1->DXR	= (uint32) 0x00;
				bi2cstat	= 2;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )                   /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 2:
			i2cSetStart( i2cREG1 );
			i2cSetDirection( i2cREG1, I2C_RECEIVER );
			bi2cstat = 3;
			break;
		case 3:
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_RX_INT) != 0U )
			{
				i2c_datah	= (uint8) i2cREG1->DRR; /* read temperature MSB byte */
				bi2cstat	= 4;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )                           /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 4:                                                 /* Judge the reading flag and start to read temperature LSB byte */
			i2cInit();
			i2cSetOwnAdd( i2cREG1, 0x20 );
			i2cSetStart( i2cREG1 );
			i2cSetSlaveAdd( i2cREG1, 0x48 );
			i2cSendByte( i2cREG1, 0x90 );                   /* activate a write for Read mode */
			ovtime		= 0;
			bi2cstat	= 5;
			break;
		case 5:                                                 /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_TX_INT) != 0U )
			{
				i2cREG1->DXR	= (uint32) 0x01;
				bi2cstat	= 6;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )                           /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 6:                                                 /* wait for transimition done */
			i2cSetStart( i2cREG1 );
			i2cSetDirection( i2cREG1, I2C_RECEIVER );
			bi2cstat = 7;
			break;
		case 7:                                                 /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_RX_INT) != 0U )
			{
				i2c_datal	= (uint8) i2cREG1->DRR; /* read temperature LSB byte */
				bi2cstat	= 8;
				ovtime		= 0;
				temperature	= (i2c_datal | (i2c_datah << 8) );
				temperature	= temperature / 8;
			}
			if ( ovtime > 10000 )                           /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
			/*********** I2C temperature sampling process end ************/
			/*********** I2C time reading process start ************/
		case 8:                                                 /* second reading */
			i2cInit();
			i2cSetOwnAdd( i2cREG1, 0x20 );
			i2cSetStart( i2cREG1 );
			i2cSetSlaveAdd( i2cREG1, 0x51 );
			i2cSendByte( i2cREG1, 0xA2 );                   /* activate a write for Read mode */
			ovtime		= 0;                            /*  */
			bi2cstat	= 9;
			break;
		case 9:                                                 /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_TX_INT) != 0U )
			{
				i2cREG1->DXR	= (uint32) 0x02;
				bi2cstat	= 10;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )   /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 10:                        /* wait for transimition done */
			i2cSetStart( i2cREG1 );
			i2cSetDirection( i2cREG1, I2C_RECEIVER );
			bi2cstat = 11;
			break;
		case 11:                        /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_RX_INT) != 0U )
			{
				second		= (uint8) i2cREG1->DRR;
				bi2cstat	= 12;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )           /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 12:                                /* minute reading */
			i2cInit();
			i2cSetOwnAdd( i2cREG1, 0x20 );
			i2cSetStart( i2cREG1 );
			i2cSetSlaveAdd( i2cREG1, 0x51 );
			i2cSendByte( i2cREG1, 0xA2 );   /* activate a write for Read mode */
			ovtime		= 0;            /*  */
			bi2cstat	= 13;
			break;
		case 13:                                /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_TX_INT) != 0U )
			{
				i2cREG1->DXR	= (uint32) 0x03;
				bi2cstat	= 14;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )   /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 14:                        /* wait for transimition done */
			i2cSetStart( i2cREG1 );
			i2cSetDirection( i2cREG1, I2C_RECEIVER );
			bi2cstat = 15;
			break;
		case 15:                        /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_RX_INT) != 0U )
			{
				tem_t		= (uint8) i2cREG1->DRR;
				minute		= tem_t & 0x7f;
				bi2cstat	= 16;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )           /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 16:                                /* hour reading */
			i2cInit();
			i2cSetOwnAdd( i2cREG1, 0x20 );
			i2cSetStart( i2cREG1 );
			i2cSetSlaveAdd( i2cREG1, 0x51 );
			i2cSendByte( i2cREG1, 0xA2 );   /* activate a write for Read mode */
			ovtime		= 0;            /*  */
			bi2cstat	= 17;
			break;
		case 17:                                /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_TX_INT) != 0U )
			{
				i2cREG1->DXR	= (uint32) 0x04;
				bi2cstat	= 18;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )   /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 18:                        /* wait for transimition done */
			i2cSetStart( i2cREG1 );
			i2cSetDirection( i2cREG1, I2C_RECEIVER );
			bi2cstat = 19;
			break;
		case 19:                        /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_RX_INT) != 0U )
			{
				tem_t		= (uint8) i2cREG1->DRR;
				hour		= tem_t & 0x3f;
				bi2cstat	= 20;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )           /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 20:                                /* day reading */
			i2cInit();
			i2cSetOwnAdd( i2cREG1, 0x20 );
			i2cSetStart( i2cREG1 );
			i2cSetSlaveAdd( i2cREG1, 0x51 );
			i2cSendByte( i2cREG1, 0xA2 );   /* activate a write for Read mode */
			ovtime		= 0;            /*  */
			bi2cstat	= 21;
			break;
		case 21:                                /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_TX_INT) != 0U )
			{
				i2cREG1->DXR	= (uint32) 0x05;
				bi2cstat	= 22;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )   /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 22:                        /* wait for transimition done */
			i2cSetStart( i2cREG1 );
			i2cSetDirection( i2cREG1, I2C_RECEIVER );
			bi2cstat = 23;
			break;
		case 23:                        /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_RX_INT) != 0U )
			{
				tem_t		= (uint8) i2cREG1->DRR;
				day		= tem_t & 0x3f;
				bi2cstat	= 24;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )           /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 24:                                /* month reading */
			i2cInit();
			i2cSetOwnAdd( i2cREG1, 0x20 );
			i2cSetStart( i2cREG1 );
			i2cSetSlaveAdd( i2cREG1, 0x51 );
			i2cSendByte( i2cREG1, 0xA2 );   /* activate a write for Read mode */
			ovtime		= 0;            /*  */
			bi2cstat	= 25;
			break;
		case 25:                                /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_TX_INT) != 0U )
			{
				i2cREG1->DXR	= (uint32) 0x07;
				bi2cstat	= 26;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )   /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 26:                        /* wait for transimition done */
			i2cSetStart( i2cREG1 );
			i2cSetDirection( i2cREG1, I2C_RECEIVER );
			bi2cstat = 27;
			break;
		case 27:                        /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_RX_INT) != 0U )
			{
				tem_t		= (uint8) i2cREG1->DRR;
				month		= tem_t & 0x1f;
				bi2cstat	= 28;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )           /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 28:                                /* year reading */
			i2cInit();
			i2cSetOwnAdd( i2cREG1, 0x20 );
			i2cSetStart( i2cREG1 );
			i2cSetSlaveAdd( i2cREG1, 0x51 );
			i2cSendByte( i2cREG1, 0xA2 );   /* activate a write for Read mode */
			ovtime		= 0;            /*  */
			bi2cstat	= 29;
			break;
		case 29:                                /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_TX_INT) != 0U )
			{
				i2cREG1->DXR	= (uint32) 0x08;
				bi2cstat	= 30;
				ovtime		= 0;
			}
			if ( ovtime > 10000 )   /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		case 30:                        /* wait for transimition done */
			i2cSetStart( i2cREG1 );
			i2cSetDirection( i2cREG1, I2C_RECEIVER );
			bi2cstat = 31;
			break;
		case 31:                        /* wait for transimition done */
			ovtime++;
			if ( (i2cREG1->STR & (uint32) I2C_RX_INT) != 0U )
			{
				tem_t		= (uint8) i2cREG1->DRR;
				year		= tem_t & 0xff;
				bi2cstat	= 0;
				ovtime		= 0;
			}
			if ( ovtime > 10000 ) /* overtime error, return back to status 0 */
				bi2cstat = 0;
			break;
		default:
			bi2cstat = 0;
			break;
			/*********** I2C time reading process end ************/
		}
		/************* I2C temperature sampling and time recording process end ************/
		/***************************** UART Receive loop start ****************************/
		switch(brxstat)
		{
		case 0:
			if(i_rcv != 0)
			{
				ovtimeX=0;
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x05)		    // read UART status
					brxstat = 1;
				else if(rxdata == 0x02)		// mode STANDBY
					brxstat = 5;
				else if(rxdata == 0x03)		// mode OPERATION
					brxstat = 9;
				else if(rxdata == 0x07)		// read Tool ID
					brxstat = 24;
				else if(rxdata == 0x06)		// write Tool ID
					brxstat = 28;
				else if(rxdata == 0x08)		// read Firmware Version
					brxstat = 42;
				else if(rxdata == 0x13)		// write Baud rate and raw data stroage cycle
					brxstat = 46;
				else if(rxdata == 0x12)		// read time
					brxstat = 53;
				else if(rxdata == 0x11)		// write time in seconds
					brxstat = 57;
				else if(rxdata == 0x50)		// rms
					brxstat = 68;
				else if(rxdata == 0x51)		// avg
					brxstat = 72;
				else if(rxdata == 0x52)		// peak to peak
					brxstat = 76;
				else if(rxdata == 0x53)		// temperature
					brxstat = 80;
				else if(rxdata == 0x54)		// r
					brxstat = 84;
				else if(rxdata == 0x2A)		// bulk erase
					brxstat = 88;
				else if(rxdata == 0x27)		// read N LB
					brxstat = 98;
				else if(rxdata == 0x21)		// read the current address
					brxstat = 110;
				else if(rxdata == 0x55)		// read levels
					brxstat = 114;
				else if(rxdata == 0x15)		// write calibration parameter table
					brxstat = 118;
				else if(rxdata == 0x16)		// read calibration parameter table
					brxstat = 180;
				else if(rxdata == 0x5A)		// read real time data
					brxstat = 190;
				else if(rxdata == 0x14)     // read Baud rate and raw data stroage cycle
					brxstat = 195;
				else if(rxdata == 0x17)     // write thresh parameter table
					brxstat = 199;
				else if(rxdata == 0x18)     // read thresh parameter table
					brxstat = 200;
				else if(rxdata == 0x56)     // read thresh parameter table
					brxstat = 201;
				else
					brxstat = 0;
			}
			break;
		case 1:                // read UART status
			
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			data[0]=0x0003;
			data[1]=(bComHighSpeed<<8)|sendmode;
			data[2]=BootCount;
			data[3]=data[0]^data[1]^data[2]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;			

		case 5:             // mode STANDBY
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			mode=0x00;
			bSave=0;
			bSend=0;
			FMWREN ();
			WriteFMByte(0x0000, bSave);
			data[0]=0x0003;
			data[1]=(bComHighSpeed<<8)|sendmode;
			data[2]=0xff00|mode;
			data[3]=data[0]^data[1]^data[2]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;
		case 9:         // mode OPERATION
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			mode=0x01;
			bSave=1;
			bSend=0;
			FMWREN ();
			WriteFMByte(0x0000, bSave);
		  bSaveFM=1;
			sendmode=0;
//			bSave=0;
			data[0]=0x0003;
			data[1]=(bComHighSpeed<<8)|sendmode;
			data[2]=0xff00|mode;
			data[3]=data[0]^data[1]^data[2]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;

		case 24:           // read Tool ID
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			data[0]=0x0005;
			data[1]=(ReadFMByte(0x0010)<<8)|ReadFMByte(0x0011);
			data[2]=(ReadFMByte(0x0012)<<8)|ReadFMByte(0x0013);
			data[3]=(ReadFMByte(0x0014)<<8)|ReadFMByte(0x0015);
			data[4]=(ReadFMByte(0x0016)<<8)|ReadFMByte(0x0017);
			data[5]=data[0]^data[1]^data[2]^data[3]^data[4]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;
		case 28:           // write Tool ID  7bit+CHKS
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 29;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 29:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 30;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 30:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x05)
				{
					check=0x0005^0xffff;
					data[0]=0x0005;
					brxstat = 31;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 31:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				tool_ID0 = rxdata;
				i_rcv--;
				tool_ID= rxdata;
				ovtimeX= 0;
				brxstat = 32;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 31;
			break;
		case 32:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				tool_ID1 = rxdata;
				i_rcv--;
				data[1]= (tool_ID0<<8|tool_ID1);
				check ^= data[1];
				tool_ID=(tool_ID<<8)|rxdata;
				ovtimeX= 0;
				brxstat = 33;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 32;
			break;
		case 33:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				tool_ID2 = rxdata;
				i_rcv--;
				tool_ID=(tool_ID<<8)|rxdata;
				ovtimeX= 0;
				brxstat = 34;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 33;
			break;
		case 34:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				tool_ID3 = rxdata;
				i_rcv--;
				data[2]= (tool_ID2<<8|tool_ID3);
				check ^= data[2];
				tool_ID=(tool_ID<<8)|rxdata;
				ovtimeX= 0;
				brxstat = 35;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 34;
			break;
		case 35:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				tool_ID4 = rxdata;
				i_rcv--;
				tool_ID=(tool_ID<<8)|rxdata;
				ovtimeX= 0;
				brxstat = 36;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 35;
			break;
		case 36:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				tool_ID5 = rxdata;
				i_rcv--;
				data[3]= (tool_ID4<<8|tool_ID5);
				check ^= data[3];
				tool_ID=(tool_ID<<8)|rxdata;
				ovtimeX= 0;
				brxstat = 37;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 36;
			break;
		case 37:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				tool_ID6 = rxdata;
				i_rcv--;
				tool_ID=(tool_ID<<8)|rxdata;
				ovtimeX= 0;
				brxstat = 38;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 37;
			break;
		case 38:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				tool_ID7 = rxdata;
				i_rcv--;
				data[4]= (tool_ID6<<8|tool_ID7);
				check ^= data[4];
				tool_ID=(tool_ID<<48)|rxdata;
				ovtimeX= 0;
				brxstat = 39;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 38;
			break;
		case 39:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				brxstat = 40;
				ovtimeX= 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 39;
			break;
		case 40:    // to check the sum
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				CHKS=rxdata1<<8|rxdata2;
				if(CHKS==check)
				{
					brxstat = 41;
					data[5]= check;
					ovtimeX= 0;
				}
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 40;
			break;
		case 41:
			MysciSend(data);
			FMWREN ();
			WriteFMByte(0x0010, tool_ID0);
			FMWREN ();
			WriteFMByte(0x0011, tool_ID1);
			FMWREN ();
			WriteFMByte(0x0012, tool_ID2);
			FMWREN ();
			WriteFMByte(0x0013, tool_ID3);
			FMWREN ();
			WriteFMByte(0x0014, tool_ID4);
			FMWREN ();
			WriteFMByte(0x0015, tool_ID5);
			FMWREN ();
			WriteFMByte(0x0016, tool_ID6);
			FMWREN ();
			WriteFMByte(0x0017, tool_ID7);
			brxstat = 0;
			break;
		case 42:         // read Firmware Version
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			data[0]=0x0006;     // const char FW_Version[]="VST_1.2.30";
			/*
		  data[1]=0x5653;
			data[2]=0x545f;
			data[3]=0x312e;
			data[4]=0x322e;
			data[5]=0x3330;
		  */
		    ipWord = (uint16*)FW_Version;
			for(itemp = 1; itemp <6; itemp++)
			{
					data[itemp] = *ipWord;
					ipWord++;
			}
			data[6]=data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;
		case 46:         // set baud rate
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					ovtimeX= 0;
					brxstat = 47;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 47:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					ovtimeX= 0;
					brxstat =48;
				}
				else
					brxstat = 47;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 48:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x03)
				{
					ovtimeX= 0;
					brxstat = 49;
				}
				else
					brxstat = 48;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 49:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX= 0;
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				baud=(rxdata<<8);
				brxstat = 50;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 50:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				baud|=rxdata;
				if(baud == 0xE100)
				{
					ovtimeX= 0;
					brxstat = 300;
					bComHighSpeed=1;
					check=0xE100^0x0003^0xffff;
				}
				else if(baud == 0x0060)
				{
					ovtimeX= 0;
					brxstat = 300;
					bComHighSpeed=0;
					check=0x0060^0x0003^0xffff;
				}
				else
					brxstat = 50;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 300:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX= 0;
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				raw_data_storage_cycle=(rxdata<<8);
				brxstat = 301;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;		
		case 301:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				raw_data_storage_cycle|=rxdata;
				if ((raw_data_storage_cycle >= 1) && (raw_data_storage_cycle <= 6))
				{
					check=check^raw_data_storage_cycle;
					brxstat = 51;
				}
				else
					brxstat = 300;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;

		case 51:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				CHKS=rxdata1;
				brxstat = 52;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 52:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				CHKS=(CHKS<<8)|rxdata2;
				if(CHKS==check)
				{
					if(bComHighSpeed==1)
					{
						brxstat = 0;
						ovtimeX= 0;
						scilinREG->BRS = 42U;  /* baudrate */		 //set high speed
					}
					else
					{
						brxstat = 0;
						ovtimeX= 0;
						scilinREG->BRS = 520U;  /* baudrate */		 //set low speed
					}
				}
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 53:         // read time
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					ovtimeX= 0;
					brxstat = 54;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 54:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					ovtimeX= 0;
					brxstat = 55;
				}
				else
					brxstat = 54;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 55:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					ovtimeX= 0;
					brxstat = 56;
				}
				else
					brxstat = 55;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 56:
			gioSetBit(gioPORTA,2,1);
			sciDisableNotification(scilinREG,SCI_RX_INT);
			data[0]=4;
			data[1]=year;
			data[1]=(data[1]<<8)+month;
			data[2]=day;
			data[2]=(data[2]<<8)+hour;
			data[3]=minute;
			data[3]=(data[3]<<8)+second;
			data[4]=data[0]^data[1]^data[2]^data[3]^0xFFFF;
			MysciSend(data);
			sciEnableNotification(scilinREG,SCI_RX_INT);
			gioSetBit(gioPORTA,2,0);
			brxstat = 0;
			break;
		case 57:         // write second time
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51 || rxdata == 0x00)
				{
					brxstat = 58;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 58:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 59;
					ovtimeX= 0;
				}
				else
					brxstat = 58;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 59:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x03)
				{
					check=0x0003^0xffff;
					brxstat = 60;
					ovtimeX= 0;
				}
				else
					brxstat = 59;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 60:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX= 0;
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				TimeSec=(rxdata1<<24);
				brxstat = 61;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 61:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX= 0;
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				TimeSec|=(rxdata2<<16);
				check^=(rxdata1<<8|rxdata2);
				brxstat = 62;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 62:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX= 0;
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				TimeSec|=(rxdata1<<8);
				brxstat = 63;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 63:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX= 0;
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				TimeSec|=rxdata2;
				check^=(rxdata1<<8|rxdata2);
				brxstat = 64;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 64:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX= 0;
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				brxstat = 65;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 65:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX = 0;
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				CHKS=(rxdata1<<8|rxdata2);
				if(CHKS==check)
				{
					ovtimeX= 0;
					brxstat = 0;
					CacuClockTime(TimeSec);
					gioSetBit(gioPORTA,2,1);
					sciDisableNotification(scilinREG,SCI_RX_INT);
					/////////////////////////////
					data[0]=0x0004;
					check=0x0004^0xffff;
					data[1]=(year<<8)|month;
					check^=data[1];
					data[2]=(day<<8)|hour;
					check^=data[2];
					data[3]=(minute<<8)|second;
					check^=data[3];
					data[4]=check;
					MysciSend(data);
					i2cInit();
					i2cSetOwnAdd(i2cREG1,0x20);
					i2cSetStart(i2cREG1);
					i2cSetSlaveAdd(i2cREG1,0x51);
					i2cSendByte(i2cREG1,0xA2);
					i2cSendByte(i2cREG1,0x08);
					i2cSendByte(i2cREG1,year);
					while(i2cIsBusBusy(i2cREG1)) {};
					i2cInit();
					i2cSetOwnAdd(i2cREG1,0x20);
					i2cSetStart(i2cREG1);
					i2cSetSlaveAdd(i2cREG1,0x51);
					i2cSendByte(i2cREG1,0xA2);
					i2cSendByte(i2cREG1,0x07);
					i2cSendByte(i2cREG1,(month&0x1f));
					while(i2cIsBusBusy(i2cREG1)) {};
					i2cInit();
					i2cSetOwnAdd(i2cREG1,0x20);
					i2cSetStart(i2cREG1);
					i2cSetSlaveAdd(i2cREG1,0x51);
					i2cSendByte(i2cREG1,0xA2);
					i2cSendByte(i2cREG1,0x05);
					i2cSendByte(i2cREG1,(day&0x3f));
					while(i2cIsBusBusy(i2cREG1)) {};
					i2cInit();
					i2cSetOwnAdd(i2cREG1,0x20);
					i2cSetStart(i2cREG1);
					i2cSetSlaveAdd(i2cREG1,0x51);
					i2cSendByte(i2cREG1,0xA2);
					i2cSendByte(i2cREG1,0x04);
					i2cSendByte(i2cREG1,(hour&0x3f));
					while(i2cIsBusBusy(i2cREG1)) {};
					i2cInit();
					i2cSetOwnAdd(i2cREG1,0x20);
					i2cSetStart(i2cREG1);
					i2cSetSlaveAdd(i2cREG1,0x51);
					i2cSendByte(i2cREG1,0xA2);
					i2cSendByte(i2cREG1,0x03);
					i2cSendByte(i2cREG1,(minute&0x7f));
					while(i2cIsBusBusy(i2cREG1)) {};
					i2cInit();
					i2cSetOwnAdd(i2cREG1,0x20);
					i2cSetStart(i2cREG1);
					i2cSetSlaveAdd(i2cREG1,0x51);
					i2cSendByte(i2cREG1,0xA2);
					i2cSendByte(i2cREG1,0x02);
					i2cSendByte(i2cREG1,(second&0x7f));
					while(i2cIsBusBusy(i2cREG1)) {};
					sciEnableNotification(scilinREG,SCI_RX_INT);
					gioSetBit(gioPORTA,2,0);
					bSaveFM=1;
				}
				else
					brxstat = 65;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 68:         // read the value of rms
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 69;
					ovtimeX = 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 69:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 70;
					ovtimeX = 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 70:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 71;
					ovtimeX = 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 71:
			data[0]=0x0007;
			check=0x0007^0xffff;
			txdata1=(rms_sec_cur_x>>24)&0xff;
			txdata2=(rms_sec_cur_x>>16)&0xff;
			data[1]=(txdata1<<8)|txdata2;
			check ^= data[1];
			txdata1=(rms_sec_cur_x>>8)&0xff;
			txdata2=rms_sec_cur_x&0xff;
			data[2]=(txdata1<<8)|txdata2;
			check ^= data[2];
			txdata1=(rms_sec_cur_y>>24)&0xff;
			txdata2=(rms_sec_cur_y>>16)&0xff;
			data[3]=(txdata1<<8)|txdata2;
			check ^= data[3];
			txdata1=(rms_sec_cur_y>>8)&0xff;
			txdata2=rms_sec_cur_y&0xff;
			data[4]=(txdata1<<8)|txdata2;
			check ^= data[4];
			txdata1=(rms_sec_cur_z>>24)&0xff;
			txdata2=(rms_sec_cur_z>>16)&0xff;
			data[5]=(txdata1<<8)|txdata2;
			check ^= data[5];
			txdata1=(rms_sec_cur_z>>8)&0xff;
			txdata2=rms_sec_cur_z&0xff;
			data[6]=(txdata1<<8)|txdata2;
			check ^= data[6];
			data[7]= check;
			MysciSend(data);
			brxstat = 0;
			break;
		case 72:         // read the value of avg
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 73;
					ovtimeX = 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 73:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 74;
					ovtimeX = 0;
				}
				else
					brxstat = 73;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 74:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 75;
					ovtimeX = 0;
				}
				else
					brxstat = 74;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 75:
			data[0]=0x0007;
			check=0x0007^0xffff;
			txdata1=(avg_sec_cur_x>>24)&0xff;
			txdata2=(avg_sec_cur_x>>16)&0xff;
			data[1]=(txdata1<<8)|txdata2;
			check ^= data[1];
			txdata1=(avg_sec_cur_x>>8)&0xff;
			txdata2=avg_sec_cur_x&0xff;
			data[2]=(txdata1<<8)|txdata2;
			check ^= data[2];
			txdata1=(avg_sec_cur_y>>24)&0xff;
			txdata2=(avg_sec_cur_y>>16)&0xff;
			data[3]=(txdata1<<8)|txdata2;
			check ^= data[3];
			txdata1=(avg_sec_cur_y>>8)&0xff;
			txdata2=avg_sec_cur_y&0xff;
			data[4]=(txdata1<<8)|txdata2;
			check ^= data[4];
			txdata1=(avg_sec_cur_z>>24)&0xff;
			txdata2=(avg_sec_cur_z>>16)&0xff;
			data[5]=(txdata1<<8)|txdata2;
			check ^= data[5];
			txdata1=(avg_sec_cur_z>>8)&0xff;
			txdata2=avg_sec_cur_z&0xff;
			data[6]=(txdata1<<8)|txdata2;
			check ^= data[6];
			data[7]= check;
			MysciSend(data);
			brxstat = 0;
			break;
		case 76:         // read the value of peak-to-peak
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}

			data[0]=0x0007;
			check=data[0]^0xffff;
			txdata1=(peak_sec_cur_x>>24)&0xff;
			txdata2=(peak_sec_cur_x>>16)&0xff;
			data[1]=(txdata1<<8)|txdata2;
			check ^= data[1];
			txdata1=(peak_sec_cur_x>>8)&0xff;
			txdata2=peak_sec_cur_x&0xff;
			data[2]=(txdata1<<8)|txdata2;
			check ^= data[2];
			txdata1=(peak_sec_cur_y>>24)&0xff;
			txdata2=(peak_sec_cur_y>>16)&0xff;
			data[3]=(txdata1<<8)|txdata2;
			check ^= data[3];
			txdata1=(peak_sec_cur_y>>8)&0xff;
			txdata2=peak_sec_cur_y&0xff;
			data[4]=(txdata1<<8)|txdata2;
			check ^= data[4];
			txdata1=(peak_sec_cur_z>>24)&0xff;
			txdata2=(peak_sec_cur_z>>16)&0xff;
			data[5]=(txdata1<<8)|txdata2;
			check ^= data[5];
			txdata1=(peak_sec_cur_z>>8)&0xff;
			txdata2=peak_sec_cur_z&0xff;
			data[6]=(txdata1<<8)|txdata2;
			check ^= data[6];
			data[7]= check;
			MysciSend(data);
			brxstat = 0;
			break;
		case 80:         // read temperature
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			data[0]=0x0003;
			check=0x0003^0xffff;

			temperature_float = temperature * 0.0625;
			p = (char*)&temperature_float;
			txdata1 = *p;
			p++;
			txdata2 = *p;
			p++;
			data[1]= txdata1<<8| txdata2;

			txdata1 = *p;
			p++;
			txdata2 = *p;
			p++;
			data[2]= txdata1<<8| txdata2;
			check = check ^ data[1] ^ data[2];
			data[3]= check;
			MysciSend(data);
			brxstat = 0;
			break;
		case 84:         // read r
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			data[0]=0x0003;
			check=0x0003^0xffff;

			p = (char*)&accrfloatsend;
			txdata1 = *p;
			p++;
			txdata2 = *p;
			p++;
			data[1]= txdata1<<8| txdata2;

			txdata1 = *p;
			p++;
			txdata2 = *p;
			p++;
			data[2]= txdata1<<8| txdata2;
			check = check ^ data[1] ^ data[2];
			data[3]= check;
			MysciSend(data);
			brxstat = 0;

			break;
		case 88:         // New bulk erase
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 89;
					ovtimeX = 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 89:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 90;
					ovtimeX = 0;
				}
				else
					brxstat = 89;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 90:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x03)
				{
					check=0x0003^0xffff;
					brxstat = 91;
					ovtimeX = 0;
				}
				else
					brxstat = 90;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 91:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 92;
					ovtimeX = 0;
				}
				else
					brxstat = 91;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 92:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x01)
				{
					bulk_select=0x0001;
					check^=bulk_select;
					brxstat = 93;
					ovtimeX = 0;
				}
				else if(rxdata == 0x02)
				{
					bulk_select=0x0002;
					check^=bulk_select;
					brxstat = 93;
					ovtimeX = 0;
				}
				else
					brxstat = 92;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 93:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x2A)
				{
					brxstat = 94;
					ovtimeX = 0;
				}
				else
					brxstat = 93;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 94:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					check^=0x2A51;
					brxstat = 95;
					ovtimeX = 0;
				}
				else
					brxstat = 94;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 95:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				brxstat = 96;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 96:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				CHKS=rxdata1<<8|rxdata2;
				if(check == CHKS)
				{
					brxstat = 97;
					ovtimeX = 0;
				}
				else
					brxstat = 95;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 97:
			data[0]=0x0001;
			data[1]=bulk_select;
			MysciSend(data);			  
			if(bulk_select==0x01)
				bulk_erase_character();
			else
				bulk_erase_primitive();
			MysciSend(data);		     
			brxstat = 0;
			break;
		case 98:         // read N LB
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 99;
					ovtimeX = 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 99:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 100;
					ovtimeX = 0;
				}
				else
					brxstat = 99;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 100:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x03)
				{
					check=0x0003^0xffff;
					brxstat = 101;
					ovtimeX = 0;
				}
				else
					brxstat = 100;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 101:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX = 0;
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				LB=rxdata1;
				brxstat = 102;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 102:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX = 0;
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				LB=(LB<<8)|rxdata2;
				check ^=(rxdata1<<8|rxdata2);
				brxstat = 103;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 103:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX = 0;
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				LB=(LB<<8)|rxdata1;
				brxstat = 104;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 104:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX = 0;
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				LBN=rxdata2;
				check ^=(rxdata1<<8|rxdata2);
				brxstat = 105;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 105:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX = 0;
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				brxstat = 106;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 106:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX = 0;
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				CHKS=(rxdata1<<8|rxdata2);
				if(check==CHKS)
				{
					brxstat = 107;
				}
				else
				{
					brxstat = 105;
				}
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 107:
//       scilinREG->BRS = 42U;  /* baudrate */		 //set high speed
			gioSetBit(gioPORTA,2,1);
			sciDisableNotification(scilinREG,SCI_RX_INT);
			wait(0x5ff);
			// MysciSendByte(0x01);
			// MysciSendByte(0x10);
			data[0]=LBN*128+1;
			wait(40000);
			MysciSend2Bytes(data[0]);
			check=data[0]^0xffff;
			sciEnableNotification(scilinREG,SCI_RX_INT);
			gioSetBit(gioPORTA,2,0);
			brxstat =108;
			break;
		case 108:
			gioSetBit(gioPORTA,2,1);
			sciDisableNotification(scilinREG,SCI_RX_INT);
			read_LB(LB<<8);
			sciEnableNotification(scilinREG,SCI_RX_INT);
			gioSetBit(gioPORTA,2,0);
			LBN--;
			if(LBN==0)
			{
				brxstat = 109;
				LB =0;
			}
			else
			{
				LB++;
				brxstat =108;
			}
			break;
		case 109:
			gioSetBit(gioPORTA,2,1);
			sciDisableNotification(scilinREG,SCI_RX_INT);
			MysciSend2Bytes(check);
			sciEnableNotification(scilinREG,SCI_RX_INT);
			gioSetBit(gioPORTA,2,0);
			brxstat = 0;
			break;
		case 110:                // read sav_addr
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 111;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 111:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 112;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 112:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 113;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 113:
			data[0]=0x0005;
			check=0x012f^0xffff;
			txdata1=(sav_addr0>>24)&0xff;
			txdata2=(sav_addr0>>16)&0xff;
			data[1]=(txdata1<<8)|txdata2;
			check ^= data[1];
			txdata1=(sav_addr0>>8)&0xff;
			txdata2=sav_addr0&0xff;
			data[2]=(txdata1<<8)|txdata2;
			check ^= data[2];
			txdata1=(sav_addr1>>24)&0xff;
			txdata2=(sav_addr1>>16)&0xff;
			data[3]=(txdata1<<8)|txdata2;
			check ^= data[3];
			txdata1=(sav_addr1>>8)&0xff;
			txdata2=sav_addr1&0xff;
			data[4]=(txdata1<<8)|txdata2;			
			    data[4]=(txdata1<<8)|txdata2;
			data[4]=(txdata1<<8)|txdata2;			
			check ^= data[4];
			data[5]= check;
			MysciSend(data);
			brxstat = 0;
			break;
		case 114:                // read importat data
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}

			data[0]=0x0008;
			check=0x0008^0xffff;
			data[1]=xylevel;
			check^=data[1];
			data[2]=zlevel;
			check^=data[2];
			data[3]=rlevel;
			check^=data[3];
			if(accrfloatsend<0)
				data[4]=0x0001;
			else
				data[4]=0x0000;
			check ^= data[4];
			data[5]=absf(accrfloatsend);
			check^=data[5];
			data[6]=s1;
			check^=data[6];
			data[7]=s2;
			check^=data[7];
			data[8]=check;
			MysciSend(data);
			brxstat = 0;
			break;
		case 118:           // write parameter table
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 119;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 119:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 120;
					ovtimeX= 0;
				}
				else
					brxstat = 119;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 120:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x15)    //40 bytes parameter, 2 bytes CHKS
				{
					check=0x0015^0xffff;
					brxstat = 121;
					ovtimeX= 0;
				}
				else
					brxstat = 120;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 121:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 122;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 121;
			break;
		case 122:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 123;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 122;
			break;
		case 123:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 124;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 123;
			break;
		case 124:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				ar=parameter;         // save ar
				parameter=0;
				brxstat = 125;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 124;
			break;
		case 125:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 126;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 125;
			break;
		case 126:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 127;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 126;
			break;
		case 127:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 128;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 127;
			break;
		case 128:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				br=parameter;
				parameter=0;
				brxstat = 129;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 128;
			break;
		case 129:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 130;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 129;
			break;
		case 130:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 131;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 130;
			break;
		case 131:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 132;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 131;
			break;
		case 132:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				cr=parameter;
				parameter=0;
				brxstat = 133;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 132;
			break;
		case 133:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 134;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 133;
			break;
		case 134:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 135;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 134;
			break;
		case 135:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 136;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 135;
			break;
		case 136:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				dr=parameter;
				parameter=0;
				brxstat = 137;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 136;
			break;
		case 137:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 138;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 137;
			break;
		case 138:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 139;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 138;
			break;
		case 139:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 140;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 139;
			break;
		case 140:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				Bx=parameter;    // save xy1
				parameter=0;
				brxstat = 141;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 140;
			break;
		case 141:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 142;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 141;
			break;
		case 142:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 143;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 142;
			break;
		case 143:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 144;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 143;
			break;
		case 144:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				Ax=parameter;     // save bx
				parameter=0;
				brxstat = 145;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 144;
			break;
		case 145:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 146;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 145;
			break;
		case 146:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 147;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 146;
			break;
		case 147:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 148;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 147;
			break;
		case 148:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				By=parameter;     // save ay
				parameter=0;
				brxstat = 149;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 148;
			break;
		case 149:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 150;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 149;
			break;
		case 150:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 151;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 150;
			break;
		case 151:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 152;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 151;
			break;
		case 152:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				Ay=parameter;    // save by
				parameter=0;
				brxstat = 157;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 152;
			break;
		case 157:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 158;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 157;
			break;
		case 158:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 159;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 158;
			break;
		case 159:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 160;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 159;
			break;
		case 160:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				Bz=parameter;
				parameter=0;
				brxstat = 161;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 160;
			break;
		case 161:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 162;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 161;
			break;
		case 162:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 163;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 162;
			break;
		case 163:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 164;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 163;
			break;
		case 164:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				Az=parameter;
				parameter=0;
				brxstat = 177;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 164;
			break;

		case 177:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX = 0;
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				brxstat = 178;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 178:
			ovtimeX++;
			if(i_rcv != 0)
			{
				ovtimeX = 0;
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				CHKS=(rxdata1<<8|rxdata2);
				if(check==CHKS)
				{
					brxstat = 179;
				}
				else
				{
					brxstat = 177;
				}
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 179:
			p = (char*)&ar;
			fm_addr = 0x101a;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&br;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);


			p = (char*)&cr;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&dr;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&Bx;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&Ax;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&By;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&Ay;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&Bz;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&Az;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);	
			
			gioSetBit(gioPORTA,2,1);
			sciDisableNotification(scilinREG,SCI_RX_INT);
			data[0]=0x0001;
			data[1]=check;
			MysciSend(data);
			gioSetBit(gioPORTA,2,0);
			sciEnableNotification(scilinREG,SCI_RX_INT);
			brxstat = 0;			
			break;
		case 180:                // read parameters
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			data[0]=0x0015;
			data[1]=((ar&0xffff0000)>>16);			
			data[2]=(ar&0x0000ffff);    //read ar
			data[3]=((br&0xffff0000)>>16);
			data[4]=(br&0x0000ffff);    //read br
			data[5]=((cr&0xffff0000)>>16);
			data[6]=(cr&0x0000ffff);    //read cr
			data[7]=((dr&0xffff0000)>>16);
			data[8]=(dr&0x0000ffff);    //read dr
			data[9]=((Bx&0xffff0000)>>16);
			data[10]=(Bx&0x0000ffff);    //read xy1
			data[11]=((Ax&0xffff0000)>>16);
			data[12]=(Ax&0x0000ffff);    //read xy2
			data[13]=((By&0xffff0000)>>16);
			data[14]=(By&0x0000ffff);    //read xy3
			data[15]=((Ay&0xffff0000)>>16);
			data[16]=(Ay&0x0000ffff);    //read xy4
			data[17]=((Bz&0xffff0000)>>16);
			data[18]=(Bz&0x0000ffff);    //read z1
			data[19]=((Az&0xffff0000)>>16);
			data[20]=(Az&0x0000ffff);    //read z2
				
			data[21]=data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[20]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;
		case 190:                         // read real time data
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			data[0]=0x0004;
			data[1]=accx_send;
			data[2]=accy_send;
			data[3]=accz_send;
			data[4]=data[0]^data[1]^data[2]^data[3]^0xFFFF;
			MysciSend(data);
			brxstat = 0;
			break;
		

		case 195:
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			
			data[0]=0x0003;
			if (bComHighSpeed == 1)
				data[1] = 0xE100;
			else
				data[1] = 0x0060;
			data[2]=raw_data_storage_cycle;
			data[3]=data[0]^data[1]^data[2]^0xFFFF;
			MysciSend(data);
			brxstat = 0;
			break;

		case 199:           // write shock & rms thresh table
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x12){brxstat = 0;}

			check = 0x0012^0xffff;

			while(i_rcv == 0){}
			parameter = rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			check = check ^ (parameter&0xffff0000 >> 16);
			check = check ^ (parameter&0xffff);

			shock_g_thresh = parameter;


			while(i_rcv == 0){}
			parameter = rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			check = check ^ (parameter&0xffff0000 >> 16);
			check = check ^ (parameter&0xffff);

			shock_xy_num_thresh[0] = parameter;

			while(i_rcv == 0){}
			parameter = rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			check = check ^ (parameter&0xffff0000 >> 16);
			check = check ^ (parameter&0xffff);

			shock_xy_num_thresh[1] = parameter;

			while(i_rcv == 0){}
			parameter = rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			check = check ^ (parameter&0xffff0000 >> 16);
			check = check ^ (parameter&0xffff);

			shock_xy_num_thresh[2] = parameter;

			while(i_rcv == 0){}
			parameter = rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			check = check ^ (parameter&0xffff0000 >> 16);
			check = check ^ (parameter&0xffff);

			shock_xy_num_thresh[3] = parameter;


			while(i_rcv == 0){}
			parameter = rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			check = check ^ (parameter&0xffff0000 >> 16);
			check = check ^ (parameter&0xffff);

			shock_z_num_thresh[0] = parameter;

			while(i_rcv == 0){}
			parameter = rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			check = check ^ (parameter&0xffff0000 >> 16);
			check = check ^ (parameter&0xffff);

			shock_z_num_thresh[1] = parameter;

			while(i_rcv == 0){}
			parameter = rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			check = check ^ (parameter&0xffff0000 >> 16);
			check = check ^ (parameter&0xffff);

			shock_z_num_thresh[2] = parameter;

			while(i_rcv == 0){}
			parameter = rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			while(i_rcv == 0){}
			parameter = (parameter<<8) | rcvdata[i_rcv];
			i_rcv--;
			check = check ^ (parameter&0xffff0000 >> 16);
			check = check ^ (parameter&0xffff);

			shock_z_num_thresh[3] = parameter;

			p = (char*)&shock_g_thresh;
			fm_addr = 0x1042;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&shock_xy_num_thresh[0];
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);


			p = (char*)&shock_xy_num_thresh[1];
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&shock_xy_num_thresh[2];
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&shock_xy_num_thresh[3];
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&shock_z_num_thresh[0];
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);


			p = (char*)&shock_z_num_thresh[1];
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&shock_z_num_thresh[2];
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);

			p = (char*)&shock_z_num_thresh[3];
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);
			p++;
			fm_addr++;
			FMWREN ();
			WriteFMByte(fm_addr, *p);


			data[0]=0x0003;
			data[1]=(bComHighSpeed<<8)|sendmode;
			data[2]=BootCount;
			data[3]=data[0]^data[1]^data[2]^0xFFFF;
			MysciSend(data);
			brxstat = 0;			
		
		case 200:
		
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}

			data[0]=0x0013;
			data[1]=((shock_g_thresh&0xffff0000)>>16);			
			data[2]=(shock_g_thresh&0x0000ffff);    //read ar
			data[3]=((shock_xy_num_thresh[0]&0xffff0000)>>16);
			data[4]=(shock_xy_num_thresh[0]&0x0000ffff);    //read 
			data[5]=((shock_xy_num_thresh[1]&0xffff0000)>>16);
			data[6]=(shock_xy_num_thresh[1]&0x0000ffff);    //read 
			data[7]=((shock_xy_num_thresh[2]&0xffff0000)>>16);
			data[8]=(shock_xy_num_thresh[2]&0x0000ffff);    //read 
			data[9]=((shock_xy_num_thresh[3]&0xffff0000)>>16);
			data[10]=(shock_xy_num_thresh[3]&0x0000ffff);    //read 
			data[11]=((shock_z_num_thresh[0]&0xffff0000)>>16);
			data[12]=(shock_z_num_thresh[0]&0x0000ffff);    //read 
			data[13]=((shock_z_num_thresh[1]&0xffff0000)>>16);
			data[14]=(shock_z_num_thresh[1]&0x0000ffff);    //read 
			data[15]=((shock_z_num_thresh[2]&0xffff0000)>>16);
			data[16]=(shock_z_num_thresh[2]&0x0000ffff);    //read 
			data[17]=((shock_z_num_thresh[3]&0xffff0000)>>16);
			data[18]=(shock_z_num_thresh[3]&0x0000ffff);    //read 
			data[19]=data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;

		case 201:
		
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x51){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}
			while(i_rcv == 0){}
			rxdata = rcvdata[i_rcv];
			i_rcv--;
			if(rxdata != 0x00){brxstat = 0;}

			data[0]=0x0007;
			data[1]=rms_sec_cur_x*100 & 0xffff;
			data[2]=rms_sec_cur_y*100 & 0xffff;
			data[3]=rms_sec_cur_z*100 & 0xffff;
			data[4]=shock_cnt_xy;
			data[5]=shock_cnt_z;

			data[6]=data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;		
		}
		/***************************** UART Receive loop end   ****************************/

		/************************* Flash Memory Saving loop start *************************/
		switch(sav_stat)
		{
		case 0:
			if(bSave)
			{

				bSaveFM=1;
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x01);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif
				mode=0x01;
				ia2k_sav_cnt=0;
				sav_stat=2;
			}
			break;
		case 2:
			if(bAdc==1)
			{
				bAdc=0;
				sav_stat=3;
			}
			break;
		case 3:
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x02);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif	

			sav_stat=4;
			break;
		case 4:
			acc2k_time=CacuSecondTime(year,month,day,hour,minute,second);
			bsav_data = 1;
			sav_stat=5;
			break;
			//=====================================================================
			//
			//*******************Saving character data start****************
			//
			//=======================================================================
		case 5:
			if(sav_addr0<0xfffffff)
			{
				gioA_tmp0=0xff;
				gioB_tmp0=0xef;        // Select F4
				sav_stat=6;
			}
			else
			{
				bSave=0;
				sav_stat=0;
			}
			break;
		case 6:		
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x10);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif	
				for(i=0;i<4;i++)
				{
					tmp_spi=0xff;
					while ( (tmp_spi & 0x01) == 0x01 )
					{
						switch ( i )
						{
						case 0: /* Flash F1 CS */
							gioSetBit( gioPORTA, 4, 0 );
							break;
						case 1: /* Flash F2 CS */
							gioSetBit( gioPORTA, 1, 0 );
							break;
						case 2: /* Flash F3 CS */
							gioSetBit( gioPORTB, 7, 0 );
							break;
						case 3: /* Flash F4 CS */
							gioSetBit( gioPORTB, 4, 0 );
							break;
						default:
							break;
						}
						spiTransmitAndReceiveData( spiREG2, &dataconfig1_t, 2, ReadStatusData, ReceiveStatusData );
						tmp_spi = ReceiveStatusData[1];
						/*  MysciSendByte(tmp); */
						MyGioSetPortA( 0xff );
						gioSetPort( gioPORTB, 0xff );
#ifdef watchdog
		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
#endif	
						/* wait(fff); */
					}
					tmp_spi = 0x00;
					while ( (tmp_spi&0x80) != 0x80 )
					{
						switch ( i )
						{
						case 0: /* Flash F1 CS */
							gioSetBit( gioPORTA, 4, 0 );
							break;
						case 1: /* Flash F2 CS */
							gioSetBit( gioPORTA, 1, 0 );
							break;
						case 2: /* Flash F3 CS */
							gioSetBit( gioPORTB, 7, 0 );
							break;
						case 3: /* Flash F4 CS */
							gioSetBit( gioPORTB, 4, 0 );
							break;
						default:
							break;
						}
						spiTransmitAndReceiveData( spiREG2, &dataconfig1_t, 2, ReadFlagStatusData, ReceiveFlagStatusData );
						tmp_spi = ReceiveFlagStatusData[1];
						/*  MysciSendByte(tmp); */
						MyGioSetPortA( 0xff );
						gioSetPort( gioPORTB, 0xff );	
#ifdef watchdog
		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
#endif			
					}
					
				}		
					
		
			MyGioSetPortA(0xff);
			gioSetPort(gioPORTB,0xff);
			MyGioSetPortA(gioA_tmp0);
			gioSetPort(gioPORTB,gioB_tmp0);
			spi2sendByte(0x06); 	  //write enable
			sav_stat = 7;
			break;
		case 7:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp0);
				gioSetPort(gioPORTB,gioB_tmp0);
				spi2sendByte(0xC5);  //WRITE Segment Choose Register
				sav_stat = 8;
			}
			break;
		case 8:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr0&0x03000000)>>24);   //Segment Choose
				sav_stat = 9;			
			}
			break;
		case 9:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp0);
				gioSetPort(gioPORTB,gioB_tmp0);
				spi2sendByte(0x06); 	  //write enable
				sav_stat = 10;
			}
			break;
		case 10:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp0);
				gioSetPort(gioPORTB,gioB_tmp0);
				spi2sendByte(0x02);  //WRITE
				sav_stat = 11;
			}
			break;
		case 11:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr0&0xff0000)>>16);  //address
				sav_stat = 12;
			}
			break;
		case 12:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr0&0x00ff00)>>8);
				sav_stat = 13;
			}
			break;
		case 13:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(sav_addr0&0x0000ff);
				sav_stat = 14;
			}
			break;
		case 14:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x18);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif	
				tmp_spi = spiREG2->BUF;
				spi2sendByte((acc2k_time>>24) & 0xff);					// Send time
				sav_stat = 15;
			}
			break;
		case 15:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((acc2k_time>>16) & 0xff);					// Send time
				sav_stat = 16;
			}
			break;
		case 16:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((acc2k_time>>8) & 0xff);					// Send time
				sav_stat = 17;
			}
			break;
		case 17:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(acc2k_time & 0xff);					// Send time
				sav_stat = 18;
			}
			break;
		case 18:		// Sending temperature information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				temp_write0 = temperature;
				spi2sendByte(shock_level_xy<<4+rms_level_xy);				// Send temperature
				sav_stat = 19;
			}
			break;
		case 19:		// Sending temperature information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(shock_level_z<<4+rms_level_z);					  // Send temperature
				sav_stat = 20;
			}
			break;
		case 20:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(rlevel);					// Send xylevel
				sav_stat = 21;
			}
			break;
		case 21:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((max_rms_xy*100>>8)&0xff);					// Send zlevel
				sav_stat = 22;
			}
			break;
		case 22:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(max_rms_xy*100&0xff);					// Send rlevel
				sav_stat = 23;
			}
			break;
		case 23:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_cur_bulk*100>>8)&0xff); 					// Send xy RMS
				sav_stat = 24;
			}
			break;
		case 24:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(rms_cur_bulk*100&0xff); 					      // Send xy RMS
				sav_stat = 25;
			}
			break;
		case 25:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((max_rms_z*100>>8)&0xff); 				// Send z RMS
				sav_stat = 26;
			}
			break;
		case 26:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(max_rms_z*100&0xff); 				// Send z RMS
				sav_stat = 27;
			}
			break;
		case 27:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_sec_cur_z*100>>8)&0xff); 					// Send z RMS
				sav_stat = 28;
			}
			break;
		case 28:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(rms_sec_cur_z*100&0xff); 					    // Send z RMS
				sav_stat = 29;
			}
			break;

		case 29:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				tmp_spi = avg_sec_cur_r*100;
				p = (char*)&tmp_spi;
				spi2sendByte((*p)&0xff);          // Send avg r
				sav_stat = 30;
			}
			break;
		case 30:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				p++;
				spi2sendByte(*p&0xff); 					    // Send avg r
				sav_stat = 31;
			}
			break;
		case 31:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				tmp_spi = peak_sec_cur_r*100;
				p = (char*)&tmp_spi;
				spi2sendByte(*p&0xff);         // Send peak r
				sav_stat = 32;
			}
			break;
		case 32:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				p++;
				spi2sendByte(*p&0xff); 					  // Send peak r
				sav_stat = 33;
			}
			break;
		case 33:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				tmp_spi = bottom_sec_cur_r*100;
				p = (char*)&tmp_spi;
				spi2sendByte(*p&0xff);       // Send bottom r
				sav_stat = 34;
			}
			break;
		case 34:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				p++;
				spi2sendByte(*p&0xff); 					  // Send bottom r
				sav_stat = 38;
			}
			break;
		case 38:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(BootCount&0xff); 					  // Send bootcount
				sav_stat = 39;
			}
			break;

		case 39:		//finish character data storage
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x49);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif							
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				sav_addr0+=0x20	;
				tmp_spi = 0x00 ;
				sav_stat = 40;
			}
			break;
		case 40:	          //  Inquire	Flag Status Register
			if((tmp_spi&0x80)==0x00)
			{
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x50);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif							
				gioSetBit(gioPORTB,4,0);		// Select F4
				spi2sendByte(0x70);
				sav_stat = 41;
			}
			else
			{
				sav_stat = 51;
			}
			break;
		case 41:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(0x00);
				sav_stat = 42;
			}
			break;
		case 42:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				sav_stat = 40;
			}
			break;
			//=====================================================================
			//
			//*******************Saving character data END****************
			//
			//=======================================================================
		case 51:		//
			raw_data_storage_cnt++;
			if (raw_data_storage_cnt % raw_data_storage_cycle == 1)
			{
				sav_stat = 0;
				break;
			}
			if(bsav_data==1)
			{
				if(sav_addr1<0x04000000)
				{
					gioA_tmp1 = 0xef;
					gioB_tmp1 = 0xff;   // Select F1
					sav_stat = 52;
				}
				else if(sav_addr1<0x08000000)
				{
					gioA_tmp1 = 0xfd;
					gioB_tmp1 = 0xff;		// Select F2
					sav_stat = 52;
				}
				else if(sav_addr1<0x0c000000)
				{
					gioA_tmp1 = 0xff;
					gioB_tmp1 = 0x7f;		// Select F3
					sav_stat = 52;
				}
				else if(sav_addr1<0x10000000)
				{
					gioA_tmp1 = 0xff;
					gioB_tmp1 = 0xef;		// Select F4
					sav_stat = 52;
				}
				else
				{
					sav_stat = 0;
				}
				bsav_data =0;
			}
			else
				sav_stat = 0;
			break;
		case 52:
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x52);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif			
			MyGioSetPortA(0xff);
			gioSetPort(gioPORTB,0xff);
			MyGioSetPortA(gioA_tmp1);
			gioSetPort(gioPORTB,gioB_tmp1);
			spi2sendByte(0x06);        //wirte enable
			sav_stat = 53;
		case 53:	//WRITE Segment Choose Register
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)	// write enabled
			{
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x53);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif				
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp1);
				gioSetPort(gioPORTB,gioB_tmp1);
				spi2sendByte(0xC5);       //WRITE Segment Choose Register
				sav_stat = 54;
			}
			break;
		case 54:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr1&0x03000000)>>24);   // Segment Choose
				sav_stat = 55;
			}
			break;
		case 55:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp1);
				gioSetPort(gioPORTB,gioB_tmp1);
				spi2sendByte(0x06);        //wirte enable
				sav_stat = 56;
			}
			break;
		case 56:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)	// write enabled
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp1);
				gioSetPort(gioPORTB,gioB_tmp1);
				spi2sendByte(0x02);
				sav_stat = 57;
			}
			break;
		case 57:		// send address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr1&0xff0000)>>16);   // Send address HSB
				sav_stat = 58;
			}
			break;
		case 58:		// send address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr1&0x00ff00)>>8);   // Send address MSB
				sav_stat = 59;
			}
			break;
		case 59:		// send address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(sav_addr1&0x0000ff);	 				// Send address LSB
				sav_stat = 60;
			}
			break;
		case 60:		// Finish address Sending, Start sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((acc2k_time>>24) & 0xff);   // Send Time HSB
				sav_stat = 61;
			}
			break;
		case 61:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((acc2k_time>>16) & 0xff);   // Send Time MSB
				sav_stat = 62;
			}
			break;
		case 62:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((acc2k_time>>8) & 0xff); 		// Send Time MSB
				sav_stat = 63;
			}
			break;
		case 63:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(acc2k_time & 0xff); 					// Send Time LSB
				sav_stat = 69;
			}
			break;
		case 69:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				(sav_addr1)+=0x100;
				sav_stat = 102;
				tmp_spi = 0x00;
			}
			break;
		case 102:	          //  Inquire	Flag Status Register
			if((tmp_spi&0x80)==0x00)
			{
				if((sav_addr1)<0x04000000)
				{
					gioSetBit(gioPORTA,4,0);		// Select F1
				}
				else if((sav_addr1)<0x08000000)
				{
					gioSetBit(gioPORTA,1,0);		// Select F2
				}
				else if((sav_addr1)<0x0c000000)
				{
					gioSetBit(gioPORTB,7,0);		// Select F3
				}
				else if((sav_addr1)<0x10000000)
				{
					gioSetBit(gioPORTB,4,0);		// Select F4
				}
				spi2sendByte(0x70);
				sav_stat = 103;
			}
			else
			{
				sav_stat = 70;
			}
			break;
		case 103:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(0x00);
				sav_stat = 104;
			}
			break;
		case 104:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				sav_stat = 102;
			}
			break;
		case 70:		//
			if(sav_addr1<0x04000000)
			{
				gioA_tmp1 = 0xef;
				gioB_tmp1 = 0xff;   // Select F1
				sav_stat = 71;
			}
			else if(sav_addr1<0x08000000)
			{
				gioA_tmp1 = 0xfd;
				gioB_tmp1 = 0xff;		// Select F2
				sav_stat = 71;
			}
			else if(sav_addr1<0x0c000000)
			{
				gioA_tmp1 = 0xff;
				gioB_tmp1 = 0x7f;		// Select F3
				sav_stat = 71;
			}
			else if(sav_addr1<0x10000000)
			{
				gioA_tmp1 = 0xff;
				gioB_tmp1 = 0xef;		// Select F4
				sav_stat = 71;
			}
			else
			{
				sav_stat = 71;
			}
			break;
		case 71:
			MyGioSetPortA(0xff);
			gioSetPort(gioPORTB,0xff);
			MyGioSetPortA(gioA_tmp1);
			gioSetPort(gioPORTB,gioB_tmp1);
			spi2sendByte(0x06);
			// Write enable to Flash
			sav_stat = 75;
			break;
		case 75:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)	//WRITE Segment Choose Register
			{
				tmp_spi = spiREG2->BUF;
				// disable CS
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp1);
				gioSetPort(gioPORTB,gioB_tmp1);
				spi2sendByte(0xC5);
				sav_stat = 201;
			}
			break;
		case 201:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr1&0x03000000)>>24);    //Segment Choose
				sav_stat = 76;
			}
			break;
		case 76:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp1);
				gioSetPort(gioPORTB,gioB_tmp1);
				spi2sendByte(0x06);     //Write enable
				sav_stat = 77;
			}
			break;
		case 77:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				// disable CS
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp1);
				gioSetPort(gioPORTB,gioB_tmp1);
				spi2sendByte(0x02);     //write
				sav_stat = 78;
			}
			break;
		case 78:		//  Sending address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr1&0xff0000)>>16);
				sav_stat = 79;
			}
			break;
		case 79:		//  Sending address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr1&0x00ff00)>>8);
				sav_stat = 80;
			}
			break;
		case 80:		//  Sending address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(sav_addr1&0x0000ff);
				ia2k_sav_cnt=0;
				sav_stat = 81;
			}
			break;
		case 81:		// Finish address Sending, Start sending data
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				if (adcflag==0)
				{
					data_send=accx_2k_buf1[ia2k_sav_cnt];
				}
				else
				{
					data_send=accx_2k_buf0[ia2k_sav_cnt];
				}
				tmp_spi = spiREG2->BUF;
				spi2sendByte((data_send>>8) & 0xff);
				sav_stat = 82;
			}
			break;
		case 82:		// Sending primitive data
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(data_send & 0xff);
				sav_stat = 83;
			}
			break;
		case 83:		// Sending primitive data
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				if (adcflag==0)
				{
					data_send=accy_2k_buf1[ia2k_sav_cnt];
				}
				else
					data_send=accy_2k_buf0[ia2k_sav_cnt];
				tmp_spi = spiREG2->BUF;
				spi2sendByte((data_send>>8) & 0xff);
				sav_stat = 84;
			}
			break;
		case 84:		// Sending primitive data
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(data_send & 0xff);
				sav_stat = 85;
			}
			break;
		case 85:		// Sending primitive data
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				if (adcflag==0)
				{
					data_send=accz_2k_buf1[ia2k_sav_cnt];
				}
				else
					data_send=accz_2k_buf0[ia2k_sav_cnt];
				tmp_spi = spiREG2->BUF;
				spi2sendByte((data_send>>8) & 0xff);
				sav_stat = 86;
			}
			break;
		case 86:		// Sending primitive data
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(data_send & 0xff);
				sav_stat = 87;
			}
			break;
		case 87:		// Sending primitive data
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				if (adcflag==0)
				{
					data_send=accr_2k_buf1[ia2k_sav_cnt];
				}
				else
					data_send=accr_2k_buf0[ia2k_sav_cnt];
				tmp_spi = spiREG2->BUF;
				spi2sendByte((data_send>>8)&0xff);
				sav_stat = 88;
			}
			break;
		case 88:		// Sending primitive data
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((data_send) & 0xff);
				sav_cnt++;
				ia2k_sav_cnt++;
				if(sav_cnt==32)
				{
					sav_stat = 89;
					sav_cnt=0;
				}
				else if (ia2k_sav_cnt==10000)
				{
					sav_stat = 89;
					sav_cnt=0;						
				}
				else
				{
					sav_stat = 81;
				}
			}
			break;
		case 89:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				tmp_spi = 0x00;
				sav_stat = 90;
			}
			break;
		case 90:	          //  Inquire	Flag Status Register
			if((tmp_spi&0x80)==0x00)
			{
				if((sav_addr1)<0x04000000)
				{
					gioSetBit(gioPORTA,4,0);		// Select F1
				}
				else if((sav_addr1)<0x08000000)
				{
					gioSetBit(gioPORTA,1,0);		// Select F2
				}
				else if((sav_addr1)<0x0c000000)
				{
					gioSetBit(gioPORTB,7,0);		// Select F3
				}
				else if((sav_addr1)<0x10000000)
				{
					gioSetBit(gioPORTB,4,0);		// Select F4
				}
				spi2sendByte(0x70);
				sav_stat = 91;
			}
			else
				sav_stat = 93;
			break;
		case 91:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(0x00);
				sav_stat = 92;
			}
			break;
		case 92:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				sav_stat = 90;
			}
			break;
		case 93:
			(sav_addr1)=(sav_addr1)+256	;
			if (ia2k_sav_cnt==10000)
			{
				sav_stat=0;
			}
			else
			{
				sav_stat=70;
			}
			break;
		}
		/************************* Flash Memory Saving loop end ***************************/
		/**************************** FMCL Saving loop end ********************************/
		switch(sav_fm_stat)
		{
		case 0:
			if(bSaveFM)
			{
				gioSetBit(gioPORTA,0,1);
				gioSetBit(gioPORTA,0,0);
				// Send write enalbe command
				spi1sendByte(0x06);
				bSaveFM = 0;
				sav_fm_stat = 3;
			}
			break;
		case 3:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				gioSetBit(gioPORTA,0,1);
				gioSetBit(gioPORTA,0,0);
				spi1sendByte(0x02);	//WRITE
				sav_fm_stat = 4;
			}
			break;
		case 4:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(0x10);   //send FEE address
				sav_fm_stat = 5;
			}
			break;
		case 5:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(0x00);   //send FEE address
				sav_fm_stat = 6;
			}
			break;
		case 6:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(gioA_tmp0);
				sav_fm_stat = 7;
			}
			break;
		case 7:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(gioB_tmp0);
				sav_fm_stat = 8;
			}
			break;
		case 8:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(gioA_tmp1);
				sav_fm_stat = 9;
			}
			break;
		case 9:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(gioB_tmp1);
				cksmspi1=(gioA_tmp0<<24)|(gioB_tmp0<<16)|(gioA_tmp1<<8)|(gioB_tmp1);
				sav_fm_stat = 10;
			}
			break;
		case 10:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(year);
				sav_fm_stat = 11;
			}
			break;
		case 11:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(month);
				sav_fm_stat = 12;
			}
			break;
		case 12:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(day);
				sav_fm_stat = 13;
			}
			break;
		case 13:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(hour);
				cksmspi1+=(year<<24)|(month<<16)|(day<<8)|(hour);
				sav_fm_stat = 14;
			}
			break;
		case 14:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(minute);
				sav_fm_stat = 15;
			}
			break;
		case 15:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(second);
				cksmspi1+=(minute<<8)|(second);
				sav_fm_stat = 16;
			}
			break;
		case 16:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr0>>24)&0xff);
				sav_fm_stat = 17;
			}
			break;
		case 17:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr0>>16)&0xff);
				sav_fm_stat = 18;
			}
			break;
		case 18:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr0>>8)&0xff);
				sav_fm_stat = 19;
			}
			break;
		case 19:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr0&0xff);
				cksmspi1+=sav_addr0;
				sav_fm_stat = 20;
			}
			break;
		case 20:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1>>24)&0xff);
				sav_fm_stat = 21;
			}
			break;
		case 21:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1>>16)&0xff);
				sav_fm_stat = 22;
			}
			break;
		case 22:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1>>8)&0xff);
				sav_fm_stat = 23;
			}
			break;
		case 23:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1&0xff);
				cksmspi1+=sav_addr1;
				sav_fm_stat = 24;
			}
			break;

		case 24:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((BootCount>>24)&0xff);
				sav_fm_stat = 25;
			}
			break;
		case 25:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((BootCount>>16)&0xff);
				sav_fm_stat = 26;
			}
			break;
		case 26:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((BootCount>>8)&0xff);
				sav_fm_stat = 27;
			}
			break;
		case 27:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(BootCount&0xff);
				cksmspi1+=BootCount;
				sav_fm_stat = 28;
			}
			break;



		case 28:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((cksmspi1>>24)&0xff);
				sav_fm_stat = 29;
			}
			break;
		case 29:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((cksmspi1>>16)&0xff);
				sav_fm_stat = 30;
			}
			break;			
		case 30:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((cksmspi1>>8)&0xff);
				sav_fm_stat = 31;
			}
			break;	
		case 31:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(cksmspi1&0xff);
				cksmspi1=0;
				sav_fm_stat = 32;
			}
			break;		
		case 32:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				gioSetBit(gioPORTA,0,1);
				sav_fm_stat = 0;
			}
			break;
		default:
			sav_fm_stat = 0;
			gioSetBit(gioPORTA,0,1);
			break;
		}
		/**************************** FMCL Saving loop end ********************************/
	}
	/* USER CODE END */
}

/* USER CODE BEGIN (4) */
void ADC_init(void)
{
	adcInit();
	adcStartConversion(adcREG1,adcGROUP1);
	adcStartConversion(adcREG2,adcGROUP1);
	adcEnableNotification(adcREG1,adcGROUP1);
}
void UART_init()
{
	sciInit();
	while((scilinREG->FLR & (uint32)SCI_TX_INT) == 0U)
	{
		;
	}
	gioSetBit(gioPORTA,2,0);  //sci receive mode
	sciEnableNotification(scilinREG,SCI_RX_INT);			//enable rx full interrput
}
int ReadFlashID(int i)
{
	uint16 TX_Data_Master[3] = {0x9f,0x00,0x00};
	uint16 RX_Data_Master[3] = {0};
	int ID = 0;
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
	switch(i)
	{
	case 0: // Flash F1 CS
		gioSetBit(gioPORTA,4,0);
		break;
	case 1: // Flash F2 CS
		gioSetBit(gioPORTA,1,0);
		break;
	case 2: // Flash F3 CS
		gioSetBit(gioPORTB,7,0);
		break;
	case 3: // Flash F4 CS
		gioSetBit(gioPORTB,4,0);
		break;
	default:
		break;
	}
	spiTransmitAndReceiveData(spiREG2, &dataconfig1_t, 3, TX_Data_Master, RX_Data_Master);
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
	ID = RX_Data_Master[1];
	gioSetBit(gioPORTA,2,1);
	sciDisableNotification(scilinREG,SCI_RX_INT);
	MysciSendByte(RX_Data_Master[0]);
	MysciSendByte(RX_Data_Master[1]);
	MysciSendByte(RX_Data_Master[2]);
	gioSetBit(gioPORTA,2,0);
	sciEnableNotification(scilinREG,SCI_RX_INT);
	return ID;
}

void SPI_init(void)
{
	int id = 0;
	int j = 0;
	spiInit();
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
	for(j=0; j<10; j++) 		// Get which Flashs are exiting
	{
		id = ReadFlashID(j);
		if(id == 0x20)
		{
			bF[j] = 1;
		}
	}
}
unsigned char ReadByte(uint32 addr)
{
	uint16 ReadByteInstruction[6]= {0};
	uint16 ReceiveByteData[6]= {0};
	unsigned char data; //falsh read byte data
	ReadByteInstruction[0]= 0x13;
	ReadByteInstruction[1]= (addr & 0x03000000)>>24;
	ReadByteInstruction[2]= (addr & 0x00ff0000)>>16;
	ReadByteInstruction[3]= (addr & 0x0000ff00)>>8;
	ReadByteInstruction[4]= addr & 0x000000ff;
	ReadByteInstruction[5]= 0x00;
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
	if(addr<0x04000000)
	{
		gioSetBit(gioPORTA,4,0);		// Select F1
	}
	else if(addr<0x08000000)
	{
		gioSetBit(gioPORTA,1,0);		// Select F2
	}
	else if(addr<0x0c000000)
	{
		gioSetBit(gioPORTB,7,0);		// Select F3
	}
	else if(addr<0x10000000)
	{
		gioSetBit(gioPORTB,4,0);		// Select F4
	}
	else
	{
		return 0xff;			// if select wrong chip, return 0xff
	}
	spiTransmitAndReceiveData(spiREG2, &dataconfig1_t, 6, ReadByteInstruction, ReceiveByteData);
	data=ReceiveByteData[5];
//	spiTransmitAndReceiveData(spiREG2, &dataconfig1_t, 5, ReadByteInstruction, ReceiveByteData);
//  data=ReceiveByteData[4];
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
//        #ifdef debug
//    		gioSetBit(gioPORTA,2,1);
//        sciDisableNotification(scilinREG,SCI_RX_INT);
//        MysciSendByte(ReceiveByteData[1]);
//        MysciSendByte(ReceiveByteData[2]);
//        MysciSendByte(ReceiveByteData[3]);
//        MysciSendByte(ReceiveByteData[4]);
//        MysciSendByte(ReceiveByteData[5]);
//	      gioSetBit(gioPORTA,2,0);
//	      sciEnableNotification(scilinREG,SCI_RX_INT);
//        #endif
//  wait(0xffff);
	return data;
}
void WriteByte(uint32 addr, int data)
{
	uint16 WREN[1]= {0x06};
	uint16 WriteByteInstruction[5]= {0};
	uint16 SegmentSelect[2]= {0};
	uint16 ReadFlagStatusReg[2]= {0x70,0x00};
	uint16 ReceiveByteData[2]= {0};
	SegmentSelect[0]= 0xC5;
	SegmentSelect[1]= (addr & 0x03000000)>>24;
	WriteByteInstruction[0]= 0x02;
	WriteByteInstruction[1]= (addr & 0x00ff0000)>>16;
	WriteByteInstruction[2]= (addr & 0x0000ff00)>>8;
	WriteByteInstruction[3]= addr & 0x000000ff;
	WriteByteInstruction[4]= data;
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
	if(addr<0x04000000)
	{
		gioSetBit(gioPORTA,4,0);		// Select F1
	}
	else if(addr<0x08000000)
	{
		gioSetBit(gioPORTA,1,0);		// Select F2
	}
	else if(addr<0x0c000000)
	{
		gioSetBit(gioPORTB,7,0);		// Select F3
	}
	else if(addr<0x10000000)
	{
		gioSetBit(gioPORTB,4,0);		// Select F4
	}
	spiTransmitData(spiREG2, &dataconfig1_t, 1, WREN );
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
	if(addr<0x04000000)
	{
		gioSetBit(gioPORTA,4,0);		// Select F1
	}
	else if(addr<0x08000000)
	{
		gioSetBit(gioPORTA,1,0);		// Select F2
	}
	else if(addr<0x0c000000)
	{
		gioSetBit(gioPORTB,7,0);		// Select F3
	}
	else if(addr<0x10000000)
	{
		gioSetBit(gioPORTB,4,0);		// Select F4
	}
	spiTransmitData(spiREG2, &dataconfig1_t, 2, SegmentSelect );
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
	if(addr<0x04000000)
	{
		gioSetBit(gioPORTA,4,0);		// Select F1
	}
	else if(addr<0x08000000)
	{
		gioSetBit(gioPORTA,1,0);		// Select F2
	}
	else if(addr<0x0c000000)
	{
		gioSetBit(gioPORTB,7,0);		// Select F3
	}
	else if(addr<0x10000000)
	{
		gioSetBit(gioPORTB,4,0);		// Select F4
	}
	spiTransmitData(spiREG2, &dataconfig1_t, 1, WREN );
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
	if(addr<0x04000000)
	{
		gioSetBit(gioPORTA,4,0);		// Select F1
	}
	else if(addr<0x08000000)
	{
		gioSetBit(gioPORTA,1,0);		// Select F2
	}
	else if(addr<0x0c000000)
	{
		gioSetBit(gioPORTB,7,0);		// Select F3
	}
	else if(addr<0x10000000)
	{
		gioSetBit(gioPORTB,4,0);		// Select F4
	}
	spiTransmitData(spiREG2, &dataconfig1_t, 5, WriteByteInstruction);
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
	tmp=0x00;
	while(tmp==0x00)
	{
		if(addr<0x04000000)
		{
			gioSetBit(gioPORTA,4,0);		// Select F1
		}
		else if(addr<0x08000000)
		{
			gioSetBit(gioPORTA,1,0);		// Select F2
		}
		else if(addr<0x0c000000)
		{
			gioSetBit(gioPORTB,7,0);		// Select F3
		}
		else if(addr<0x10000000)
		{
			gioSetBit(gioPORTB,4,0);		// Select F4
		}
		spiTransmitAndReceiveData(spiREG2, &dataconfig1_t, 2, ReadFlagStatusReg, ReceiveByteData);
		MyGioSetPortA(0xff);
		gioSetPort(gioPORTB,0xff);
		tmp=((ReceiveByteData[1]&0x80)>>7);
	}
}
void WREN (int x)
{
	uint16 WREN[1]= {0x06};
	switch(x)
	{
	case 0: // Flash F1 CS
		gioSetBit(gioPORTA,4,0);
		break;
	case 1: // Flash F2 CS
		gioSetBit(gioPORTA,1,0);
		break;
	case 2: // Flash F3 CS
		gioSetBit(gioPORTB,7,0);
		break;
	case 3: // Flash F4 CS
		gioSetBit(gioPORTB,4,0);
		break;
	default:
		break;
	}
	spiTransmitData(spiREG2, &dataconfig1_t, 1, WREN );
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
}
void StartAdd(uint16 addr)
{
	uint16 WriteByteInstruction[3]= {0};
	WriteByteInstruction[0]= 0x03;
	WriteByteInstruction[1]= (addr & 0x7f00)>>8;
	WriteByteInstruction[2]= (addr & 0x00ff);
	gioSetBit(gioPORTA,0,1);
	gioSetBit(gioPORTA,0,0);
	spiTransmitData(spiREG1, &dataconfig1_t, 3, WriteByteInstruction);
}
void spi1transmitByte(uint8 data)
{
	spiREG1->DAT1 =  ((uint32)(&dataconfig1_t)->DFSEL << 24U) |
	                 ((uint32)0u << 16U)|
	                 (0x04000000U)|
	                 (0u) |
	                 (uint32)(data);  //WRITE ENABLE
	while((spiREG1->FLG & 0x00000100U) != 0x00000100U) {} /* Wait */
}
void recovery(void)
{
	
//	sav_stat = 0;
//	bsav_level = 0;
//  bsav_data = 0;
	StartAdd(0x0000);
	spi1transmitByte(0x00);
	bSave=spiREG1->BUF;
	// recovery process!!
	cksmrecover=0;
	rcvspicksm=0;
	StartAdd(0x1000);		       // address 1000
	spi1transmitByte(0x00);
	gioA_tmp0=spiREG1->BUF;        // read  gioA_tmp
	spi1transmitByte(0x00);
	gioB_tmp0=spiREG1->BUF;        // read  gioB_tmp
	spi1transmitByte(0x00);
	gioA_tmp1=spiREG1->BUF;
	spi1transmitByte(0x00);
	gioB_tmp1=spiREG1->BUF;
	cksmrecover=(gioA_tmp0<<24)|(gioB_tmp0<<16)|(gioA_tmp1<<8)|(gioB_tmp1);
	spi1transmitByte(0x00);
	year=spiREG1->BUF;
	spi1transmitByte(0x00);
	month=spiREG1->BUF;
	spi1transmitByte(0x00);
	day=spiREG1->BUF;
	spi1transmitByte(0x00);
	hour=spiREG1->BUF;
	cksmrecover+=(year<<24)|(month<<16)|(day<<8)|(hour);
	spi1transmitByte(0x00);
	minute=spiREG1->BUF;
	spi1transmitByte(0x00);
	second=spiREG1->BUF;
	cksmrecover+=(minute<<8)|(second);
	sav_addr0=spiReadAddr();		        //addr 0x100a
	cksmrecover+=sav_addr0;
	sav_addr1=spiReadAddr();    //100e
	cksmrecover+=sav_addr1;
	BootCount=spiReadAddr();    //1012
	cksmrecover+=BootCount;   
	rcvspicksm=spiReadAddr();    //1016

	StartAdd(0x101a);
	ar=spiReadAddr();					
	br=spiReadAddr();				 	
	cr=spiReadAddr();           	
	dr=spiReadAddr();
	Bx=spiReadAddr();					
	Ax=spiReadAddr();				 	
	By=spiReadAddr();           	
	Ay=spiReadAddr();
	Bz=spiReadAddr();           	
	Az=spiReadAddr();

	shock_g_thresh = spiReadAddr();   // 1042
	shock_xy_num_thresh[0]=spiReadAddr();
	shock_xy_num_thresh[1]=spiReadAddr();				  	
	shock_xy_num_thresh[2]=spiReadAddr();				  	
	shock_xy_num_thresh[3]=spiReadAddr();
	shock_z_num_thresh[0]=spiReadAddr();				    
	shock_z_num_thresh[1]=spiReadAddr();				    
	shock_z_num_thresh[2]=spiReadAddr();				    
	shock_z_num_thresh[3]=spiReadAddr();


	gioSetBit(gioPORTA,0,1);

	BootCount += 1;
	
	wait(0x1000000);  //time for the initialization of i2c
}
int abs(int x)
{
	if(x<0)
		return -x;
	else
		return x;
}
float absf(float x)
{
	if(x<0)
		return -x;
	else
		return x;
}
adcData_t adc_data1[3];
adcData_t adc_data2[3];
void adcNotification( adcBASE_t *adc, uint32 group )
{
	unsigned static int delay=0;
	adcGetData( adcREG1, adcGROUP1, &adc_data1[0] );
	adcGetData( adcREG2, adcGROUP1, &adc_data2[0] );
	accx_f0		= adc_data1[0].value;
	accy_f0		= adc_data1[1].value;
	accz_f0		= adc_data1[2].value;
	accr_f0		= adc_data2[0].value;
	rtemperature_f0 = adc_data2[1].value;
	accx_sum		+= accx_f0;
	accy_sum		+= accy_f0;
	accz_sum		+= accz_f0;
	accr_sum		+= accr_f0;
	rtemperature_sum	+= rtemperature_f0;
	iTick++;
	if ( iTick == 8 ) /* filter to alter the sample frequency to 1000Hz */
	{
		ia2k += 1;
		accx_send	= accx_sum / 8;
		accy_send	= accy_sum / 8;
		accz_send	= accz_sum / 8;
		/*				accrfloatsend= accr_sum*(2.67695-(0.0000791*rtemperature))/8+(0.1394408*rtemperature)-5420.99; */
		accr_send	= accr_sum / 8;
		rtemperature	= rtemperature_sum / 8;
		accx_sum		= 0;
		accy_sum		= 0;
		accz_sum		= 0;
		accr_sum		= 0;
		rtemperature_sum	= 0;
		iTick		= 0;
		bStartI2c	= 1;
		bstart_tx	= 1;


		if ( adcflag == 0 )
		{
			accx_2k_buf0[ia2k]	= accx_send;
			accy_2k_buf0[ia2k]	= accy_send;
			accz_2k_buf0[ia2k]	= accz_send;
			accr_2k_buf0[ia2k]	= accr_send;
		}
		else
		{
			accx_2k_buf1[ia2k]	= accx_send;
			accy_2k_buf1[ia2k]	= accy_send;
			accz_2k_buf1[ia2k]	= accz_send;
			accr_2k_buf1[ia2k]	= accr_send;
		}



		avg_sec_x_float_100ms += accx_send;
		avg_sec_y_float_100ms += accy_send;
		avg_sec_z_float_100ms += accz_send;
		square_sec_x_float_100ms += accx_send * accx_send;
		square_sec_y_float_100ms += accy_send * accy_send;
		square_sec_z_float_100ms += accz_send * accz_send;

		avg_sec_x_float += accx_send;
		avg_sec_y_float += accy_send;
		avg_sec_z_float += accz_send;


		if(peak_sec_x < accx_send)
		{
			// Calculate peak value, accx
			peak_sec_x = accx_send;
		}
		if(peak_sec_y < accy_send)
		{
			// Calculate peak value, accy
			peak_sec_y = accy_send;
		}
		if(peak_sec_z < accz_send)
		{
			// Calculate peak value, accz
			peak_sec_z = accz_send;
		}

		arfloat = *((float*)&ar);
		brfloat = *((float*)&br);
		crfloat = *((float*)&cr);
		drfloat = *((float*)&dr);
		accrfloatsend = (brfloat + (arfloat * rtemperature) ) * ( (crfloat * rtemperature) + drfloat + accr_send) / 6;
		if ( accrfloatsend < 0 )
		{
			minusR++;
		}                                               /* count when r is negative */
		avg_sec_r_float += accrfloatsend;
		/* Calculate average value, accr */
		if ( peak_sec_r_float < accrfloatsend )         /* Calculate peak value, accr */
		{
			peak_sec_r_float	= accrfloatsend;
			peak_sec_r		= accrfloatsend;
		}
		if ( bottom_sec_r_float > accrfloatsend )       /* Calculate bottom value, accr */
		{
			bottom_sec_r_float	= accrfloatsend;
			bottom_sec_r		= accrfloatsend;
		}
	}
	if ( ia2k == 64 )
	{
		  bAdc = 0;
  	}
	if ( ia2k % 100 == 0)  // 100ms
	{
		Bx_float = *((float*)&Bx);
		Ax_float = *((float*)&Ax);
		By_float = *((float*)&By);
		Ay_float = *((float*)&Ay);
		Bz_float = *((float*)&Bz);
		Az_float = *((float*)&Az);	

		avg_sec_x_float_100ms = avg_sec_x_float_100ms/100;
		avg_sec_y_float_100ms = avg_sec_y_float_100ms/100;
		avg_sec_z_float_100ms = avg_sec_z_float_100ms/100;

		rms_buf_100ms_x[ia_rms] = Bx_float*(sqrt(square_sec_x_float_100ms/100) - avg_sec_x_float_100ms) + Ax_float;
		rms_buf_100ms_y[ia_rms] = By_float*(sqrt(square_sec_y_float_100ms/100) - avg_sec_y_float_100ms) + Ay_float;
		rms_buf_100ms_z[ia_rms] = Bz_float*(sqrt(square_sec_z_float_100ms/100) - avg_sec_z_float_100ms) + Az_float;

		if ((rms_buf_100ms_x[ia_rms] + rms_buf_100ms_y[ia_rms])/2 > shock_g_thresh)
		{
			shock_cnt_xy++;
		}
		if (rms_buf_100ms_z[ia_rms] > shock_g_thresh)
		{
			shock_cnt_z++;
		}

		rms_sec_x += rms_buf_100ms_x[ia_rms];
		rms_sec_y += rms_buf_100ms_y[ia_rms];		
		rms_sec_z += rms_buf_100ms_z[ia_rms];

		if(peak_rms_xy < rms_buf_100ms_x[ia_rms])
			peak_rms_xy = rms_buf_100ms_x[ia_rms];
		if(peak_rms_xy < rms_buf_100ms_y[ia_rms])
			peak_rms_xy = rms_buf_100ms_y[ia_rms];
		if(peak_rms_z < rms_buf_100ms_z[ia_rms])
			peak_rms_z = rms_buf_100ms_z[ia_rms];

		avg_sec_x_float_100ms = 0;
		avg_sec_y_float_100ms = 0;
		avg_sec_z_float_100ms = 0;
		ia_rms += 1;
	}
	if ( ia_rms == 100 )  //10s
	{
		ia2k = 0;
		ia_rms = 0;
		peak_sec_cur_x = peak_sec_x;
		peak_sec_cur_y = peak_sec_y;
		peak_sec_cur_z = peak_sec_z;

		avg_sec_cur_x	= avg_sec_x_float / 10000; /* normalization */
		avg_sec_cur_y	= avg_sec_y_float / 10000;
		avg_sec_cur_z	= avg_sec_z_float / 10000;
		avg_sec_cur_r	= avg_sec_r_float / 10000;
		avg_sec_cur_r_float  =  avg_sec_r_float / 10000;
		peak_sec_cur_r_float	= peak_sec_r_float;
		bottom_sec_cur_r_float	= bottom_sec_r_float;
		peak_sec_cur_r		= (int)peak_sec_r_float;
		bottom_sec_cur_r	= (int)bottom_sec_r_float;

		rms_sec_cur_z = rms_sec_z/100;
		rms_sec_cur_x = rms_sec_x/100;
		rms_sec_cur_y = rms_sec_y/100;
		rms_cur_bulk = (rms_sec_cur_x + rms_sec_cur_y)/2;
		max_rms_xy = peak_rms_xy;
		max_rms_z = peak_rms_z;

		rms_sec_x = 0;
		rms_sec_y = 0;
		rms_sec_z = 0;
		peak_sec_x = 0;
		peak_sec_y = 0;
		peak_sec_z = 0;
		avg_sec_x_float		= 0;
		avg_sec_y_float		= 0;
		avg_sec_z_float		= 0;
		avg_sec_r_float		= 0;
		peak_sec_r_float	= 0;
		bottom_sec_r_float	= 10000;
		peak_rms_xy = 0;
		peak_rms_z = 0;
		delay++;

		// calc_rms_level_xy
		rms_level_xy = 0;
		while (rms_level_xy<15) {
			if (max_rms_xy > rms_xy_thresh[i])
				rms_level_xy++;
			else
			    break;
		}

		// calc_rms_level_z
		rms_level_z = 0;
		while (rms_level_z<15) {
			if (max_rms_xy > rms_z_thresh[i])
				i++;
			else
			    break;
		}

		// calc_shock_level_xy
		shock_level_xy = 0;
		while (shock_level_xy<4) {
			if (shock_cnt_xy > shock_xy_num_thresh[i])
				shock_level_xy++;
			else
			    break;
		}

		// calc_shock_level_z
		shock_level_z = 0;
		while (shock_level_z<4) {
			if (shock_cnt_z > shock_z_num_thresh[i])
				shock_level_z++;
			else
			    break;
		}



		// calc_ss_level()
		if(avg_sec_cur_r_float < 0.1)
		{
			s1=0;
			s2=0;
		}    //to set a zero zone for avg_sec_cur_r_float
		else
		{
			s1=absf((peak_sec_cur_r_float-bottom_sec_cur_r_float)/(2*avg_sec_cur_r_float));
			s2=(minusR+0.01)/10000;
		}
		if (s1<0.05)    //calculate rlevel
		{
			rlevel=0x00;
		}
		else if (s1<0.1)
		{
			rlevel=0x01;
		}
		else if (s1<0.2)
		{
			rlevel=0x02;
		}
		else if (s1<0.4)
		{
			rlevel=0x03;
		}
		else if (s1<0.8)
		{
			rlevel=0x04;
		}
		else if (s1<1.2)
		{
			rlevel=0x05;
		}
		else 		
		{
			rlevel=0x06;
		}		
		if (s2>0.1)
		{
			rlevel=0x07;
		}

		minusR = 0;
		// character data to be saved: 
		// shock_level_xy, rms_level_xy, shock_level_z, rms_level_z, rlevel,
		// max_rms_xy, avg_rms_xy, max_rms_z, avg_rms_z,
		// avg_sec_cur_r, peak_sec_cur_r, bottom_sec_cur_r, BootCount




		if(delay>1)     // adc delay when power-up
		{
			bAdc	= 1;
		}
		if ( adcflag == 1 )
			adcflag = 0;
		else
			adcflag = 1;
	}
}

void sciNotification(sciBASE_t *sci, uint32 flags)
{
	i_rcv++;
	rcvdata[i_rcv] = sciReceiveByte(scilinREG);
}
uint8 ReadFMByte(uint16 addr)
{
	uint16 ReadByteInstruction[4];
	uint16 ReceiveByteData[4]= {0};
	uint8 data; //falsh read byte data
	ReadByteInstruction[0]= 0x03;
	ReadByteInstruction[1]= (addr & 0x7f00)>>8;
	ReadByteInstruction[2]= (addr & 0x00ff);
	ReadByteInstruction[3]= 0x00;
	gioSetBit(gioPORTA,0,1);
	gioSetBit(gioPORTA,0,0);
	spiTransmitAndReceiveData(spiREG1, &dataconfig1_t, 4, ReadByteInstruction, ReceiveByteData);
	data=ReceiveByteData[3];
	gioSetBit(gioPORTA,0,1);
	return data;
}
void ReadFMStatus(void)
{
	uint16 TX_Data_Master[2] = { 0x05, 0x00};
	uint16 RX_Data_Master[2] = { 0 };
	gioSetBit(gioPORTA,0,1);
	gioSetBit(gioPORTA,0,0);
	spiTransmitAndReceiveData(spiREG1, &dataconfig1_t, 2, TX_Data_Master, RX_Data_Master);
	gioSetBit(gioPORTA,0,1);
}
void FMWREN (void)
{
	uint16 WREN[1]= {0x06};
	gioSetBit(gioPORTA,0,1);
	gioSetBit(gioPORTA,0,0);
	spiTransmitData(spiREG1, &dataconfig1_t, 1, WREN );
	gioSetBit(gioPORTA,0,1);
}
void WriteFMByte(uint16 addr, uint8 data)
{
	uint16 WriteByteInstruction[4]= {0};
	WriteByteInstruction[0]= 0x02;
	WriteByteInstruction[1]= (addr & 0x7f00)>>8;
	WriteByteInstruction[2]= (addr & 0x00ff);
	WriteByteInstruction[3]= data;
	gioSetBit(gioPORTA,0,1);
	gioSetBit(gioPORTA,0,0);
	spiTransmitData(spiREG1, &dataconfig1_t, 4, WriteByteInstruction);
	gioSetBit(gioPORTA,0,1);
}
void MyGioSetPortA( uint8 value )
{
	if ( gioGetBit( gioPORTA, 0 ) == 0 )
		value = value & 0xfe;
	else
		value = value;
	if ( gioGetBit( gioPORTA, 2 ) == 0 )
		value = value & 0xfb;
	else
		value = value;
	gioSetPort( gioPORTA, value );
}
void read_LB(uint32 addr_read)
{
	int i=0;
	uint16 tmp1,tmp2=0;
	for(i=0; i<128; i++)
	{
		tmp1 = ReadByte(addr_read);
		MysciSendByte(tmp1);
		tmp2 = ReadByte(addr_read+0x01);
		MysciSendByte(tmp2);
		check^=(tmp1<<8|tmp2);
		addr_read+=0x02;
	}
}
void WriteFM4Bytes(uint32 x,uint32 firstaddr)
{
	uint8 data;
	data=((x&0xff000000)>>24);
	FMWREN ();
	WriteFMByte(firstaddr, data);
	firstaddr++;
	data=((x&0x00ff0000)>>16);
	FMWREN ();
	WriteFMByte(firstaddr, data);
	firstaddr++;
	data=((x&0x0000ff00)>>8);
	FMWREN ();
	WriteFMByte(firstaddr, data);
	firstaddr++;
	data=((x&0x000000ff));
	FMWREN ();
	WriteFMByte(firstaddr, data);
}
void bulk_erase_character()
{
	uint16	WREN[1]			= { 0x06 };
	uint16	sector_erase[4];
	uint16  SegmentSelect[2]= {0xc5, 0x03};
	uint16	ReadStatusData[2]	= { 0x05, 0x00 };
    uint16  ReadFlagStatusData[2] = {0x70, 0x00};
	uint16	ReceiveStatusData[2]	= { 0 };
	uint16	ReceiveFlagStatusData[2]	= { 0 };
	uint8   tmp1, tmp2;
	uint32  RecoverAddress= 0xf6dce00;
	sector_erase[0]=0xd8;
	while(RecoverAddress<0x10000000)
	{

		sector_erase[1]=((RecoverAddress&0xff0000)>>16);
		sector_erase[2]=((RecoverAddress&0x00ff00)>>8);
		sector_erase[3]=((RecoverAddress&0x0000ff)>>16);
		gioSetPort( gioPORTB, 0xff );
		gioSetBit( gioPORTB, 4, 0 );
		spiTransmitData( spiREG2, &dataconfig1_t, 1, WREN );
		gioSetPort( gioPORTB, 0xff );
		gioSetBit( gioPORTB, 4, 0 );
		spiTransmitData( spiREG2, &dataconfig1_t, 2, SegmentSelect);
		gioSetPort( gioPORTB, 0xff );
		gioSetBit( gioPORTB, 4, 0 );
		spiTransmitData( spiREG2, &dataconfig1_t, 1, WREN );
		gioSetPort( gioPORTB, 0xff );
		gioSetBit( gioPORTB, 4, 0 );
		spiTransmitData( spiREG2, &dataconfig1_t, 4, sector_erase );
		gioSetPort( gioPORTB, 0xff );
		RecoverAddress+=64*1024;	            //1 sector= 64kB
		tmp1=0xff;
		while ( (tmp1 & 0x01) == 0x01 )
		{
			gioSetBit( gioPORTB, 4, 0 );
			spiTransmitAndReceiveData( spiREG2, &dataconfig1_t, 2, ReadStatusData, ReceiveStatusData );
			tmp1 = ReceiveStatusData[1];
			/*  MysciSendByte(tmp); */
			MyGioSetPortA( 0xff );
			gioSetPort( gioPORTB, 0xff );
#ifdef watchdog
		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
#endif	
			/* wait(fff); */
		}
		tmp2 = 0x00;
		while ( (tmp2&0x80) != 0x80 )
		{
			gioSetBit( gioPORTB, 4, 0 );
			spiTransmitAndReceiveData( spiREG2, &dataconfig1_t, 2, ReadFlagStatusData, ReceiveFlagStatusData );
			tmp2 = ReceiveFlagStatusData[1];
			/*  MysciSendByte(tmp); */
			MyGioSetPortA( 0xff );
			gioSetPort( gioPORTB, 0xff );	
#ifdef watchdog
		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
#endif			
		}
//#ifdef debug
//	sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
//	gioSetBit(gioPORTA,2,1);
//	MysciSendByte(tmp1);  //for debug
//	MysciSendByte(tmp2);
//	gioSetBit(gioPORTA,2,0);
//	sciEnableNotification(scilinREG,SCI_RX_INT);
//#endif

//		gioSetPort( gioPORTB, 0xff );	
			/* wait(fff); */
	}
	sav_addr0 = 0xf6dce00;
	WriteFM4Bytes(0xf6dce00,0x100a);	      //FM storage
	bSaveFM=1;
}
void bulk_erase_primitive(void)
{
	uint8	  i	= 0;
	uint16	WREN[1]			= { 0x06 };
	uint16	bulk_erase[4]		= {0xc4,0x00,0x01,0x00};
	uint16  SegmentSelect[2] = {0xc5,0x00};
	uint16	ReadStatusData[2]	= { 0x05, 0x00 };
	uint16	ReceiveStatusData[2]	= { 0 };
  uint16  ReadFlagStatusData[2] = {0x70, 0x00};
	uint16	ReceiveFlagStatusData[2]	= { 0 };
	uint8   tmp1,tmp2;
	check = 0;
	MyGioSetPortA( 0xff );
	gioSetPort( gioPORTB, 0xff );
	for ( i = 0; i < 4; i++ )
	{
		SegmentSelect[1]=0x00;	 //erase upper segment
		switch ( i )
		{
		case 0: /* Flash F1 CS */
			gioSetBit( gioPORTA, 4, 0 );
			break;
		case 1: /* Flash F2 CS */
			gioSetBit( gioPORTA, 1, 0 );
			break;
		case 2: /* Flash F3 CS */
			gioSetBit( gioPORTB, 7, 0 );
			break;
		case 3: /* Flash F4 CS */
			gioSetBit( gioPORTB, 4, 0 );
			break;
		default:
			break;
		}
		spiTransmitData( spiREG2, &dataconfig1_t, 1, WREN );
		MyGioSetPortA( 0xff );
		gioSetPort( gioPORTB, 0xff );
		switch ( i )
		{
		case 0: /* Flash F1 CS */
			gioSetBit( gioPORTA, 4, 0 );
			break;
		case 1: /* Flash F2 CS */
			gioSetBit( gioPORTA, 1, 0 );
			break;
		case 2: /* Flash F3 CS */
			gioSetBit( gioPORTB, 7, 0 );
			break;
		case 3: /* Flash F4 CS */
			gioSetBit( gioPORTB, 4, 0 );
			break;
		default:
			break;
		}
		spiTransmitData( spiREG2, &dataconfig1_t, 2, SegmentSelect);
		MyGioSetPortA( 0xff );
		gioSetPort( gioPORTB, 0xff );
		switch ( i )
		{
		case 0: /* Flash F1 CS */
			gioSetBit( gioPORTA, 4, 0 );
			break;
		case 1: /* Flash F2 CS */
			gioSetBit( gioPORTA, 1, 0 );
			break;
		case 2: /* Flash F3 CS */
			gioSetBit( gioPORTB, 7, 0 );
			break;
		case 3: /* Flash F4 CS */
			gioSetBit( gioPORTB, 4, 0 );
			break;
		default:
			break;
		}
		spiTransmitData( spiREG2, &dataconfig1_t, 1, WREN );
		MyGioSetPortA( 0xff );
		gioSetPort( gioPORTB, 0xff );
		switch ( i )
		{
		case 0: /* Flash F1 CS */
			gioSetBit( gioPORTA, 4, 0 );
			break;
		case 1: /* Flash F2 CS */
			gioSetBit( gioPORTA, 1, 0 );
			break;
		case 2: /* Flash F3 CS */
			gioSetBit( gioPORTB, 7, 0 );
			break;
		case 3: /* Flash F4 CS */
			gioSetBit( gioPORTB, 4, 0 );
			break;
		default:
			break;
		}
		spiTransmitData( spiREG2, &dataconfig1_t, 4, bulk_erase );
		MyGioSetPortA( 0xff );
		gioSetPort( gioPORTB, 0xff );
  }

	for(i=0;i<4;i++)
	{
		tmp1=0xff;
		while ( (tmp1 & 0x01) == 0x01 )
		{
			switch ( i )
			{
			case 0: /* Flash F1 CS */
				gioSetBit( gioPORTA, 4, 0 );
				break;
			case 1: /* Flash F2 CS */
				gioSetBit( gioPORTA, 1, 0 );
				break;
			case 2: /* Flash F3 CS */
				gioSetBit( gioPORTB, 7, 0 );
				break;
			case 3: /* Flash F4 CS */
				gioSetBit( gioPORTB, 4, 0 );
				break;
			default:
				break;
			}
			spiTransmitAndReceiveData( spiREG2, &dataconfig1_t, 2, ReadStatusData, ReceiveStatusData );
			tmp1 = ReceiveStatusData[1];
			/*  MysciSendByte(tmp); */
			MyGioSetPortA( 0xff );
			gioSetPort( gioPORTB, 0xff );
#ifdef watchdog
		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
#endif	
			/* wait(fff); */
		}
		tmp2 = 0x00;
		while ( (tmp2&0x80) != 0x80 )
		{
			switch ( i )
			{
			case 0: /* Flash F1 CS */
				gioSetBit( gioPORTA, 4, 0 );
				break;
			case 1: /* Flash F2 CS */
				gioSetBit( gioPORTA, 1, 0 );
				break;
			case 2: /* Flash F3 CS */
				gioSetBit( gioPORTB, 7, 0 );
				break;
			case 3: /* Flash F4 CS */
				gioSetBit( gioPORTB, 4, 0 );
				break;
			default:
				break;
			}
			spiTransmitAndReceiveData( spiREG2, &dataconfig1_t, 2, ReadFlagStatusData, ReceiveFlagStatusData );
			tmp2 = ReceiveFlagStatusData[1];
			/*  MysciSendByte(tmp); */
			MyGioSetPortA( 0xff );
			gioSetPort( gioPORTB, 0xff );	
#ifdef watchdog
		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
#endif			
		}
		
  }


	for ( i = 0; i < 4; i++ )
	{		
		SegmentSelect[1]=0x02;				    //change to upper segment
		switch ( i )
		{
		case 0: /* Flash F1 CS */
			gioSetBit( gioPORTA, 4, 0 );
			break;
		case 1: /* Flash F2 CS */
			gioSetBit( gioPORTA, 1, 0 );
			break;
		case 2: /* Flash F3 CS */
			gioSetBit( gioPORTB, 7, 0 );
			break;
		case 3: /* Flash F4 CS */
			gioSetBit( gioPORTB, 4, 0 );
			break;
		default:
			break;
		}
		spiTransmitData( spiREG2, &dataconfig1_t, 1, WREN );
		MyGioSetPortA( 0xff );
		gioSetPort( gioPORTB, 0xff );
		switch ( i )
		{
		case 0: /* Flash F1 CS */
			gioSetBit( gioPORTA, 4, 0 );
			break;
		case 1: /* Flash F2 CS */
			gioSetBit( gioPORTA, 1, 0 );
			break;
		case 2: /* Flash F3 CS */
			gioSetBit( gioPORTB, 7, 0 );
			break;
		case 3: /* Flash F4 CS */
			gioSetBit( gioPORTB, 4, 0 );
			break;
		default:
			break;
		}
		spiTransmitData( spiREG2, &dataconfig1_t, 2, SegmentSelect);
		MyGioSetPortA( 0xff );
		gioSetPort( gioPORTB, 0xff );
		switch ( i )
		{
		case 0: /* Flash F1 CS */
			gioSetBit( gioPORTA, 4, 0 );
			break;
		case 1: /* Flash F2 CS */
			gioSetBit( gioPORTA, 1, 0 );
			break;
		case 2: /* Flash F3 CS */
			gioSetBit( gioPORTB, 7, 0 );
			break;
		case 3: /* Flash F4 CS */
			gioSetBit( gioPORTB, 4, 0 );
			break;
		default:
			break;
		}
		spiTransmitData( spiREG2, &dataconfig1_t, 1, WREN );
		MyGioSetPortA( 0xff );
		gioSetPort( gioPORTB, 0xff );
		switch ( i )
		{
		case 0: /* Flash F1 CS */
			gioSetBit( gioPORTA, 4, 0 );
			break;
		case 1: /* Flash F2 CS */
			gioSetBit( gioPORTA, 1, 0 );
			break;
		case 2: /* Flash F3 CS */
			gioSetBit( gioPORTB, 7, 0 );
			break;
		case 3: /* Flash F4 CS */
			gioSetBit( gioPORTB, 4, 0 );
			break;
		default:
			break;
		}
		spiTransmitData( spiREG2, &dataconfig1_t, 4, bulk_erase );
		MyGioSetPortA( 0xff );
		gioSetPort( gioPORTB, 0xff );	

	}
	for(i=0;i<4;i++)
	{
		tmp1=0xff;
		while ( (tmp1 & 0x01) == 0x01 )
		{
			switch ( i )
			{
			case 0: /* Flash F1 CS */
				gioSetBit( gioPORTA, 4, 0 );
				break;
			case 1: /* Flash F2 CS */
				gioSetBit( gioPORTA, 1, 0 );
				break;
			case 2: /* Flash F3 CS */
				gioSetBit( gioPORTB, 7, 0 );
				break;
			case 3: /* Flash F4 CS */
				gioSetBit( gioPORTB, 4, 0 );
				break;
			default:
				break;
			}
			spiTransmitAndReceiveData( spiREG2, &dataconfig1_t, 2, ReadStatusData, ReceiveStatusData );
			tmp1 = ReceiveStatusData[1];
			/*  MysciSendByte(tmp); */
			MyGioSetPortA( 0xff );
			gioSetPort( gioPORTB, 0xff );
#ifdef watchdog
		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
#endif	
			/* wait(fff); */
		}
		tmp2 = 0x00;
		while ( (tmp2&0x80) != 0x80 )
		{
			switch ( i )
			{
			case 0: /* Flash F1 CS */
				gioSetBit( gioPORTA, 4, 0 );
				break;
			case 1: /* Flash F2 CS */
				gioSetBit( gioPORTA, 1, 0 );
				break;
			case 2: /* Flash F3 CS */
				gioSetBit( gioPORTB, 7, 0 );
				break;
			case 3: /* Flash F4 CS */
				gioSetBit( gioPORTB, 4, 0 );
				break;
			default:
				break;
			}
			spiTransmitAndReceiveData( spiREG2, &dataconfig1_t, 2, ReadFlagStatusData, ReceiveFlagStatusData );
			tmp2 = ReceiveFlagStatusData[1];
			/*  MysciSendByte(tmp); */
			MyGioSetPortA( 0xff );
			gioSetPort( gioPORTB, 0xff );	
#ifdef watchdog
		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
#endif			
		}
		
  }
	sav_addr1 = 0x0000000;
	bSaveFM=1;		
}
void spi2sendByte(uint8 x)
{
	//tmp_spi = spiREG2->BUF;
	spiREG2->DAT1 =  ((uint32)(&dataconfig1_t)->DFSEL << 24U) |
	                 ((uint32)0u << 16U)|
	                 (0x04000000U)|
	                 (0u) |
	                 (uint32)(x);
}
void spi1sendByte(uint8 x)
{
	tmp_spi = spiREG2->BUF;
	spiREG1->DAT1 =  ((uint32)(&dataconfig1_t)->DFSEL << 24U) |
	                 ((uint32)0u << 16U)|
	                 (0x04000000U)|
	                 (0u) |
	                 (uint32)(x);
}
void MysciSendByte(uint8 byte)
{
	while ((scilinREG->FLR & (uint32)0x800) == 0U)
	{
	} /* Wait for TX empty */
	scilinREG->TD = byte;
	while ((scilinREG->FLR & (uint32)0x800) == 0U)
	{
	} /* Wait */
}
void MysciSend2Bytes (uint16 x)
{
	uint8 y,z;
	y=((x&0xff00)>>8);
	z=(x&0x00ff);
	while ((scilinREG->FLR & (uint32)0x800) == 0U)
	{
	} /* Wait for TX empty */
	scilinREG->TD = y;
	while ((scilinREG->FLR & (uint32)0x800) == 0U)
	{
	} /* Wait */
	while ((scilinREG->FLR & (uint32)0x800) == 0U)
	{
	} /* Wait for TX empty */
	scilinREG->TD = z;
	while ((scilinREG->FLR & (uint32)0x800) == 0U)
	{
	} /* Wait */
}
void MysciSend(uint16 * data)
{
	wait(40000);   //LH delay
	gioSetBit(gioPORTA,2,1);
	sciDisableNotification(scilinREG,SCI_RX_INT);
//  wait(0x5ff);
	datalength=*data+1;
	while(datalength--)
	{
		MysciSend2Bytes(*data);
		data++;
	}
	gioSetBit(gioPORTA,2,0);
	sciEnableNotification(scilinREG,SCI_RX_INT);
}
uint32 CacuSecondTime(int t1,int t2,int t3,int t4,int t5,int t6)
{
	int DayDelay=0;
	int TimeDelay=0;
	int Year=1970;
	int Month=1;
	t1=((uint8)(t1/16))*10+(t1&0x0f);       //decimal to binary
	t2=((uint8)(t2/16))*10+(t2&0x0f);
	t3=((uint8)(t3/16))*10+(t3&0x0f);
	t4=((uint8)(t4/16))*10+(t4&0x0f)-8;
	t5=((uint8)(t5/16))*10+(t5&0x0f);
	t6=((uint8)(t6/16))*10+(t6&0x0f);
	while (Year<(t1+1970))
	{
		if ((Year%4==0&&Year%100!=0)||Year%400==0)  //leap year
			DayDelay += 366;
		else
			DayDelay += 365;
		Year++;
	}
	while(Month<t2)
	{
		if (Month == 1 || Month == 3 || Month == 5
		        || Month == 7 || Month == 8 || Month == 10 || Month == 12)
		{
			DayDelay+=31;
		}
		else if (Month == 2)
		{
			if ((Year%4==0&&Year%100!= 0)||Year % 400 == 0)  //leap year
				DayDelay += 29;
			else
				DayDelay += 28;
		}
		else
		{
			DayDelay+=30;
		}
		Month++;
	}
	TimeDelay = (((DayDelay +t3-1)* 24 + t4) * 60 + t5) * 60 + t6;
	return TimeDelay;
}
void CacuClockTime(uint32 TimeSec)
{
	int YearDelay = 365*24*60*60;
	int MonthDelay = 31 *24 * 60 * 60;
	int DayDelay = 24 * 60* 60;
	int HourDelay = 60* 60;
	int MiniteDelay = 60;
	year=0;
	month=1;
	day=1;
	hour=0;
	minute=0;
	second=0;
	realyear=1970;
	realmonth=1;
	realday=1;
	realhour=0;
	realminute=0;
	realsecond=0;
	
	TimeSec = TimeSec +8*60*60;
	while(TimeSec >=YearDelay)
	{
		realyear++;
		year++;
		TimeSec-=YearDelay;
		if ((realyear % 4 == 0 && realyear % 100 != 0)||realyear % 400 == 0)  //leap year
			YearDelay=366*24*60*60;
		else
			YearDelay=365*24*60*60;
	}
	while(TimeSec>=MonthDelay)
	{
		realmonth++;
		month++;
		TimeSec-=MonthDelay;
		if (realmonth == 1 || realmonth== 3 || realmonth == 5 || realmonth == 7 || realmonth == 8 || realmonth == 10 || realmonth == 12)
			MonthDelay=31*24*60*60;
		else if (realmonth == 2)
		{
			if ((realyear % 4 == 0 &&realyear % 100 != 0 )||realyear % 400 == 0)  //leap year
				MonthDelay=29*24*60*60;
			else
				MonthDelay=28*24*60*60;
		}
		else
			MonthDelay=30*24*60*60;
	}
	while(TimeSec>=DayDelay)
	{
		day++;
		realday++;
		TimeSec-=DayDelay;
	}
	while(TimeSec>=HourDelay)
	{
		hour++;
		realhour++;
		TimeSec-=HourDelay;
	}
	while(TimeSec>=MiniteDelay)
	{
		minute++;
		realminute++;
		TimeSec-=MiniteDelay;
	}
	second=TimeSec;
	realsecond=TimeSec;
	second=second/10*16+second%10;         // binary to decimal
	minute=minute/10*16+minute%10;
	hour=hour;
	hour=hour/10*16+hour%10;
	day=day/10*16+day%10;
	month=month/10*16+month%10;
	year=year/10*16+year%10;
}
uint32 spiReadAddr(void)
{
	uint32 x;
	spi1transmitByte(0x00);
	x=spiREG1->BUF;
	spi1transmitByte(0x00);
	x=(x<<8)|spiREG1->BUF;
	spi1transmitByte(0x00);
	x=(x<<8)|spiREG1->BUF;
	spi1transmitByte(0x00);
	x=(x<<8)|spiREG1->BUF;
	return x;
}
/* USER CODE END */
