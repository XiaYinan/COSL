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
int bF[10] = {0};  /*flash existence */
uint16 temperature = 0;				/* temperature value */
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
unsigned char brxstat = 0;			/* UART receive data processing status */
uint16 accx_send, accy_send, accz_send,accr_send,temp_send = 0;	/* used for UART data transmition */
uint16 accr_tmp;
float accrfloatsend,accrfloatavg=0;
int accrintsend=0;
float acczfloatsend,acczfloatavg=0;
float accxfloatsend,accxfloatavg=0;
float accyfloatsend,accyfloatavg=0;
uint8 rcvdata[64] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};			/* UART receive data buffer */
uint16  ReadFlagStatusData[2] = {0x70, 0x00};
uint16	ReceiveFlagStatusData[2]	= { 0 };
uint16	ReadStatusData[2]	= { 0x05, 0x00 };
uint16	ReceiveStatusData[2]	= { 0 };
unsigned char i_rcv = 0;												/* UART receive data pointer */
uint16 accx_2k_buf0[10240], accy_2k_buf0[10240],	accz_2k_buf0[10240], accr_2k_buf0[10240];	/* 1kHz sampling data buffers for accx, accy, accz (8x accumulation results)*/
uint16 accx_2k_buf1[10240], accy_2k_buf1[10240],	accz_2k_buf1[10240], accr_2k_buf1[10240];	/* 1kHz sampling data buffers for accx, accy, accz (8x accumulation results)*/
bool adcflag=0;
uint16 floatx=0,floaty=0,floatz=0;
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
unsigned int sav_addr0 = 0xf6dce00;			/* Global address of character data storage, the range is 0x10000000~0x14000000 */
unsigned int * psav_addr1;        // pointer for sav_addr1
/*  Address table for ininialization */
unsigned int sav_addr1_xy0z0r0=0x0000000, sav_addr1_xy0z0r1=0x01A5500, sav_addr1_xy0z0r2=0x034AA00, sav_addr1_xy0z0r3=0x04EFF00, sav_addr1_xy0z0r4=0x0695400, sav_addr1_xy0z0r5=0x083A900;
unsigned int sav_addr1_xy0z1r0=0x09DFE00, sav_addr1_xy0z1r1=0x0B85300, sav_addr1_xy0z1r2=0x0D2A800, sav_addr1_xy0z1r3=0x0ECFD00, sav_addr1_xy0z1r4=0x1075200, sav_addr1_xy0z1r5=0x121A700;
unsigned int sav_addr1_xy0z2r0=0x13BFC00, sav_addr1_xy0z2r1=0x1565100, sav_addr1_xy0z2r2=0x170A600, sav_addr1_xy0z2r3=0x18AFB00, sav_addr1_xy0z2r4=0x1A55000, sav_addr1_xy0z2r5=0x1BFA500;
unsigned int sav_addr1_xy0z3r0=0x1D9FA00, sav_addr1_xy0z3r1=0x1F44F00, sav_addr1_xy0z3r2=0x20EA400, sav_addr1_xy0z3r3=0x228F900, sav_addr1_xy0z3r4=0x2434E00, sav_addr1_xy0z3r5=0x25DA300;
unsigned int sav_addr1_xy0z4r0=0x277F800, sav_addr1_xy0z4r1=0x2924D00, sav_addr1_xy0z4r2=0x2ACA200, sav_addr1_xy0z4r3=0x2C6F700, sav_addr1_xy0z4r4=0x2E14C00, sav_addr1_xy0z4r5=0x2FBA100;
unsigned int sav_addr1_xy1z0r0=0x315F600, sav_addr1_xy1z0r1=0x3304B00, sav_addr1_xy1z0r2=0x34AA000, sav_addr1_xy1z0r3=0x364F500, sav_addr1_xy1z0r4=0x37F4A00, sav_addr1_xy1z0r5=0x3999F00;
unsigned int sav_addr1_xy1z1r0=0x3B3F400, sav_addr1_xy1z1r1=0x3CE4900, sav_addr1_xy1z1r2=0x3E89E00, sav_addr1_xy1z1r3=0x402F300, sav_addr1_xy1z1r4=0x41D4800, sav_addr1_xy1z1r5=0x4379D00;
unsigned int sav_addr1_xy1z2r0=0x451F200, sav_addr1_xy1z2r1=0x46C4700, sav_addr1_xy1z2r2=0x4869C00, sav_addr1_xy1z2r3=0x4A0F100, sav_addr1_xy1z2r4=0x4BB4600, sav_addr1_xy1z2r5=0x4D59B00;
unsigned int sav_addr1_xy1z3r0=0x4EFF000, sav_addr1_xy1z3r1=0x50A4500, sav_addr1_xy1z3r2=0x5249A00, sav_addr1_xy1z3r3=0x53EEF00, sav_addr1_xy1z3r4=0x5594400, sav_addr1_xy1z3r5=0x5739900;
unsigned int sav_addr1_xy1z4r0=0x58DEE00, sav_addr1_xy1z4r1=0x5A84300, sav_addr1_xy1z4r2=0x5C29800, sav_addr1_xy1z4r3=0x5DCED00, sav_addr1_xy1z4r4=0x5F74200, sav_addr1_xy1z4r5=0x6119700;
unsigned int sav_addr1_xy2z0r0=0x62BEC00, sav_addr1_xy2z0r1=0x6464100, sav_addr1_xy2z0r2=0x6609600, sav_addr1_xy2z0r3=0x67AEB00, sav_addr1_xy2z0r4=0x6954000, sav_addr1_xy2z0r5=0x6AF9500;
unsigned int sav_addr1_xy2z1r0=0x6C9EA00, sav_addr1_xy2z1r1=0x6E43F00, sav_addr1_xy2z1r2=0x6FE9400, sav_addr1_xy2z1r3=0x718E900, sav_addr1_xy2z1r4=0x7333E00, sav_addr1_xy2z1r5=0x74D9300;
unsigned int sav_addr1_xy2z2r0=0x767E800, sav_addr1_xy2z2r1=0x7823D00, sav_addr1_xy2z2r2=0x79C9200, sav_addr1_xy2z2r3=0x7B6E700, sav_addr1_xy2z2r4=0x7D13C00, sav_addr1_xy2z2r5=0x7EB9100;
unsigned int sav_addr1_xy2z3r0=0x805E600, sav_addr1_xy2z3r1=0x8203B00, sav_addr1_xy2z3r2=0x83A9000, sav_addr1_xy2z3r3=0x854E500, sav_addr1_xy2z3r4=0x86F3A00, sav_addr1_xy2z3r5=0x8898F00;
unsigned int sav_addr1_xy2z4r0=0x8A3E400, sav_addr1_xy2z4r1=0x8BE3900, sav_addr1_xy2z4r2=0x8D88E00, sav_addr1_xy2z4r3=0x8F2E300, sav_addr1_xy2z4r4=0x90D3800, sav_addr1_xy2z4r5=0x9278D00;
unsigned int sav_addr1_xy3z0r0=0x941E200, sav_addr1_xy3z0r1=0x95C3700, sav_addr1_xy3z0r2=0x9768C00, sav_addr1_xy3z0r3=0x990E100, sav_addr1_xy3z0r4=0x9AB3600, sav_addr1_xy3z0r5=0x9C58B00;
unsigned int sav_addr1_xy3z1r0=0x9DFE000, sav_addr1_xy3z1r1=0x9FA3500, sav_addr1_xy3z1r2=0xA148A00, sav_addr1_xy3z1r3=0xA2EDF00, sav_addr1_xy3z1r4=0xA493400, sav_addr1_xy3z1r5=0xA638900;
unsigned int sav_addr1_xy3z2r0=0xA7DDE00, sav_addr1_xy3z2r1=0xA983300, sav_addr1_xy3z2r2=0xAB28800, sav_addr1_xy3z2r3=0xACCDD00, sav_addr1_xy3z2r4=0xAE73200, sav_addr1_xy3z2r5=0xB018700;
unsigned int sav_addr1_xy3z3r0=0xB1BDC00, sav_addr1_xy3z3r1=0xB363100, sav_addr1_xy3z3r2=0xB508600, sav_addr1_xy3z3r3=0xB6ADB00, sav_addr1_xy3z3r4=0xB853000, sav_addr1_xy3z3r5=0xB9F8500;
unsigned int sav_addr1_xy3z4r0=0xBB9DA00, sav_addr1_xy3z4r1=0xBD42F00, sav_addr1_xy3z4r2=0xBEE8400, sav_addr1_xy3z4r3=0xC08D900, sav_addr1_xy3z4r4=0xC232E00, sav_addr1_xy3z4r5=0xC3D8300;
unsigned int sav_addr1_xy4z0r0=0xC57D800, sav_addr1_xy4z0r1=0xC722D00, sav_addr1_xy4z0r2=0xC8C8200, sav_addr1_xy4z0r3=0xCA6D700, sav_addr1_xy4z0r4=0xCC12C00, sav_addr1_xy4z0r5=0xCDB8100;
unsigned int sav_addr1_xy4z1r0=0xCF5D600, sav_addr1_xy4z1r1=0xD102B00, sav_addr1_xy4z1r2=0xD2A8000, sav_addr1_xy4z1r3=0xD44D500, sav_addr1_xy4z1r4=0xD5F2A00, sav_addr1_xy4z1r5=0xD797F00;
unsigned int sav_addr1_xy4z2r0=0xD93D400, sav_addr1_xy4z2r1=0xDAE2900, sav_addr1_xy4z2r2=0xDC87E00, sav_addr1_xy4z2r3=0xDE2D300, sav_addr1_xy4z2r4=0xDFD2800, sav_addr1_xy4z2r5=0xE177D00;
unsigned int sav_addr1_xy4z3r0=0xE31D200, sav_addr1_xy4z3r1=0xE4C2700, sav_addr1_xy4z3r2=0xE667C00, sav_addr1_xy4z3r3=0xE80D100, sav_addr1_xy4z3r4=0xE9B2600, sav_addr1_xy4z3r5=0xEB57B00;
unsigned int sav_addr1_xy4z4r0=0xECFD000, sav_addr1_xy4z4r1=0xEEA2500, sav_addr1_xy4z4r2=0xF047A00, sav_addr1_xy4z4r3=0xF1ECF00, sav_addr1_xy4z4r4=0xF392400, sav_addr1_xy4z4r5=0xF537900;
unsigned int sav_cnt = 0;
unsigned int page_cnt = 0;
unsigned int ia2k = 0;				/* index for 2k buffer */
int ia2k_sav_cnt = 0;
unsigned char bsav_level = 0,bsav_data = 0;			/* flag for saving 2k data */
int accx_sum = 0, accy_sum = 0, accz_sum = 0, accr_sum = 0;	/* used for acculumating accx, accy and accz data for 2kHz sampling */
long long rms_sum_sec_x = 0, rms_sum_sec_y = 0, rms_sum_sec_z = 0;		/* calculate the rms value of 10 seconds for three axises */
float rms_sum_sec_x_float=0, rms_sum_sec_y_float=0, rms_sum_sec_z_float=0;
long long rms_sec_cur_x = 0, rms_sec_cur_y = 0, rms_sec_cur_z = 0;		/* storing the rms value of 10 seconds for three axises */
float rms_sec_cur_z_float=0;
int peak_sec_x = 0, peak_sec_y = 0, peak_sec_z = 0,peak_sec_r = 0;					/* calculate the peak value of 10 seconds for three axises */
float peak_sec_x_float=0, peak_sec_y_float=0, peak_sec_z_float=0,peak_sec_r_float=0;
int peak_sec_cur_x = 0, peak_sec_cur_y = 0, peak_sec_cur_z = 0,peak_sec_cur_r = 0;		/* storing the peak value of 10 seconds for three axises */
float peak_sec_cur_r_float = 0;
int bottom_sec_cur_r=0,bottom_sec_r=0;
float bottom_sec_cur_r_float=10000,bottom_sec_r_float=10000;
int avg_sec_x = 0, avg_sec_y = 0, avg_sec_z = 0, avg_sec_r = 0;					/* calculate the average value of 10 seconds for three axises */
float avg_sec_x_float=0, avg_sec_y_float=0, avg_sec_z_float=0,avg_sec_r_float=0;
uint16 avg_sec_cur_x = 0, avg_sec_cur_y = 0, avg_sec_cur_z = 0,avg_sec_cur_r = 0;		/* storing the average value of 10 seconds for three axises */
float avg_sec_cur_r_float=0;
long long rms_sum_bulk = 0;
float rms_sum_bulk_float=0;
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
uint16 minusR=0;
uint8 xylevel=0,zlevel=0,rlevel=0,level=0;
float s1=0.00,s2=0.00;
int iTick = 0;					/* I2C to sampling temperature interval count, default 0.5sps*/
unsigned char bStartI2c = 0;	/* flag for start temperature convert and time reading */
uint32 ar,br,cr,dr = 0;                   //parameters to calibrate the primitive data
uint32 xy1,xy2,xy3,xy4,xy5=0;             //threshold value to calibrate the lateral vibration
float xy1float,xy2float,xy3float,xy4float,xy5float=0;  //threshold value to calibrate the lateral vibration(float)
uint32 z1,z2,z3,z4,z5=0;                  //threshold value to calibrate the vibration of axis z
float z1float,z2float,z3float,z4float,z5float=0;  //threshold value to calibrate the vibration of axis z (float)
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
void ReadStatus(int i);   /* flash status read */
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
				else if(rxdata == 0x60)		// mode CALIBRATION
					brxstat = 13;
				else if(rxdata == 0x07)		// read Tool ID
					brxstat = 24;
				else if(rxdata == 0x58)		// write Tool ID
					brxstat = 28;
				else if(rxdata == 0x08)		// read Firmware Version
					brxstat = 42;
				else if(rxdata == 0x56)		// write Baud rate
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
				else if(rxdata == 0x13)		// write parameter table
					brxstat = 118;
				else if(rxdata == 0x14)		// read parameter table
					brxstat = 180;
				else if(rxdata == 0x5A)		// read real time data
					brxstat = 190;
				else
					brxstat = 0;
			}
			break;
		case 1:                // read UART status
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 2;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 2:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 3;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 3:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 4;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 4:
			data[0]=0x0003;
			data[1]=(bComHighSpeed<<8)|sendmode;
			data[2]=0xff00|mode;
			data[3]=data[0]^data[1]^data[2]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;
		case 5:             // mode STANDBY
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 6;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 6:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 7;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 7:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 8;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 8:
			mode=0x00;
			bSave=0;
			bSend=0;
			FMWREN ();
			WriteFMByte(0x0000, bSave);
			sendmode=0;
			data[0]=0x0003;
			data[1]=(bComHighSpeed<<8)|sendmode;
			data[2]=0xff00|mode;
			data[3]=data[0]^data[1]^data[2]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;
		case 9:         // mode OPERATION
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 10;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 10:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 11;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 11:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 12;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 12:
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
		case 13:                // mode CALIBRATION
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 14;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 14:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 15;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 15:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x02)
				{
					brxstat = 16;
					ovtimeX= 0;
					check=0x0002^0xffff;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 16:
			ovtimeX++;
			if(i_rcv != 0)
			{
				mode=0x10;
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata1 == 0x00)           // sendmode=0;
				{
					brxstat = 18;
				}
				else if(rxdata1 == 0x01)      // sendmode=1;
				{
					brxstat = 18;
				}
				else if(rxdata1 == 0x02)      // sendmode=2;
				{
					brxstat = 18;
				}
				else if(rxdata1 == 0x03)      // sendmode=3;
				{
					brxstat = 18;
				}
				else if(rxdata1 == 0x04)      // sendmode=4;
				{
					brxstat = 18;
				}
				else
					brxstat = 16;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 18:
			ovtimeX++;
			if(i_rcv != 0)
			{
				mode=0x10;
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata1==rxdata2)
				{
					if(rxdata2 == 0x00)           // sendmode=0;
					{
						brxstat = 19;
						sendmode=0;
						check^=0x0000;
					}
					else if(rxdata2 == 0x01)      // sendmode=1;
					{
						brxstat = 19;
						sendmode=1;
						check^=0x0101;
					}
					else if(rxdata2 == 0x02)      // sendmode=2;
					{
						brxstat = 19;
						sendmode= 2;
						check^=0x0202;
					}
					else if(rxdata2 == 0x03)      // sendmode=3;
					{
						brxstat = 19;
						sendmode= 3;
						check^=0x0303;
					}
					else if(rxdata2 == 0x04)      // sendmode=4;
					{
						brxstat = 19;
						sendmode= 4;
						check^=0x0404;
					}
					else
						brxstat = 18;
				}
				else
					brxstat = 18;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 19:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				brxstat = 20;
				ovtimeX= 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 20;
			break;
		case 20:    // to check the sum
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				CHKS=(rxdata1<<8)^rxdata2;
				if(check==CHKS)
				{
					brxstat = 21;
					ovtimeX= 0;
				}
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 0;
			break;
		case 21:
			bSave=0;
			bSend=1;
			bComHighSpeed=1;
			FMWREN();
			WriteFMByte(0x0000, bSave);
			data[0]=0x0003;
			data[1]=(bComHighSpeed<<8)|sendmode;
			data[2]=0xff00|mode;
			data[3]=data[0]^data[1]^data[2]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;
		case 24:           // read Tool ID
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 25;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 25:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 26;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 26:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 27;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 27:
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
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 43;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 43:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 44;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 44:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 45;
					ovtimeX= 0;
				}
				else
					brxstat = 44;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 45:
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
				if(rxdata == 0x00)
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
				if(rxdata == 0x02)
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
					brxstat = 51;
					bComHighSpeed=1;
					check=0xE100^0x0002^0xffff;
				}
				else if(baud == 0x0060)
				{
					ovtimeX= 0;
					brxstat = 51;
					bComHighSpeed=0;
					check=0x0060^0x0002^0xffff;
				}
				else
					brxstat = 50;
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
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 77;
					ovtimeX = 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 77:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 78;
					ovtimeX = 0;
				}
				else
					brxstat = 77;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 78:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 79;
					ovtimeX = 0;
				}
				else
					brxstat = 78;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 79:
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
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 81;
					ovtimeX = 0;
				}
				else
					brxstat = 80;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 81:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 82;
					ovtimeX = 0;
				}
				else
					brxstat = 81;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 82:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 83;
					ovtimeX = 0;
				}
				else
					brxstat = 82;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 83:
			data[0]=0x0002;
			check=0x0002^0xffff;
			data[1]=temperature;
			check ^= data[1];
			data[2]= check;
			MysciSend(data);
			brxstat = 0;
			break;
		case 84:         // read r
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 85;
					ovtimeX = 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 85:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 86;
					ovtimeX = 0;
				}
				else
					brxstat = 85;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 86:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 87;
					ovtimeX = 0;
				}
				else
					brxstat = 86;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 87:
			data[0]=0x0003;
			check=0x0003^0xffff;
			if(accrfloatsend<0)
				data[1]=0x0001;
			else
				data[1]=0x0000;
			check ^= data[1];
			data[2]=absf(accrfloatsend);
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
			MysciSend(data);			  //��ʼ����
			if(bulk_select==0x01)
				bulk_erase_character();
			else
				bulk_erase_primitive();
			MysciSend(data);		     //��������
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
			/*    data[0]=0x0005;          //for debug
			    check=0x0005^0xffff;
					txdata1=(sav_addr0>>24)&0xff;
					txdata2=(sav_addr0>>16)&0xff;
			    data[1]=(txdata1<<8)|txdata2;
				  check ^= data[1];
					  txdata1=(sav_addr0>>8)&0xff;
					  txdata2=sav_addr0&0xff;
			    data[2]=(txdata1<<8)|txdata2;
				    check ^= data[2];
					  txdata1=(sav_addr1_xy0z0r0>>24)&0xff;
					  txdata2=(sav_addr1_xy0z0r0>>16)&0xff;
			    data[3]=(txdata1<<8)|txdata2;
				    check ^= data[3];
					  txdata1=(sav_addr1_xy0z0r0>>8)&0xff;
					  txdata2=sav_addr1_xy0z0r0&0xff;
			    data[4]=(txdata1<<8)|txdata2;
				    check ^= data[4];
			    data[5]= check;
			    MysciSend(data);
						brxstat = 0; */
			data[0]=0x012f;
			check=0x012f^0xffff;
			txdata1=(sav_addr0>>24)&0xff;
			txdata2=(sav_addr0>>16)&0xff;
			data[1]=(txdata1<<8)|txdata2;
			check ^= data[1];
			txdata1=(sav_addr0>>8)&0xff;
			txdata2=sav_addr0&0xff;
			data[2]=(txdata1<<8)|txdata2;
			check ^= data[2];
			txdata1=(sav_addr1_xy0z0r0>>24)&0xff;
			txdata2=(sav_addr1_xy0z0r0>>16)&0xff;
			data[3]=(txdata1<<8)|txdata2;
			check ^= data[3];
			txdata1=(sav_addr1_xy0z0r0>>8)&0xff;
			txdata2=sav_addr1_xy0z0r0&0xff;
			data[4]=(txdata1<<8)|txdata2;
			check ^= data[4];
			txdata1=(sav_addr1_xy0z0r1>>24)&0xff;
			txdata2=(sav_addr1_xy0z0r1>>16)&0xff;
			data[5]=(txdata1<<8)|txdata2;
			check ^= data[5];
			txdata1=(sav_addr1_xy0z0r1>>8)&0xff;
			txdata2=sav_addr1_xy0z0r1&0xff;
			data[6]=(txdata1<<8)|txdata2;
			check ^= data[6];
			txdata1=(sav_addr1_xy0z0r2>>24)&0xff;
			txdata2=(sav_addr1_xy0z0r2>>16)&0xff;
			data[7]=(txdata1<<8)|txdata2;
			check ^= data[7];
			txdata1=(sav_addr1_xy0z0r2>>8)&0xff;
			txdata2=sav_addr1_xy0z0r2&0xff;
			data[8]=(txdata1<<8)|txdata2;
			check ^= data[8];
			txdata1=(sav_addr1_xy0z0r3>>24)&0xff;
			txdata2=(sav_addr1_xy0z0r3>>16)&0xff;
			data[9]=(txdata1<<8)|txdata2;
			check ^= data[9];
			txdata1=(sav_addr1_xy0z0r3>>8)&0xff;
			txdata2=sav_addr1_xy0z0r3&0xff;
			data[10]=(txdata1<<8)|txdata2;
			check ^= data[10];
			txdata1=(sav_addr1_xy0z0r4>>24)&0xff;
			txdata2=(sav_addr1_xy0z0r4>>16)&0xff;
			data[11]=(txdata1<<8)|txdata2;
			check ^= data[11];
			txdata1=(sav_addr1_xy0z0r4>>8)&0xff;
			txdata2=sav_addr1_xy0z0r4&0xff;
			data[12]=(txdata1<<8)|txdata2;
			check ^= data[12];
			txdata1=(sav_addr1_xy0z0r5>>24)&0xff;
			txdata2=(sav_addr1_xy0z0r5>>16)&0xff;
			data[13]=(txdata1<<8)|txdata2;
			check ^= data[13];
			txdata1=(sav_addr1_xy0z0r5>>8)&0xff;
			txdata2=sav_addr1_xy0z0r5&0xff;
			data[14]=(txdata1<<8)|txdata2;
			check ^= data[14];
			txdata1=(sav_addr1_xy0z1r0>>24)&0xff;
			txdata2=(sav_addr1_xy0z1r0>>16)&0xff;
			data[15]=(txdata1<<8)|txdata2;
			check ^= data[15];
			txdata1=(sav_addr1_xy0z1r0>>8)&0xff;
			txdata2=sav_addr1_xy0z1r0&0xff;
			data[16]=(txdata1<<8)|txdata2;
			check ^= data[16];
			txdata1=(sav_addr1_xy0z1r1>>24)&0xff;
			txdata2=(sav_addr1_xy0z1r1>>16)&0xff;
			data[17]=(txdata1<<8)|txdata2;
			check ^= data[17];
			txdata1=(sav_addr1_xy0z1r1>>8)&0xff;
			txdata2=sav_addr1_xy0z1r1&0xff;
			data[18]=(txdata1<<8)|txdata2;
			check ^= data[18];
			txdata1=(sav_addr1_xy0z1r2>>24)&0xff;
			txdata2=(sav_addr1_xy0z1r2>>16)&0xff;
			data[19]=(txdata1<<8)|txdata2;
			check ^= data[19];
			txdata1=(sav_addr1_xy0z1r2>>8)&0xff;
			txdata2=sav_addr1_xy0z1r2&0xff;
			data[20]=(txdata1<<8)|txdata2;
			check ^= data[20];
			txdata1=(sav_addr1_xy0z1r3>>24)&0xff;
			txdata2=(sav_addr1_xy0z1r3>>16)&0xff;
			data[21]=(txdata1<<8)|txdata2;
			check ^= data[21];
			txdata1=(sav_addr1_xy0z1r3>>8)&0xff;
			txdata2=sav_addr1_xy0z1r3&0xff;
			data[22]=(txdata1<<8)|txdata2;
			check ^= data[22];
			txdata1=(sav_addr1_xy0z1r4>>24)&0xff;
			txdata2=(sav_addr1_xy0z1r4>>16)&0xff;
			data[23]=(txdata1<<8)|txdata2;
			check ^= data[23];
			txdata1=(sav_addr1_xy0z1r4>>8)&0xff;
			txdata2=sav_addr1_xy0z1r4&0xff;
			data[24]=(txdata1<<8)|txdata2;
			check ^= data[24];
			txdata1=(sav_addr1_xy0z1r5>>24)&0xff;
			txdata2=(sav_addr1_xy0z1r5>>16)&0xff;
			data[25]=(txdata1<<8)|txdata2;
			check ^= data[25];
			txdata1=(sav_addr1_xy0z1r5>>8)&0xff;
			txdata2=sav_addr1_xy0z1r5&0xff;
			data[26]=(txdata1<<8)|txdata2;
			check ^= data[26];
			txdata1=(sav_addr1_xy0z2r0>>24)&0xff;
			txdata2=(sav_addr1_xy0z2r0>>16)&0xff;
			data[27]=(txdata1<<8)|txdata2;
			check ^= data[27];
			txdata1=(sav_addr1_xy0z2r0>>8)&0xff;
			txdata2=sav_addr1_xy0z2r0&0xff;
			data[28]=(txdata1<<8)|txdata2;
			check ^= data[28];
			txdata1=(sav_addr1_xy0z2r1>>24)&0xff;
			txdata2=(sav_addr1_xy0z2r1>>16)&0xff;
			data[29]=(txdata1<<8)|txdata2;
			check ^= data[29];
			txdata1=(sav_addr1_xy0z2r1>>8)&0xff;
			txdata2=sav_addr1_xy0z2r1&0xff;
			data[30]=(txdata1<<8)|txdata2;
			check ^= data[30];
			txdata1=(sav_addr1_xy0z2r2>>24)&0xff;
			txdata2=(sav_addr1_xy0z2r2>>16)&0xff;
			data[31]=(txdata1<<8)|txdata2;
			check ^= data[31];
			txdata1=(sav_addr1_xy0z2r2>>8)&0xff;
			txdata2=sav_addr1_xy0z2r2&0xff;
			data[32]=(txdata1<<8)|txdata2;
			check ^= data[32];
			txdata1=(sav_addr1_xy0z2r3>>24)&0xff;
			txdata2=(sav_addr1_xy0z2r3>>16)&0xff;
			data[33]=(txdata1<<8)|txdata2;
			check ^= data[33];
			txdata1=(sav_addr1_xy0z2r3>>8)&0xff;
			txdata2=sav_addr1_xy0z2r3&0xff;
			data[34]=(txdata1<<8)|txdata2;
			check ^= data[34];
			txdata1=(sav_addr1_xy0z2r4>>24)&0xff;
			txdata2=(sav_addr1_xy0z2r4>>16)&0xff;
			data[35]=(txdata1<<8)|txdata2;
			check ^= data[35];
			txdata1=(sav_addr1_xy0z2r4>>8)&0xff;
			txdata2=sav_addr1_xy0z2r4&0xff;
			data[36]=(txdata1<<8)|txdata2;
			check ^= data[36];
			txdata1=(sav_addr1_xy0z2r5>>24)&0xff;
			txdata2=(sav_addr1_xy0z2r5>>16)&0xff;
			data[37]=(txdata1<<8)|txdata2;
			check ^= data[37];
			txdata1=(sav_addr1_xy0z2r5>>8)&0xff;
			txdata2=sav_addr1_xy0z2r5&0xff;
			data[38]=(txdata1<<8)|txdata2;
			check ^= data[38];
			txdata1=(sav_addr1_xy0z3r0>>24)&0xff;
			txdata2=(sav_addr1_xy0z3r0>>16)&0xff;
			data[39]=(txdata1<<8)|txdata2;
			check ^= data[39];
			txdata1=(sav_addr1_xy0z3r0>>8)&0xff;
			txdata2=sav_addr1_xy0z3r0&0xff;
			data[40]=(txdata1<<8)|txdata2;
			check ^= data[40];
			txdata1=(sav_addr1_xy0z3r1>>24)&0xff;
			txdata2=(sav_addr1_xy0z3r1>>16)&0xff;
			data[41]=(txdata1<<8)|txdata2;
			check ^= data[41];
			txdata1=(sav_addr1_xy0z3r1>>8)&0xff;
			txdata2=sav_addr1_xy0z3r1&0xff;
			data[42]=(txdata1<<8)|txdata2;
			check ^= data[42];
			txdata1=(sav_addr1_xy0z3r2>>24)&0xff;
			txdata2=(sav_addr1_xy0z3r2>>16)&0xff;
			data[43]=(txdata1<<8)|txdata2;
			check ^= data[43];
			txdata1=(sav_addr1_xy0z3r2>>8)&0xff;
			txdata2=sav_addr1_xy0z3r2&0xff;
			data[44]=(txdata1<<8)|txdata2;
			check ^= data[44];
			txdata1=(sav_addr1_xy0z3r3>>24)&0xff;
			txdata2=(sav_addr1_xy0z3r3>>16)&0xff;
			data[45]=(txdata1<<8)|txdata2;
			check ^= data[45];
			txdata1=(sav_addr1_xy0z3r3>>8)&0xff;
			txdata2=sav_addr1_xy0z3r3&0xff;
			data[46]=(txdata1<<8)|txdata2;
			check ^= data[46];
			txdata1=(sav_addr1_xy0z3r4>>24)&0xff;
			txdata2=(sav_addr1_xy0z3r4>>16)&0xff;
			data[47]=(txdata1<<8)|txdata2;
			check ^= data[47];
			txdata1=(sav_addr1_xy0z3r4>>8)&0xff;
			txdata2=sav_addr1_xy0z3r4&0xff;
			data[48]=(txdata1<<8)|txdata2;
			check ^= data[48];
			txdata1=(sav_addr1_xy0z3r5>>24)&0xff;
			txdata2=(sav_addr1_xy0z3r5>>16)&0xff;
			data[49]=(txdata1<<8)|txdata2;
			check ^= data[49];
			txdata1=(sav_addr1_xy0z3r5>>8)&0xff;
			txdata2=sav_addr1_xy0z3r5&0xff;
			data[50]=(txdata1<<8)|txdata2;
			check ^= data[50];
			txdata1=(sav_addr1_xy0z4r0>>24)&0xff;
			txdata2=(sav_addr1_xy0z4r0>>16)&0xff;
			data[51]=(txdata1<<8)|txdata2;
			check ^= data[51];
			txdata1=(sav_addr1_xy0z4r0>>8)&0xff;
			txdata2=sav_addr1_xy0z4r0&0xff;
			data[52]=(txdata1<<8)|txdata2;
			check ^= data[52];
			txdata1=(sav_addr1_xy0z4r1>>25)&0xff;
			txdata2=(sav_addr1_xy0z4r1>>16)&0xff;
			data[53]=(txdata1<<8)|txdata2;
			check ^= data[53];
			txdata1=(sav_addr1_xy0z4r1>>8)&0xff;
			txdata2=sav_addr1_xy0z4r1&0xff;
			data[54]=(txdata1<<8)|txdata2;
			check ^= data[54];
			txdata1=(sav_addr1_xy0z4r2>>24)&0xff;
			txdata2=(sav_addr1_xy0z4r2>>16)&0xff;
			data[55]=(txdata1<<8)|txdata2;
			check ^= data[55];
			txdata1=(sav_addr1_xy0z4r2>>8)&0xff;
			txdata2=sav_addr1_xy0z4r2&0xff;
			data[56]=(txdata1<<8)|txdata2;
			check ^= data[56];
			txdata1=(sav_addr1_xy0z4r3>>24)&0xff;
			txdata2=(sav_addr1_xy0z4r3>>16)&0xff;
			data[57]=(txdata1<<8)|txdata2;
			check ^= data[57];
			txdata1=(sav_addr1_xy0z4r3>>8)&0xff;
			txdata2=sav_addr1_xy0z4r3&0xff;
			data[58]=(txdata1<<8)|txdata2;
			check ^= data[58];
			txdata1=(sav_addr1_xy0z4r4>>24)&0xff;
			txdata2=(sav_addr1_xy0z4r4>>16)&0xff;
			data[59]=(txdata1<<8)|txdata2;
			check ^= data[59];
			txdata1=(sav_addr1_xy0z4r4>>8)&0xff;
			txdata2=sav_addr1_xy0z4r4&0xff;
			data[60]=(txdata1<<8)|txdata2;
			check ^= data[60];
			txdata1=(sav_addr1_xy0z4r5>>24)&0xff;
			txdata2=(sav_addr1_xy0z4r5>>16)&0xff;
			data[61]=(txdata1<<8)|txdata2;
			check ^= data[61];
			txdata1=(sav_addr1_xy0z4r5>>8)&0xff;
			txdata2=sav_addr1_xy0z4r5&0xff;
			data[62]=(txdata1<<8)|txdata2;
			check ^= data[62];
			txdata1=(sav_addr1_xy1z0r0>>24)&0xff;
			txdata2=(sav_addr1_xy1z0r0>>16)&0xff;
			data[63]=(txdata1<<8)|txdata2;
			check ^= data[63];
			txdata1=(sav_addr1_xy1z0r0>>8)&0xff;
			txdata2=sav_addr1_xy1z0r0&0xff;
			data[64]=(txdata1<<8)|txdata2;
			check ^= data[64];
			txdata1=(sav_addr1_xy1z0r1>>24)&0xff;
			txdata2=(sav_addr1_xy1z0r1>>16)&0xff;
			data[65]=(txdata1<<8)|txdata2;
			check ^= data[65];
			txdata1=(sav_addr1_xy1z0r1>>8)&0xff;
			txdata2=sav_addr1_xy1z0r1&0xff;
			data[66]=(txdata1<<8)|txdata2;
			check ^= data[66];
			txdata1=(sav_addr1_xy1z0r2>>24)&0xff;
			txdata2=(sav_addr1_xy1z0r2>>16)&0xff;
			data[67]=(txdata1<<8)|txdata2;
			check ^= data[67];
			txdata1=(sav_addr1_xy1z0r2>>8)&0xff;
			txdata2=sav_addr1_xy1z0r2&0xff;
			data[68]=(txdata1<<8)|txdata2;
			check ^= data[68];
			txdata1=(sav_addr1_xy1z0r3>>24)&0xff;
			txdata2=(sav_addr1_xy1z0r3>>16)&0xff;
			data[69]=(txdata1<<8)|txdata2;
			check ^= data[69];
			txdata1=(sav_addr1_xy1z0r3>>8)&0xff;
			txdata2=sav_addr1_xy1z0r3&0xff;
			data[70]=(txdata1<<8)|txdata2;
			check ^= data[70];
			txdata1=(sav_addr1_xy1z0r4>>24)&0xff;
			txdata2=(sav_addr1_xy1z0r4>>16)&0xff;
			data[71]=(txdata1<<8)|txdata2;
			check ^= data[71];
			txdata1=(sav_addr1_xy1z0r4>>8)&0xff;
			txdata2=sav_addr1_xy1z0r4&0xff;
			data[72]=(txdata1<<8)|txdata2;
			check ^= data[72];
			txdata1=(sav_addr1_xy1z0r5>>24)&0xff;
			txdata2=(sav_addr1_xy1z0r5>>16)&0xff;
			data[73]=(txdata1<<8)|txdata2;
			check ^= data[73];
			txdata1=(sav_addr1_xy1z0r5>>8)&0xff;
			txdata2=sav_addr1_xy1z0r5&0xff;
			data[74]=(txdata1<<8)|txdata2;
			check ^= data[74];
			txdata1=(sav_addr1_xy1z1r0>>24)&0xff;
			txdata2=(sav_addr1_xy1z1r0>>16)&0xff;
			data[75]=(txdata1<<8)|txdata2;
			check ^= data[75];
			txdata1=(sav_addr1_xy1z1r0>>8)&0xff;
			txdata2=sav_addr1_xy1z1r0&0xff;
			data[76]=(txdata1<<8)|txdata2;
			check ^= data[76];
			txdata1=(sav_addr1_xy1z1r1>>24)&0xff;
			txdata2=(sav_addr1_xy1z1r1>>16)&0xff;
			data[77]=(txdata1<<8)|txdata2;
			check ^= data[77];
			txdata1=(sav_addr1_xy1z1r1>>8)&0xff;
			txdata2=sav_addr1_xy1z1r1&0xff;
			data[78]=(txdata1<<8)|txdata2;
			check ^= data[78];
			txdata1=(sav_addr1_xy1z1r2>>24)&0xff;
			txdata2=(sav_addr1_xy1z1r2>>16)&0xff;
			data[79]=(txdata1<<8)|txdata2;
			check ^= data[79];
			txdata1=(sav_addr1_xy1z1r2>>8)&0xff;
			txdata2=sav_addr1_xy1z1r2&0xff;
			data[80]=(txdata1<<8)|txdata2;
			check ^= data[80];
			txdata1=(sav_addr1_xy1z1r3>>24)&0xff;
			txdata2=(sav_addr1_xy1z1r3>>16)&0xff;
			data[81]=(txdata1<<8)|txdata2;
			check ^= data[81];
			txdata1=(sav_addr1_xy1z1r3>>8)&0xff;
			txdata2=sav_addr1_xy1z1r3&0xff;
			data[82]=(txdata1<<8)|txdata2;
			check ^= data[82];
			txdata1=(sav_addr1_xy1z1r4>>24)&0xff;
			txdata2=(sav_addr1_xy1z1r4>>16)&0xff;
			data[83]=(txdata1<<8)|txdata2;
			check ^= data[83];
			txdata1=(sav_addr1_xy1z1r4>>8)&0xff;
			txdata2=sav_addr1_xy1z1r4&0xff;
			data[84]=(txdata1<<8)|txdata2;
			check ^= data[84];
			txdata1=(sav_addr1_xy1z1r5>>24)&0xff;
			txdata2=(sav_addr1_xy1z1r5>>16)&0xff;
			data[85]=(txdata1<<8)|txdata2;
			check ^= data[85];
			txdata1=(sav_addr1_xy1z1r5>>8)&0xff;
			txdata2=sav_addr1_xy1z1r5&0xff;
			data[86]=(txdata1<<8)|txdata2;
			check ^= data[86];
			txdata1=(sav_addr1_xy1z2r0>>24)&0xff;
			txdata2=(sav_addr1_xy1z2r0>>16)&0xff;
			data[87]=(txdata1<<8)|txdata2;
			check ^= data[87];
			txdata1=(sav_addr1_xy1z2r0>>8)&0xff;
			txdata2=sav_addr1_xy1z2r0&0xff;
			data[88]=(txdata1<<8)|txdata2;
			check ^= data[88];
			txdata1=(sav_addr1_xy1z2r1>>24)&0xff;
			txdata2=(sav_addr1_xy1z2r1>>16)&0xff;
			data[89]=(txdata1<<8)|txdata2;
			check ^= data[89];
			txdata1=(sav_addr1_xy1z2r1>>8)&0xff;
			txdata2=sav_addr1_xy1z2r1&0xff;
			data[90]=(txdata1<<8)|txdata2;
			check ^= data[90];
			txdata1=(sav_addr1_xy1z2r2>>24)&0xff;
			txdata2=(sav_addr1_xy1z2r2>>16)&0xff;
			data[91]=(txdata1<<8)|txdata2;
			check ^= data[91];
			txdata1=(sav_addr1_xy1z2r2>>8)&0xff;
			txdata2=sav_addr1_xy1z2r2&0xff;
			data[92]=(txdata1<<8)|txdata2;
			check ^= data[92];
			txdata1=(sav_addr1_xy1z2r3>>24)&0xff;
			txdata2=(sav_addr1_xy1z2r3>>16)&0xff;
			data[93]=(txdata1<<8)|txdata2;
			check ^= data[93];
			txdata1=(sav_addr1_xy1z2r3>>8)&0xff;
			txdata2=sav_addr1_xy1z2r3&0xff;
			data[94]=(txdata1<<8)|txdata2;
			check ^= data[94];
			txdata1=(sav_addr1_xy1z2r4>>24)&0xff;
			txdata2=(sav_addr1_xy1z2r4>>16)&0xff;
			data[95]=(txdata1<<8)|txdata2;
			check ^= data[95];
			txdata1=(sav_addr1_xy1z2r4>>8)&0xff;
			txdata2=sav_addr1_xy1z2r4&0xff;
			data[96]=(txdata1<<8)|txdata2;
			check ^= data[96];
			txdata1=(sav_addr1_xy1z2r5>>24)&0xff;
			txdata2=(sav_addr1_xy1z2r5>>16)&0xff;
			data[97]=(txdata1<<8)|txdata2;
			check ^= data[97];
			txdata1=(sav_addr1_xy1z2r5>>8)&0xff;
			txdata2=sav_addr1_xy1z2r5&0xff;
			data[98]=(txdata1<<8)|txdata2;
			check ^= data[98];
			txdata1=(sav_addr1_xy1z3r0>>24)&0xff;
			txdata2=(sav_addr1_xy1z3r0>>16)&0xff;
			data[99]=(txdata1<<8)|txdata2;
			check ^= data[99];
			txdata1=(sav_addr1_xy1z3r0>>8)&0xff;
			txdata2=sav_addr1_xy1z3r0&0xff;
			data[100]=(txdata1<<8)|txdata2;
			check ^= data[100];
			txdata1=(sav_addr1_xy1z3r1>>24)&0xff;
			txdata2=(sav_addr1_xy1z3r1>>16)&0xff;
			data[101]=(txdata1<<8)|txdata2;
			check ^= data[101];
			txdata1=(sav_addr1_xy1z3r1>>8)&0xff;
			txdata2=sav_addr1_xy1z3r1&0xff;
			data[102]=(txdata1<<8)|txdata2;
			check ^= data[102];
			txdata1=(sav_addr1_xy1z3r2>>24)&0xff;
			txdata2=(sav_addr1_xy1z3r2>>16)&0xff;
			data[103]=(txdata1<<8)|txdata2;
			check ^= data[103];
			txdata1=(sav_addr1_xy1z3r2>>8)&0xff;
			txdata2=sav_addr1_xy1z3r2&0xff;
			data[104]=(txdata1<<8)|txdata2;
			check ^= data[104];
			txdata1=(sav_addr1_xy1z3r3>>24)&0xff;
			txdata2=(sav_addr1_xy1z3r3>>16)&0xff;
			data[105]=(txdata1<<8)|txdata2;
			check ^= data[105];
			txdata1=(sav_addr1_xy1z3r3>>8)&0xff;
			txdata2=sav_addr1_xy1z3r3&0xff;
			data[106]=(txdata1<<8)|txdata2;
			check ^= data[106];
			txdata1=(sav_addr1_xy1z3r4>>24)&0xff;
			txdata2=(sav_addr1_xy1z3r4>>16)&0xff;
			data[107]=(txdata1<<8)|txdata2;
			check ^= data[107];
			txdata1=(sav_addr1_xy1z3r4>>8)&0xff;
			txdata2=sav_addr1_xy1z3r4&0xff;
			data[108]=(txdata1<<8)|txdata2;
			check ^= data[108];
			txdata1=(sav_addr1_xy1z3r5>>24)&0xff;
			txdata2=(sav_addr1_xy1z3r5>>16)&0xff;
			data[109]=(txdata1<<8)|txdata2;
			check ^= data[109];
			txdata1=(sav_addr1_xy1z3r5>>8)&0xff;
			txdata2=sav_addr1_xy1z3r5&0xff;
			data[110]=(txdata1<<8)|txdata2;
			check ^= data[110];
			txdata1=(sav_addr1_xy1z4r0>>24)&0xff;
			txdata2=(sav_addr1_xy1z4r0>>16)&0xff;
			data[111]=(txdata1<<8)|txdata2;
			check ^= data[111];
			txdata1=(sav_addr1_xy1z4r0>>8)&0xff;
			txdata2=sav_addr1_xy1z4r0&0xff;
			data[112]=(txdata1<<8)|txdata2;
			check ^= data[112];
			txdata1=(sav_addr1_xy1z4r1>>25)&0xff;
			txdata2=(sav_addr1_xy1z4r1>>16)&0xff;
			data[113]=(txdata1<<8)|txdata2;
			check ^= data[113];
			txdata1=(sav_addr1_xy1z4r1>>8)&0xff;
			txdata2=sav_addr1_xy1z4r1&0xff;
			data[114]=(txdata1<<8)|txdata2;
			check ^= data[114];
			txdata1=(sav_addr1_xy1z4r2>>24)&0xff;
			txdata2=(sav_addr1_xy1z4r2>>16)&0xff;
			data[115]=(txdata1<<8)|txdata2;
			check ^= data[115];
			txdata1=(sav_addr1_xy1z4r2>>8)&0xff;
			txdata2=sav_addr1_xy1z4r2&0xff;
			data[116]=(txdata1<<8)|txdata2;
			check ^= data[116];
			txdata1=(sav_addr1_xy1z4r3>>24)&0xff;
			txdata2=(sav_addr1_xy1z4r3>>16)&0xff;
			data[117]=(txdata1<<8)|txdata2;
			check ^= data[117];
			txdata1=(sav_addr1_xy1z4r3>>8)&0xff;
			txdata2=sav_addr1_xy1z4r3&0xff;
			data[118]=(txdata1<<8)|txdata2;
			check ^= data[118];
			txdata1=(sav_addr1_xy1z4r4>>24)&0xff;
			txdata2=(sav_addr1_xy1z4r4>>16)&0xff;
			data[119]=(txdata1<<8)|txdata2;
			check ^= data[119];
			txdata1=(sav_addr1_xy1z4r4>>8)&0xff;
			txdata2=sav_addr1_xy1z4r4&0xff;
			data[120]=(txdata1<<8)|txdata2;
			check ^= data[120];
			txdata1=(sav_addr1_xy1z4r5>>24)&0xff;
			txdata2=(sav_addr1_xy1z4r5>>16)&0xff;
			data[121]=(txdata1<<8)|txdata2;
			check ^= data[121];
			txdata1=(sav_addr1_xy1z4r5>>8)&0xff;
			txdata2=sav_addr1_xy1z4r5&0xff;
			data[122]=(txdata1<<8)|txdata2;
			check ^= data[122];
			txdata1=(sav_addr1_xy2z0r0>>24)&0xff;
			txdata2=(sav_addr1_xy2z0r0>>16)&0xff;
			data[123]=(txdata1<<8)|txdata2;
			check ^= data[123];
			txdata1=(sav_addr1_xy2z0r0>>8)&0xff;
			txdata2=sav_addr1_xy2z0r0&0xff;
			data[124]=(txdata1<<8)|txdata2;
			check ^= data[124];
			txdata1=(sav_addr1_xy2z0r1>>24)&0xff;
			txdata2=(sav_addr1_xy2z0r1>>16)&0xff;
			data[125]=(txdata1<<8)|txdata2;
			check ^= data[125];
			txdata1=(sav_addr1_xy2z0r1>>8)&0xff;
			txdata2=sav_addr1_xy2z0r1&0xff;
			data[126]=(txdata1<<8)|txdata2;
			check ^= data[126];
			txdata1=(sav_addr1_xy2z0r2>>24)&0xff;
			txdata2=(sav_addr1_xy2z0r2>>16)&0xff;
			data[127]=(txdata1<<8)|txdata2;
			check ^= data[127];
			txdata1=(sav_addr1_xy2z0r2>>8)&0xff;
			txdata2=sav_addr1_xy2z0r2&0xff;
			data[128]=(txdata1<<8)|txdata2;
			check ^= data[128];
			txdata1=(sav_addr1_xy2z0r3>>24)&0xff;
			txdata2=(sav_addr1_xy2z0r3>>16)&0xff;
			data[129]=(txdata1<<8)|txdata2;
			check ^= data[129];
			txdata1=(sav_addr1_xy2z0r3>>8)&0xff;
			txdata2=sav_addr1_xy2z0r3&0xff;
			data[130]=(txdata1<<8)|txdata2;
			check ^= data[130];
			txdata1=(sav_addr1_xy2z0r4>>24)&0xff;
			txdata2=(sav_addr1_xy2z0r4>>16)&0xff;
			data[131]=(txdata1<<8)|txdata2;
			check ^= data[131];
			txdata1=(sav_addr1_xy2z0r4>>8)&0xff;
			txdata2=sav_addr1_xy2z0r4&0xff;
			data[132]=(txdata1<<8)|txdata2;
			check ^= data[132];
			txdata1=(sav_addr1_xy2z0r5>>24)&0xff;
			txdata2=(sav_addr1_xy2z0r5>>16)&0xff;
			data[133]=(txdata1<<8)|txdata2;
			check ^= data[133];
			txdata1=(sav_addr1_xy2z0r5>>8)&0xff;
			txdata2=sav_addr1_xy2z0r5&0xff;
			data[134]=(txdata1<<8)|txdata2;
			check ^= data[134];
			txdata1=(sav_addr1_xy2z1r0>>24)&0xff;
			txdata2=(sav_addr1_xy2z1r0>>16)&0xff;
			data[135]=(txdata1<<8)|txdata2;
			check ^= data[135];
			txdata1=(sav_addr1_xy2z1r0>>8)&0xff;
			txdata2=sav_addr1_xy2z1r0&0xff;
			data[136]=(txdata1<<8)|txdata2;
			check ^= data[136];
			txdata1=(sav_addr1_xy2z1r1>>24)&0xff;
			txdata2=(sav_addr1_xy2z1r1>>16)&0xff;
			data[137]=(txdata1<<8)|txdata2;
			check ^= data[137];
			txdata1=(sav_addr1_xy2z1r1>>8)&0xff;
			txdata2=sav_addr1_xy2z1r1&0xff;
			data[138]=(txdata1<<8)|txdata2;
			check ^= data[138];
			txdata1=(sav_addr1_xy2z1r2>>24)&0xff;
			txdata2=(sav_addr1_xy2z1r2>>16)&0xff;
			data[139]=(txdata1<<8)|txdata2;
			check ^= data[139];
			txdata1=(sav_addr1_xy2z1r2>>8)&0xff;
			txdata2=sav_addr1_xy2z1r2&0xff;
			data[140]=(txdata1<<8)|txdata2;
			check ^= data[140];
			txdata1=(sav_addr1_xy2z1r3>>24)&0xff;
			txdata2=(sav_addr1_xy2z1r3>>16)&0xff;
			data[141]=(txdata1<<8)|txdata2;
			check ^= data[141];
			txdata1=(sav_addr1_xy2z1r3>>8)&0xff;
			txdata2=sav_addr1_xy2z1r3&0xff;
			data[142]=(txdata1<<8)|txdata2;
			check ^= data[142];
			txdata1=(sav_addr1_xy2z1r4>>24)&0xff;
			txdata2=(sav_addr1_xy2z1r4>>16)&0xff;
			data[143]=(txdata1<<8)|txdata2;
			check ^= data[143];
			txdata1=(sav_addr1_xy2z1r4>>8)&0xff;
			txdata2=sav_addr1_xy2z1r4&0xff;
			data[144]=(txdata1<<8)|txdata2;
			check ^= data[144];
			txdata1=(sav_addr1_xy2z1r5>>24)&0xff;
			txdata2=(sav_addr1_xy2z1r5>>16)&0xff;
			data[145]=(txdata1<<8)|txdata2;
			check ^= data[145];
			txdata1=(sav_addr1_xy2z1r5>>8)&0xff;
			txdata2=sav_addr1_xy2z1r5&0xff;
			data[146]=(txdata1<<8)|txdata2;
			check ^= data[146];
			txdata1=(sav_addr1_xy2z2r0>>24)&0xff;
			txdata2=(sav_addr1_xy2z2r0>>16)&0xff;
			data[147]=(txdata1<<8)|txdata2;
			check ^= data[147];
			txdata1=(sav_addr1_xy2z2r0>>8)&0xff;
			txdata2=sav_addr1_xy2z2r0&0xff;
			data[148]=(txdata1<<8)|txdata2;
			check ^= data[148];
			txdata1=(sav_addr1_xy2z2r1>>24)&0xff;
			txdata2=(sav_addr1_xy2z2r1>>16)&0xff;
			data[149]=(txdata1<<8)|txdata2;
			check ^= data[149];
			txdata1=(sav_addr1_xy2z2r1>>8)&0xff;
			txdata2=sav_addr1_xy2z2r1&0xff;
			data[150]=(txdata1<<8)|txdata2;
			check ^= data[150];
			txdata1=(sav_addr1_xy2z2r2>>24)&0xff;
			txdata2=(sav_addr1_xy2z2r2>>16)&0xff;
			data[151]=(txdata1<<8)|txdata2;
			check ^= data[151];
			txdata1=(sav_addr1_xy2z2r2>>8)&0xff;
			txdata2=sav_addr1_xy2z2r2&0xff;
			data[152]=(txdata1<<8)|txdata2;
			check ^= data[152];
			txdata1=(sav_addr1_xy2z2r3>>24)&0xff;
			txdata2=(sav_addr1_xy2z2r3>>16)&0xff;
			data[153]=(txdata1<<8)|txdata2;
			check ^= data[153];
			txdata1=(sav_addr1_xy2z2r3>>8)&0xff;
			txdata2=sav_addr1_xy2z2r3&0xff;
			data[154]=(txdata1<<8)|txdata2;
			check ^= data[154];
			txdata1=(sav_addr1_xy2z2r4>>24)&0xff;
			txdata2=(sav_addr1_xy2z2r4>>16)&0xff;
			data[155]=(txdata1<<8)|txdata2;
			check ^= data[155];
			txdata1=(sav_addr1_xy2z2r4>>8)&0xff;
			txdata2=sav_addr1_xy2z2r4&0xff;
			data[156]=(txdata1<<8)|txdata2;
			check ^= data[156];
			txdata1=(sav_addr1_xy2z2r5>>24)&0xff;
			txdata2=(sav_addr1_xy2z2r5>>16)&0xff;
			data[157]=(txdata1<<8)|txdata2;
			check ^= data[157];
			txdata1=(sav_addr1_xy2z2r5>>8)&0xff;
			txdata2=sav_addr1_xy2z2r5&0xff;
			data[158]=(txdata1<<8)|txdata2;
			check ^= data[158];
			txdata1=(sav_addr1_xy2z3r0>>24)&0xff;
			txdata2=(sav_addr1_xy2z3r0>>16)&0xff;
			data[159]=(txdata1<<8)|txdata2;
			check ^= data[159];
			txdata1=(sav_addr1_xy2z3r0>>8)&0xff;
			txdata2=sav_addr1_xy2z3r0&0xff;
			data[160]=(txdata1<<8)|txdata2;
			check ^= data[160];
			txdata1=(sav_addr1_xy2z3r1>>24)&0xff;
			txdata2=(sav_addr1_xy2z3r1>>16)&0xff;
			data[161]=(txdata1<<8)|txdata2;
			check ^= data[161];
			txdata1=(sav_addr1_xy2z3r1>>8)&0xff;
			txdata2=sav_addr1_xy2z3r1&0xff;
			data[162]=(txdata1<<8)|txdata2;
			check ^= data[162];
			txdata1=(sav_addr1_xy2z3r2>>24)&0xff;
			txdata2=(sav_addr1_xy2z3r2>>16)&0xff;
			data[163]=(txdata1<<8)|txdata2;
			check ^= data[163];
			txdata1=(sav_addr1_xy2z3r2>>8)&0xff;
			txdata2=sav_addr1_xy2z3r2&0xff;
			data[164]=(txdata1<<8)|txdata2;
			check ^= data[164];
			txdata1=(sav_addr1_xy2z3r3>>24)&0xff;
			txdata2=(sav_addr1_xy2z3r3>>16)&0xff;
			data[165]=(txdata1<<8)|txdata2;
			check ^= data[165];
			txdata1=(sav_addr1_xy2z3r3>>8)&0xff;
			txdata2=sav_addr1_xy2z3r3&0xff;
			data[166]=(txdata1<<8)|txdata2;
			check ^= data[166];
			txdata1=(sav_addr1_xy2z3r4>>24)&0xff;
			txdata2=(sav_addr1_xy2z3r4>>16)&0xff;
			data[167]=(txdata1<<8)|txdata2;
			check ^= data[167];
			txdata1=(sav_addr1_xy2z3r4>>8)&0xff;
			txdata2=sav_addr1_xy2z3r4&0xff;
			data[168]=(txdata1<<8)|txdata2;
			check ^= data[168];
			txdata1=(sav_addr1_xy2z3r5>>24)&0xff;
			txdata2=(sav_addr1_xy2z3r5>>16)&0xff;
			data[169]=(txdata1<<8)|txdata2;
			check ^= data[169];
			txdata1=(sav_addr1_xy2z3r5>>8)&0xff;
			txdata2=sav_addr1_xy2z3r5&0xff;
			data[170]=(txdata1<<8)|txdata2;
			check ^= data[170];
			txdata1=(sav_addr1_xy2z4r0>>24)&0xff;
			txdata2=(sav_addr1_xy2z4r0>>16)&0xff;
			data[171]=(txdata1<<8)|txdata2;
			check ^= data[171];
			txdata1=(sav_addr1_xy2z4r0>>8)&0xff;
			txdata2=sav_addr1_xy2z4r0&0xff;
			data[172]=(txdata1<<8)|txdata2;
			check ^= data[172];
			txdata1=(sav_addr1_xy2z4r1>>25)&0xff;
			txdata2=(sav_addr1_xy2z4r1>>16)&0xff;
			data[173]=(txdata1<<8)|txdata2;
			check ^= data[173];
			txdata1=(sav_addr1_xy2z4r1>>8)&0xff;
			txdata2=sav_addr1_xy2z4r1&0xff;
			data[174]=(txdata1<<8)|txdata2;
			check ^= data[174];
			txdata1=(sav_addr1_xy2z4r2>>24)&0xff;
			txdata2=(sav_addr1_xy2z4r2>>16)&0xff;
			data[175]=(txdata1<<8)|txdata2;
			check ^= data[175];
			txdata1=(sav_addr1_xy2z4r2>>8)&0xff;
			txdata2=sav_addr1_xy2z4r2&0xff;
			data[176]=(txdata1<<8)|txdata2;
			check ^= data[176];
			txdata1=(sav_addr1_xy2z4r3>>24)&0xff;
			txdata2=(sav_addr1_xy2z4r3>>16)&0xff;
			data[177]=(txdata1<<8)|txdata2;
			check ^= data[177];
			txdata1=(sav_addr1_xy2z4r3>>8)&0xff;
			txdata2=sav_addr1_xy2z4r3&0xff;
			data[178]=(txdata1<<8)|txdata2;
			check ^= data[178];
			txdata1=(sav_addr1_xy2z4r4>>24)&0xff;
			txdata2=(sav_addr1_xy2z4r4>>16)&0xff;
			data[179]=(txdata1<<8)|txdata2;
			check ^= data[179];
			txdata1=(sav_addr1_xy2z4r4>>8)&0xff;
			txdata2=sav_addr1_xy2z4r4&0xff;
			data[180]=(txdata1<<8)|txdata2;
			check ^= data[180];
			txdata1=(sav_addr1_xy2z4r5>>24)&0xff;
			txdata2=(sav_addr1_xy2z4r5>>16)&0xff;
			data[181]=(txdata1<<8)|txdata2;
			check ^= data[181];
			txdata1=(sav_addr1_xy2z4r5>>8)&0xff;
			txdata2=sav_addr1_xy2z4r5&0xff;
			data[182]=(txdata1<<8)|txdata2;
			check ^= data[182];
			txdata1=(sav_addr1_xy3z0r0>>24)&0xff;
			txdata2=(sav_addr1_xy3z0r0>>16)&0xff;
			data[183]=(txdata1<<8)|txdata2;
			check ^= data[183];
			txdata1=(sav_addr1_xy3z0r0>>8)&0xff;
			txdata2=sav_addr1_xy3z0r0&0xff;
			data[184]=(txdata1<<8)|txdata2;
			check ^= data[184];
			txdata1=(sav_addr1_xy3z0r1>>24)&0xff;
			txdata2=(sav_addr1_xy3z0r1>>16)&0xff;
			data[185]=(txdata1<<8)|txdata2;
			check ^= data[185];
			txdata1=(sav_addr1_xy3z0r1>>8)&0xff;
			txdata2=sav_addr1_xy3z0r1&0xff;
			data[186]=(txdata1<<8)|txdata2;
			check ^= data[186];
			txdata1=(sav_addr1_xy3z0r2>>24)&0xff;
			txdata2=(sav_addr1_xy3z0r2>>16)&0xff;
			data[187]=(txdata1<<8)|txdata2;
			check ^= data[187];
			txdata1=(sav_addr1_xy3z0r2>>8)&0xff;
			txdata2=sav_addr1_xy3z0r2&0xff;
			data[188]=(txdata1<<8)|txdata2;
			check ^= data[188];
			txdata1=(sav_addr1_xy3z0r3>>24)&0xff;
			txdata2=(sav_addr1_xy3z0r3>>16)&0xff;
			data[189]=(txdata1<<8)|txdata2;
			check ^= data[189];
			txdata1=(sav_addr1_xy3z0r3>>8)&0xff;
			txdata2=sav_addr1_xy3z0r3&0xff;
			data[190]=(txdata1<<8)|txdata2;
			check ^= data[190];
			txdata1=(sav_addr1_xy3z0r4>>24)&0xff;
			txdata2=(sav_addr1_xy3z0r4>>16)&0xff;
			data[191]=(txdata1<<8)|txdata2;
			check ^= data[191];
			txdata1=(sav_addr1_xy3z0r4>>8)&0xff;
			txdata2=sav_addr1_xy3z0r4&0xff;
			data[192]=(txdata1<<8)|txdata2;
			check ^= data[192];
			txdata1=(sav_addr1_xy3z0r5>>24)&0xff;
			txdata2=(sav_addr1_xy3z0r5>>16)&0xff;
			data[193]=(txdata1<<8)|txdata2;
			check ^= data[193];
			txdata1=(sav_addr1_xy3z0r5>>8)&0xff;
			txdata2=sav_addr1_xy3z0r5&0xff;
			data[194]=(txdata1<<8)|txdata2;
			check ^= data[194];
			txdata1=(sav_addr1_xy3z1r0>>24)&0xff;
			txdata2=(sav_addr1_xy3z1r0>>16)&0xff;
			data[195]=(txdata1<<8)|txdata2;
			check ^= data[195];
			txdata1=(sav_addr1_xy3z1r0>>8)&0xff;
			txdata2=sav_addr1_xy3z1r0&0xff;
			data[196]=(txdata1<<8)|txdata2;
			check ^= data[196];
			txdata1=(sav_addr1_xy3z1r1>>24)&0xff;
			txdata2=(sav_addr1_xy3z1r1>>16)&0xff;
			data[197]=(txdata1<<8)|txdata2;
			check ^= data[197];
			txdata1=(sav_addr1_xy3z1r1>>8)&0xff;
			txdata2=sav_addr1_xy3z1r1&0xff;
			data[198]=(txdata1<<8)|txdata2;
			check ^= data[198];
			txdata1=(sav_addr1_xy3z1r2>>24)&0xff;
			txdata2=(sav_addr1_xy3z1r2>>16)&0xff;
			data[199]=(txdata1<<8)|txdata2;
			check ^= data[199];
			txdata1=(sav_addr1_xy3z1r2>>8)&0xff;
			txdata2=sav_addr1_xy3z1r2&0xff;
			data[200]=(txdata1<<8)|txdata2;
			check ^= data[200];
			txdata1=(sav_addr1_xy3z1r3>>24)&0xff;
			txdata2=(sav_addr1_xy3z1r3>>16)&0xff;
			data[201]=(txdata1<<8)|txdata2;
			check ^= data[201];
			txdata1=(sav_addr1_xy3z1r3>>8)&0xff;
			txdata2=sav_addr1_xy3z1r3&0xff;
			data[202]=(txdata1<<8)|txdata2;
			check ^= data[202];
			txdata1=(sav_addr1_xy3z1r4>>24)&0xff;
			txdata2=(sav_addr1_xy3z1r4>>16)&0xff;
			data[203]=(txdata1<<8)|txdata2;
			check ^= data[203];
			txdata1=(sav_addr1_xy3z1r4>>8)&0xff;
			txdata2=sav_addr1_xy3z1r4&0xff;
			data[204]=(txdata1<<8)|txdata2;
			check ^= data[204];
			txdata1=(sav_addr1_xy3z1r5>>24)&0xff;
			txdata2=(sav_addr1_xy3z1r5>>16)&0xff;
			data[205]=(txdata1<<8)|txdata2;
			check ^= data[205];
			txdata1=(sav_addr1_xy3z1r5>>8)&0xff;
			txdata2=sav_addr1_xy3z1r5&0xff;
			data[206]=(txdata1<<8)|txdata2;
			check ^= data[206];
			txdata1=(sav_addr1_xy3z2r0>>24)&0xff;
			txdata2=(sav_addr1_xy3z2r0>>16)&0xff;
			data[207]=(txdata1<<8)|txdata2;
			check ^= data[207];
			txdata1=(sav_addr1_xy3z2r0>>8)&0xff;
			txdata2=sav_addr1_xy3z2r0&0xff;
			data[208]=(txdata1<<8)|txdata2;
			check ^= data[208];
			txdata1=(sav_addr1_xy3z2r1>>24)&0xff;
			txdata2=(sav_addr1_xy3z2r1>>16)&0xff;
			data[209]=(txdata1<<8)|txdata2;
			check ^= data[209];
			txdata1=(sav_addr1_xy3z2r1>>8)&0xff;
			txdata2=sav_addr1_xy3z2r1&0xff;
			data[210]=(txdata1<<8)|txdata2;
			check ^= data[210];
			txdata1=(sav_addr1_xy3z2r2>>24)&0xff;
			txdata2=(sav_addr1_xy3z2r2>>16)&0xff;
			data[211]=(txdata1<<8)|txdata2;
			check ^= data[211];
			txdata1=(sav_addr1_xy3z2r2>>8)&0xff;
			txdata2=sav_addr1_xy3z2r2&0xff;
			data[212]=(txdata1<<8)|txdata2;
			check ^= data[212];
			txdata1=(sav_addr1_xy3z2r3>>24)&0xff;
			txdata2=(sav_addr1_xy3z2r3>>16)&0xff;
			data[213]=(txdata1<<8)|txdata2;
			check ^= data[213];
			txdata1=(sav_addr1_xy3z2r3>>8)&0xff;
			txdata2=sav_addr1_xy3z2r3&0xff;
			data[214]=(txdata1<<8)|txdata2;
			check ^= data[214];
			txdata1=(sav_addr1_xy3z2r4>>24)&0xff;
			txdata2=(sav_addr1_xy3z2r4>>16)&0xff;
			data[215]=(txdata1<<8)|txdata2;
			check ^= data[215];
			txdata1=(sav_addr1_xy3z2r4>>8)&0xff;
			txdata2=sav_addr1_xy3z2r4&0xff;
			data[216]=(txdata1<<8)|txdata2;
			check ^= data[216];
			txdata1=(sav_addr1_xy3z2r5>>24)&0xff;
			txdata2=(sav_addr1_xy3z2r5>>16)&0xff;
			data[217]=(txdata1<<8)|txdata2;
			check ^= data[217];
			txdata1=(sav_addr1_xy3z2r5>>8)&0xff;
			txdata2=sav_addr1_xy3z2r5&0xff;
			data[218]=(txdata1<<8)|txdata2;
			check ^= data[218];
			txdata1=(sav_addr1_xy3z3r0>>24)&0xff;
			txdata2=(sav_addr1_xy3z3r0>>16)&0xff;
			data[219]=(txdata1<<8)|txdata2;
			check ^= data[219];
			txdata1=(sav_addr1_xy3z3r0>>8)&0xff;
			txdata2=sav_addr1_xy3z3r0&0xff;
			data[220]=(txdata1<<8)|txdata2;
			check ^= data[220];
			txdata1=(sav_addr1_xy3z3r1>>24)&0xff;
			txdata2=(sav_addr1_xy3z3r1>>16)&0xff;
			data[221]=(txdata1<<8)|txdata2;
			check ^= data[221];
			txdata1=(sav_addr1_xy3z3r1>>8)&0xff;
			txdata2=sav_addr1_xy3z3r1&0xff;
			data[222]=(txdata1<<8)|txdata2;
			check ^= data[222];
			txdata1=(sav_addr1_xy3z3r2>>24)&0xff;
			txdata2=(sav_addr1_xy3z3r2>>16)&0xff;
			data[223]=(txdata1<<8)|txdata2;
			check ^= data[223];
			txdata1=(sav_addr1_xy3z3r2>>8)&0xff;
			txdata2=sav_addr1_xy3z3r2&0xff;
			data[224]=(txdata1<<8)|txdata2;
			check ^= data[224];
			txdata1=(sav_addr1_xy3z3r3>>24)&0xff;
			txdata2=(sav_addr1_xy3z3r3>>16)&0xff;
			data[225]=(txdata1<<8)|txdata2;
			check ^= data[225];
			txdata1=(sav_addr1_xy3z3r3>>8)&0xff;
			txdata2=sav_addr1_xy3z3r3&0xff;
			data[226]=(txdata1<<8)|txdata2;
			check ^= data[226];
			txdata1=(sav_addr1_xy3z3r4>>24)&0xff;
			txdata2=(sav_addr1_xy3z3r4>>16)&0xff;
			data[227]=(txdata1<<8)|txdata2;
			check ^= data[227];
			txdata1=(sav_addr1_xy3z3r4>>8)&0xff;
			txdata2=sav_addr1_xy3z3r4&0xff;
			data[228]=(txdata1<<8)|txdata2;
			check ^= data[228];
			txdata1=(sav_addr1_xy3z3r5>>24)&0xff;
			txdata2=(sav_addr1_xy3z3r5>>16)&0xff;
			data[229]=(txdata1<<8)|txdata2;
			check ^= data[229];
			txdata1=(sav_addr1_xy3z3r5>>8)&0xff;
			txdata2=sav_addr1_xy3z3r5&0xff;
			data[230]=(txdata1<<8)|txdata2;
			check ^= data[230];
			txdata1=(sav_addr1_xy3z4r0>>24)&0xff;
			txdata2=(sav_addr1_xy3z4r0>>16)&0xff;
			data[231]=(txdata1<<8)|txdata2;
			check ^= data[231];
			txdata1=(sav_addr1_xy3z4r0>>8)&0xff;
			txdata2=sav_addr1_xy3z4r0&0xff;
			data[232]=(txdata1<<8)|txdata2;
			check ^= data[232];
			txdata1=(sav_addr1_xy3z4r1>>25)&0xff;
			txdata2=(sav_addr1_xy3z4r1>>16)&0xff;
			data[233]=(txdata1<<8)|txdata2;
			check ^= data[233];
			txdata1=(sav_addr1_xy3z4r1>>8)&0xff;
			txdata2=sav_addr1_xy3z4r1&0xff;
			data[234]=(txdata1<<8)|txdata2;
			check ^= data[234];
			txdata1=(sav_addr1_xy3z4r2>>24)&0xff;
			txdata2=(sav_addr1_xy3z4r2>>16)&0xff;
			data[235]=(txdata1<<8)|txdata2;
			check ^= data[235];
			txdata1=(sav_addr1_xy3z4r2>>8)&0xff;
			txdata2=sav_addr1_xy3z4r2&0xff;
			data[236]=(txdata1<<8)|txdata2;
			check ^= data[236];
			txdata1=(sav_addr1_xy3z4r3>>24)&0xff;
			txdata2=(sav_addr1_xy3z4r3>>16)&0xff;
			data[237]=(txdata1<<8)|txdata2;
			check ^= data[237];
			txdata1=(sav_addr1_xy3z4r3>>8)&0xff;
			txdata2=sav_addr1_xy3z4r3&0xff;
			data[238]=(txdata1<<8)|txdata2;
			check ^= data[238];
			txdata1=(sav_addr1_xy3z4r4>>24)&0xff;
			txdata2=(sav_addr1_xy3z4r4>>16)&0xff;
			data[239]=(txdata1<<8)|txdata2;
			check ^= data[239];
			txdata1=(sav_addr1_xy3z4r4>>8)&0xff;
			txdata2=sav_addr1_xy3z4r4&0xff;
			data[240]=(txdata1<<8)|txdata2;
			check ^= data[240];
			txdata1=(sav_addr1_xy3z4r5>>24)&0xff;
			txdata2=(sav_addr1_xy3z4r5>>16)&0xff;
			data[241]=(txdata1<<8)|txdata2;
			check ^= data[241];
			txdata1=(sav_addr1_xy3z4r5>>8)&0xff;
			txdata2=sav_addr1_xy3z4r5&0xff;
			data[242]=(txdata1<<8)|txdata2;
			check ^= data[242];
			txdata1=(sav_addr1_xy4z0r0>>24)&0xff;
			txdata2=(sav_addr1_xy4z0r0>>16)&0xff;
			data[243]=(txdata1<<8)|txdata2;
			check ^= data[243];
			txdata1=(sav_addr1_xy4z0r0>>8)&0xff;
			txdata2=sav_addr1_xy4z0r0&0xff;
			data[244]=(txdata1<<8)|txdata2;
			check ^= data[244];
			txdata1=(sav_addr1_xy4z0r1>>24)&0xff;
			txdata2=(sav_addr1_xy4z0r1>>16)&0xff;
			data[245]=(txdata1<<8)|txdata2;
			check ^= data[245];
			txdata1=(sav_addr1_xy4z0r1>>8)&0xff;
			txdata2=sav_addr1_xy4z0r1&0xff;
			data[246]=(txdata1<<8)|txdata2;
			check ^= data[246];
			txdata1=(sav_addr1_xy4z0r2>>24)&0xff;
			txdata2=(sav_addr1_xy4z0r2>>16)&0xff;
			data[247]=(txdata1<<8)|txdata2;
			check ^= data[247];
			txdata1=(sav_addr1_xy4z0r2>>8)&0xff;
			txdata2=sav_addr1_xy4z0r2&0xff;
			data[248]=(txdata1<<8)|txdata2;
			check ^= data[248];
			txdata1=(sav_addr1_xy4z0r3>>24)&0xff;
			txdata2=(sav_addr1_xy4z0r3>>16)&0xff;
			data[249]=(txdata1<<8)|txdata2;
			check ^= data[249];
			txdata1=(sav_addr1_xy4z0r3>>8)&0xff;
			txdata2=sav_addr1_xy4z0r3&0xff;
			data[250]=(txdata1<<8)|txdata2;
			check ^= data[250];
			txdata1=(sav_addr1_xy4z0r4>>24)&0xff;
			txdata2=(sav_addr1_xy4z0r4>>16)&0xff;
			data[251]=(txdata1<<8)|txdata2;
			check ^= data[251];
			txdata1=(sav_addr1_xy4z0r4>>8)&0xff;
			txdata2=sav_addr1_xy4z0r4&0xff;
			data[252]=(txdata1<<8)|txdata2;
			check ^= data[252];
			txdata1=(sav_addr1_xy4z0r5>>24)&0xff;
			txdata2=(sav_addr1_xy4z0r5>>16)&0xff;
			data[253]=(txdata1<<8)|txdata2;
			check ^= data[253];
			txdata1=(sav_addr1_xy4z0r5>>8)&0xff;
			txdata2=sav_addr1_xy4z0r5&0xff;
			data[254]=(txdata1<<8)|txdata2;
			check ^= data[254];
			txdata1=(sav_addr1_xy4z1r0>>24)&0xff;
			txdata2=(sav_addr1_xy4z1r0>>16)&0xff;
			data[255]=(txdata1<<8)|txdata2;
			check ^= data[255];
			txdata1=(sav_addr1_xy4z1r0>>8)&0xff;
			txdata2=sav_addr1_xy4z1r0&0xff;
			data[256]=(txdata1<<8)|txdata2;
			check ^= data[256];
			txdata1=(sav_addr1_xy4z1r1>>24)&0xff;
			txdata2=(sav_addr1_xy4z1r1>>16)&0xff;
			data[257]=(txdata1<<8)|txdata2;
			check ^= data[257];
			txdata1=(sav_addr1_xy4z1r1>>8)&0xff;
			txdata2=sav_addr1_xy4z1r1&0xff;
			data[258]=(txdata1<<8)|txdata2;
			check ^= data[258];
			txdata1=(sav_addr1_xy4z1r2>>24)&0xff;
			txdata2=(sav_addr1_xy4z1r2>>16)&0xff;
			data[259]=(txdata1<<8)|txdata2;
			check ^= data[259];
			txdata1=(sav_addr1_xy4z1r2>>8)&0xff;
			txdata2=sav_addr1_xy4z1r2&0xff;
			data[260]=(txdata1<<8)|txdata2;
			check ^= data[260];
			txdata1=(sav_addr1_xy4z1r3>>24)&0xff;
			txdata2=(sav_addr1_xy4z1r3>>16)&0xff;
			data[261]=(txdata1<<8)|txdata2;
			check ^= data[261];
			txdata1=(sav_addr1_xy4z1r3>>8)&0xff;
			txdata2=sav_addr1_xy4z1r3&0xff;
			data[262]=(txdata1<<8)|txdata2;
			check ^= data[262];
			txdata1=(sav_addr1_xy4z1r4>>24)&0xff;
			txdata2=(sav_addr1_xy4z1r4>>16)&0xff;
			data[263]=(txdata1<<8)|txdata2;
			check ^= data[263];
			txdata1=(sav_addr1_xy4z1r4>>8)&0xff;
			txdata2=sav_addr1_xy4z1r4&0xff;
			data[264]=(txdata1<<8)|txdata2;
			check ^= data[264];
			txdata1=(sav_addr1_xy4z1r5>>24)&0xff;
			txdata2=(sav_addr1_xy4z1r5>>16)&0xff;
			data[265]=(txdata1<<8)|txdata2;
			check ^= data[265];
			txdata1=(sav_addr1_xy4z1r5>>8)&0xff;
			txdata2=sav_addr1_xy4z1r5&0xff;
			data[266]=(txdata1<<8)|txdata2;
			check ^= data[266];
			txdata1=(sav_addr1_xy4z2r0>>24)&0xff;
			txdata2=(sav_addr1_xy4z2r0>>16)&0xff;
			data[267]=(txdata1<<8)|txdata2;
			check ^= data[267];
			txdata1=(sav_addr1_xy4z2r0>>8)&0xff;
			txdata2=sav_addr1_xy4z2r0&0xff;
			data[268]=(txdata1<<8)|txdata2;
			check ^= data[268];
			txdata1=(sav_addr1_xy4z2r1>>24)&0xff;
			txdata2=(sav_addr1_xy4z2r1>>16)&0xff;
			data[269]=(txdata1<<8)|txdata2;
			check ^= data[269];
			txdata1=(sav_addr1_xy4z2r1>>8)&0xff;
			txdata2=sav_addr1_xy4z2r1&0xff;
			data[270]=(txdata1<<8)|txdata2;
			check ^= data[270];
			txdata1=(sav_addr1_xy4z2r2>>24)&0xff;
			txdata2=(sav_addr1_xy4z2r2>>16)&0xff;
			data[271]=(txdata1<<8)|txdata2;
			check ^= data[271];
			txdata1=(sav_addr1_xy4z2r2>>8)&0xff;
			txdata2=sav_addr1_xy4z2r2&0xff;
			data[272]=(txdata1<<8)|txdata2;
			check ^= data[272];
			txdata1=(sav_addr1_xy4z2r3>>24)&0xff;
			txdata2=(sav_addr1_xy4z2r3>>16)&0xff;
			data[273]=(txdata1<<8)|txdata2;
			check ^= data[273];
			txdata1=(sav_addr1_xy4z2r3>>8)&0xff;
			txdata2=sav_addr1_xy4z2r3&0xff;
			data[274]=(txdata1<<8)|txdata2;
			check ^= data[274];
			txdata1=(sav_addr1_xy4z2r4>>24)&0xff;
			txdata2=(sav_addr1_xy4z2r4>>16)&0xff;
			data[275]=(txdata1<<8)|txdata2;
			check ^= data[275];
			txdata1=(sav_addr1_xy4z2r4>>8)&0xff;
			txdata2=sav_addr1_xy4z2r4&0xff;
			data[276]=(txdata1<<8)|txdata2;
			check ^= data[276];
			txdata1=(sav_addr1_xy4z2r5>>24)&0xff;
			txdata2=(sav_addr1_xy4z2r5>>16)&0xff;
			data[277]=(txdata1<<8)|txdata2;
			check ^= data[277];
			txdata1=(sav_addr1_xy4z2r5>>8)&0xff;
			txdata2=sav_addr1_xy4z2r5&0xff;
			data[278]=(txdata1<<8)|txdata2;
			check ^= data[278];
			txdata1=(sav_addr1_xy4z3r0>>24)&0xff;
			txdata2=(sav_addr1_xy4z3r0>>16)&0xff;
			data[279]=(txdata1<<8)|txdata2;
			check ^= data[279];
			txdata1=(sav_addr1_xy4z3r0>>8)&0xff;
			txdata2=sav_addr1_xy4z3r0&0xff;
			data[280]=(txdata1<<8)|txdata2;
			check ^= data[280];
			txdata1=(sav_addr1_xy4z3r1>>24)&0xff;
			txdata2=(sav_addr1_xy4z3r1>>16)&0xff;
			data[281]=(txdata1<<8)|txdata2;
			check ^= data[281];
			txdata1=(sav_addr1_xy4z3r1>>8)&0xff;
			txdata2=sav_addr1_xy4z3r1&0xff;
			data[282]=(txdata1<<8)|txdata2;
			check ^= data[282];
			txdata1=(sav_addr1_xy4z3r2>>24)&0xff;
			txdata2=(sav_addr1_xy4z3r2>>16)&0xff;
			data[283]=(txdata1<<8)|txdata2;
			check ^= data[283];
			txdata1=(sav_addr1_xy4z3r2>>8)&0xff;
			txdata2=sav_addr1_xy4z3r2&0xff;
			data[284]=(txdata1<<8)|txdata2;
			check ^= data[284];
			txdata1=(sav_addr1_xy4z3r3>>24)&0xff;
			txdata2=(sav_addr1_xy4z3r3>>16)&0xff;
			data[285]=(txdata1<<8)|txdata2;
			check ^= data[285];
			txdata1=(sav_addr1_xy4z3r3>>8)&0xff;
			txdata2=sav_addr1_xy4z3r3&0xff;
			data[286]=(txdata1<<8)|txdata2;
			check ^= data[286];
			txdata1=(sav_addr1_xy4z3r4>>24)&0xff;
			txdata2=(sav_addr1_xy4z3r4>>16)&0xff;
			data[287]=(txdata1<<8)|txdata2;
			check ^= data[287];
			txdata1=(sav_addr1_xy4z3r4>>8)&0xff;
			txdata2=sav_addr1_xy4z3r4&0xff;
			data[288]=(txdata1<<8)|txdata2;
			check ^= data[288];
			txdata1=(sav_addr1_xy4z3r5>>24)&0xff;
			txdata2=(sav_addr1_xy4z3r5>>16)&0xff;
			data[289]=(txdata1<<8)|txdata2;
			check ^= data[289];
			txdata1=(sav_addr1_xy4z3r5>>8)&0xff;
			txdata2=sav_addr1_xy4z3r5&0xff;
			data[290]=(txdata1<<8)|txdata2;
			check ^= data[290];
			txdata1=(sav_addr1_xy4z4r0>>24)&0xff;
			txdata2=(sav_addr1_xy4z4r0>>16)&0xff;
			data[291]=(txdata1<<8)|txdata2;
			check ^= data[291];
			txdata1=(sav_addr1_xy4z4r0>>8)&0xff;
			txdata2=sav_addr1_xy4z4r0&0xff;
			data[292]=(txdata1<<8)|txdata2;
			check ^= data[292];
			txdata1=(sav_addr1_xy4z4r1>>25)&0xff;
			txdata2=(sav_addr1_xy4z4r1>>16)&0xff;
			data[293]=(txdata1<<8)|txdata2;
			check ^= data[293];
			txdata1=(sav_addr1_xy4z4r1>>8)&0xff;
			txdata2=sav_addr1_xy4z4r1&0xff;
			data[294]=(txdata1<<8)|txdata2;
			check ^= data[294];
			txdata1=(sav_addr1_xy4z4r2>>24)&0xff;
			txdata2=(sav_addr1_xy4z4r2>>16)&0xff;
			data[295]=(txdata1<<8)|txdata2;
			check ^= data[295];
			txdata1=(sav_addr1_xy4z4r2>>8)&0xff;
			txdata2=sav_addr1_xy4z4r2&0xff;
			data[296]=(txdata1<<8)|txdata2;
			check ^= data[296];
			txdata1=(sav_addr1_xy4z4r3>>24)&0xff;
			txdata2=(sav_addr1_xy4z4r3>>16)&0xff;
			data[297]=(txdata1<<8)|txdata2;
			check ^= data[297];
			txdata1=(sav_addr1_xy4z4r3>>8)&0xff;
			txdata2=sav_addr1_xy4z4r3&0xff;
			data[298]=(txdata1<<8)|txdata2;
			check ^= data[298];
			txdata1=(sav_addr1_xy4z4r4>>24)&0xff;
			txdata2=(sav_addr1_xy4z4r4>>16)&0xff;
			data[299]=(txdata1<<8)|txdata2;
			check ^= data[299];
			txdata1=(sav_addr1_xy4z4r4>>8)&0xff;
			txdata2=sav_addr1_xy4z4r4&0xff;
			data[300]=(txdata1<<8)|txdata2;
			check ^= data[300];
			txdata1=(sav_addr1_xy4z4r5>>24)&0xff;
			txdata2=(sav_addr1_xy4z4r5>>16)&0xff;
			data[301]=(txdata1<<8)|txdata2;
			check ^= data[301];
			txdata1=(sav_addr1_xy4z4r5>>8)&0xff;
			txdata2=sav_addr1_xy4z4r5&0xff;
			data[302]=(txdata1<<8)|txdata2;
			check ^= data[302];
			data[303]= check;
			MysciSend(data);
			brxstat = 0;
			break;
		case 114:                // read importat data
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 115;
					ovtimeX= 0;
				}
				else
					brxstat = 114;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 115:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 116;
					ovtimeX= 0;
				}
				else
					brxstat = 115;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 116:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 117;
					ovtimeX= 0;
				}
				else
					brxstat = 116;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 117:
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
				if(rxdata == 0x19)    //40 bytes parameter, 2 bytes CHKS
				{
					check=0x0019^0xffff;
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
				xy1=parameter;    // save xy1
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
				xy2=parameter;     // save bx
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
				xy3=parameter;     // save ay
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
				xy4=parameter;    // save by
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
				z1=parameter;
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
				z2=parameter;
				parameter=0;
				brxstat = 165;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 164;
			break;
		case 165:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 166;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 165;
			break;
		case 166:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 167;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 166;
			break;
		case 167:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 168;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 167;
			break;
		case 168:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				z3=parameter;
				parameter=0;
				brxstat = 169;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 168;
			break;
		case 169:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter= rxdata;
				ovtimeX= 0;
				brxstat = 170;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 169;
			break;
		case 170:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata;
				check ^= parameter;
				ovtimeX= 0;
				brxstat = 171;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 170;
			break;
		case 171:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata1 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata1;
				ovtimeX= 0;
				brxstat = 172;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 171;
			break;
		case 172:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata2 = rcvdata[i_rcv];
				i_rcv--;
				parameter=(parameter<<8)|rxdata2;
				check ^= (rxdata1<<8)|rxdata2;
				ovtimeX= 0;
				z4=parameter;
				parameter=0;
				brxstat = 177;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			else
				brxstat = 172;
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
			FMWREN ();
			WriteFMByte(0x14d4, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14d5, *p);
			p++;				
			FMWREN ();
			WriteFMByte(0x14d6, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14d7, *p);
		
			p = (char*)&br;
			FMWREN ();
			WriteFMByte(0x14d8, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14d9, *p);
			p++;					
			FMWREN ();
			WriteFMByte(0x14da, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14db, *p);
			p = (char*)&cr;
			FMWREN ();
			WriteFMByte(0x14dc, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14dd, *p);	
			p++;	
			FMWREN ();
			WriteFMByte(0x14de, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14df, *p);
			p = (char*)&dr;
			FMWREN ();
			WriteFMByte(0x14e0, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14e1, *p);		
			p++;		
			FMWREN ();
			WriteFMByte(0x14e2, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14e3, *p);
			p = (char*)&xy1;
			FMWREN ();
			WriteFMByte(0x14e4, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14e5, *p);			
			p++;
			FMWREN ();
			WriteFMByte(0x14e6, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14e7, *p);
			p = (char*)&xy2;
			FMWREN ();
			WriteFMByte(0x14e8, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14e9, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14ea, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14eb, *p);
			p = (char*)&xy3;
			FMWREN ();
			WriteFMByte(0x14ec, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14ed, *p);			
			p++;
			FMWREN ();
			WriteFMByte(0x14ee, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14ef, *p);
			p = (char*)&xy4;
			FMWREN ();
			WriteFMByte(0x14f0, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14f1, *p);
			p++;		
			FMWREN ();
			WriteFMByte(0x14f2, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14f3, *p);
			p = (char*)&z1;
			FMWREN ();
			WriteFMByte(0x14f4, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14f5, *p);
			p++;		
			FMWREN ();
			WriteFMByte(0x14f6, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14f7, *p);
			p = (char*)&z2;
			FMWREN ();
			WriteFMByte(0x14f8, *p);
			p++;
			FMWREN ();
			WriteFMByte(0x14f9, *p);		
			p++;			
			FMWREN ();
			WriteFMByte(0x14fa, *p);
			p++;
			FMWREN ();			
			WriteFMByte(0x14fb, *p);
			p = (char*)&z3;	
			FMWREN ();			
			WriteFMByte(0x14fc, *p);
			p++;			
			FMWREN ();			
			WriteFMByte(0x14fd, *p);		
			p++;
			FMWREN ();			
			WriteFMByte(0x14fe, *p);
			p++;			
			FMWREN ();			
			WriteFMByte(0x14ff, *p);		
			p = (char*)&z4;	
			FMWREN ();			
			WriteFMByte(0x1500, *p);
			p++;			
			FMWREN ();			
			WriteFMByte(0x1501, *p);		
			p++;
			FMWREN ();			
			WriteFMByte(0x1502, *p);
			p++;			
			FMWREN ();			
			WriteFMByte(0x1503, *p);		
			
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
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 181;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 181:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 182;
					ovtimeX= 0;
				}
				else
					brxstat = 181;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 182:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 184;
					ovtimeX= 0;
				}
				else
					brxstat = 182;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 184:
			data[0]=0x0019;
//			data[1]=(ReadFMByte(0x1266)<<8)|ReadFMByte(0x1267);
//			data[2]=(ReadFMByte(0x1268)<<8)|ReadFMByte(0x1269);    //read ar
//			data[3]=(ReadFMByte(0x126a)<<8)|ReadFMByte(0x126b);
//			data[4]=(ReadFMByte(0x126c)<<8)|ReadFMByte(0x126d);    //read br
//			data[5]=(ReadFMByte(0x126e)<<8)|ReadFMByte(0x126f);
//			data[6]=(ReadFMByte(0x1270)<<8)|ReadFMByte(0x1271);    //read cr
//			data[7]=(ReadFMByte(0x1272)<<8)|ReadFMByte(0x1273);
//			data[8]=(ReadFMByte(0x1274)<<8)|ReadFMByte(0x1275);    //read dr
//			data[9]=(ReadFMByte(0x1276)<<8)|ReadFMByte(0x1277);
//			data[10]=(ReadFMByte(0x1278)<<8)|ReadFMByte(0x1279);    //read xy1
//			data[11]=(ReadFMByte(0x127a)<<8)|ReadFMByte(0x127b);
//			data[12]=(ReadFMByte(0x127c)<<8)|ReadFMByte(0x127d);    //read xy2
//			data[13]=(ReadFMByte(0x127e)<<8)|ReadFMByte(0x127f);
//			data[14]=(ReadFMByte(0x1280)<<8)|ReadFMByte(0x1281);    //read xy3
//			data[15]=(ReadFMByte(0x1282)<<8)|ReadFMByte(0x1283);
//			data[16]=(ReadFMByte(0x1284)<<8)|ReadFMByte(0x1285);    //read xy4
//			data[17]=(ReadFMByte(0x1286)<<8)|ReadFMByte(0x1287);
//			data[18]=(ReadFMByte(0x1288)<<8)|ReadFMByte(0x1289);    //read z1
//			data[19]=(ReadFMByte(0x128a)<<8)|ReadFMByte(0x128b);
//			data[20]=(ReadFMByte(0x128c)<<8)|ReadFMByte(0x128d);    //read z2
//			data[21]=(ReadFMByte(0x128e)<<8)|ReadFMByte(0x128f);
//			data[22]=(ReadFMByte(0x1290)<<8)|ReadFMByte(0x1291);    //read z3
//			data[23]=(ReadFMByte(0x1292)<<8)|ReadFMByte(0x1293);
//			data[24]=(ReadFMByte(0x1294)<<8)|ReadFMByte(0x1295);    //read z4
			data[1]=((ar&0xffff0000)>>16);			
			data[2]=(ar&0x0000ffff);    //read ar
			data[3]=((br&0xffff0000)>>16);
			data[4]=(br&0x0000ffff);    //read br
			data[5]=((cr&0xffff0000)>>16);
			data[6]=(cr&0x0000ffff);    //read cr
			data[7]=((dr&0xffff0000)>>16);
			data[8]=(dr&0x0000ffff);    //read dr
			data[9]=((xy1&0xffff0000)>>16);
			data[10]=(xy1&0x0000ffff);    //read xy1
			data[11]=((xy2&0xffff0000)>>16);
			data[12]=(xy2&0x0000ffff);    //read xy2
			data[13]=((xy3&0xffff0000)>>16);
			data[14]=(xy3&0x0000ffff);    //read xy3
			data[15]=((xy4&0xffff0000)>>16);
			data[16]=(xy4&0x0000ffff);    //read xy4
			data[17]=((z1&0xffff0000)>>16);
			data[18]=(z1&0x0000ffff);    //read z1
			data[19]=((z2&0xffff0000)>>16);
			data[20]=(z2&0x0000ffff);    //read z2
			data[21]=((z3&0xffff0000)>>16);
			data[22]=(z3&0x0000ffff);    //read z3
			data[23]=((z4&0xffff0000)>>16);
			data[24]=(z4&0x0000ffff);    //read z4						
			data[25]=data[0]^data[1]^data[2]^data[3]^data[4]^data[5]^data[6]^data[7]^data[8]^data[9]^data[10]^data[11]^data[12]^data[13]^data[14]^data[15]^data[16]^data[17]^data[18]^data[19]^data[20]^data[21]^data[22]^data[23]^data[24]^0xffff;
			MysciSend(data);
			brxstat = 0;
			break;
		case 190:                         // read real time data
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x51)
				{
					brxstat = 191;
					ovtimeX= 0;
				}
				else
					brxstat = 0;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 191:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 192;
					ovtimeX= 0;
				}
				else
					brxstat = 191;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 192:
			ovtimeX++;
			if(i_rcv != 0)
			{
				rxdata = rcvdata[i_rcv];
				i_rcv--;
				if(rxdata == 0x00)
				{
					brxstat = 194;
					ovtimeX= 0;
				}
				else
					brxstat = 192;
			}
			else if(ovtimeX>999)	// overtime error, return back to status 0
				brxstat = 0;
			break;
		case 194:
			data[0]=0x0004;
			data[1]=accx_send;
			data[2]=accy_send;
			data[3]=accz_send;
			data[4]=data[0]^data[1]^data[2]^data[3]^0xFFFF;
			MysciSend(data);
			brxstat = 0;
			break;
		}
		/***************************** UART Receive loop end   ****************************/
		/***************************** UART transmit loop start ***************************/
		switch(btxstat)	
		{
		case 0:
			if(bstart_tx)
			{
				if(bSend && bComHighSpeed)		// when in the status of bSend and COM is high speed mode, then go to data sending
				{
					gioSetBit(gioPORTA,2,1);
//          sciSetBaudrate(scilinREG,115200U);
					btxstat = 4;
					sciDisableNotification(scilinREG,SCI_RX_INT);			//disable rx full interrput
//          MysciSendByte(0x67);
				}
				bstart_tx = 0;
			}
			break;
		case 4:
//			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
//			{
			switch(sendmode)
			{
			case 0:
//						scilinREG->TD = 0xda;				// send out 2kHz accx, accy, accz, ID
				temp_send = rtemperature;	// refresh temperature value
				btxstat = 11;
				break;
			case 1:
//						scilinREG->TD = 0xd0;
				btxstat = 5;
				break;
			case 2:
//						scilinREG->TD = 0xd1;
				btxstat = 7;
				break;
			case 3:
//						scilinREG->TD = 0xd2;
				btxstat = 9;
				break;
			case 4:
//						scilinREG->TD = 0xd3;
				btxstat = 20;
				break;
			default:
				break;
			}
//			}
			break;
		case 5:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = ((accx_send>>8) & 0xff);			// send out accx low byte
				scilinREG->TD = tmp;
				btxstat = 6;
			}
			break;
		case 6:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = (accx_send & 0xff);			// send out accx high byte
				scilinREG->TD = tmp;
				btxstat = 14;
			}
			break;
		case 7:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = ((accy_send>>8) & 0xff) ;			// send out accy low byte
				scilinREG->TD = tmp;
				btxstat = 8;
			}
			break;
		case 8:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = (accy_send & 0xff);			// send out accy high byte
				scilinREG->TD = tmp;
				btxstat = 14;
			}
			break;
		case 9:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = ((accz_send>>8) & 0xff);			// send out accz low byte
				scilinREG->TD = tmp;
				btxstat = 10;
			}
			break;
		case 10:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = (accz_send & 0xff);			// send out accy high byte
				scilinREG->TD = tmp;
				btxstat = 14;
			}
			break;
		case 11:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = ((temp_send>>8) & 0xff);			// send out temperature low byte
				scilinREG->TD = tmp;
				btxstat = 12;
			}
			break;
		case 12:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = (temp_send & 0xff);			// send out temperature high byte
				scilinREG->TD = tmp;
				btxstat = 14;
			}
			break;
		case 20:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = ((accr_send& 0xff00)>>8 );			// send out accr low byte
				scilinREG->TD =(uint8)tmp;
				btxstat = 21;
			}
			break;
		case 21:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected
			{
				tmp = (accr_send & 0x00ff);			// send out accr low byte
				scilinREG->TD = (uint8)tmp;
				btxstat = 14;
			}
			break;
		case 14:
			if((scilinREG->FLR & (uint32)SCI_TX_INT) != 0U)	// transmition empty detected, enable receive
			{
//				sciEnableNotification(scilinREG, SCI_RX_INT);			// enable rx full interrupt
				btxstat = 4;
			}
			break;
		default:
			btxstat = 0;
			break;
		}
		/***************************** UART transmit loop end *****************************/
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
				pword=&ar;
				arfloat=*((float *)pword);
				pword=&br;
				brfloat=*((float *)pword);
				pword=&cr;
				crfloat=*((float *)pword);
				pword=&dr;
				drfloat=*((float *)pword);
				pword=&xy1;
				xy1float=*((float *)pword);
				pword=&xy2;
				xy2float=*((float *)pword);
				pword=&xy3;
				xy3float=*((float *)pword);
				pword=&xy4;
				xy4float=*((float *)pword);
				pword=&z1;
				z1float=*((float *)pword);
				pword=&z2;
				z2float=*((float *)pword);
				pword=&z3;
				z3float=*((float *)pword);
				pword=&z4;
				z4float=*((float *)pword);
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
			for(ia2k_sav_cnt=0; ia2k_sav_cnt<10240; ia2k_sav_cnt++)
			{
				if (adcflag==0)
				{
					floatx=abs(accx_2k_buf1[ia2k_sav_cnt]-avg_sec_cur_x);     // to eliminate the DC component
					floaty=abs(accy_2k_buf1[ia2k_sav_cnt]-avg_sec_cur_y);
					floatz=abs(accz_2k_buf1[ia2k_sav_cnt]-avg_sec_cur_z);
				}
				else
				{
					floatx=abs(accx_2k_buf0[ia2k_sav_cnt]-avg_sec_cur_x);     // to eliminate the DC component
					floaty=abs(accy_2k_buf0[ia2k_sav_cnt]-avg_sec_cur_y);
					floatz=abs(accz_2k_buf0[ia2k_sav_cnt]-avg_sec_cur_z);
				}
				rms_sum_sec_x=rms_sum_sec_x+(floatx*floatx);
				rms_sum_sec_y=rms_sum_sec_y+(floaty*floaty);
				rms_sum_sec_z=rms_sum_sec_z+(floatz*floatz);
				if(abs(peak_sec_x) < abs(floatx))
				{
					// Calculate peak value, accx
					peak_sec_x = abs(floatx);
				}
				if(abs(peak_sec_y) < abs(floaty))
				{
					// Calculate peak value, accy
					peak_sec_y = abs(floaty);
				}
				if(abs(peak_sec_z) < abs(floatz))
				{
					// Calculate peak value, accz
					peak_sec_z = abs(floatz);
				}
			}
//        bSave=0;
			sav_stat=4;
			break;
		case 4:
			ia2k_sav_cnt = 0;
			acc2k_time=CacuSecondTime(year,month,day,hour,minute,second);
			sav_stat=5;
			break;
		case 5:
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x03);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif	
			rms_sec_cur_x = rms_sum_sec_x/10240;  //normalization
			rms_sec_cur_y = rms_sum_sec_y/10240;
			rms_sec_cur_z = rms_sum_sec_z/10240;
			rms_cur_bulk=(rms_sum_sec_x+rms_sum_sec_y)/20480;
//        rms_cur_bulk_float=(rms_sum_sec_x+rms_sum_sec_y)/10240;
//			  rms_sec_cur_z_float = rms_sum_sec_z_float/10240;
			rms_sum_sec_x = 0;
			rms_sum_sec_y = 0;
			rms_sum_sec_z = 0;
			rms_sum_bulk =0;
//        rms_sum_bulk_float=0;
//				rms_sum_sec_x_float=0;
//				rms_sum_sec_y_float=0;
//				rms_sum_sec_z_float=0;
			peak_sec_cur_x = peak_sec_x;
			peak_sec_cur_y = peak_sec_y;
			peak_sec_cur_z = peak_sec_z;
			peak_sec_x = 0;
			peak_sec_y = 0;
			peak_sec_z = 0;
			sav_stat=6;
			break;
		case 6:
			if(avg_sec_cur_r_float<0.1)
			{
				s1=0;
				s2=0;
			}    //to set a zero zone for avg_sec_cur_r_float
			else
			{
				s1=absf((peak_sec_cur_r_float-bottom_sec_cur_r_float)/(2*avg_sec_cur_r_float));
				s2=(minusR+0.01)/10240;
			}
			sav_stat=7;
			break;
		case 7:
			if (rms_cur_bulk<xy1float)       // calculate xylevel 0.2g
			{
				xylevel=0x00;
			}
			else if (rms_cur_bulk<xy2float)       // calculate xylevel 0.5g
			{
				xylevel=0x01;
			}
			else if (rms_cur_bulk<xy3float)       // calculate xylevel 1g
			{
				xylevel=0x02;
			}
			else if (rms_cur_bulk<xy4float)    //calculate xylevel 2g
			{
				xylevel=0x03;
			}
			else
			{
				xylevel=0x04;
			}
			if (rms_sec_cur_z<z1float)       // calculate zlevel 0.2g
			{
				zlevel=0x00;
			}
			else if (rms_sec_cur_z<z2float)      // calculate zlevel 0.5g
			{
				zlevel=0x01;
			}
			else if (rms_sec_cur_z<z3float)       // calculate zlevel 1g
			{
				zlevel=0x02;
			}
			else if (rms_sec_cur_z<z4float)    //calculate zlevel 2g
			{
				zlevel=0x03;
			}
			else
			{
				zlevel=0x04;
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
			else if (s1<0.5)
			{
				rlevel=0x03;
			}
			else
			{
				rlevel=0x04;
			}
			if (s2>0.1)
			{
				rlevel=0x05;
			}
			minusR=0;
			sav_stat=8;
			break;
		case 8:
			bsav_data = 1;	
			switch(xylevel)
			{
			case 0:
				switch(zlevel)
				{
				case 0:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy0z0r0<0x1A5500)
							psav_addr1=&sav_addr1_xy0z0r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy0z0r1<0x34AA00)
							psav_addr1=&sav_addr1_xy0z0r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy0z0r2<0x4EFF00)
							psav_addr1=&sav_addr1_xy0z0r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy0z0r3<0x695400)
							psav_addr1=&sav_addr1_xy0z0r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy0z0r4<0x83A900)
							psav_addr1=&sav_addr1_xy0z0r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy0z0r5<0x9DFE00)
							psav_addr1=&sav_addr1_xy0z0r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 1:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy0z1r0<0xB85300)
							psav_addr1=&sav_addr1_xy0z1r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy0z1r1<0xD2A800)
							psav_addr1=&sav_addr1_xy0z1r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy0z1r2<0xECFD00)
							psav_addr1=&sav_addr1_xy0z1r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy0z1r3<0x1075200)
							psav_addr1=&sav_addr1_xy0z1r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy0z1r4<0x121A700)
							psav_addr1=&sav_addr1_xy0z1r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy0z1r5<0x13BFC00)
							psav_addr1=&sav_addr1_xy0z1r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 2:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy0z2r0<0x1565100)
							psav_addr1=&sav_addr1_xy0z2r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy0z2r1<0x170A600)
							psav_addr1=&sav_addr1_xy0z2r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy0z2r2<0x18AFB00)
							psav_addr1=&sav_addr1_xy0z2r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy0z2r3<0x1A55000)
							psav_addr1=&sav_addr1_xy0z2r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy0z2r4<0x1BFA500)
							psav_addr1=&sav_addr1_xy0z2r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy0z2r5<0x1D9FA00)
							psav_addr1=&sav_addr1_xy0z2r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 3:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy0z3r0<0x1F44F00)
							psav_addr1=&sav_addr1_xy0z3r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy0z3r1<0x20EA400)
							psav_addr1=&sav_addr1_xy0z3r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy0z3r2<0x228F900)
							psav_addr1=&sav_addr1_xy0z3r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy0z3r3<0x2434E00)
							psav_addr1=&sav_addr1_xy0z3r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy0z3r4<0x25DA300)
							psav_addr1=&sav_addr1_xy0z3r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy0z3r5<0x277F800)
							psav_addr1=&sav_addr1_xy0z3r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 4:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy0z4r0<0x2924D00)
							psav_addr1=&sav_addr1_xy0z4r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy0z4r1<0x2ACA200)
							psav_addr1=&sav_addr1_xy0z4r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy0z4r2<0x2C6F700)
							psav_addr1=&sav_addr1_xy0z4r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy0z4r3<0x2E14C00)
							psav_addr1=&sav_addr1_xy0z4r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy0z4r4<0x2FBA100)
							psav_addr1=&sav_addr1_xy0z4r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy0z4r5<0x315F600)
							psav_addr1=&sav_addr1_xy0z4r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				}
				break;
			case 1:
				switch(zlevel)
				{
				case 0:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy1z0r0<0x3304B00)
							psav_addr1=&sav_addr1_xy1z0r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy1z0r1<0x34AA000)
							psav_addr1=&sav_addr1_xy1z0r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy1z0r2<0x364F500)
							psav_addr1=&sav_addr1_xy1z0r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy1z0r3<0x37F4A00)
							psav_addr1=&sav_addr1_xy1z0r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy1z0r4<0x3999F00)
							psav_addr1=&sav_addr1_xy1z0r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy1z0r5<0x3B3F400)
							psav_addr1=&sav_addr1_xy1z0r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 1:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy1z1r0<0x3CE4900)
							psav_addr1=&sav_addr1_xy1z1r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy1z1r1<0x3E89E00)
							psav_addr1=&sav_addr1_xy1z1r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy1z1r2<0x402F300)
							psav_addr1=&sav_addr1_xy1z1r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy1z1r3<0x41D4800)
							psav_addr1=&sav_addr1_xy1z1r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy1z1r4<0x4379D00)
							psav_addr1=&sav_addr1_xy1z1r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy1z1r5<0x451F200)
							psav_addr1=&sav_addr1_xy1z1r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 2:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy1z2r0<0x46C4700)
							psav_addr1=&sav_addr1_xy1z2r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy1z2r1<0x4869C00)
							psav_addr1=&sav_addr1_xy1z2r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy1z2r2<0x4A0F100)
							psav_addr1=&sav_addr1_xy1z2r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy1z2r3<0x4BB4600)
							psav_addr1=&sav_addr1_xy1z2r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy1z2r4<0x4D59B00)
							psav_addr1=&sav_addr1_xy1z2r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy1z2r5<0x4EFF000)
							psav_addr1=&sav_addr1_xy1z2r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 3:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy1z3r0<0x50A4500)
							psav_addr1=&sav_addr1_xy1z3r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy1z3r1<0x5249A00)
							psav_addr1=&sav_addr1_xy1z3r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy1z3r2<0x53EEF00)
							psav_addr1=&sav_addr1_xy1z3r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy1z3r3<0x5594400)
							psav_addr1=&sav_addr1_xy1z3r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy1z3r4<0x5739900)
							psav_addr1=&sav_addr1_xy1z3r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy1z3r5<0x58DEE00)
							psav_addr1=&sav_addr1_xy1z3r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 4:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy1z4r0<0x5A84300)
							psav_addr1=&sav_addr1_xy1z4r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy1z4r1<0x5C29800)
							psav_addr1=&sav_addr1_xy1z4r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy1z4r2<0x5DCED00)
							psav_addr1=&sav_addr1_xy1z4r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy1z4r3<0x5F74200)
							psav_addr1=&sav_addr1_xy1z4r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy1z4r4<0x6119700)
							psav_addr1=&sav_addr1_xy1z4r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy1z4r5<0x62BEC00)
							psav_addr1=&sav_addr1_xy1z4r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				}
				break;
			case 2:
				switch(zlevel)
				{
				case 0:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy2z0r0<0x6464100)
							psav_addr1=&sav_addr1_xy2z0r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy2z0r1<0x6609600)
							psav_addr1=&sav_addr1_xy2z0r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy2z0r2<0x67AEB00)
							psav_addr1=&sav_addr1_xy2z0r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy2z0r3<0x6954000)
							psav_addr1=&sav_addr1_xy2z0r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy2z0r4<0x6AF9500)
							psav_addr1=&sav_addr1_xy2z0r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy2z0r5<0x6C9EA00)
							psav_addr1=&sav_addr1_xy2z0r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 1:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy2z1r0<0x6E43F00)
							psav_addr1=&sav_addr1_xy2z1r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy2z1r1<0x6FE9400)
							psav_addr1=&sav_addr1_xy2z1r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy2z1r2<0x718E900)
							psav_addr1=&sav_addr1_xy2z1r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy2z1r3<0x7333E00)
							psav_addr1=&sav_addr1_xy2z1r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy2z1r4<0x74D9300)
							psav_addr1=&sav_addr1_xy2z1r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy2z1r5<0x767E800)
							psav_addr1=&sav_addr1_xy2z1r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 2:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy2z2r0<0x7823D00)
							psav_addr1=&sav_addr1_xy2z2r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy2z2r1<0x79C9200)
							psav_addr1=&sav_addr1_xy2z2r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy2z2r2<0x7B6E700)
							psav_addr1=&sav_addr1_xy2z2r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy2z2r3<0x7D13C00)
							psav_addr1=&sav_addr1_xy2z2r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy2z2r4<0x7EB9100)
							psav_addr1=&sav_addr1_xy2z2r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy2z2r5<0x805E600)
							psav_addr1=&sav_addr1_xy2z2r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 3:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy2z3r0<0x8203B00)
							psav_addr1=&sav_addr1_xy2z3r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy2z3r1<0x83A9000)
							psav_addr1=&sav_addr1_xy2z3r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy2z3r2<0x854E500)
							psav_addr1=&sav_addr1_xy2z3r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy2z3r3<0x86F3A00)
							psav_addr1=&sav_addr1_xy2z3r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy2z3r4<0x8898F00)
							psav_addr1=&sav_addr1_xy2z3r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy2z3r5<0x8A3E400)
							psav_addr1=&sav_addr1_xy2z3r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 4:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy2z4r0<0x8BE3900)
							psav_addr1=&sav_addr1_xy2z4r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy2z4r1<0x8D88E00)
							psav_addr1=&sav_addr1_xy2z4r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy2z4r2<0x8F2E300)
							psav_addr1=&sav_addr1_xy2z4r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy2z4r3<0x90D3800)
							psav_addr1=&sav_addr1_xy2z4r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy2z4r4<0x9278D00)
							psav_addr1=&sav_addr1_xy2z4r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy2z4r5<0x941E200)
							psav_addr1=&sav_addr1_xy2z4r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				}
				break;
			case 3:
				switch(zlevel)
				{
				case 0:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy3z0r0<0x95C3700)
							psav_addr1=&sav_addr1_xy3z0r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy3z0r1<0x9768C00)
							psav_addr1=&sav_addr1_xy3z0r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy3z0r2<0x990E100)
							psav_addr1=&sav_addr1_xy3z0r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy3z0r3<0x9AB3600)
							psav_addr1=&sav_addr1_xy3z0r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy3z0r4<0x9C58B00)
							psav_addr1=&sav_addr1_xy3z0r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy3z0r5<0x9DFE000)
							psav_addr1=&sav_addr1_xy3z0r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 1:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy3z1r0<0x9FA3500)
							psav_addr1=&sav_addr1_xy3z1r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy3z1r1<0xA148A00)
							psav_addr1=&sav_addr1_xy3z1r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy3z1r2<0xA2EDF00)
							psav_addr1=&sav_addr1_xy3z1r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy3z1r3<0xA493400)
							psav_addr1=&sav_addr1_xy3z1r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy3z1r4<0xA638900)
							psav_addr1=&sav_addr1_xy3z1r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy3z1r5<0xA7DDE00)
							psav_addr1=&sav_addr1_xy3z1r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 2:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy3z2r0<0xA983300)
							psav_addr1=&sav_addr1_xy3z2r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy3z2r1<0xAB28800)
							psav_addr1=&sav_addr1_xy3z2r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy3z2r2<0xACCDD00)
							psav_addr1=&sav_addr1_xy3z2r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy3z2r3<0xAE73200)
							psav_addr1=&sav_addr1_xy3z2r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy3z2r4<0xB018700)
							psav_addr1=&sav_addr1_xy3z2r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy3z2r5<0xB1BDC00)
							psav_addr1=&sav_addr1_xy3z2r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 3:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy3z3r0<0xB363100)
							psav_addr1=&sav_addr1_xy3z3r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy3z3r1<0xB508600)
							psav_addr1=&sav_addr1_xy3z3r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy3z3r2<0xB6ADB00)
							psav_addr1=&sav_addr1_xy3z3r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy3z3r3<0xB853000)
							psav_addr1=&sav_addr1_xy3z3r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy3z3r4<0xB9F8500)
							psav_addr1=&sav_addr1_xy3z3r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy3z3r5<0xBB9DA00)
							psav_addr1=&sav_addr1_xy3z3r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 4:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy3z4r0<0xBD42F00)
							psav_addr1=&sav_addr1_xy3z4r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy3z4r1<0xBEE8400)
							psav_addr1=&sav_addr1_xy3z4r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy3z4r2<0xC08D900)
							psav_addr1=&sav_addr1_xy3z4r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy3z4r3<0xC232E00)
							psav_addr1=&sav_addr1_xy3z4r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy3z4r4<0xC3D8300)
							psav_addr1=&sav_addr1_xy3z4r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy3z4r5<0xC57D800)
							psav_addr1=&sav_addr1_xy3z4r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				}
				break;
			case 4:
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x03);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif	
				switch(zlevel)
				{
				case 0:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy4z0r0<0xC722D00)
							psav_addr1=&sav_addr1_xy4z0r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy4z0r1<0xC8C8200)
							psav_addr1=&sav_addr1_xy4z0r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy4z0r2<0xCA6D700)
							psav_addr1=&sav_addr1_xy4z0r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy4z0r3<0xCC12C00)
							psav_addr1=&sav_addr1_xy4z0r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy4z0r4<0xCDB8100)
							psav_addr1=&sav_addr1_xy4z0r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy4z0r5<0xCF5D600)
							psav_addr1=&sav_addr1_xy4z0r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 1:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy4z1r0<0xD102B00)
							psav_addr1=&sav_addr1_xy4z1r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy4z1r1<0xD2A8000)
							psav_addr1=&sav_addr1_xy4z1r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy4z1r2<0xD44D500)
							psav_addr1=&sav_addr1_xy4z1r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy4z1r3<0xD5F2A00)
							psav_addr1=&sav_addr1_xy4z1r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy4z1r4<0xD797F00)
							psav_addr1=&sav_addr1_xy4z1r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy4z1r5<0xD93D400)
							psav_addr1=&sav_addr1_xy4z1r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 2:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy4z2r0<0xDAE2900)
							psav_addr1=&sav_addr1_xy4z2r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy4z2r1<0xDC87E00)
							psav_addr1=&sav_addr1_xy4z2r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy4z2r2<0xDE2D300)
							psav_addr1=&sav_addr1_xy4z2r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy4z2r3<0xDFD2800)
							psav_addr1=&sav_addr1_xy4z2r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy4z2r4<0xE177D00)
							psav_addr1=&sav_addr1_xy4z2r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy4z2r5<0xE31D200)
							psav_addr1=&sav_addr1_xy4z2r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 3:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy4z3r0<0xE4C2700)
							psav_addr1=&sav_addr1_xy4z3r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy4z3r1<0xE667C00)
							psav_addr1=&sav_addr1_xy4z3r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy4z3r2<0xE80D100)
							psav_addr1=&sav_addr1_xy4z3r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy4z3r3<0xE9B2600)
							psav_addr1=&sav_addr1_xy4z3r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy4z3r4<0xEB57B00)
							psav_addr1=&sav_addr1_xy4z3r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy4z3r5<0xECFD000)
							psav_addr1=&sav_addr1_xy4z3r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				case 4:
					switch(rlevel)
					{
					case 0:
						if(sav_addr1_xy4z4r0<0xEEA2500)
							psav_addr1=&sav_addr1_xy4z4r0;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 1:
						if(sav_addr1_xy4z4r1<0xF047A00)
							psav_addr1=&sav_addr1_xy4z4r1;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 2:
						if(sav_addr1_xy4z4r2<0xF1ECF00)
							psav_addr1=&sav_addr1_xy4z4r2;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 3:
						if(sav_addr1_xy4z4r3<0xF392400)
							psav_addr1=&sav_addr1_xy4z4r3;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 4:
						if(sav_addr1_xy4z4r4<0xF537900)
							psav_addr1=&sav_addr1_xy4z4r4;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					case 5:
						if(sav_addr1_xy4z4r5<0xF6DCE00)
							psav_addr1=&sav_addr1_xy4z4r5;
						else
						{
							psav_addr1=NULL;
							bsav_data=0;
						}  // if this block is full, stop the storage.
						break;
					default:
						break;
					}
					break;
				}
				break;
			default:
				break;
			}
			sav_stat=9;
			break;
			//=====================================================================
			//
			//*******************Saving character data start****************
			//
			//=======================================================================
		case 9:
			if(sav_addr0<0xfffffff)
			{
				gioA_tmp0=0xff;
				gioB_tmp0=0xef;        // Select F4
				sav_stat=10;
			}
			else
			{
				bSave=0;
				sav_stat=0;
			}
			break;
		case 10:		
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
			sav_stat = 11;
			break;
		case 11:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp0);
				gioSetPort(gioPORTB,gioB_tmp0);
				spi2sendByte(0xC5);  //WRITE Segment Choose Register
				sav_stat = 12;
			}
			break;
		case 12:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr0&0x03000000)>>24);   //Segment Choose
				sav_stat = 13;
				
				tmp_spi = 0x00;
//				while ( (tmp_spi&0x80) != 0x80 )
//				{

//					gioSetBit( gioPORTB, 4, 0 );
//					spiTransmitAndReceiveData( spiREG2, &dataconfig1_t, 2, ReadFlagStatusData, ReceiveFlagStatusData );
//					tmp_spi = ReceiveFlagStatusData[1];
//					/*  MysciSendByte(tmp); */
//					MyGioSetPortA( 0xff );
//					gioSetPort( gioPORTB, 0xff );	
//#ifdef watchdog
//		rtiREG1->WDKEY=0xE51A;   //feed the watchdog
//		rtiREG1->WDKEY=0xA35C;	 //feed the watchdog
//#endif			
//				}
				
			}
			break;
		case 13:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp0);
				gioSetPort(gioPORTB,gioB_tmp0);
				spi2sendByte(0x06); 	  //write enable
				sav_stat = 14;
			}
			break;
		case 14:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				MyGioSetPortA(gioA_tmp0);
				gioSetPort(gioPORTB,gioB_tmp0);
				spi2sendByte(0x02);  //WRITE
				sav_stat = 15;
			}
			break;
		case 15:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr0&0xff0000)>>16);  //address
				sav_stat = 16;
			}
			break;
		case 16:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((sav_addr0&0x00ff00)>>8);
				sav_stat = 17;
			}
			break;
		case 17:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(sav_addr0&0x0000ff);
				sav_stat = 18;
			}
			break;
		case 18:		// Sending time information
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
				sav_stat = 19;
			}
			break;
		case 19:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((acc2k_time>>16) & 0xff);					// Send time
				sav_stat = 20;
			}
			break;
		case 20:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((acc2k_time>>8) & 0xff);					// Send time
				sav_stat = 21;
			}
			break;
		case 21:		// Sending time information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(acc2k_time & 0xff);					// Send time
				sav_stat = 22;
			}
			break;
		case 22:		// Sending temperature information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				temp_write0 = temperature;
				spi2sendByte((temp_write0>>8) & 0xff);				// Send temperature
				sav_stat = 23;
			}
			break;
		case 23:		// Sending temperature information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(temp_write0 & 0xff);					  // Send temperature
				sav_stat = 24;
			}
			break;
		case 24:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(xylevel);					// Send xylevel
				sav_stat = 25;
			}
			break;
		case 25:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(zlevel);					// Send zlevel
				sav_stat = 26;
			}
			break;
		case 26:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(rlevel);					// Send rlevel
				sav_stat = 27;
			}
			break;
		case 27:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
#ifdef debug
				sciDisableNotification(scilinREG,SCI_RX_INT);		 //for debug			///////////2016.09.05
				gioSetBit(gioPORTA,2,1);
				MysciSendByte(0x27);  //for debug
				gioSetBit(gioPORTA,2,0);
				sciEnableNotification(scilinREG,SCI_RX_INT);
#endif	
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_cur_bulk>>56)&0xff); 					// Send xy RMS
				sav_stat = 28;
			}
			break;
		case 28:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_cur_bulk>>48)&0xff); 					// Send xy RMS
				sav_stat = 29;
			}
			break;
		case 29:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_cur_bulk>>40)&0xff); 					// Send xy RMS
				sav_stat = 30;
			}
			break;
		case 30:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_cur_bulk>>32)&0xff); 					// Send xy RMS
				sav_stat = 31;
			}
			break;
		case 31:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_cur_bulk>>24)&0xff); 					// Send xy RMS
				sav_stat = 32;
			}
			break;
		case 32:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_cur_bulk>>16)&0xff); 					// Send xy RMS
				sav_stat = 33;
			}
			break;
		case 33:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_cur_bulk>>8)&0xff); 					// Send xy RMS
				sav_stat = 34;
			}
			break;
		case 34:		// Sending RMSxy information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(rms_cur_bulk&0xff); 					      // Send xy RMS
				sav_stat = 35;
			}
			break;
		case 35:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_sec_cur_z>>56)&0xff); 				// Send z RMS
				sav_stat = 36;
			}
			break;
		case 36:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_sec_cur_z>>48)&0xff); 				// Send z RMS
				sav_stat = 37;
			}
			break;
		case 37:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_sec_cur_z>>40)&0xff); 				// Send z RMS
				sav_stat = 38;
			}
			break;
		case 38:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_sec_cur_z>>32)&0xff); 				// Send z RMS
				sav_stat = 39;
			}
			break;
		case 39:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_sec_cur_z>>24)&0xff); 				// Send z RMS
				sav_stat = 40;
			}
			break;
		case 40:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_sec_cur_z>>16)&0xff); 				// Send z RMS
				sav_stat = 41;
			}
			break;
		case 41:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((rms_sec_cur_z>>8)&0xff); 					// Send z RMS
				sav_stat = 42;
			}
			break;
		case 42:		// Sending RMSz information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(rms_sec_cur_z&0xff); 					    // Send z RMS
				sav_stat = 43;
			}
			break;
		case 43:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((peak_sec_cur_r>>8)&0xff);         // Send peak r
				sav_stat = 44;
			}
			break;
		case 44:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((peak_sec_cur_r)&0xff); 					  // Send peak r
				sav_stat = 45;
			}
			break;
		case 45:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((bottom_sec_cur_r>>8)&0xff);       // Send bottom r
				sav_stat = 46;
			}
			break;
		case 46:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(bottom_sec_cur_r&0xff); 					  // Send bottom r
//        savcnt0 = 0;		// set save data count equal 0 used to count 31 acc data
				sav_stat = 47;
			}
			break;
		case 47:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((avg_sec_cur_r>>8)&0xff);          // Send avg r
				sav_stat = 48;
			}
			break;
		case 48:		//
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(avg_sec_cur_r&0xff); 					    // Send avg r
				sav_stat = 49;
			}
			break;
		case 49:		//finish character data storage
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
				sav_stat = 50;
			}
			break;
		case 50:	          //  Inquire	Flag Status Register
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
				sav_stat = 100;
			}
			else
			{
				sav_stat = 51;
			}
			break;
		case 100:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(0x00);
				sav_stat = 101;
			}
			break;
		case 101:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				sav_stat = 50;
			}
			break;
			//=====================================================================
			//
			//*******************Saving character data END****************
			//
			//=======================================================================
		case 51:		//
			if(bsav_data==1)
			{
				if(*psav_addr1<0x04000000)
				{
					gioA_tmp1 = 0xef;
					gioB_tmp1 = 0xff;   // Select F1
					sav_stat = 52;
				}
				else if(*psav_addr1<0x08000000)
				{
					gioA_tmp1 = 0xfd;
					gioB_tmp1 = 0xff;		// Select F2
					sav_stat = 52;
				}
				else if(*psav_addr1<0x0c000000)
				{
					gioA_tmp1 = 0xff;
					gioB_tmp1 = 0x7f;		// Select F3
					sav_stat = 52;
				}
				else if(*psav_addr1<0x10000000)
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
				spi2sendByte((*psav_addr1&0x03000000)>>24);   // Segment Choose
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
				spi2sendByte((*psav_addr1&0xff0000)>>16);   // Send address HSB
				sav_stat = 58;
			}
			break;
		case 58:		// send address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((*psav_addr1&0x00ff00)>>8);   // Send address MSB
				sav_stat = 59;
			}
			break;
		case 59:		// send address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(*psav_addr1&0x0000ff);	 				// Send address LSB
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
				sav_stat = 64;
			}
			break;
		case 64:		// Sending temperature information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				temp_write0 = temperature;						   // refresh temperature value
				spi2sendByte((temp_write0>>8) & 0xff); 				// Send temperature HSB
				sav_stat = 65;
			}
			break;
		case 65:		// Sending temperature information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(temp_write0 & 0xff); 					// Send temperature LSB
				sav_stat = 66;
			}
			break;
		case 66:		// Sending xylevel information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(xylevel); 					// Send xylevel
				sav_stat = 67;
			}
			break;
		case 67:		// Sending zlevel information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(zlevel); 					// Send zlevel
				sav_stat = 68;
			}
			break;
		case 68:		// Sending rlevel information
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(rlevel); 					// Send rlevel
				sav_stat = 69;
			}
			break;
		case 69:
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				MyGioSetPortA(0xff);
				gioSetPort(gioPORTB,0xff);
				(*psav_addr1)+=0x100;
				sav_stat = 102;
				tmp_spi = 0x00;
			}
			break;
		case 102:	          //  Inquire	Flag Status Register
			if((tmp_spi&0x80)==0x00)
			{
				if((*psav_addr1)<0x04000000)
				{
					gioSetBit(gioPORTA,4,0);		// Select F1
				}
				else if((*psav_addr1)<0x08000000)
				{
					gioSetBit(gioPORTA,1,0);		// Select F2
				}
				else if((*psav_addr1)<0x0c000000)
				{
					gioSetBit(gioPORTB,7,0);		// Select F3
				}
				else if((*psav_addr1)<0x10000000)
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
			if(*psav_addr1<0x04000000)
			{
				gioA_tmp1 = 0xef;
				gioB_tmp1 = 0xff;   // Select F1
				sav_stat = 71;
			}
			else if(*psav_addr1<0x08000000)
			{
				gioA_tmp1 = 0xfd;
				gioB_tmp1 = 0xff;		// Select F2
				sav_stat = 71;
			}
			else if(*psav_addr1<0x0c000000)
			{
				gioA_tmp1 = 0xff;
				gioB_tmp1 = 0x7f;		// Select F3
				sav_stat = 71;
			}
			else if(*psav_addr1<0x10000000)
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
				spi2sendByte((*psav_addr1&0x03000000)>>24);    //Segment Choose
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
				spi2sendByte((*psav_addr1&0xff0000)>>16);
				sav_stat = 79;
			}
			break;
		case 79:		//  Sending address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte((*psav_addr1&0x00ff00)>>8);
				sav_stat = 80;
			}
			break;
		case 80:		//  Sending address
			if((spiREG2->FLG & 0x00000100U) == 0x00000100U)
			{
				tmp_spi = spiREG2->BUF;
				spi2sendByte(*psav_addr1&0x0000ff);
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
				if((*psav_addr1)<0x04000000)
				{
					gioSetBit(gioPORTA,4,0);		// Select F1
				}
				else if((*psav_addr1)<0x08000000)
				{
					gioSetBit(gioPORTA,1,0);		// Select F2
				}
				else if((*psav_addr1)<0x0c000000)
				{
					gioSetBit(gioPORTB,7,0);		// Select F3
				}
				else if((*psav_addr1)<0x10000000)
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
			(*psav_addr1)=(*psav_addr1)+256	;
			page_cnt++;
			if(page_cnt==320)
			{
				page_cnt=0;
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
				spi1sendByte((sav_addr1_xy0z0r0>>24)&0xff);
				sav_fm_stat = 21;
			}
			break;
		case 21:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r0>>16)&0xff);
				sav_fm_stat = 22;
			}
			break;
		case 22:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r0>>8)&0xff);
				sav_fm_stat = 23;
			}
			break;
		case 23:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r0&0xff);
				cksmspi1+=sav_addr1_xy0z0r0;
				sav_fm_stat = 24;
			}
			break;
		case 24:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r1>>24)&0xff);
				sav_fm_stat = 25;
			}
			break;
		case 25:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r1>>16)&0xff);
				sav_fm_stat = 26;
			}
			break;
		case 26:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r1>>8)&0xff);
				sav_fm_stat = 27;
			}
			break;
		case 27:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r1&0xff);
				cksmspi1+=sav_addr1_xy0z0r1;
				sav_fm_stat = 28;
			}
			break;
		case 28:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r2>>24)&0xff);
				sav_fm_stat = 29;
			}
			break;
		case 29:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r2>>16)&0xff);
				sav_fm_stat = 30;
			}
			break;
		case 30:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r2>>8)&0xff);
				sav_fm_stat = 31;
			}
			break;
		case 31:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r2&0xff);
				cksmspi1+=sav_addr1_xy0z0r2;
				sav_fm_stat = 32;
			}
			break;
		case 32:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r3>>24)&0xff);
				sav_fm_stat = 33;
			}
			break;
		case 33:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r3>>16)&0xff);
				sav_fm_stat = 34;
			}
			break;
		case 34:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r3>>8)&0xff);
				sav_fm_stat = 35;
			}
			break;
		case 35:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r3&0xff);
				cksmspi1+=sav_addr1_xy0z0r3;
				sav_fm_stat = 36;
			}
			break;
		case 36:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r4>>24)&0xff);
				sav_fm_stat = 37;
			}
			break;
		case 37:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r4>>16)&0xff);
				sav_fm_stat = 38;
			}
			break;
		case 38:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r4>>8)&0xff);
				sav_fm_stat = 39;
			}
			break;
		case 39:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r4&0xff);
				cksmspi1+=sav_addr1_xy0z0r4;
				sav_fm_stat = 40;
			}
			break;
		case 40:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r5>>24)&0xff);
				sav_fm_stat = 41;
			}
			break;
		case 41:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r5>>16)&0xff);
				sav_fm_stat = 42;
			}
			break;
		case 42:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r5>>8)&0xff);
				sav_fm_stat = 43;
			}
			break;
		case 43:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r5&0xff);
				cksmspi1+=sav_addr1_xy0z0r5;
				sav_fm_stat = 44;
			}
			break;
		case 44:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r0>>24)&0xff);
				sav_fm_stat = 45;
			}
			break;
		case 45:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r0>>16)&0xff);
				sav_fm_stat = 46;
			}
			break;
		case 46:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r0>>8)&0xff);
				sav_fm_stat = 47;
			}
			break;
		case 47:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r0&0xff);
				cksmspi1+=sav_addr1_xy0z1r0;
				sav_fm_stat = 48;
			}
			break;
		case 48:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r1>>24)&0xff);
				sav_fm_stat = 49;
			}
			break;
		case 49:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r1>>16)&0xff);
				sav_fm_stat = 50;
			}
			break;
		case 50:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r1>>8)&0xff);
				sav_fm_stat = 51;
			}
			break;
		case 51:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r1&0xff);
				cksmspi1+=sav_addr1_xy0z1r1;
				sav_fm_stat = 52;
			}
			break;
		case 52:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r2>>24)&0xff);
				sav_fm_stat = 53;
			}
			break;
		case 53:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r2>>16)&0xff);
				sav_fm_stat = 54;
			}
			break;
		case 54:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r2>>8)&0xff);
				sav_fm_stat = 55;
			}
			break;
		case 55:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r2&0xff);
				cksmspi1+=sav_addr1_xy0z1r2;
				sav_fm_stat = 56;
			}
			break;
		case 56:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r3>>24)&0xff);
				sav_fm_stat = 57;
			}
			break;
		case 57:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r3>>16)&0xff);
				sav_fm_stat = 58;
			}
			break;
		case 58:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r3>>8)&0xff);
				sav_fm_stat = 59;
			}
			break;
		case 59:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r3&0xff);
				cksmspi1+=sav_addr1_xy0z1r3;
				sav_fm_stat = 60;
			}
			break;
		case 60:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r4>>24)&0xff);
				sav_fm_stat = 61;
			}
			break;
		case 61:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r4>>16)&0xff);
				sav_fm_stat = 62;
			}
			break;
		case 62:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r4>>8)&0xff);
				sav_fm_stat = 63;
			}
			break;
		case 63:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r4&0xff);
				cksmspi1+=sav_addr1_xy0z1r4;
				sav_fm_stat = 64;
			}
			break;
		case 64:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r5>>24)&0xff);
				sav_fm_stat = 65;
			}
			break;
		case 65:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r5>>16)&0xff);
				sav_fm_stat = 66;
			}
			break;
		case 66:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r5>>8)&0xff);
				sav_fm_stat = 67;
			}
			break;
		case 67:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r5&0xff);
				cksmspi1+=sav_addr1_xy0z1r5;
				sav_fm_stat = 68;
			}
			break;
		case 68:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r0>>24)&0xff);
				sav_fm_stat = 69;
			}
			break;
		case 69:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r0>>16)&0xff);
				sav_fm_stat = 70;
			}
			break;
		case 70:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r0>>8)&0xff);
				sav_fm_stat = 71;
			}
			break;
		case 71:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r0&0xff);
				cksmspi1+=sav_addr1_xy0z2r0;
				sav_fm_stat = 72;
			}
			break;
		case 72:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r1>>24)&0xff);
				sav_fm_stat = 73;
			}
			break;
		case 73:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r1>>16)&0xff);
				sav_fm_stat = 74;
			}
			break;
		case 74:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r1>>8)&0xff);
				sav_fm_stat = 75;
			}
			break;
		case 75:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r1&0xff);
				cksmspi1+=sav_addr1_xy0z2r1;
				sav_fm_stat = 76;
			}
			break;
		case 76:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r2>>24)&0xff);
				sav_fm_stat = 77;
			}
			break;
		case 77:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r2>>16)&0xff);
				sav_fm_stat = 78;
			}
			break;
		case 78:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r2>>8)&0xff);
				sav_fm_stat = 79;
			}
			break;
		case 79:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r2&0xff);
				cksmspi1+=sav_addr1_xy0z2r2;
				sav_fm_stat = 80;
			}
			break;
		case 80:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r3>>24)&0xff);
				sav_fm_stat = 81;
			}
			break;
		case 81:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r3>>16)&0xff);
				sav_fm_stat = 82;
			}
			break;
		case 82:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r3>>8)&0xff);
				sav_fm_stat = 83;
			}
			break;
		case 83:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r3&0xff);
				cksmspi1+=sav_addr1_xy0z2r3;
				sav_fm_stat = 84;
			}
			break;
		case 84:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r4>>24)&0xff);
				sav_fm_stat = 85;
			}
			break;
		case 85:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r4>>16)&0xff);
				sav_fm_stat = 86;
			}
			break;
		case 86:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r4>>8)&0xff);
				sav_fm_stat = 87;
			}
			break;
		case 87:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r4&0xff);
				cksmspi1+=sav_addr1_xy0z2r4;
				sav_fm_stat = 88;
			}
			break;
		case 88:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r5>>24)&0xff);
				sav_fm_stat = 89;
			}
			break;
		case 89:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r5>>16)&0xff);
				sav_fm_stat = 90;
			}
			break;
		case 90:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r5>>8)&0xff);
				sav_fm_stat = 91;
			}
			break;
		case 91:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r5&0xff);
				cksmspi1+=sav_addr1_xy0z2r5;
				sav_fm_stat = 92;
			}
			break;
		case 92:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r0>>24)&0xff);
				sav_fm_stat = 93;
			}
			break;
		case 93:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r0>>16)&0xff);
				sav_fm_stat = 94;
			}
			break;
		case 94:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r0>>8)&0xff);
				sav_fm_stat = 95;
			}
			break;
		case 95:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r0&0xff);
				cksmspi1+=sav_addr1_xy0z3r0;
				sav_fm_stat = 96;
			}
			break;
		case 96:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r1>>24)&0xff);
				sav_fm_stat = 97;
			}
			break;
		case 97:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r1>>16)&0xff);
				sav_fm_stat = 98;
			}
			break;
		case 98:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r1>>8)&0xff);
				sav_fm_stat = 99;
			}
			break;
		case 99:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r1&0xff);
				cksmspi1+=sav_addr1_xy0z3r1;
				sav_fm_stat = 100;
			}
			break;
		case 100:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r2>>24)&0xff);
				sav_fm_stat = 101;
			}
			break;
		case 101:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r2>>16)&0xff);
				sav_fm_stat = 102;
			}
			break;
		case 102:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r2>>8)&0xff);
				sav_fm_stat = 103;
			}
			break;
		case 103:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r2&0xff);
				cksmspi1+=sav_addr1_xy0z3r2;
				sav_fm_stat = 104;
			}
			break;
		case 104:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r3>>24)&0xff);
				sav_fm_stat = 105;
			}
			break;
		case 105:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r3>>16)&0xff);
				sav_fm_stat = 106;
			}
			break;
		case 106:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r3>>8)&0xff);
				sav_fm_stat = 107;
			}
			break;
		case 107:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r3&0xff);
				cksmspi1+=sav_addr1_xy0z3r3;
				sav_fm_stat = 108;
			}
			break;
		case 108:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r4>>24)&0xff);
				sav_fm_stat = 109;
			}
			break;
		case 109:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r4>>16)&0xff);
				sav_fm_stat = 110;
			}
			break;
		case 110:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r4>>8)&0xff);
				sav_fm_stat = 111;
			}
			break;
		case 111:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r4&0xff);
				cksmspi1+=sav_addr1_xy0z3r4;
				sav_fm_stat = 112;
			}
			break;
		case 112:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r5>>24)&0xff);
				sav_fm_stat = 113;
			}
			break;
		case 113:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r5>>16)&0xff);
				sav_fm_stat = 114;
			}
			break;
		case 114:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r5>>8)&0xff);
				sav_fm_stat = 115;
			}
			break;
		case 115:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r5&0xff);
				cksmspi1+=sav_addr1_xy0z3r5;
				sav_fm_stat = 116;
			}
			break;
		case 116:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r0>>24)&0xff);
				sav_fm_stat = 117;
			}
			break;
		case 117:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r0>>16)&0xff);
				sav_fm_stat = 118;
			}
			break;
		case 118:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r0>>8)&0xff);
				sav_fm_stat = 119;
			}
			break;
		case 119:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r0&0xff);
				cksmspi1+=sav_addr1_xy0z4r0;
				sav_fm_stat = 120;
			}
			break;
		case 120:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r1>>24)&0xff);
				sav_fm_stat = 121;
			}
			break;
		case 121:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r1>>16)&0xff);
				sav_fm_stat = 122;
			}
			break;
		case 122:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r1>>8)&0xff);
				sav_fm_stat = 123;
			}
			break;
		case 123:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r1&0xff);
				cksmspi1+=sav_addr1_xy0z4r1;
				sav_fm_stat = 124;
			}
			break;
		case 124:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r2>>24)&0xff);
				sav_fm_stat = 125;
			}
			break;
		case 125:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r2>>16)&0xff);
				sav_fm_stat = 126;
			}
			break;
		case 126:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r2>>8)&0xff);
				sav_fm_stat = 127;
			}
			break;
		case 127:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r2&0xff);
				cksmspi1+=sav_addr1_xy0z4r2;
				sav_fm_stat = 128;
			}
			break;
		case 128:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r3>>24)&0xff);
				sav_fm_stat = 129;
			}
			break;
		case 129:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r3>>16)&0xff);
				sav_fm_stat = 130;
			}
			break;
		case 130:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r3>>8)&0xff);
				sav_fm_stat = 131;
			}
			break;
		case 131:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r3&0xff);
				cksmspi1+=sav_addr1_xy0z4r3;
				sav_fm_stat = 132;
			}
			break;
		case 132:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r4>>24)&0xff);
				sav_fm_stat = 133;
			}
			break;
		case 133:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r4>>16)&0xff);
				sav_fm_stat = 134;
			}
			break;
		case 134:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r4>>8)&0xff);
				sav_fm_stat = 135;
			}
			break;
		case 135:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r4&0xff);
				cksmspi1+=sav_addr1_xy0z4r4;
				sav_fm_stat = 136;
			}
			break;
		case 136:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r5>>24)&0xff);
				sav_fm_stat = 137;
			}
			break;
		case 137:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r5>>16)&0xff);
				sav_fm_stat = 138;
			}
			break;
		case 138:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r5>>8)&0xff);
				sav_fm_stat = 139;
			}
			break;
		case 139:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r5&0xff);
				cksmspi1+=sav_addr1_xy0z4r5;
				sav_fm_stat = 140;
			}
			break;
		case 140:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r0>>24)&0xff);
				sav_fm_stat = 141;
			}
			break;
		case 141:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r0>>16)&0xff);
				sav_fm_stat = 142;
			}
			break;
		case 142:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r0>>8)&0xff);
				sav_fm_stat = 143;
			}
			break;
		case 143:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r0&0xff);
				cksmspi1+=sav_addr1_xy1z0r0;
				sav_fm_stat = 144;
			}
			break;
		case 144:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r1>>24)&0xff);
				sav_fm_stat = 145;
			}
			break;
		case 145:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r1>>16)&0xff);
				sav_fm_stat = 146;
			}
			break;
		case 146:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r1>>8)&0xff);
				sav_fm_stat = 147;
			}
			break;
		case 147:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r1&0xff);
				cksmspi1+=sav_addr1_xy1z0r1;
				sav_fm_stat = 148;
			}
			break;
		case 148:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r2>>24)&0xff);
				sav_fm_stat = 149;
			}
			break;
		case 149:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r2>>16)&0xff);
				sav_fm_stat = 150;
			}
			break;
		case 150:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r2>>8)&0xff);
				sav_fm_stat = 151;
			}
			break;
		case 151:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r2&0xff);
				cksmspi1+=sav_addr1_xy1z0r2;
				sav_fm_stat = 152;
			}
			break;
		case 152:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r3>>24)&0xff);
				sav_fm_stat = 153;
			}
			break;
		case 153:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r3>>16)&0xff);
				sav_fm_stat = 154;
			}
			break;
		case 154:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r3>>8)&0xff);
				sav_fm_stat = 155;
			}
			break;
		case 155:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r3&0xff);
				cksmspi1+=sav_addr1_xy1z0r3;
				sav_fm_stat = 156;
			}
			break;
		case 156:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r4>>24)&0xff);
				sav_fm_stat = 157;
			}
			break;
		case 157:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r4>>16)&0xff);
				sav_fm_stat = 158;
			}
			break;
		case 158:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r4>>8)&0xff);
				sav_fm_stat = 159;
			}
			break;
		case 159:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r4&0xff);
				cksmspi1+=sav_addr1_xy1z0r4;
				sav_fm_stat = 160;
			}
			break;
		case 160:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r5>>24)&0xff);
				sav_fm_stat = 161;
			}
			break;
		case 161:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r5>>16)&0xff);
				sav_fm_stat = 162;
			}
			break;
		case 162:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r5>>8)&0xff);
				sav_fm_stat = 163;
			}
			break;
		case 163:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r5&0xff);
				cksmspi1+=sav_addr1_xy1z0r5;
				sav_fm_stat = 164;
			}
			break;
		case 164:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r0>>24)&0xff);
				sav_fm_stat = 165;
			}
			break;
		case 165:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r0>>16)&0xff);
				sav_fm_stat = 166;
			}
			break;
		case 166:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r0>>8)&0xff);
				sav_fm_stat = 167;
			}
			break;
		case 167:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r0&0xff);
				cksmspi1+=sav_addr1_xy1z1r0;
				sav_fm_stat = 168;
			}
			break;
		case 168:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r1>>24)&0xff);
				sav_fm_stat = 169;
			}
			break;
		case 169:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r1>>16)&0xff);
				sav_fm_stat = 170;
			}
			break;
		case 170:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r1>>8)&0xff);
				sav_fm_stat = 171;
			}
			break;
		case 171:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r1&0xff);
				cksmspi1+=sav_addr1_xy1z1r1;
				sav_fm_stat = 172;
			}
			break;
		case 172:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r2>>24)&0xff);
				sav_fm_stat = 173;
			}
			break;
		case 173:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r2>>16)&0xff);
				sav_fm_stat = 174;
			}
			break;
		case 174:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r2>>8)&0xff);
				sav_fm_stat = 175;
			}
			break;
		case 175:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r2&0xff);
				cksmspi1+=sav_addr1_xy1z1r2;
				sav_fm_stat = 176;
			}
			break;
		case 176:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r3>>24)&0xff);
				sav_fm_stat = 177;
			}
			break;
		case 177:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r3>>16)&0xff);
				sav_fm_stat = 178;
			}
			break;
		case 178:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r3>>8)&0xff);
				sav_fm_stat = 179;
			}
			break;
		case 179:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r3&0xff);
				cksmspi1+=sav_addr1_xy1z1r3;
				sav_fm_stat = 180;
			}
			break;
		case 180:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r4>>24)&0xff);
				sav_fm_stat = 181;
			}
			break;
		case 181:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r4>>16)&0xff);
				sav_fm_stat = 182;
			}
			break;
		case 182:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r4>>8)&0xff);
				sav_fm_stat = 183;
			}
			break;
		case 183:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r4&0xff);
				cksmspi1+=sav_addr1_xy1z1r4;
				sav_fm_stat = 184;
			}
			break;
		case 184:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r5>>24)&0xff);
				sav_fm_stat = 185;
			}
			break;
		case 185:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r5>>16)&0xff);
				sav_fm_stat = 186;
			}
			break;
		case 186:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r5>>8)&0xff);
				sav_fm_stat = 187;
			}
			break;
		case 187:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r5&0xff);
				cksmspi1+=sav_addr1_xy1z1r5;
				sav_fm_stat = 188;
			}
			break;
		case 188:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r0>>24)&0xff);
				sav_fm_stat = 189;
			}
			break;
		case 189:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r0>>16)&0xff);
				sav_fm_stat = 190;
			}
			break;
		case 190:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r0>>8)&0xff);
				sav_fm_stat = 191;
			}
			break;
		case 191:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r0&0xff);
				cksmspi1+=sav_addr1_xy1z2r0;
				sav_fm_stat = 192;
			}
			break;
		case 192:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r1>>24)&0xff);
				sav_fm_stat = 193;
			}
			break;
		case 193:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r1>>16)&0xff);
				sav_fm_stat = 194;
			}
			break;
		case 194:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r1>>8)&0xff);
				sav_fm_stat = 195;
			}
			break;
		case 195:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r1&0xff);
				cksmspi1+=sav_addr1_xy1z2r1;
				sav_fm_stat = 196;
			}
			break;
		case 196:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r2>>24)&0xff);
				sav_fm_stat = 197;
			}
			break;
		case 197:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r2>>16)&0xff);
				sav_fm_stat = 198;
			}
			break;
		case 198:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r2>>8)&0xff);
				sav_fm_stat = 199;
			}
			break;
		case 199:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r2&0xff);
				cksmspi1+=sav_addr1_xy1z2r2;
				sav_fm_stat = 200;
			}
			break;
		case 200:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r3>>24)&0xff);
				sav_fm_stat = 201;
			}
			break;
		case 201:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r3>>16)&0xff);
				sav_fm_stat = 202;
			}
			break;
		case 202:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r3>>8)&0xff);
				sav_fm_stat = 203;
			}
			break;
		case 203:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r3&0xff);
				cksmspi1+=sav_addr1_xy1z2r3;
				sav_fm_stat = 204;
			}
			break;
		case 204:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r4>>24)&0xff);
				sav_fm_stat = 205;
			}
			break;
		case 205:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r4>>16)&0xff);
				sav_fm_stat = 206;
			}
			break;
		case 206:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r4>>8)&0xff);
				sav_fm_stat = 207;
			}
			break;
		case 207:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r4&0xff);
				cksmspi1+=sav_addr1_xy1z2r4;
				sav_fm_stat = 208;
			}
			break;
		case 208:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r5>>24)&0xff);
				sav_fm_stat = 209;
			}
			break;
		case 209:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r5>>16)&0xff);
				sav_fm_stat = 210;
			}
			break;
		case 210:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r5>>8)&0xff);
				sav_fm_stat = 211;
			}
			break;
		case 211:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r5&0xff);
				cksmspi1+=sav_addr1_xy1z2r5;
				sav_fm_stat = 212;
			}
			break;
		case 212:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r0>>24)&0xff);
				sav_fm_stat = 213;
			}
			break;
		case 213:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r0>>16)&0xff);
				sav_fm_stat = 214;
			}
			break;
		case 214:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r0>>8)&0xff);
				sav_fm_stat = 215;
			}
			break;
		case 215:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r0&0xff);
				cksmspi1+=sav_addr1_xy1z3r0;
				sav_fm_stat = 216;
			}
			break;
		case 216:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r1>>24)&0xff);
				sav_fm_stat = 217;
			}
			break;
		case 217:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r1>>16)&0xff);
				sav_fm_stat = 218;
			}
			break;
		case 218:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r1>>8)&0xff);
				sav_fm_stat = 219;
			}
			break;
		case 219:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r1&0xff);
				cksmspi1+=sav_addr1_xy1z3r1;
				sav_fm_stat = 220;
			}
			break;
		case 220:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r2>>24)&0xff);
				sav_fm_stat = 221;
			}
			break;
		case 221:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r2>>16)&0xff);
				sav_fm_stat = 222;
			}
			break;
		case 222:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r2>>8)&0xff);
				sav_fm_stat = 223;
			}
			break;
		case 223:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r2&0xff);
				cksmspi1+=sav_addr1_xy1z3r2;
				sav_fm_stat = 224;
			}
			break;
		case 224:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r3>>24)&0xff);
				sav_fm_stat = 225;
			}
			break;
		case 225:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r3>>16)&0xff);
				sav_fm_stat = 226;
			}
			break;
		case 226:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r3>>8)&0xff);
				sav_fm_stat = 227;
			}
			break;
		case 227:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r3&0xff);
				cksmspi1+=sav_addr1_xy1z3r3;
				sav_fm_stat = 228;
			}
			break;
		case 228:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r4>>24)&0xff);
				sav_fm_stat = 229;
			}
			break;
		case 229:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r4>>16)&0xff);
				sav_fm_stat = 230;
			}
			break;
		case 230:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r4>>8)&0xff);
				sav_fm_stat = 231;
			}
			break;
		case 231:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r4&0xff);
				cksmspi1+=sav_addr1_xy1z3r4;
				sav_fm_stat = 232;
			}
			break;
		case 232:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r5>>24)&0xff);
				sav_fm_stat = 233;
			}
			break;
		case 233:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r5>>16)&0xff);
				sav_fm_stat = 234;
			}
			break;
		case 234:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r5>>8)&0xff);
				sav_fm_stat = 235;
			}
			break;
		case 235:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r5&0xff);
				cksmspi1+=sav_addr1_xy1z3r5;
				sav_fm_stat = 236;
			}
			break;
		case 236:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r0>>24)&0xff);
				sav_fm_stat = 237;
			}
			break;
		case 237:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r0>>16)&0xff);
				sav_fm_stat = 238;
			}
			break;
		case 238:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r0>>8)&0xff);
				sav_fm_stat = 239;
			}
			break;
		case 239:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r0&0xff);
				cksmspi1+=sav_addr1_xy1z4r0;
				sav_fm_stat = 240;
			}
			break;
		case 240:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r1>>24)&0xff);
				sav_fm_stat = 241;
			}
			break;
		case 241:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r1>>16)&0xff);
				sav_fm_stat = 242;
			}
			break;
		case 242:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r1>>8)&0xff);
				sav_fm_stat = 243;
			}
			break;
		case 243:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r1&0xff);
				cksmspi1+=sav_addr1_xy1z4r1;
				sav_fm_stat = 244;
			}
			break;
		case 244:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r2>>24)&0xff);
				sav_fm_stat = 245;
			}
			break;
		case 245:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r2>>16)&0xff);
				sav_fm_stat = 246;
			}
			break;
		case 246:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r2>>8)&0xff);
				sav_fm_stat = 247;
			}
			break;
		case 247:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r2&0xff);
				cksmspi1+=sav_addr1_xy1z4r2;
				sav_fm_stat = 248;
			}
			break;
		case 248:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r3>>24)&0xff);
				sav_fm_stat = 249;
			}
			break;
		case 249:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r3>>16)&0xff);
				sav_fm_stat = 250;
			}
			break;
		case 250:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r3>>8)&0xff);
				sav_fm_stat = 251;
			}
			break;
		case 251:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r3&0xff);
				cksmspi1+=sav_addr1_xy1z4r3;
				sav_fm_stat = 252;
			}
			break;
		case 252:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r4>>24)&0xff);
				sav_fm_stat = 253;
			}
			break;
		case 253:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r4>>16)&0xff);
				sav_fm_stat = 254;
			}
			break;
		case 254:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r4>>8)&0xff);
				sav_fm_stat = 255;
			}
			break;
		case 255:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r4&0xff);
				cksmspi1+=sav_addr1_xy1z4r4;
				sav_fm_stat = 256;
			}
			break;
		case 256:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r5>>24)&0xff);
				sav_fm_stat = 257;
			}
			break;
		case 257:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r5>>16)&0xff);
				sav_fm_stat = 258;
			}
			break;
		case 258:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r5>>8)&0xff);
				sav_fm_stat = 259;
			}
			break;
		case 259:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r5&0xff);
				cksmspi1+=sav_addr1_xy1z4r5;
				sav_fm_stat = 260;
			}
			break;
		case 260:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r0>>24)&0xff);
				sav_fm_stat = 261;
			}
			break;
		case 261:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r0>>16)&0xff);
				sav_fm_stat = 262;
			}
			break;
		case 262:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r0>>8)&0xff);
				sav_fm_stat = 263;
			}
			break;
		case 263:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r0&0xff);
				cksmspi1+=sav_addr1_xy2z0r0;
				sav_fm_stat = 264;
			}
			break;
		case 264:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r1>>24)&0xff);
				sav_fm_stat = 265;
			}
			break;
		case 265:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r1>>16)&0xff);
				sav_fm_stat = 266;
			}
			break;
		case 266:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r1>>8)&0xff);
				sav_fm_stat = 267;
			}
			break;
		case 267:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r1&0xff);
				cksmspi1+=sav_addr1_xy2z0r1;
				sav_fm_stat = 268;
			}
			break;
		case 268:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r2>>24)&0xff);
				sav_fm_stat = 269;
			}
			break;
		case 269:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r2>>16)&0xff);
				sav_fm_stat = 270;
			}
			break;
		case 270:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r2>>8)&0xff);
				sav_fm_stat = 271;
			}
			break;
		case 271:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r2&0xff);
				cksmspi1+=sav_addr1_xy2z0r2;
				sav_fm_stat = 272;
			}
			break;
		case 272:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r3>>24)&0xff);
				sav_fm_stat = 273;
			}
			break;
		case 273:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r3>>16)&0xff);
				sav_fm_stat = 274;
			}
			break;
		case 274:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r3>>8)&0xff);
				sav_fm_stat = 275;
			}
			break;
		case 275:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r3&0xff);
				cksmspi1+=sav_addr1_xy2z0r3;
				sav_fm_stat = 276;
			}
			break;
		case 276:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r4>>24)&0xff);
				sav_fm_stat = 277;
			}
			break;
		case 277:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r4>>16)&0xff);
				sav_fm_stat = 278;
			}
			break;
		case 278:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r4>>8)&0xff);
				sav_fm_stat = 279;
			}
			break;
		case 279:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r4&0xff);
				cksmspi1+=sav_addr1_xy2z0r4;
				sav_fm_stat = 280;
			}
			break;
		case 280:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r5>>24)&0xff);
				sav_fm_stat = 281;
			}
			break;
		case 281:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r5>>16)&0xff);
				sav_fm_stat = 282;
			}
			break;
		case 282:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r5>>8)&0xff);
				sav_fm_stat = 283;
			}
			break;
		case 283:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r5&0xff);
				cksmspi1+=sav_addr1_xy2z0r5;
				sav_fm_stat = 284;
			}
			break;
		case 284:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r0>>24)&0xff);
				sav_fm_stat = 285;
			}
			break;
		case 285:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r0>>16)&0xff);
				sav_fm_stat = 286;
			}
			break;
		case 286:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r0>>8)&0xff);
				sav_fm_stat = 287;
			}
			break;
		case 287:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r0&0xff);
				cksmspi1+=sav_addr1_xy2z1r0;
				sav_fm_stat = 288;
			}
			break;
		case 288:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r1>>24)&0xff);
				sav_fm_stat = 289;
			}
			break;
		case 289:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r1>>16)&0xff);
				sav_fm_stat = 290;
			}
			break;
		case 290:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r1>>8)&0xff);
				sav_fm_stat = 291;
			}
			break;
		case 291:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r1&0xff);
				cksmspi1+=sav_addr1_xy2z1r1;
				sav_fm_stat = 292;
			}
			break;
		case 292:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r2>>24)&0xff);
				sav_fm_stat = 293;
			}
			break;
		case 293:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r2>>16)&0xff);
				sav_fm_stat = 294;
			}
			break;
		case 294:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r2>>8)&0xff);
				sav_fm_stat = 295;
			}
			break;
		case 295:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r2&0xff);
				cksmspi1+=sav_addr1_xy2z1r2;
				sav_fm_stat = 296;
			}
			break;
		case 296:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r3>>24)&0xff);
				sav_fm_stat = 297;
			}
			break;
		case 297:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r3>>16)&0xff);
				sav_fm_stat = 298;
			}
			break;
		case 298:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r3>>8)&0xff);
				sav_fm_stat = 299;
			}
			break;
		case 299:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r3&0xff);
				cksmspi1+=sav_addr1_xy2z1r3;
				sav_fm_stat = 300;
			}
			break;
		case 300:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r4>>24)&0xff);
				sav_fm_stat = 301;
			}
			break;
		case 301:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r4>>16)&0xff);
				sav_fm_stat = 302;
			}
			break;
		case 302:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r4>>8)&0xff);
				sav_fm_stat = 303;
			}
			break;
		case 303:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r4&0xff);
				cksmspi1+=sav_addr1_xy2z1r4;
				sav_fm_stat = 304;
			}
			break;
		case 304:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r5>>24)&0xff);
				sav_fm_stat = 305;
			}
			break;
		case 305:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r5>>16)&0xff);
				sav_fm_stat = 306;
			}
			break;
		case 306:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r5>>8)&0xff);
				sav_fm_stat = 307;
			}
			break;
		case 307:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r5&0xff);
				cksmspi1+=sav_addr1_xy2z1r5;
				sav_fm_stat = 308;
			}
			break;
		case 308:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r0>>24)&0xff);
				sav_fm_stat = 309;
			}
			break;
		case 309:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r0>>16)&0xff);
				sav_fm_stat = 310;
			}
			break;
		case 310:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r0>>8)&0xff);
				sav_fm_stat = 311;
			}
			break;
		case 311:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r0&0xff);
				cksmspi1+=sav_addr1_xy2z2r0;
				sav_fm_stat = 312;
			}
			break;
		case 312:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r1>>24)&0xff);
				sav_fm_stat = 313;
			}
			break;
		case 313:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r1>>16)&0xff);
				sav_fm_stat = 314;
			}
			break;
		case 314:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r1>>8)&0xff);
				sav_fm_stat = 315;
			}
			break;
		case 315:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r1&0xff);
				cksmspi1+=sav_addr1_xy2z2r1;
				sav_fm_stat = 316;
			}
			break;
		case 316:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r2>>24)&0xff);
				sav_fm_stat = 317;
			}
			break;
		case 317:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r2>>16)&0xff);
				sav_fm_stat = 318;
			}
			break;
		case 318:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r2>>8)&0xff);
				sav_fm_stat = 319;
			}
			break;
		case 319:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r2&0xff);
				cksmspi1+=sav_addr1_xy2z2r2;
				sav_fm_stat = 320;
			}
			break;
		case 320:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r3>>24)&0xff);
				sav_fm_stat = 321;
			}
			break;
		case 321:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r3>>16)&0xff);
				sav_fm_stat = 322;
			}
			break;
		case 322:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r3>>8)&0xff);
				sav_fm_stat = 323;
			}
			break;
		case 323:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r3&0xff);
				cksmspi1+=sav_addr1_xy2z2r3;
				sav_fm_stat = 324;
			}
			break;
		case 324:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r4>>24)&0xff);
				sav_fm_stat = 325;
			}
			break;
		case 325:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r4>>16)&0xff);
				sav_fm_stat = 326;
			}
			break;
		case 326:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r4>>8)&0xff);
				sav_fm_stat = 327;
			}
			break;
		case 327:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r4&0xff);
				cksmspi1+=sav_addr1_xy2z2r4;
				sav_fm_stat = 328;
			}
			break;
		case 328:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r5>>24)&0xff);
				sav_fm_stat = 329;
			}
			break;
		case 329:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r5>>16)&0xff);
				sav_fm_stat = 330;
			}
			break;
		case 330:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r5>>8)&0xff);
				sav_fm_stat = 331;
			}
			break;
		case 331:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r5&0xff);
				cksmspi1+=sav_addr1_xy2z2r5;
				sav_fm_stat = 332;
			}
			break;
		case 332:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r0>>24)&0xff);
				sav_fm_stat = 333;
			}
			break;
		case 333:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r0>>16)&0xff);
				sav_fm_stat = 334;
			}
			break;
		case 334:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r0>>8)&0xff);
				sav_fm_stat = 335;
			}
			break;
		case 335:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r0&0xff);
				cksmspi1+=sav_addr1_xy2z3r0;
				sav_fm_stat = 336;
			}
			break;
		case 336:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r1>>24)&0xff);
				sav_fm_stat = 337;
			}
			break;
		case 337:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r1>>16)&0xff);
				sav_fm_stat = 338;
			}
			break;
		case 338:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r1>>8)&0xff);
				sav_fm_stat = 339;
			}
			break;
		case 339:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r1&0xff);
				cksmspi1+=sav_addr1_xy2z3r1;
				sav_fm_stat = 340;
			}
			break;
		case 340:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r2>>24)&0xff);
				sav_fm_stat = 341;
			}
			break;
		case 341:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r2>>16)&0xff);
				sav_fm_stat = 342;
			}
			break;
		case 342:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r2>>8)&0xff);
				sav_fm_stat = 343;
			}
			break;
		case 343:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r2&0xff);
				cksmspi1+=sav_addr1_xy2z3r2;
				sav_fm_stat = 344;
			}
			break;
		case 344:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r3>>24)&0xff);
				sav_fm_stat = 345;
			}
			break;
		case 345:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r3>>16)&0xff);
				sav_fm_stat = 346;
			}
			break;
		case 346:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r3>>8)&0xff);
				sav_fm_stat = 347;
			}
			break;
		case 347:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r3&0xff);
				cksmspi1+=sav_addr1_xy2z3r3;
				sav_fm_stat = 348;
			}
			break;
		case 348:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r4>>24)&0xff);
				sav_fm_stat = 349;
			}
			break;
		case 349:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r4>>16)&0xff);
				sav_fm_stat = 350;
			}
			break;
		case 350:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r4>>8)&0xff);
				sav_fm_stat = 351;
			}
			break;
		case 351:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r4&0xff);
				cksmspi1+=sav_addr1_xy2z3r4;
				sav_fm_stat = 352;
			}
			break;
		case 352:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r5>>24)&0xff);
				sav_fm_stat = 353;
			}
			break;
		case 353:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r5>>16)&0xff);
				sav_fm_stat = 354;
			}
			break;
		case 354:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r5>>8)&0xff);
				sav_fm_stat = 355;
			}
			break;
		case 355:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r5&0xff);
				cksmspi1+=sav_addr1_xy2z3r5;
				sav_fm_stat = 356;
			}
			break;
		case 356:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r0>>24)&0xff);
				sav_fm_stat = 357;
			}
			break;
		case 357:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r0>>16)&0xff);
				sav_fm_stat = 358;
			}
			break;
		case 358:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r0>>8)&0xff);
				sav_fm_stat = 359;
			}
			break;
		case 359:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r0&0xff);
				cksmspi1+=sav_addr1_xy2z4r0;
				sav_fm_stat = 360;
			}
			break;
		case 360:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r1>>24)&0xff);
				sav_fm_stat = 361;
			}
			break;
		case 361:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r1>>16)&0xff);
				sav_fm_stat = 362;
			}
			break;
		case 362:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r1>>8)&0xff);
				sav_fm_stat = 363;
			}
			break;
		case 363:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r1&0xff);
				cksmspi1+=sav_addr1_xy2z4r1;
				sav_fm_stat = 364;
			}
			break;
		case 364:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r2>>24)&0xff);
				sav_fm_stat = 365;
			}
			break;
		case 365:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r2>>16)&0xff);
				sav_fm_stat = 366;
			}
			break;
		case 366:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r2>>8)&0xff);
				sav_fm_stat = 367;
			}
			break;
		case 367:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r2&0xff);
				cksmspi1+=sav_addr1_xy2z4r2;
				sav_fm_stat = 368;
			}
			break;
		case 368:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r3>>24)&0xff);
				sav_fm_stat = 369;
			}
			break;
		case 369:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r3>>16)&0xff);
				sav_fm_stat = 370;
			}
			break;
		case 370:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r3>>8)&0xff);
				sav_fm_stat = 371;
			}
			break;
		case 371:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r3&0xff);
				cksmspi1+=sav_addr1_xy2z4r3;
				sav_fm_stat = 372;
			}
			break;
		case 372:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r4>>24)&0xff);
				sav_fm_stat = 373;
			}
			break;
		case 373:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r4>>16)&0xff);
				sav_fm_stat = 374;
			}
			break;
		case 374:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r4>>8)&0xff);
				sav_fm_stat = 375;
			}
			break;
		case 375:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r4&0xff);
				cksmspi1+=sav_addr1_xy2z4r4;
				sav_fm_stat = 376;
			}
			break;
		case 376:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r5>>24)&0xff);
				sav_fm_stat = 377;
			}
			break;
		case 377:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r5>>16)&0xff);
				sav_fm_stat = 378;
			}
			break;
		case 378:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r5>>8)&0xff);
				sav_fm_stat = 379;
			}
			break;
		case 379:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r5&0xff);
				cksmspi1+=sav_addr1_xy2z4r5;
				sav_fm_stat = 380;
			}
			break;
		case 380:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r0>>24)&0xff);
				sav_fm_stat = 381;
			}
			break;
		case 381:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r0>>16)&0xff);
				sav_fm_stat = 382;
			}
			break;
		case 382:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r0>>8)&0xff);
				sav_fm_stat = 383;
			}
			break;
		case 383:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r0&0xff);
				cksmspi1+=sav_addr1_xy3z0r0;
				sav_fm_stat = 384;
			}
			break;
		case 384:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r1>>24)&0xff);
				sav_fm_stat = 385;
			}
			break;
		case 385:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r1>>16)&0xff);
				sav_fm_stat = 386;
			}
			break;
		case 386:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r1>>8)&0xff);
				sav_fm_stat = 387;
			}
			break;
		case 387:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r1&0xff);
				cksmspi1+=sav_addr1_xy3z0r1;
				sav_fm_stat = 388;
			}
			break;
		case 388:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r2>>24)&0xff);
				sav_fm_stat = 389;
			}
			break;
		case 389:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r2>>16)&0xff);
				sav_fm_stat = 390;
			}
			break;
		case 390:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r2>>8)&0xff);
				sav_fm_stat = 391;
			}
			break;
		case 391:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r2&0xff);
				cksmspi1+=sav_addr1_xy3z0r2;
				sav_fm_stat = 392;
			}
			break;
		case 392:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r3>>24)&0xff);
				sav_fm_stat = 393;
			}
			break;
		case 393:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r3>>16)&0xff);
				sav_fm_stat = 394;
			}
			break;
		case 394:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r3>>8)&0xff);
				sav_fm_stat = 395;
			}
			break;
		case 395:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r3&0xff);
				cksmspi1+=sav_addr1_xy3z0r3;
				sav_fm_stat = 396;
			}
			break;
		case 396:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r4>>24)&0xff);
				sav_fm_stat = 397;
			}
			break;
		case 397:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r4>>16)&0xff);
				sav_fm_stat = 398;
			}
			break;
		case 398:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r4>>8)&0xff);
				sav_fm_stat = 399;
			}
			break;
		case 399:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r4&0xff);
				cksmspi1+=sav_addr1_xy3z0r4;
				sav_fm_stat = 400;
			}
			break;
		case 400:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r5>>24)&0xff);
				sav_fm_stat = 401;
			}
			break;
		case 401:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r5>>16)&0xff);
				sav_fm_stat = 402;
			}
			break;
		case 402:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r5>>8)&0xff);
				sav_fm_stat = 403;
			}
			break;
		case 403:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r5&0xff);
				cksmspi1+=sav_addr1_xy3z0r5;
				sav_fm_stat = 404;
			}
			break;
		case 404:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r0>>24)&0xff);
				sav_fm_stat = 405;
			}
			break;
		case 405:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r0>>16)&0xff);
				sav_fm_stat = 406;
			}
			break;
		case 406:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r0>>8)&0xff);
				sav_fm_stat = 407;
			}
			break;
		case 407:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r0&0xff);
				cksmspi1+=sav_addr1_xy3z1r0;
				sav_fm_stat = 408;
			}
			break;
		case 408:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r1>>24)&0xff);
				sav_fm_stat = 409;
			}
			break;
		case 409:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r1>>16)&0xff);
				sav_fm_stat = 410;
			}
			break;
		case 410:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r1>>8)&0xff);
				sav_fm_stat = 411;
			}
			break;
		case 411:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r1&0xff);
				cksmspi1+=sav_addr1_xy3z1r1;
				sav_fm_stat = 412;
			}
			break;
		case 412:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r2>>24)&0xff);
				sav_fm_stat = 413;
			}
			break;
		case 413:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r2>>16)&0xff);
				sav_fm_stat = 414;
			}
			break;
		case 414:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r2>>8)&0xff);
				sav_fm_stat = 415;
			}
			break;
		case 415:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r2&0xff);
				cksmspi1+=sav_addr1_xy3z1r2;
				sav_fm_stat = 416;
			}
			break;
		case 416:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r3>>24)&0xff);
				sav_fm_stat = 417;
			}
			break;
		case 417:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r3>>16)&0xff);
				sav_fm_stat = 418;
			}
			break;
		case 418:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r3>>8)&0xff);
				sav_fm_stat = 419;
			}
			break;
		case 419:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r3&0xff);
				cksmspi1+=sav_addr1_xy3z1r3;
				sav_fm_stat = 420;
			}
			break;
		case 420:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r4>>24)&0xff);
				sav_fm_stat = 421;
			}
			break;
		case 421:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r4>>16)&0xff);
				sav_fm_stat = 422;
			}
			break;
		case 422:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r4>>8)&0xff);
				sav_fm_stat = 423;
			}
			break;
		case 423:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r4&0xff);
				cksmspi1+=sav_addr1_xy3z1r4;
				sav_fm_stat = 424;
			}
			break;
		case 424:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r5>>24)&0xff);
				sav_fm_stat = 425;
			}
			break;
		case 425:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r5>>16)&0xff);
				sav_fm_stat = 426;
			}
			break;
		case 426:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r5>>8)&0xff);
				sav_fm_stat = 427;
			}
			break;
		case 427:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r5&0xff);
				cksmspi1+=sav_addr1_xy3z1r5;
				sav_fm_stat = 428;
			}
			break;
		case 428:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r0>>24)&0xff);
				sav_fm_stat = 429;
			}
			break;
		case 429:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r0>>16)&0xff);
				sav_fm_stat = 430;
			}
			break;
		case 430:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r0>>8)&0xff);
				sav_fm_stat = 431;
			}
			break;
		case 431:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r0&0xff);
				cksmspi1+=sav_addr1_xy3z2r0;
				sav_fm_stat = 432;
			}
			break;
		case 432:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r1>>24)&0xff);
				sav_fm_stat = 433;
			}
			break;
		case 433:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r1>>16)&0xff);
				sav_fm_stat = 434;
			}
			break;
		case 434:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r1>>8)&0xff);
				sav_fm_stat = 435;
			}
			break;
		case 435:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r1&0xff);
				cksmspi1+=sav_addr1_xy3z2r1;
				sav_fm_stat = 436;
			}
			break;
		case 436:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r2>>24)&0xff);
				sav_fm_stat = 437;
			}
			break;
		case 437:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r2>>16)&0xff);
				sav_fm_stat = 438;
			}
			break;
		case 438:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r2>>8)&0xff);
				sav_fm_stat = 439;
			}
			break;
		case 439:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r2&0xff);
				cksmspi1+=sav_addr1_xy3z2r2;
				sav_fm_stat = 440;
			}
			break;
		case 440:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r3>>24)&0xff);
				sav_fm_stat = 441;
			}
			break;
		case 441:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r3>>16)&0xff);
				sav_fm_stat = 442;
			}
			break;
		case 442:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r3>>8)&0xff);
				sav_fm_stat = 443;
			}
			break;
		case 443:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r3&0xff);
				cksmspi1+=sav_addr1_xy3z2r3;
				sav_fm_stat = 444;
			}
			break;
		case 444:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r4>>24)&0xff);
				sav_fm_stat = 445;
			}
			break;
		case 445:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r4>>16)&0xff);
				sav_fm_stat = 446;
			}
			break;
		case 446:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r4>>8)&0xff);
				sav_fm_stat = 447;
			}
			break;
		case 447:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r4&0xff);
				cksmspi1+=sav_addr1_xy3z2r4;
				sav_fm_stat = 448;
			}
			break;
		case 448:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r5>>24)&0xff);
				sav_fm_stat = 449;
			}
			break;
		case 449:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r5>>16)&0xff);
				sav_fm_stat = 450;
			}
			break;
		case 450:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r5>>8)&0xff);
				sav_fm_stat = 451;
			}
			break;
		case 451:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r5&0xff);
				cksmspi1+=sav_addr1_xy3z2r5;
				sav_fm_stat = 452;
			}
			break;
		case 452:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r0>>24)&0xff);
				sav_fm_stat = 453;
			}
			break;
		case 453:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r0>>16)&0xff);
				sav_fm_stat = 454;
			}
			break;
		case 454:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r0>>8)&0xff);
				sav_fm_stat = 455;
			}
			break;
		case 455:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r0&0xff);
				cksmspi1+=sav_addr1_xy3z3r0;
				sav_fm_stat = 456;
			}
			break;
		case 456:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r1>>24)&0xff);
				sav_fm_stat = 457;
			}
			break;
		case 457:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r1>>16)&0xff);
				sav_fm_stat = 458;
			}
			break;
		case 458:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r1>>8)&0xff);
				sav_fm_stat = 459;
			}
			break;
		case 459:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r1&0xff);
				cksmspi1+=sav_addr1_xy3z3r1;
				sav_fm_stat = 460;
			}
			break;
		case 460:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r2>>24)&0xff);
				sav_fm_stat = 461;
			}
			break;
		case 461:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r2>>16)&0xff);
				sav_fm_stat = 462;
			}
			break;
		case 462:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r2>>8)&0xff);
				sav_fm_stat = 463;
			}
			break;
		case 463:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r2&0xff);
				cksmspi1+=sav_addr1_xy3z3r2;
				sav_fm_stat = 464;
			}
			break;
		case 464:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r3>>24)&0xff);
				sav_fm_stat = 465;
			}
			break;
		case 465:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r3>>16)&0xff);
				sav_fm_stat = 466;
			}
			break;
		case 466:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r3>>8)&0xff);
				sav_fm_stat = 467;
			}
			break;
		case 467:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r3&0xff);
				cksmspi1+=sav_addr1_xy3z3r3;
				sav_fm_stat = 468;
			}
			break;
		case 468:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r4>>24)&0xff);
				sav_fm_stat = 469;
			}
			break;
		case 469:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r4>>16)&0xff);
				sav_fm_stat = 470;
			}
			break;
		case 470:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r4>>8)&0xff);
				sav_fm_stat = 471;
			}
			break;
		case 471:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r4&0xff);
				cksmspi1+=sav_addr1_xy3z3r4;
				sav_fm_stat = 472;
			}
			break;
		case 472:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r5>>24)&0xff);
				sav_fm_stat = 473;
			}
			break;
		case 473:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r5>>16)&0xff);
				sav_fm_stat = 474;
			}
			break;
		case 474:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r5>>8)&0xff);
				sav_fm_stat = 475;
			}
			break;
		case 475:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r5&0xff);
				cksmspi1+=sav_addr1_xy3z3r5;
				sav_fm_stat = 476;
			}
			break;
		case 476:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r0>>24)&0xff);
				sav_fm_stat = 477;
			}
			break;
		case 477:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r0>>16)&0xff);
				sav_fm_stat = 478;
			}
			break;
		case 478:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r0>>8)&0xff);
				sav_fm_stat = 479;
			}
			break;
		case 479:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r0&0xff);
				cksmspi1+=sav_addr1_xy3z4r0;
				sav_fm_stat = 480;
			}
			break;
		case 480:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r1>>24)&0xff);
				sav_fm_stat = 481;
			}
			break;
		case 481:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r1>>16)&0xff);
				sav_fm_stat = 482;
			}
			break;
		case 482:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r1>>8)&0xff);
				sav_fm_stat = 483;
			}
			break;
		case 483:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r1&0xff);
				cksmspi1+=sav_addr1_xy3z4r1;
				sav_fm_stat = 484;
			}
			break;
		case 484:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r2>>24)&0xff);
				sav_fm_stat = 485;
			}
			break;
		case 485:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r2>>16)&0xff);
				sav_fm_stat = 486;
			}
			break;
		case 486:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r2>>8)&0xff);
				sav_fm_stat = 487;
			}
			break;
		case 487:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r2&0xff);
				cksmspi1+=sav_addr1_xy3z4r2;
				sav_fm_stat = 488;
			}
			break;
		case 488:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r3>>24)&0xff);
				sav_fm_stat = 489;
			}
			break;
		case 489:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r3>>16)&0xff);
				sav_fm_stat = 490;
			}
			break;
		case 490:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r3>>8)&0xff);
				sav_fm_stat = 491;
			}
			break;
		case 491:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r3&0xff);
				cksmspi1+=sav_addr1_xy3z4r3;
				sav_fm_stat = 492;
			}
			break;
		case 492:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r4>>24)&0xff);
				sav_fm_stat = 493;
			}
			break;
		case 493:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r4>>16)&0xff);
				sav_fm_stat = 494;
			}
			break;
		case 494:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r4>>8)&0xff);
				sav_fm_stat = 495;
			}
			break;
		case 495:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r4&0xff);
				cksmspi1+=sav_addr1_xy3z4r4;
				sav_fm_stat = 496;
			}
			break;
		case 496:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r5>>24)&0xff);
				sav_fm_stat = 497;
			}
			break;
		case 497:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r5>>16)&0xff);
				sav_fm_stat = 498;
			}
			break;
		case 498:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r5>>8)&0xff);
				sav_fm_stat = 499;
			}
			break;
		case 499:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r5&0xff);
				cksmspi1+=sav_addr1_xy3z4r5;
				sav_fm_stat = 500;
			}
			break;
		case 500:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r0>>24)&0xff);
				sav_fm_stat = 501;
			}
			break;
		case 501:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r0>>16)&0xff);
				sav_fm_stat = 502;
			}
			break;
		case 502:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r0>>8)&0xff);
				sav_fm_stat = 503;
			}
			break;
		case 503:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r0&0xff);
				cksmspi1+=sav_addr1_xy4z0r0;
				sav_fm_stat = 504;
			}
			break;
		case 504:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r1>>24)&0xff);
				sav_fm_stat = 505;
			}
			break;
		case 505:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r1>>16)&0xff);
				sav_fm_stat = 506;
			}
			break;
		case 506:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r1>>8)&0xff);
				sav_fm_stat = 507;
			}
			break;
		case 507:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r1&0xff);
				cksmspi1+=sav_addr1_xy4z0r1;
				sav_fm_stat = 508;
			}
			break;
		case 508:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r2>>24)&0xff);
				sav_fm_stat = 509;
			}
			break;
		case 509:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r2>>16)&0xff);
				sav_fm_stat = 510;
			}
			break;
		case 510:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r2>>8)&0xff);
				sav_fm_stat = 511;
			}
			break;
		case 511:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r2&0xff);
				cksmspi1+=sav_addr1_xy4z0r2;
				sav_fm_stat = 512;
			}
			break;
		case 512:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r3>>24)&0xff);
				sav_fm_stat = 513;
			}
			break;
		case 513:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r3>>16)&0xff);
				sav_fm_stat = 514;
			}
			break;
		case 514:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r3>>8)&0xff);
				sav_fm_stat = 515;
			}
			break;
		case 515:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r3&0xff);
				cksmspi1+=sav_addr1_xy4z0r3;
				sav_fm_stat = 516;
			}
			break;
		case 516:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r4>>24)&0xff);
				sav_fm_stat = 517;
			}
			break;
		case 517:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r4>>16)&0xff);
				sav_fm_stat = 518;
			}
			break;
		case 518:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r4>>8)&0xff);
				sav_fm_stat = 519;
			}
			break;
		case 519:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r4&0xff);
				cksmspi1+=sav_addr1_xy4z0r4;
				sav_fm_stat = 520;
			}
			break;
		case 520:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r5>>24)&0xff);
				sav_fm_stat = 521;
			}
			break;
		case 521:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r5>>16)&0xff);
				sav_fm_stat = 522;
			}
			break;
		case 522:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r5>>8)&0xff);
				sav_fm_stat = 523;
			}
			break;
		case 523:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r5&0xff);
				cksmspi1+=sav_addr1_xy4z0r5;
				sav_fm_stat = 524;
			}
			break;
		case 524:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r0>>24)&0xff);
				sav_fm_stat = 525;
			}
			break;
		case 525:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r0>>16)&0xff);
				sav_fm_stat = 526;
			}
			break;
		case 526:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r0>>8)&0xff);
				sav_fm_stat = 527;
			}
			break;
		case 527:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r0&0xff);
				cksmspi1+=sav_addr1_xy4z1r0;
				sav_fm_stat = 528;
			}
			break;
		case 528:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r1>>24)&0xff);
				sav_fm_stat = 529;
			}
			break;
		case 529:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r1>>16)&0xff);
				sav_fm_stat = 530;
			}
			break;
		case 530:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r1>>8)&0xff);
				sav_fm_stat = 531;
			}
			break;
		case 531:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r1&0xff);
				cksmspi1+=sav_addr1_xy4z1r1;
				sav_fm_stat = 532;
			}
			break;
		case 532:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r2>>24)&0xff);
				sav_fm_stat = 533;
			}
			break;
		case 533:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r2>>16)&0xff);
				sav_fm_stat = 534;
			}
			break;
		case 534:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r2>>8)&0xff);
				sav_fm_stat = 535;
			}
			break;
		case 535:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r2&0xff);
				cksmspi1+=sav_addr1_xy4z1r2;
				sav_fm_stat = 536;
			}
			break;
		case 536:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r3>>24)&0xff);
				sav_fm_stat = 537;
			}
			break;
		case 537:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r3>>16)&0xff);
				sav_fm_stat = 538;
			}
			break;
		case 538:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r3>>8)&0xff);
				sav_fm_stat = 539;
			}
			break;
		case 539:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r3&0xff);
				cksmspi1+=sav_addr1_xy4z1r3;
				sav_fm_stat = 540;
			}
			break;
		case 540:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r4>>24)&0xff);
				sav_fm_stat = 541;
			}
			break;
		case 541:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r4>>16)&0xff);
				sav_fm_stat = 542;
			}
			break;
		case 542:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r4>>8)&0xff);
				sav_fm_stat = 543;
			}
			break;
		case 543:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r4&0xff);
				cksmspi1+=sav_addr1_xy4z1r4;
				sav_fm_stat = 544;
			}
			break;
		case 544:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r5>>24)&0xff);
				sav_fm_stat = 545;
			}
			break;
		case 545:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r5>>16)&0xff);
				sav_fm_stat = 546;
			}
			break;
		case 546:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r5>>8)&0xff);
				sav_fm_stat = 547;
			}
			break;
		case 547:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r5&0xff);
				cksmspi1+=sav_addr1_xy4z1r5;
				sav_fm_stat = 548;
			}
			break;
		case 548:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r0>>24)&0xff);
				sav_fm_stat = 549;
			}
			break;
		case 549:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r0>>16)&0xff);
				sav_fm_stat = 550;
			}
			break;
		case 550:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r0>>8)&0xff);
				sav_fm_stat = 551;
			}
			break;
		case 551:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r0&0xff);
				cksmspi1+=sav_addr1_xy4z2r0;
				sav_fm_stat = 552;
			}
			break;
		case 552:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r1>>24)&0xff);
				sav_fm_stat = 553;
			}
			break;
		case 553:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r1>>16)&0xff);
				sav_fm_stat = 554;
			}
			break;
		case 554:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r1>>8)&0xff);
				sav_fm_stat = 555;
			}
			break;
		case 555:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r1&0xff);
				cksmspi1+=sav_addr1_xy4z2r1;
				sav_fm_stat = 556;
			}
			break;
		case 556:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r2>>24)&0xff);
				sav_fm_stat = 557;
			}
			break;
		case 557:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r2>>16)&0xff);
				sav_fm_stat = 558;
			}
			break;
		case 558:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r2>>8)&0xff);
				sav_fm_stat = 559;
			}
			break;
		case 559:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r2&0xff);
				cksmspi1+=sav_addr1_xy4z2r2;
				sav_fm_stat = 560;
			}
			break;
		case 560:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r3>>24)&0xff);
				sav_fm_stat = 561;
			}
			break;
		case 561:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r3>>16)&0xff);
				sav_fm_stat = 562;
			}
			break;
		case 562:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r3>>8)&0xff);
				sav_fm_stat = 563;
			}
			break;
		case 563:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r3&0xff);
				cksmspi1+=sav_addr1_xy4z2r3;
				sav_fm_stat = 564;
			}
			break;
		case 564:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r4>>24)&0xff);
				sav_fm_stat = 565;
			}
			break;
		case 565:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r4>>16)&0xff);
				sav_fm_stat = 566;
			}
			break;
		case 566:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r4>>8)&0xff);
				sav_fm_stat = 567;
			}
			break;
		case 567:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r4&0xff);
				cksmspi1+=sav_addr1_xy4z2r4;
				sav_fm_stat = 568;
			}
			break;
		case 568:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r5>>24)&0xff);
				sav_fm_stat = 569;
			}
			break;
		case 569:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r5>>16)&0xff);
				sav_fm_stat = 570;
			}
			break;
		case 570:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r5>>8)&0xff);
				sav_fm_stat = 571;
			}
			break;
		case 571:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r5&0xff);
				cksmspi1+=sav_addr1_xy4z2r5;
				sav_fm_stat = 572;
			}
			break;
		case 572:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r0>>24)&0xff);
				sav_fm_stat = 573;
			}
			break;
		case 573:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r0>>16)&0xff);
				sav_fm_stat = 574;
			}
			break;
		case 574:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r0>>8)&0xff);
				sav_fm_stat = 575;
			}
			break;
		case 575:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r0&0xff);
				cksmspi1+=sav_addr1_xy4z3r0;
				sav_fm_stat = 576;
			}
			break;
		case 576:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r1>>24)&0xff);
				sav_fm_stat = 577;
			}
			break;
		case 577:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r1>>16)&0xff);
				sav_fm_stat = 578;
			}
			break;
		case 578:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r1>>8)&0xff);
				sav_fm_stat = 579;
			}
			break;
		case 579:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r1&0xff);
				cksmspi1+=sav_addr1_xy4z3r1;
				sav_fm_stat = 580;
			}
			break;
		case 580:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r2>>24)&0xff);
				sav_fm_stat = 581;
			}
			break;
		case 581:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r2>>16)&0xff);
				sav_fm_stat = 582;
			}
			break;
		case 582:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r2>>8)&0xff);
				sav_fm_stat = 583;
			}
			break;
		case 583:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r2&0xff);
				cksmspi1+=sav_addr1_xy4z3r2;
				sav_fm_stat = 584;
			}
			break;
		case 584:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r3>>24)&0xff);
				sav_fm_stat = 585;
			}
			break;
		case 585:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r3>>16)&0xff);
				sav_fm_stat = 586;
			}
			break;
		case 586:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r3>>8)&0xff);
				sav_fm_stat = 587;
			}
			break;
		case 587:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r3&0xff);
				cksmspi1+=sav_addr1_xy4z3r3;
				sav_fm_stat = 588;
			}
			break;
		case 588:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r4>>24)&0xff);
				sav_fm_stat = 589;
			}
			break;
		case 589:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r4>>16)&0xff);
				sav_fm_stat = 590;
			}
			break;
		case 590:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r4>>8)&0xff);
				sav_fm_stat = 591;
			}
			break;
		case 591:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r4&0xff);
				cksmspi1+=sav_addr1_xy4z3r4;
				sav_fm_stat = 592;
			}
			break;
		case 592:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r5>>24)&0xff);
				sav_fm_stat = 593;
			}
			break;
		case 593:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r5>>16)&0xff);
				sav_fm_stat = 594;
			}
			break;
		case 594:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r5>>8)&0xff);
				sav_fm_stat = 595;
			}
			break;
		case 595:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r5&0xff);
				cksmspi1+=sav_addr1_xy4z3r5;
				sav_fm_stat = 596;
			}
			break;
		case 596:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r0>>24)&0xff);
				sav_fm_stat = 597;
			}
			break;
		case 597:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r0>>16)&0xff);
				sav_fm_stat = 598;
			}
			break;
		case 598:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r0>>8)&0xff);
				sav_fm_stat = 599;
			}
			break;
		case 599:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r0&0xff);
				cksmspi1+=sav_addr1_xy4z4r0;
				sav_fm_stat = 600;
			}
			break;
		case 600:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r1>>24)&0xff);
				sav_fm_stat = 601;
			}
			break;
		case 601:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r1>>16)&0xff);
				sav_fm_stat = 602;
			}
			break;
		case 602:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r1>>8)&0xff);
				sav_fm_stat = 603;
			}
			break;
		case 603:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r1&0xff);
				cksmspi1+=sav_addr1_xy4z4r1;
				sav_fm_stat = 604;
			}
			break;
		case 604:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r2>>24)&0xff);
				sav_fm_stat = 605;
			}
			break;
		case 605:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r2>>16)&0xff);
				sav_fm_stat = 606;
			}
			break;
		case 606:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r2>>8)&0xff);
				sav_fm_stat = 607;
			}
			break;
		case 607:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r2&0xff);
				cksmspi1+=sav_addr1_xy4z4r2;
				sav_fm_stat = 608;
			}
			break;
		case 608:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r3>>24)&0xff);
				sav_fm_stat = 609;
			}
			break;
		case 609:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r3>>16)&0xff);
				sav_fm_stat = 610;
			}
			break;
		case 610:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r3>>8)&0xff);
				sav_fm_stat = 611;
			}
			break;
		case 611:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r3&0xff);
				cksmspi1+=sav_addr1_xy4z4r3;
				sav_fm_stat = 612;
			}
			break;
		case 612:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r4>>24)&0xff);
				sav_fm_stat = 613;
			}
			break;
		case 613:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r4>>16)&0xff);
				sav_fm_stat = 614;
			}
			break;
		case 614:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r4>>8)&0xff);
				sav_fm_stat = 615;
			}
			break;
		case 615:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r4&0xff);
				cksmspi1+=sav_addr1_xy4z4r4;
				sav_fm_stat = 616;
			}
			break;
		case 616:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r5>>24)&0xff);
				sav_fm_stat = 617;
			}
			break;
		case 617:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r5>>16)&0xff);
				sav_fm_stat = 618;
			}
			break;
		case 618:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r5>>8)&0xff);
				sav_fm_stat = 619;
			}
			break;
		case 619:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r5&0xff);
				cksmspi1+=sav_addr1_xy4z4r5;
				sav_fm_stat = 620;
			}
			break;
		case 620:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((cksmspi1>>24)&0xff);
				sav_fm_stat = 1240;
			}
			break;
		case 1240:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((cksmspi1>>16)&0xff);
				sav_fm_stat = 1241;
			}
			break;			
		case 1241:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((cksmspi1>>8)&0xff);
				sav_fm_stat = 1242;
			}
			break;	
		case 1242:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(cksmspi1&0xff);
				cksmspi1=0;
				sav_fm_stat = 621;
			}
			break;	
			
			
		case 621:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(gioA_tmp0);
				sav_fm_stat = 7+615;
			}
			break;
		case 622:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(gioB_tmp0);
				sav_fm_stat = 8+615;
			}
			break;
		case 623:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(gioA_tmp1);
				sav_fm_stat = 9+615;
			}
			break;
		case 624:
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(gioB_tmp1);
				cksmspi1=(gioA_tmp0<<24)|(gioB_tmp0<<16)|(gioA_tmp1<<8)|(gioB_tmp1);;
				sav_fm_stat = 10+615;
			}
			break;
		case (10+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(year);
				sav_fm_stat = 11+615;
			}
			break;
		case (11+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(month);
				sav_fm_stat = 12+615;
			}
			break;
		case (12+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(day);
				sav_fm_stat = 13+615;
			}
			break;
		case (13+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(hour);
				cksmspi1+=(year<<24)|(month<<16)|(day<<8)|(hour);
				sav_fm_stat = 14+615;
			}
			break;
		case (14+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(minute);
				sav_fm_stat = 15+615;
			}
			break;
		case (15+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(second);
				cksmspi1+=(minute<<8)|(second);
				sav_fm_stat = 16+615;
			}
			break;
		case (16+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr0>>24)&0xff);
				sav_fm_stat = 17+615;
			}
			break;
		case (17+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr0>>16)&0xff);
				sav_fm_stat = 18+615;
			}
			break;
		case (18+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr0>>8)&0xff);
				sav_fm_stat = 19+615;
			}
			break;
		case (19+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr0&0xff);
				cksmspi1+=sav_addr0;
				sav_fm_stat = 20+615;
			}
			break;
		case (20+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r0>>24)&0xff);
				sav_fm_stat = 21+615;
			}
			break;
		case (21+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r0>>16)&0xff);
				sav_fm_stat = 22+615;
			}
			break;
		case (22+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r0>>8)&0xff);
				sav_fm_stat = 23+615;
			}
			break;
		case (23+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r0&0xff);
				cksmspi1+=sav_addr1_xy0z0r0;
				sav_fm_stat = 24+615;
			}
			break;
		case (24+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r1>>24)&0xff);
				sav_fm_stat = 25+615;
			}
			break;
		case (25+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r1>>16)&0xff);
				sav_fm_stat = 26+615;
			}
			break;
		case (26+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r1>>8)&0xff);
				sav_fm_stat = 27+615;
			}
			break;
		case (27+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r1&0xff);
				cksmspi1+=sav_addr1_xy0z0r1;
				sav_fm_stat = 28+615;
			}
			break;
		case (28+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r2>>24)&0xff);
				sav_fm_stat = 29+615;
			}
			break;
		case (29+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r2>>16)&0xff);
				sav_fm_stat = 30+615;
			}
			break;
		case (30+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r2>>8)&0xff);
				sav_fm_stat = 31+615;
			}
			break;
		case (31+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r2&0xff);
				cksmspi1+=sav_addr1_xy0z0r2;
				sav_fm_stat = 32+615;
			}
			break;
		case (32+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r3>>24)&0xff);
				sav_fm_stat = 33+615;
			}
			break;
		case (33+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r3>>16)&0xff);
				sav_fm_stat = 34+615;
			}
			break;
		case (34+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r3>>8)&0xff);
				sav_fm_stat = 35+615;
			}
			break;
		case (35+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r3&0xff);
				cksmspi1+=sav_addr1_xy0z0r3;
				sav_fm_stat = 36+615;
			}
			break;
		case (36+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r4>>24)&0xff);
				sav_fm_stat = 37+615;
			}
			break;
		case (37+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r4>>16)&0xff);
				sav_fm_stat = 38+615;
			}
			break;
		case (38+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r4>>8)&0xff);
				sav_fm_stat = 39+615;
			}
			break;
		case (39+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r4&0xff);
				cksmspi1+=sav_addr1_xy0z0r4;
				sav_fm_stat = 40+615;
			}
			break;
		case (40+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r5>>24)&0xff);
				sav_fm_stat = 41+615;
			}
			break;
		case (41+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r5>>16)&0xff);
				sav_fm_stat = 42+615;
			}
			break;
		case (42+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z0r5>>8)&0xff);
				sav_fm_stat = 43+615;
			}
			break;
		case (43+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z0r5&0xff);
				cksmspi1+=sav_addr1_xy0z0r5;
				sav_fm_stat = 44+615;
			}
			break;
		case (44+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r0>>24)&0xff);
				sav_fm_stat = 45+615;
			}
			break;
		case (45+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r0>>16)&0xff);
				sav_fm_stat = 46+615;
			}
			break;
		case (46+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r0>>8)&0xff);
				sav_fm_stat = 47+615;
			}
			break;
		case (47+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r0&0xff);
				cksmspi1+=sav_addr1_xy0z1r0;
				sav_fm_stat = 48+615;
			}
			break;
		case (48+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r1>>24)&0xff);
				sav_fm_stat = 49+615;
			}
			break;
		case (49+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r1>>16)&0xff);
				sav_fm_stat = 50+615;
			}
			break;
		case (50+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r1>>8)&0xff);
				sav_fm_stat = 51+615;
			}
			break;
		case (51+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r1&0xff);
				cksmspi1+=sav_addr1_xy0z1r1;
				sav_fm_stat = 52+615;
			}
			break;
		case (52+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r2>>24)&0xff);
				sav_fm_stat = 53+615;
			}
			break;
		case (53+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r2>>16)&0xff);
				sav_fm_stat = 54+615;
			}
			break;
		case (54+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r2>>8)&0xff);
				sav_fm_stat = 55+615;
			}
			break;
		case (55+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r2&0xff);
				cksmspi1+=sav_addr1_xy0z1r2;
				sav_fm_stat = 56+615;
			}
			break;
		case (56+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r3>>24)&0xff);
				sav_fm_stat = 57+615;
			}
			break;
		case (57+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r3>>16)&0xff);
				sav_fm_stat = 58+615;
			}
			break;
		case (58+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r3>>8)&0xff);
				sav_fm_stat = 59+615;
			}
			break;
		case (59+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r3&0xff);
				cksmspi1+=sav_addr1_xy0z1r3;
				sav_fm_stat = 60+615;
			}
			break;
		case (60+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r4>>24)&0xff);
				sav_fm_stat = 61+615;
			}
			break;
		case (61+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r4>>16)&0xff);
				sav_fm_stat = 62+615;
			}
			break;
		case (62+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r4>>8)&0xff);
				sav_fm_stat = 63+615;
			}
			break;
		case (63+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r4&0xff);
				cksmspi1+=sav_addr1_xy0z1r4;
				sav_fm_stat = 64+615;
			}
			break;
		case (64+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r5>>24)&0xff);
				sav_fm_stat = 65+615;
			}
			break;
		case (65+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r5>>16)&0xff);
				sav_fm_stat = 66+615;
			}
			break;
		case (66+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z1r5>>8)&0xff);
				sav_fm_stat = 67+615;
			}
			break;
		case (67+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z1r5&0xff);
				cksmspi1+=sav_addr1_xy0z1r5;
				sav_fm_stat = 68+615;
			}
			break;
		case (68+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r0>>24)&0xff);
				sav_fm_stat = 69+615;
			}
			break;
		case (69+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r0>>16)&0xff);
				sav_fm_stat = 70+615;
			}
			break;
		case (70+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r0>>8)&0xff);
				sav_fm_stat = 71+615;
			}
			break;
		case (71+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r0&0xff);
				cksmspi1+=sav_addr1_xy0z2r0;
				sav_fm_stat = 72+615;
			}
			break;
		case (72+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r1>>24)&0xff);
				sav_fm_stat = 73+615;
			}
			break;
		case (73+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r1>>16)&0xff);
				sav_fm_stat = 74+615;
			}
			break;
		case (74+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r1>>8)&0xff);
				sav_fm_stat = 75+615;
			}
			break;
		case (75+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r1&0xff);
				cksmspi1+=sav_addr1_xy0z2r1;
				sav_fm_stat = 76+615;
			}
			break;
		case (76+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r2>>24)&0xff);
				sav_fm_stat = 77+615;
			}
			break;
		case (77+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r2>>16)&0xff);
				sav_fm_stat = 78+615;
			}
			break;
		case (78+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r2>>8)&0xff);
				sav_fm_stat = 79+615;
			}
			break;
		case (79+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r2&0xff);
				cksmspi1+=sav_addr1_xy0z2r2;
				sav_fm_stat = 80+615;
			}
			break;
		case (80+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r3>>24)&0xff);
				sav_fm_stat = 81+615;
			}
			break;
		case (81+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r3>>16)&0xff);
				sav_fm_stat = 82+615;
			}
			break;
		case (82+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r3>>8)&0xff);
				sav_fm_stat = 83+615;
			}
			break;
		case (83+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r3&0xff);
				cksmspi1+=sav_addr1_xy0z2r3;
				sav_fm_stat = 84+615;
			}
			break;
		case (84+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r4>>24)&0xff);
				sav_fm_stat = 85+615;
			}
			break;
		case (85+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r4>>16)&0xff);
				sav_fm_stat = 86+615;
			}
			break;
		case (86+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r4>>8)&0xff);
				sav_fm_stat = 87+615;
			}
			break;
		case (87+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r4&0xff);
				cksmspi1+=sav_addr1_xy0z2r4;
				sav_fm_stat = 88+615;
			}
			break;
		case (88+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r5>>24)&0xff);
				sav_fm_stat = 89+615;
			}
			break;
		case (89+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r5>>16)&0xff);
				sav_fm_stat = 90+615;
			}
			break;
		case (90+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z2r5>>8)&0xff);
				sav_fm_stat = 91+615;
			}
			break;
		case (91+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z2r5&0xff);
				cksmspi1+=sav_addr1_xy0z2r5;
				sav_fm_stat = 92+615;
			}
			break;
		case (92+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r0>>24)&0xff);
				sav_fm_stat = 93+615;
			}
			break;
		case (93+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r0>>16)&0xff);
				sav_fm_stat = 94+615;
			}
			break;
		case (94+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r0>>8)&0xff);
				sav_fm_stat = 95+615;
			}
			break;
		case (95+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r0&0xff);
				cksmspi1+=sav_addr1_xy0z3r0;
				sav_fm_stat = 96+615;
			}
			break;
		case (96+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r1>>24)&0xff);
				sav_fm_stat = 97+615;
			}
			break;
		case (97+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r1>>16)&0xff);
				sav_fm_stat = 98+615;
			}
			break;
		case (98+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r1>>8)&0xff);
				sav_fm_stat = 99+615;
			}
			break;
		case (99+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r1&0xff);
				cksmspi1+=sav_addr1_xy0z3r1;
				sav_fm_stat = 100+615;
			}
			break;
		case (100+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r2>>24)&0xff);
				sav_fm_stat = 101+615;
			}
			break;
		case (101+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r2>>16)&0xff);
				sav_fm_stat = 102+615;
			}
			break;
		case (102+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r2>>8)&0xff);
				sav_fm_stat = 103+615;
			}
			break;
		case (103+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r2&0xff);
				cksmspi1+=sav_addr1_xy0z3r2;
				sav_fm_stat = 104+615;
			}
			break;
		case (104+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r3>>24)&0xff);
				sav_fm_stat = 105+615;
			}
			break;
		case (105+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r3>>16)&0xff);
				sav_fm_stat = 106+615;
			}
			break;
		case (106+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r3>>8)&0xff);
				sav_fm_stat = 107+615;
			}
			break;
		case (107+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r3&0xff);
				cksmspi1+=sav_addr1_xy0z3r3;
				sav_fm_stat = 108+615;
			}
			break;
		case (108+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r4>>24)&0xff);
				sav_fm_stat = 109+615;
			}
			break;
		case (109+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r4>>16)&0xff);
				sav_fm_stat = 110+615;
			}
			break;
		case (110+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r4>>8)&0xff);
				sav_fm_stat = 111+615;
			}
			break;
		case (111+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r4&0xff);
				cksmspi1+=sav_addr1_xy0z3r4;
				sav_fm_stat = 112+615;
			}
			break;
		case (112+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r5>>24)&0xff);
				sav_fm_stat = 113+615;
			}
			break;
		case (113+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r5>>16)&0xff);
				sav_fm_stat = 114+615;
			}
			break;
		case (114+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z3r5>>8)&0xff);
				sav_fm_stat = 115+615;
			}
			break;
		case (115+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z3r5&0xff);
				cksmspi1+=sav_addr1_xy0z3r5;
				sav_fm_stat = 116+615;
			}
			break;
		case (116+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r0>>24)&0xff);
				sav_fm_stat = 117+615;
			}
			break;
		case (117+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r0>>16)&0xff);
				sav_fm_stat = 118+615;
			}
			break;
		case (118+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r0>>8)&0xff);
				sav_fm_stat = 119+615;
			}
			break;
		case (119+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r0&0xff);
				cksmspi1+=sav_addr1_xy0z4r0;
				sav_fm_stat = 120+615;
			}
			break;
		case (120+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r1>>24)&0xff);
				sav_fm_stat = 121+615;
			}
			break;
		case (121+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r1>>16)&0xff);
				sav_fm_stat = 122+615;
			}
			break;
		case (122+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r1>>8)&0xff);
				sav_fm_stat = 123+615;
			}
			break;
		case (123+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r1&0xff);
				cksmspi1+=sav_addr1_xy0z4r1;
				sav_fm_stat = 124+615;
			}
			break;
		case (124+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r2>>24)&0xff);
				sav_fm_stat = 125+615;
			}
			break;
		case (125+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r2>>16)&0xff);
				sav_fm_stat = 126+615;
			}
			break;
		case (126+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r2>>8)&0xff);
				sav_fm_stat = 127+615;
			}
			break;
		case (127+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r2&0xff);
				cksmspi1+=sav_addr1_xy0z4r2;
				sav_fm_stat = 128+615;
			}
			break;
		case (128+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r3>>24)&0xff);
				sav_fm_stat = 129+615;
			}
			break;
		case (129+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r3>>16)&0xff);
				sav_fm_stat = 130+615;
			}
			break;
		case (130+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r3>>8)&0xff);
				sav_fm_stat = 131+615;
			}
			break;
		case (131+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r3&0xff);
				cksmspi1+=sav_addr1_xy0z4r3;
				sav_fm_stat = 132+615;
			}
			break;
		case (132+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r4>>24)&0xff);
				sav_fm_stat = 133+615;
			}
			break;
		case (133+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r4>>16)&0xff);
				sav_fm_stat = 134+615;
			}
			break;
		case (134+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r4>>8)&0xff);
				sav_fm_stat = 135+615;
			}
			break;
		case (135+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r4&0xff);
				cksmspi1+=sav_addr1_xy0z4r4;
				sav_fm_stat = 136+615;
			}
			break;
		case (136+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r5>>24)&0xff);
				sav_fm_stat = 137+615;
			}
			break;
		case (137+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r5>>16)&0xff);
				sav_fm_stat = 138+615;
			}
			break;
		case (138+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy0z4r5>>8)&0xff);
				sav_fm_stat = 139+615;
			}
			break;
		case (139+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy0z4r5&0xff);
				cksmspi1+=sav_addr1_xy0z4r5;
				sav_fm_stat = 140+615;
			}
			break;
		case (140+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r0>>24)&0xff);
				sav_fm_stat = 141+615;
			}
			break;
		case (141+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r0>>16)&0xff);
				sav_fm_stat = 142+615;
			}
			break;
		case (142+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r0>>8)&0xff);
				sav_fm_stat = 143+615;
			}
			break;
		case (143+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r0&0xff);
				cksmspi1+=sav_addr1_xy1z0r0;
				sav_fm_stat = 144+615;
			}
			break;
		case (144+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r1>>24)&0xff);
				sav_fm_stat = 145+615;
			}
			break;
		case (145+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r1>>16)&0xff);
				sav_fm_stat = 146+615;
			}
			break;
		case (146+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r1>>8)&0xff);
				sav_fm_stat = 147+615;
			}
			break;
		case (147+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r1&0xff);
				cksmspi1+=sav_addr1_xy1z0r1;
				sav_fm_stat = 148+615;
			}
			break;
		case (148+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r2>>24)&0xff);
				sav_fm_stat = 149+615;
			}
			break;
		case (149+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r2>>16)&0xff);
				sav_fm_stat = 150+615;
			}
			break;
		case (150+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r2>>8)&0xff);
				sav_fm_stat = 151+615;
			}
			break;
		case (151+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r2&0xff);
				cksmspi1+=sav_addr1_xy1z0r2;
				sav_fm_stat = 152+615;
			}
			break;
		case (152+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r3>>24)&0xff);
				sav_fm_stat = 153+615;
			}
			break;
		case (153+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r3>>16)&0xff);
				sav_fm_stat = 154+615;
			}
			break;
		case (154+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r3>>8)&0xff);
				sav_fm_stat = 155+615;
			}
			break;
		case (155+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r3&0xff);
				cksmspi1+=sav_addr1_xy1z0r3;
				sav_fm_stat = 156+615;
			}
			break;
		case (156+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r4>>24)&0xff);
				sav_fm_stat = 157+615;
			}
			break;
		case (157+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r4>>16)&0xff);
				sav_fm_stat = 158+615;
			}
			break;
		case (158+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r4>>8)&0xff);
				sav_fm_stat = 159+615;
			}
			break;
		case (159+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r4&0xff);
				cksmspi1+=sav_addr1_xy1z0r4;
				sav_fm_stat = 160+615;
			}
			break;
		case (160+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r5>>24)&0xff);
				sav_fm_stat = 161+615;
			}
			break;
		case (161+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r5>>16)&0xff);
				sav_fm_stat = 162+615;
			}
			break;
		case (162+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z0r5>>8)&0xff);
				sav_fm_stat = 163+615;
			}
			break;
		case (163+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z0r5&0xff);
				cksmspi1+=sav_addr1_xy1z0r5;
				sav_fm_stat = 164+615;
			}
			break;
		case (164+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r0>>24)&0xff);
				sav_fm_stat = 165+615;
			}
			break;
		case (165+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r0>>16)&0xff);
				sav_fm_stat = 166+615;
			}
			break;
		case (166+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r0>>8)&0xff);
				sav_fm_stat = 167+615;
			}
			break;
		case (167+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r0&0xff);
				cksmspi1+=sav_addr1_xy1z1r0;
				sav_fm_stat = 168+615;
			}
			break;
		case (168+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r1>>24)&0xff);
				sav_fm_stat = 169+615;
			}
			break;
		case (169+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r1>>16)&0xff);
				sav_fm_stat = 170+615;
			}
			break;
		case (170+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r1>>8)&0xff);
				sav_fm_stat = 171+615;
			}
			break;
		case (171+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r1&0xff);
				cksmspi1+=sav_addr1_xy1z1r1;
				sav_fm_stat = 172+615;
			}
			break;
		case (172+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r2>>24)&0xff);
				sav_fm_stat = 173+615;
			}
			break;
		case (173+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r2>>16)&0xff);
				sav_fm_stat = 174+615;
			}
			break;
		case (174+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r2>>8)&0xff);
				sav_fm_stat = 175+615;
			}
			break;
		case (175+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r2&0xff);
				cksmspi1+=sav_addr1_xy1z1r2;
				sav_fm_stat = 176+615;
			}
			break;
		case (176+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r3>>24)&0xff);
				sav_fm_stat = 177+615;
			}
			break;
		case (177+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r3>>16)&0xff);
				sav_fm_stat = 178+615;
			}
			break;
		case (178+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r3>>8)&0xff);
				sav_fm_stat = 179+615;
			}
			break;
		case (179+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r3&0xff);
				cksmspi1+=sav_addr1_xy1z1r3;
				sav_fm_stat = 180+615;
			}
			break;
		case (180+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r4>>24)&0xff);
				sav_fm_stat = 181+615;
			}
			break;
		case (181+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r4>>16)&0xff);
				sav_fm_stat = 182+615;
			}
			break;
		case (182+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r4>>8)&0xff);
				sav_fm_stat = 183+615;
			}
			break;
		case (183+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r4&0xff);
				cksmspi1+=sav_addr1_xy1z1r4;
				sav_fm_stat = 184+615;
			}
			break;
		case (184+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r5>>24)&0xff);
				sav_fm_stat = 185+615;
			}
			break;
		case (185+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r5>>16)&0xff);
				sav_fm_stat = 186+615;
			}
			break;
		case (186+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z1r5>>8)&0xff);
				sav_fm_stat = 187+615;
			}
			break;
		case (187+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z1r5&0xff);
				cksmspi1+=sav_addr1_xy1z1r5;
				sav_fm_stat = 188+615;
			}
			break;
		case (188+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r0>>24)&0xff);
				sav_fm_stat = 189+615;
			}
			break;
		case (189+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r0>>16)&0xff);
				sav_fm_stat = 190+615;
			}
			break;
		case (190+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r0>>8)&0xff);
				sav_fm_stat = 191+615;
			}
			break;
		case (191+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r0&0xff);
				cksmspi1+=sav_addr1_xy1z2r0;
				sav_fm_stat = 192+615;
			}
			break;
		case (192+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r1>>24)&0xff);
				sav_fm_stat = 193+615;
			}
			break;
		case (193+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r1>>16)&0xff);
				sav_fm_stat = 194+615;
			}
			break;
		case (194+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r1>>8)&0xff);
				sav_fm_stat = 195+615;
			}
			break;
		case (195+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r1&0xff);
				cksmspi1+=sav_addr1_xy1z2r1;
				sav_fm_stat = 196+615;
			}
			break;
		case (196+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r2>>24)&0xff);
				sav_fm_stat = 197+615;
			}
			break;
		case (197+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r2>>16)&0xff);
				sav_fm_stat = 198+615;
			}
			break;
		case (198+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r2>>8)&0xff);
				sav_fm_stat = 199+615;
			}
			break;
		case (199+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r2&0xff);
				cksmspi1+=sav_addr1_xy1z2r2;
				sav_fm_stat = 200+615;
			}
			break;
		case (200+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r3>>24)&0xff);
				sav_fm_stat = 201+615;
			}
			break;
		case (201+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r3>>16)&0xff);
				sav_fm_stat = 202+615;
			}
			break;
		case (202+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r3>>8)&0xff);
				sav_fm_stat = 203+615;
			}
			break;
		case (203+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r3&0xff);
				cksmspi1+=sav_addr1_xy1z2r3;
				sav_fm_stat = 204+615;
			}
			break;
		case (204+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r4>>24)&0xff);
				sav_fm_stat = 205+615;
			}
			break;
		case (205+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r4>>16)&0xff);
				sav_fm_stat = 206+615;
			}
			break;
		case (206+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r4>>8)&0xff);
				sav_fm_stat = 207+615;
			}
			break;
		case (207+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r4&0xff);
				cksmspi1+=sav_addr1_xy1z2r4;
				sav_fm_stat = 208+615;
			}
			break;
		case (208+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r5>>24)&0xff);
				sav_fm_stat = 209+615;
			}
			break;
		case (209+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r5>>16)&0xff);
				sav_fm_stat = 210+615;
			}
			break;
		case (210+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z2r5>>8)&0xff);
				sav_fm_stat = 211+615;
			}
			break;
		case (211+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z2r5&0xff);
				cksmspi1+=sav_addr1_xy1z2r5;
				sav_fm_stat = 212+615;
			}
			break;
		case (212+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r0>>24)&0xff);
				sav_fm_stat = 213+615;
			}
			break;
		case (213+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r0>>16)&0xff);
				sav_fm_stat = 214+615;
			}
			break;
		case (214+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r0>>8)&0xff);
				sav_fm_stat = 215+615;
			}
			break;
		case (215+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r0&0xff);
				cksmspi1+=sav_addr1_xy1z3r0;
				sav_fm_stat = 216+615;
			}
			break;
		case (216+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r1>>24)&0xff);
				sav_fm_stat = 217+615;
			}
			break;
		case (217+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r1>>16)&0xff);
				sav_fm_stat = 218+615;
			}
			break;
		case (218+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r1>>8)&0xff);
				sav_fm_stat = 219+615;
			}
			break;
		case (219+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r1&0xff);
				cksmspi1+=sav_addr1_xy1z3r1;
				sav_fm_stat = 220+615;
			}
			break;
		case (220+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r2>>24)&0xff);
				sav_fm_stat = 221+615;
			}
			break;
		case (221+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r2>>16)&0xff);
				sav_fm_stat = 222+615;
			}
			break;
		case (222+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r2>>8)&0xff);
				sav_fm_stat = 223+615;
			}
			break;
		case (223+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r2&0xff);
				cksmspi1+=sav_addr1_xy1z3r2;
				sav_fm_stat = 224+615;
			}
			break;
		case (224+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r3>>24)&0xff);
				sav_fm_stat = 225+615;
			}
			break;
		case (225+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r3>>16)&0xff);
				sav_fm_stat = 226+615;
			}
			break;
		case (226+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r3>>8)&0xff);
				sav_fm_stat = 227+615;
			}
			break;
		case (227+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r3&0xff);
				cksmspi1+=sav_addr1_xy1z3r3;
				sav_fm_stat = 228+615;
			}
			break;
		case (228+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r4>>24)&0xff);
				sav_fm_stat = 229+615;
			}
			break;
		case (229+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r4>>16)&0xff);
				sav_fm_stat = 230+615;
			}
			break;
		case (230+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r4>>8)&0xff);
				sav_fm_stat = 231+615;
			}
			break;
		case (231+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r4&0xff);
				cksmspi1+=sav_addr1_xy1z3r4;
				sav_fm_stat = 232+615;
			}
			break;
		case (232+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r5>>24)&0xff);
				sav_fm_stat = 233+615;
			}
			break;
		case (233+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r5>>16)&0xff);
				sav_fm_stat = 234+615;
			}
			break;
		case (234+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z3r5>>8)&0xff);
				sav_fm_stat = 235+615;
			}
			break;
		case (235+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z3r5&0xff);
				cksmspi1+=sav_addr1_xy1z3r5;
				sav_fm_stat = 236+615;
			}
			break;
		case (236+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r0>>24)&0xff);
				sav_fm_stat = 237+615;
			}
			break;
		case (237+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r0>>16)&0xff);
				sav_fm_stat = 238+615;
			}
			break;
		case (238+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r0>>8)&0xff);
				sav_fm_stat = 239+615;
			}
			break;
		case (239+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r0&0xff);
				cksmspi1+=sav_addr1_xy1z4r0;
				sav_fm_stat = 240+615;
			}
			break;
		case (240+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r1>>24)&0xff);
				sav_fm_stat = 241+615;
			}
			break;
		case (241+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r1>>16)&0xff);
				sav_fm_stat = 242+615;
			}
			break;
		case (242+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r1>>8)&0xff);
				sav_fm_stat = 243+615;
			}
			break;
		case (243+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r1&0xff);
				cksmspi1+=sav_addr1_xy1z4r1;
				sav_fm_stat = 244+615;
			}
			break;
		case (244+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r2>>24)&0xff);
				sav_fm_stat = 245+615;
			}
			break;
		case (245+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r2>>16)&0xff);
				sav_fm_stat = 246+615;
			}
			break;
		case (246+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r2>>8)&0xff);
				sav_fm_stat = 247+615;
			}
			break;
		case (247+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r2&0xff);
				cksmspi1+=sav_addr1_xy1z4r2;
				sav_fm_stat = 248+615;
			}
			break;
		case (248+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r3>>24)&0xff);
				sav_fm_stat = 249+615;
			}
			break;
		case (249+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r3>>16)&0xff);
				sav_fm_stat = 250+615;
			}
			break;
		case (250+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r3>>8)&0xff);
				sav_fm_stat = 251+615;
			}
			break;
		case (251+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r3&0xff);
				cksmspi1+=sav_addr1_xy1z4r3;
				sav_fm_stat = 252+615;
			}
			break;
		case (252+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r4>>24)&0xff);
				sav_fm_stat = 253+615;
			}
			break;
		case (253+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r4>>16)&0xff);
				sav_fm_stat = 254+615;
			}
			break;
		case (254+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r4>>8)&0xff);
				sav_fm_stat = 255+615;
			}
			break;
		case (255+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r4&0xff);
				cksmspi1+=sav_addr1_xy1z4r4;
				sav_fm_stat = 256+615;
			}
			break;
		case (256+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r5>>24)&0xff);
				sav_fm_stat = 257+615;
			}
			break;
		case (257+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r5>>16)&0xff);
				sav_fm_stat = 258+615;
			}
			break;
		case (258+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy1z4r5>>8)&0xff);
				sav_fm_stat = 259+615;
			}
			break;
		case (259+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy1z4r5&0xff);
				cksmspi1+=sav_addr1_xy1z4r5;
				sav_fm_stat = 260+615;
			}
			break;
		case (260+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r0>>24)&0xff);
				sav_fm_stat = 261+615;
			}
			break;
		case (261+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r0>>16)&0xff);
				sav_fm_stat = 262+615;
			}
			break;
		case (262+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r0>>8)&0xff);
				sav_fm_stat = 263+615;
			}
			break;
		case (263+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r0&0xff);
				cksmspi1+=sav_addr1_xy2z0r0;
				sav_fm_stat = 264+615;
			}
			break;
		case (264+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r1>>24)&0xff);
				sav_fm_stat = 265+615;
			}
			break;
		case (265+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r1>>16)&0xff);
				sav_fm_stat = 266+615;
			}
			break;
		case (266+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r1>>8)&0xff);
				sav_fm_stat = 267+615;
			}
			break;
		case (267+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r1&0xff);
				cksmspi1+=sav_addr1_xy2z0r1;
				sav_fm_stat = 268+615;
			}
			break;
		case (268+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r2>>24)&0xff);
				sav_fm_stat = 269+615;
			}
			break;
		case (269+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r2>>16)&0xff);
				sav_fm_stat = 270+615;
			}
			break;
		case (270+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r2>>8)&0xff);
				sav_fm_stat = 271+615;
			}
			break;
		case (271+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r2&0xff);
				cksmspi1+=sav_addr1_xy2z0r2;
				sav_fm_stat = 272+615;
			}
			break;
		case (272+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r3>>24)&0xff);
				sav_fm_stat = 273+615;
			}
			break;
		case (273+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r3>>16)&0xff);
				sav_fm_stat = 274+615;
			}
			break;
		case (274+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r3>>8)&0xff);
				sav_fm_stat = 275+615;
			}
			break;
		case (275+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r3&0xff);
				cksmspi1+=sav_addr1_xy2z0r3;
				sav_fm_stat = 276+615;
			}
			break;
		case (276+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r4>>24)&0xff);
				sav_fm_stat = 277+615;
			}
			break;
		case (277+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r4>>16)&0xff);
				sav_fm_stat = 278+615;
			}
			break;
		case (278+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r4>>8)&0xff);
				sav_fm_stat = 279+615;
			}
			break;
		case (279+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r4&0xff);
				cksmspi1+=sav_addr1_xy2z0r4;
				sav_fm_stat = 280+615;
			}
			break;
		case (280+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r5>>24)&0xff);
				sav_fm_stat = 281+615;
			}
			break;
		case (281+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r5>>16)&0xff);
				sav_fm_stat = 282+615;
			}
			break;
		case (282+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z0r5>>8)&0xff);
				sav_fm_stat = 283+615;
			}
			break;
		case (283+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z0r5&0xff);
				cksmspi1+=sav_addr1_xy2z0r5;
				sav_fm_stat = 284+615;
			}
			break;
		case (284+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r0>>24)&0xff);
				sav_fm_stat = 285+615;
			}
			break;
		case (285+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r0>>16)&0xff);
				sav_fm_stat = 286+615;
			}
			break;
		case (286+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r0>>8)&0xff);
				sav_fm_stat = 287+615;
			}
			break;
		case (287+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r0&0xff);
				cksmspi1+=sav_addr1_xy2z1r0;
				sav_fm_stat = 288+615;
			}
			break;
		case (288+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r1>>24)&0xff);
				sav_fm_stat = 289+615;
			}
			break;
		case (289+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r1>>16)&0xff);
				sav_fm_stat = 290+615;
			}
			break;
		case (290+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r1>>8)&0xff);
				sav_fm_stat = 291+615;
			}
			break;
		case (291+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r1&0xff);
				cksmspi1+=sav_addr1_xy2z1r1;
				sav_fm_stat = 292+615;
			}
			break;
		case (292+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r2>>24)&0xff);
				sav_fm_stat = 293+615;
			}
			break;
		case (293+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r2>>16)&0xff);
				sav_fm_stat = 294+615;
			}
			break;
		case (294+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r2>>8)&0xff);
				sav_fm_stat = 295+615;
			}
			break;
		case (295+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r2&0xff);
				cksmspi1+=sav_addr1_xy2z1r2;
				sav_fm_stat = 296+615;
			}
			break;
		case (296+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r3>>24)&0xff);
				sav_fm_stat = 297+615;
			}
			break;
		case (297+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r3>>16)&0xff);
				sav_fm_stat = 298+615;
			}
			break;
		case (298+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r3>>8)&0xff);
				sav_fm_stat = 299+615;
			}
			break;
		case (299+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r3&0xff);
				cksmspi1+=sav_addr1_xy2z1r3;
				sav_fm_stat = 300+615;
			}
			break;
		case (300+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r4>>24)&0xff);
				sav_fm_stat = 301+615;
			}
			break;
		case (301+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r4>>16)&0xff);
				sav_fm_stat = 302+615;
			}
			break;
		case (302+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r4>>8)&0xff);
				sav_fm_stat = 303+615;
			}
			break;
		case (303+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r4&0xff);
				cksmspi1+=sav_addr1_xy2z1r4;
				sav_fm_stat = 304+615;
			}
			break;
		case (304+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r5>>24)&0xff);
				sav_fm_stat = 305+615;
			}
			break;
		case (305+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r5>>16)&0xff);
				sav_fm_stat = 306+615;
			}
			break;
		case (306+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z1r5>>8)&0xff);
				sav_fm_stat = 307+615;
			}
			break;
		case (307+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z1r5&0xff);
				cksmspi1+=sav_addr1_xy2z1r5;
				sav_fm_stat = 308+615;
			}
			break;
		case (308+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r0>>24)&0xff);
				sav_fm_stat = 309+615;
			}
			break;
		case (309+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r0>>16)&0xff);
				sav_fm_stat = 310+615;
			}
			break;
		case (310+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r0>>8)&0xff);
				sav_fm_stat = 311+615;
			}
			break;
		case (311+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r0&0xff);
				cksmspi1+=sav_addr1_xy2z2r0;
				sav_fm_stat = 312+615;
			}
			break;
		case (312+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r1>>24)&0xff);
				sav_fm_stat = 313+615;
			}
			break;
		case (313+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r1>>16)&0xff);
				sav_fm_stat = 314+615;
			}
			break;
		case (314+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r1>>8)&0xff);
				sav_fm_stat = 315+615;
			}
			break;
		case (315+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r1&0xff);
				cksmspi1+=sav_addr1_xy2z2r1;
				sav_fm_stat = 316+615;
			}
			break;
		case (316+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r2>>24)&0xff);
				sav_fm_stat = 317+615;
			}
			break;
		case (317+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r2>>16)&0xff);
				sav_fm_stat = 318+615;
			}
			break;
		case (318+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r2>>8)&0xff);
				sav_fm_stat = 319+615;
			}
			break;
		case (319+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r2&0xff);
				cksmspi1+=sav_addr1_xy2z2r2;
				sav_fm_stat = 320+615;
			}
			break;
		case (320+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r3>>24)&0xff);
				sav_fm_stat = 321+615;
			}
			break;
		case (321+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r3>>16)&0xff);
				sav_fm_stat = 322+615;
			}
			break;
		case (322+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r3>>8)&0xff);
				sav_fm_stat = 323+615;
			}
			break;
		case (323+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r3&0xff);
				cksmspi1+=sav_addr1_xy2z2r3;
				sav_fm_stat = 324+615;
			}
			break;
		case (324+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r4>>24)&0xff);
				sav_fm_stat = 325+615;
			}
			break;
		case (325+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r4>>16)&0xff);
				sav_fm_stat = 326+615;
			}
			break;
		case (326+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r4>>8)&0xff);
				sav_fm_stat = 327+615;
			}
			break;
		case (327+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r4&0xff);
				cksmspi1+=sav_addr1_xy2z2r4;
				sav_fm_stat = 328+615;
			}
			break;
		case (328+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r5>>24)&0xff);
				sav_fm_stat = 329+615;
			}
			break;
		case (329+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r5>>16)&0xff);
				sav_fm_stat = 330+615;
			}
			break;
		case (330+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z2r5>>8)&0xff);
				sav_fm_stat = 331+615;
			}
			break;
		case (331+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z2r5&0xff);
				cksmspi1+=sav_addr1_xy2z2r5;
				sav_fm_stat = 332+615;
			}
			break;
		case (332+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r0>>24)&0xff);
				sav_fm_stat = 333+615;
			}
			break;
		case (333+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r0>>16)&0xff);
				sav_fm_stat = 334+615;
			}
			break;
		case (334+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r0>>8)&0xff);
				sav_fm_stat = 335+615;
			}
			break;
		case (335+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r0&0xff);
				cksmspi1+=sav_addr1_xy2z3r0;
				sav_fm_stat = 336+615;
			}
			break;
		case (336+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r1>>24)&0xff);
				sav_fm_stat = 337+615;
			}
			break;
		case (337+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r1>>16)&0xff);
				sav_fm_stat = 338+615;
			}
			break;
		case (338+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r1>>8)&0xff);
				sav_fm_stat = 339+615;
			}
			break;
		case (339+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r1&0xff);
				cksmspi1+=sav_addr1_xy2z3r1;
				sav_fm_stat = 340+615;
			}
			break;
		case (340+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r2>>24)&0xff);
				sav_fm_stat = 341+615;
			}
			break;
		case (341+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r2>>16)&0xff);
				sav_fm_stat = 342+615;
			}
			break;
		case (342+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r2>>8)&0xff);
				sav_fm_stat = 343+615;
			}
			break;
		case (343+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r2&0xff);
				cksmspi1+=sav_addr1_xy2z3r2;
				sav_fm_stat = 344+615;
			}
			break;
		case (344+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r3>>24)&0xff);
				sav_fm_stat = 345+615;
			}
			break;
		case (345+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r3>>16)&0xff);
				sav_fm_stat = 346+615;
			}
			break;
		case (346+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r3>>8)&0xff);
				sav_fm_stat = 347+615;
			}
			break;
		case (347+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r3&0xff);
				cksmspi1+=sav_addr1_xy2z3r3;
				sav_fm_stat = 348+615;
			}
			break;
		case (348+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r4>>24)&0xff);
				sav_fm_stat = 349+615;
			}
			break;
		case (349+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r4>>16)&0xff);
				sav_fm_stat = 350+615;
			}
			break;
		case (350+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r4>>8)&0xff);
				sav_fm_stat = 351+615;
			}
			break;
		case (351+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r4&0xff);
				cksmspi1+=sav_addr1_xy2z3r4;
				sav_fm_stat = 352+615;
			}
			break;
		case (352+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r5>>24)&0xff);
				sav_fm_stat = 353+615;
			}
			break;
		case (353+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r5>>16)&0xff);
				sav_fm_stat = 354+615;
			}
			break;
		case (354+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z3r5>>8)&0xff);
				sav_fm_stat = 355+615;
			}
			break;
		case (355+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z3r5&0xff);
				cksmspi1+=sav_addr1_xy2z3r5;
				sav_fm_stat = 356+615;
			}
			break;
		case (356+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r0>>24)&0xff);
				sav_fm_stat = 357+615;
			}
			break;
		case (357+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r0>>16)&0xff);
				sav_fm_stat = 358+615;
			}
			break;
		case (358+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r0>>8)&0xff);
				sav_fm_stat = 359+615;
			}
			break;
		case (359+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r0&0xff);
				cksmspi1+=sav_addr1_xy2z4r0;
				sav_fm_stat = 360+615;
			}
			break;
		case (360+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r1>>24)&0xff);
				sav_fm_stat = 361+615;
			}
			break;
		case (361+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r1>>16)&0xff);
				sav_fm_stat = 362+615;
			}
			break;
		case (362+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r1>>8)&0xff);
				sav_fm_stat = 363+615;
			}
			break;
		case (363+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r1&0xff);
				cksmspi1+=sav_addr1_xy2z4r1;
				sav_fm_stat = 364+615;
			}
			break;
		case (364+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r2>>24)&0xff);
				sav_fm_stat = 365+615;
			}
			break;
		case (365+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r2>>16)&0xff);
				sav_fm_stat = 366+615;
			}
			break;
		case (366+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r2>>8)&0xff);
				sav_fm_stat = 367+615;
			}
			break;
		case (367+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r2&0xff);
				cksmspi1+=sav_addr1_xy2z4r2;
				sav_fm_stat = 368+615;
			}
			break;
		case (368+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r3>>24)&0xff);
				sav_fm_stat = 369+615;
			}
			break;
		case (369+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r3>>16)&0xff);
				sav_fm_stat = 370+615;
			}
			break;
		case (370+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r3>>8)&0xff);
				sav_fm_stat = 371+615;
			}
			break;
		case (371+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r3&0xff);
				cksmspi1+=sav_addr1_xy2z4r3;
				sav_fm_stat = 372+615;
			}
			break;
		case (372+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r4>>24)&0xff);
				sav_fm_stat = 373+615;
			}
			break;
		case (373+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r4>>16)&0xff);
				sav_fm_stat = 374+615;
			}
			break;
		case (374+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r4>>8)&0xff);
				sav_fm_stat = 375+615;
			}
			break;
		case (375+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r4&0xff);
				cksmspi1+=sav_addr1_xy2z4r4;
				sav_fm_stat = 376+615;
			}
			break;
		case (376+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r5>>24)&0xff);
				sav_fm_stat = 377+615;
			}
			break;
		case (377+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r5>>16)&0xff);
				sav_fm_stat = 378+615;
			}
			break;
		case (378+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy2z4r5>>8)&0xff);
				sav_fm_stat = 379+615;
			}
			break;
		case (379+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy2z4r5&0xff);
				cksmspi1+=sav_addr1_xy2z4r5;
				sav_fm_stat = 380+615;
			}
			break;
		case (380+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r0>>24)&0xff);
				sav_fm_stat = 381+615;
			}
			break;
		case (381+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r0>>16)&0xff);
				sav_fm_stat = 382+615;
			}
			break;
		case (382+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r0>>8)&0xff);
				sav_fm_stat = 383+615;
			}
			break;
		case (383+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r0&0xff);
				cksmspi1+=sav_addr1_xy3z0r0;
				sav_fm_stat = 384+615;
			}
			break;
		case (384+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r1>>24)&0xff);
				sav_fm_stat = 385+615;
			}
			break;
		case (385+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r1>>16)&0xff);
				sav_fm_stat = 386+615;
			}
			break;
		case (386+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r1>>8)&0xff);
				sav_fm_stat = 387+615;
			}
			break;
		case (387+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r1&0xff);
				cksmspi1+=sav_addr1_xy3z0r1;
				sav_fm_stat = 388+615;
			}
			break;
		case (388+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r2>>24)&0xff);
				sav_fm_stat = 389+615;
			}
			break;
		case (389+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r2>>16)&0xff);
				sav_fm_stat = 390+615;
			}
			break;
		case (390+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r2>>8)&0xff);
				sav_fm_stat = 391+615;
			}
			break;
		case (391+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r2&0xff);
				cksmspi1+=sav_addr1_xy3z0r2;
				sav_fm_stat = 392+615;
			}
			break;
		case (392+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r3>>24)&0xff);
				sav_fm_stat = 393+615;
			}
			break;
		case (393+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r3>>16)&0xff);
				sav_fm_stat = 394+615;
			}
			break;
		case (394+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r3>>8)&0xff);
				sav_fm_stat = 395+615;
			}
			break;
		case (395+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r3&0xff);
				cksmspi1+=sav_addr1_xy3z0r3;
				sav_fm_stat = 396+615;
			}
			break;
		case (396+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r4>>24)&0xff);
				sav_fm_stat = 397+615;
			}
			break;
		case (397+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r4>>16)&0xff);
				sav_fm_stat = 398+615;
			}
			break;
		case (398+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r4>>8)&0xff);
				sav_fm_stat = 399+615;
			}
			break;
		case (399+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r4&0xff);
				cksmspi1+=sav_addr1_xy3z0r4;
				sav_fm_stat = 400+615;
			}
			break;
		case (400+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r5>>24)&0xff);
				sav_fm_stat = 401+615;
			}
			break;
		case (401+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r5>>16)&0xff);
				sav_fm_stat = 402+615;
			}
			break;
		case (402+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z0r5>>8)&0xff);
				sav_fm_stat = 403+615;
			}
			break;
		case (403+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z0r5&0xff);
				cksmspi1+=sav_addr1_xy3z0r5;
				sav_fm_stat = 404+615;
			}
			break;
		case (404+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r0>>24)&0xff);
				sav_fm_stat = 405+615;
			}
			break;
		case (405+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r0>>16)&0xff);
				sav_fm_stat = 406+615;
			}
			break;
		case (406+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r0>>8)&0xff);
				sav_fm_stat = 407+615;
			}
			break;
		case (407+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r0&0xff);
				cksmspi1+=sav_addr1_xy3z1r0;
				sav_fm_stat = 408+615;
			}
			break;
		case (408+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r1>>24)&0xff);
				sav_fm_stat = 409+615;
			}
			break;
		case (409+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r1>>16)&0xff);
				sav_fm_stat = 410+615;
			}
			break;
		case (410+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r1>>8)&0xff);
				sav_fm_stat = 411+615;
			}
			break;
		case (411+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r1&0xff);
				cksmspi1+=sav_addr1_xy3z1r1;
				sav_fm_stat = 412+615;
			}
			break;
		case (412+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r2>>24)&0xff);
				sav_fm_stat = 413+615;
			}
			break;
		case (413+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r2>>16)&0xff);
				sav_fm_stat = 414+615;
			}
			break;
		case (414+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r2>>8)&0xff);
				sav_fm_stat = 415+615;
			}
			break;
		case (415+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r2&0xff);
				cksmspi1+=sav_addr1_xy3z1r2;
				sav_fm_stat = 416+615;
			}
			break;
		case (416+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r3>>24)&0xff);
				sav_fm_stat = 417+615;
			}
			break;
		case (417+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r3>>16)&0xff);
				sav_fm_stat = 418+615;
			}
			break;
		case (418+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r3>>8)&0xff);
				sav_fm_stat = 419+615;
			}
			break;
		case (419+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r3&0xff);
				cksmspi1+=sav_addr1_xy3z1r3;
				sav_fm_stat = 420+615;
			}
			break;
		case (420+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r4>>24)&0xff);
				sav_fm_stat = 421+615;
			}
			break;
		case (421+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r4>>16)&0xff);
				sav_fm_stat = 422+615;
			}
			break;
		case (422+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r4>>8)&0xff);
				sav_fm_stat = 423+615;
			}
			break;
		case (423+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r4&0xff);
				cksmspi1+=sav_addr1_xy3z1r4;
				sav_fm_stat = 424+615;
			}
			break;
		case (424+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r5>>24)&0xff);
				sav_fm_stat = 425+615;
			}
			break;
		case (425+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r5>>16)&0xff);
				sav_fm_stat = 426+615;
			}
			break;
		case (426+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z1r5>>8)&0xff);
				sav_fm_stat = 427+615;
			}
			break;
		case (427+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z1r5&0xff);
				cksmspi1+=sav_addr1_xy3z1r5;
				sav_fm_stat = 428+615;
			}
			break;
		case (428+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r0>>24)&0xff);
				sav_fm_stat = 429+615;
			}
			break;
		case (429+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r0>>16)&0xff);
				sav_fm_stat = 430+615;
			}
			break;
		case (430+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r0>>8)&0xff);
				sav_fm_stat = 431+615;
			}
			break;
		case (431+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r0&0xff);
				cksmspi1+=sav_addr1_xy3z2r0;
				sav_fm_stat = 432+615;
			}
			break;
		case (432+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r1>>24)&0xff);
				sav_fm_stat = 433+615;
			}
			break;
		case (433+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r1>>16)&0xff);
				sav_fm_stat = 434+615;
			}
			break;
		case (434+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r1>>8)&0xff);
				sav_fm_stat = 435+615;
			}
			break;
		case (435+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r1&0xff);
				cksmspi1+=sav_addr1_xy3z2r1;
				sav_fm_stat = 436+615;
			}
			break;
		case (436+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r2>>24)&0xff);
				sav_fm_stat = 437+615;
			}
			break;
		case (437+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r2>>16)&0xff);
				sav_fm_stat = 438+615;
			}
			break;
		case (438+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r2>>8)&0xff);
				sav_fm_stat = 439+615;
			}
			break;
		case (439+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r2&0xff);
				cksmspi1+=sav_addr1_xy3z2r2;
				sav_fm_stat = 440+615;
			}
			break;
		case (440+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r3>>24)&0xff);
				sav_fm_stat = 441+615;
			}
			break;
		case (441+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r3>>16)&0xff);
				sav_fm_stat = 442+615;
			}
			break;
		case (442+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r3>>8)&0xff);
				sav_fm_stat = 443+615;
			}
			break;
		case (443+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r3&0xff);
				cksmspi1+=sav_addr1_xy3z2r3;
				sav_fm_stat = 444+615;
			}
			break;
		case (444+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r4>>24)&0xff);
				sav_fm_stat = 445+615;
			}
			break;
		case (445+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r4>>16)&0xff);
				sav_fm_stat = 446+615;
			}
			break;
		case (446+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r4>>8)&0xff);
				sav_fm_stat = 447+615;
			}
			break;
		case (447+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r4&0xff);
				cksmspi1+=sav_addr1_xy3z2r4;
				sav_fm_stat = 448+615;
			}
			break;
		case (448+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r5>>24)&0xff);
				sav_fm_stat = 449+615;
			}
			break;
		case (449+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r5>>16)&0xff);
				sav_fm_stat = 450+615;
			}
			break;
		case (450+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z2r5>>8)&0xff);
				sav_fm_stat = 451+615;
			}
			break;
		case (451+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z2r5&0xff);
				cksmspi1+=sav_addr1_xy3z2r5;
				sav_fm_stat = 452+615;
			}
			break;
		case (452+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r0>>24)&0xff);
				sav_fm_stat = 453+615;
			}
			break;
		case (453+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r0>>16)&0xff);
				sav_fm_stat = 454+615;
			}
			break;
		case (454+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r0>>8)&0xff);
				sav_fm_stat = 455+615;
			}
			break;
		case (455+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r0&0xff);
				cksmspi1+=sav_addr1_xy3z3r0;
				sav_fm_stat = 456+615;
			}
			break;
		case (456+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r1>>24)&0xff);
				sav_fm_stat = 457+615;
			}
			break;
		case (457+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r1>>16)&0xff);
				sav_fm_stat = 458+615;
			}
			break;
		case (458+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r1>>8)&0xff);
				sav_fm_stat = 459+615;
			}
			break;
		case (459+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r1&0xff);
				cksmspi1+=sav_addr1_xy3z3r1;
				sav_fm_stat = 460+615;
			}
			break;
		case (460+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r2>>24)&0xff);
				sav_fm_stat = 461+615;
			}
			break;
		case (461+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r2>>16)&0xff);
				sav_fm_stat = 462+615;
			}
			break;
		case (462+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r2>>8)&0xff);
				sav_fm_stat = 463+615;
			}
			break;
		case (463+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r2&0xff);
				cksmspi1+=sav_addr1_xy3z3r2;
				sav_fm_stat = 464+615;
			}
			break;
		case (464+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r3>>24)&0xff);
				sav_fm_stat = 465+615;
			}
			break;
		case (465+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r3>>16)&0xff);
				sav_fm_stat = 466+615;
			}
			break;
		case (466+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r3>>8)&0xff);
				sav_fm_stat = 467+615;
			}
			break;
		case (467+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r3&0xff);
				cksmspi1+=sav_addr1_xy3z3r3;
				sav_fm_stat = 468+615;
			}
			break;
		case (468+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r4>>24)&0xff);
				sav_fm_stat = 469+615;
			}
			break;
		case (469+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r4>>16)&0xff);
				sav_fm_stat = 470+615;
			}
			break;
		case (470+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r4>>8)&0xff);
				sav_fm_stat = 471+615;
			}
			break;
		case (471+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r4&0xff);
				cksmspi1+=sav_addr1_xy3z3r4;
				sav_fm_stat = 472+615;
			}
			break;
		case (472+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r5>>24)&0xff);
				sav_fm_stat = 473+615;
			}
			break;
		case (473+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r5>>16)&0xff);
				sav_fm_stat = 474+615;
			}
			break;
		case (474+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z3r5>>8)&0xff);
				sav_fm_stat = 475+615;
			}
			break;
		case (475+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z3r5&0xff);
				cksmspi1+=sav_addr1_xy3z3r5;
				sav_fm_stat = 476+615;
			}
			break;
		case (476+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r0>>24)&0xff);
				sav_fm_stat = 477+615;
			}
			break;
		case (477+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r0>>16)&0xff);
				sav_fm_stat = 478+615;
			}
			break;
		case (478+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r0>>8)&0xff);
				sav_fm_stat = 479+615;
			}
			break;
		case (479+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r0&0xff);
				cksmspi1+=sav_addr1_xy3z4r0;
				sav_fm_stat = 480+615;
			}
			break;
		case (480+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r1>>24)&0xff);
				sav_fm_stat = 481+615;
			}
			break;
		case (481+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r1>>16)&0xff);
				sav_fm_stat = 482+615;
			}
			break;
		case (482+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r1>>8)&0xff);
				sav_fm_stat = 483+615;
			}
			break;
		case (483+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r1&0xff);
				cksmspi1+=sav_addr1_xy3z4r1;
				sav_fm_stat = 484+615;
			}
			break;
		case (484+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r2>>24)&0xff);
				sav_fm_stat = 485+615;
			}
			break;
		case (485+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r2>>16)&0xff);
				sav_fm_stat = 486+615;
			}
			break;
		case (486+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r2>>8)&0xff);
				sav_fm_stat = 487+615;
			}
			break;
		case (487+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r2&0xff);
				cksmspi1+=sav_addr1_xy3z4r2;
				sav_fm_stat = 488+615;
			}
			break;
		case (488+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r3>>24)&0xff);
				sav_fm_stat = 489+615;
			}
			break;
		case (489+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r3>>16)&0xff);
				sav_fm_stat = 490+615;
			}
			break;
		case (490+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r3>>8)&0xff);
				sav_fm_stat = 491+615;
			}
			break;
		case (491+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r3&0xff);
				cksmspi1+=sav_addr1_xy3z4r3;
				sav_fm_stat = 492+615;
			}
			break;
		case (492+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r4>>24)&0xff);
				sav_fm_stat = 493+615;
			}
			break;
		case (493+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r4>>16)&0xff);
				sav_fm_stat = 494+615;
			}
			break;
		case (494+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r4>>8)&0xff);
				sav_fm_stat = 495+615;
			}
			break;
		case (495+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r4&0xff);
				cksmspi1+=sav_addr1_xy3z4r4;
				sav_fm_stat = 496+615;
			}
			break;
		case (496+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r5>>24)&0xff);
				sav_fm_stat = 497+615;
			}
			break;
		case (497+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r5>>16)&0xff);
				sav_fm_stat = 498+615;
			}
			break;
		case (498+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy3z4r5>>8)&0xff);
				sav_fm_stat = 499+615;
			}
			break;
		case (499+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy3z4r5&0xff);
				cksmspi1+=sav_addr1_xy3z4r5;
				sav_fm_stat = 500+615;
			}
			break;
		case (500+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r0>>24)&0xff);
				sav_fm_stat = 501+615;
			}
			break;
		case (501+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r0>>16)&0xff);
				sav_fm_stat = 502+615;
			}
			break;
		case (502+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r0>>8)&0xff);
				sav_fm_stat = 503+615;
			}
			break;
		case (503+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r0&0xff);
				cksmspi1+=sav_addr1_xy4z0r0;
				sav_fm_stat = 504+615;
			}
			break;
		case (504+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r1>>24)&0xff);
				sav_fm_stat = 505+615;
			}
			break;
		case (505+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r1>>16)&0xff);
				sav_fm_stat = 506+615;
			}
			break;
		case (506+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r1>>8)&0xff);
				sav_fm_stat = 507+615;
			}
			break;
		case (507+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r1&0xff);
				cksmspi1+=sav_addr1_xy4z0r1;
				sav_fm_stat = 508+615;
			}
			break;
		case (508+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r2>>24)&0xff);
				sav_fm_stat = 509+615;
			}
			break;
		case (509+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r2>>16)&0xff);
				sav_fm_stat = 510+615;
			}
			break;
		case (510+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r2>>8)&0xff);
				sav_fm_stat = 511+615;
			}
			break;
		case (511+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r2&0xff);
				cksmspi1+=sav_addr1_xy4z0r2;
				sav_fm_stat = 512+615;
			}
			break;
		case (512+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r3>>24)&0xff);
				sav_fm_stat = 513+615;
			}
			break;
		case (513+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r3>>16)&0xff);
				sav_fm_stat = 514+615;
			}
			break;
		case (514+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r3>>8)&0xff);
				sav_fm_stat = 515+615;
			}
			break;
		case (515+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r3&0xff);
				cksmspi1+=sav_addr1_xy4z0r3;
				sav_fm_stat = 516+615;
			}
			break;
		case (516+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r4>>24)&0xff);
				sav_fm_stat = 517+615;
			}
			break;
		case (517+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r4>>16)&0xff);
				sav_fm_stat = 518+615;
			}
			break;
		case (518+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r4>>8)&0xff);
				sav_fm_stat = 519+615;
			}
			break;
		case (519+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r4&0xff);
				cksmspi1+=sav_addr1_xy4z0r4;
				sav_fm_stat = 520+615;
			}
			break;
		case (520+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r5>>24)&0xff);
				sav_fm_stat = 521+615;
			}
			break;
		case (521+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r5>>16)&0xff);
				sav_fm_stat = 522+615;
			}
			break;
		case (522+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z0r5>>8)&0xff);
				sav_fm_stat = 523+615;
			}
			break;
		case (523+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z0r5&0xff);
				cksmspi1+=sav_addr1_xy4z0r5;
				sav_fm_stat = 524+615;
			}
			break;
		case (524+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r0>>24)&0xff);
				sav_fm_stat = 525+615;
			}
			break;
		case (525+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r0>>16)&0xff);
				sav_fm_stat = 526+615;
			}
			break;
		case (526+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r0>>8)&0xff);
				sav_fm_stat = 527+615;
			}
			break;
		case (527+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r0&0xff);
				cksmspi1+=sav_addr1_xy4z1r0;
				sav_fm_stat = 528+615;
			}
			break;
		case (528+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r1>>24)&0xff);
				sav_fm_stat = 529+615;
			}
			break;
		case (529+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r1>>16)&0xff);
				sav_fm_stat = 530+615;
			}
			break;
		case (530+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r1>>8)&0xff);
				sav_fm_stat = 531+615;
			}
			break;
		case (531+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r1&0xff);
				cksmspi1+=sav_addr1_xy4z1r1;
				sav_fm_stat = 532+615;
			}
			break;
		case (532+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r2>>24)&0xff);
				sav_fm_stat = 533+615;
			}
			break;
		case (533+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r2>>16)&0xff);
				sav_fm_stat = 534+615;
			}
			break;
		case (534+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r2>>8)&0xff);
				sav_fm_stat = 535+615;
			}
			break;
		case (535+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r2&0xff);
				cksmspi1+=sav_addr1_xy4z1r2;
				sav_fm_stat = 536+615;
			}
			break;
		case (536+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r3>>24)&0xff);
				sav_fm_stat = 537+615;
			}
			break;
		case (537+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r3>>16)&0xff);
				sav_fm_stat = 538+615;
			}
			break;
		case (538+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r3>>8)&0xff);
				sav_fm_stat = 539+615;
			}
			break;
		case (539+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r3&0xff);
				cksmspi1+=sav_addr1_xy4z1r3;
				sav_fm_stat = 540+615;
			}
			break;
		case (540+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r4>>24)&0xff);
				sav_fm_stat = 541+615;
			}
			break;
		case (541+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r4>>16)&0xff);
				sav_fm_stat = 542+615;
			}
			break;
		case (542+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r4>>8)&0xff);
				sav_fm_stat = 543+615;
			}
			break;
		case (543+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r4&0xff);
				cksmspi1+=sav_addr1_xy4z1r4;
				sav_fm_stat = 544+615;
			}
			break;
		case (544+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r5>>24)&0xff);
				sav_fm_stat = 545+615;
			}
			break;
		case (545+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r5>>16)&0xff);
				sav_fm_stat = 546+615;
			}
			break;
		case (546+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z1r5>>8)&0xff);
				sav_fm_stat = 547+615;
			}
			break;
		case (547+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z1r5&0xff);
				cksmspi1+=sav_addr1_xy4z1r5;
				sav_fm_stat = 548+615;
			}
			break;
		case (548+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r0>>24)&0xff);
				sav_fm_stat = 549+615;
			}
			break;
		case (549+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r0>>16)&0xff);
				sav_fm_stat = 550+615;
			}
			break;
		case (550+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r0>>8)&0xff);
				sav_fm_stat = 551+615;
			}
			break;
		case (551+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r0&0xff);
				cksmspi1+=sav_addr1_xy4z2r0;
				sav_fm_stat = 552+615;
			}
			break;
		case (552+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r1>>24)&0xff);
				sav_fm_stat = 553+615;
			}
			break;
		case (553+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r1>>16)&0xff);
				sav_fm_stat = 554+615;
			}
			break;
		case (554+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r1>>8)&0xff);
				sav_fm_stat = 555+615;
			}
			break;
		case (555+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r1&0xff);
				cksmspi1+=sav_addr1_xy4z2r1;
				sav_fm_stat = 556+615;
			}
			break;
		case (556+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r2>>24)&0xff);
				sav_fm_stat = 557+615;
			}
			break;
		case (557+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r2>>16)&0xff);
				sav_fm_stat = 558+615;
			}
			break;
		case (558+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r2>>8)&0xff);
				sav_fm_stat = 559+615;
			}
			break;
		case (559+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r2&0xff);
				cksmspi1+=sav_addr1_xy4z2r2;
				sav_fm_stat = 560+615;
			}
			break;
		case (560+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r3>>24)&0xff);
				sav_fm_stat = 561+615;
			}
			break;
		case (561+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r3>>16)&0xff);
				sav_fm_stat = 562+615;
			}
			break;
		case (562+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r3>>8)&0xff);
				sav_fm_stat = 563+615;
			}
			break;
		case (563+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r3&0xff);
				cksmspi1+=sav_addr1_xy4z2r3;
				sav_fm_stat = 564+615;
			}
			break;
		case (564+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r4>>24)&0xff);
				sav_fm_stat = 565+615;
			}
			break;
		case (565+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r4>>16)&0xff);
				sav_fm_stat = 566+615;
			}
			break;
		case (566+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r4>>8)&0xff);
				sav_fm_stat = 567+615;
			}
			break;
		case (567+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r4&0xff);
				cksmspi1+=sav_addr1_xy4z2r4;
				sav_fm_stat = 568+615;
			}
			break;
		case (568+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r5>>24)&0xff);
				sav_fm_stat = 569+615;
			}
			break;
		case (569+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r5>>16)&0xff);
				sav_fm_stat = 570+615;
			}
			break;
		case (570+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z2r5>>8)&0xff);
				sav_fm_stat = 571+615;
			}
			break;
		case (571+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z2r5&0xff);
				cksmspi1+=sav_addr1_xy4z2r5;
				sav_fm_stat = 572+615;
			}
			break;
		case (572+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r0>>24)&0xff);
				sav_fm_stat = 573+615;
			}
			break;
		case (573+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r0>>16)&0xff);
				sav_fm_stat = 574+615;
			}
			break;
		case (574+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r0>>8)&0xff);
				sav_fm_stat = 575+615;
			}
			break;
		case (575+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r0&0xff);
				cksmspi1+=sav_addr1_xy4z3r0;
				sav_fm_stat = 576+615;
			}
			break;
		case (576+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r1>>24)&0xff);
				sav_fm_stat = 577+615;
			}
			break;
		case (577+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r1>>16)&0xff);
				sav_fm_stat = 578+615;
			}
			break;
		case (578+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r1>>8)&0xff);
				sav_fm_stat = 579+615;
			}
			break;
		case (579+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r1&0xff);
				cksmspi1+=sav_addr1_xy4z3r1;
				sav_fm_stat = 580+615;
			}
			break;
		case (580+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r2>>24)&0xff);
				sav_fm_stat = 581+615;
			}
			break;
		case (581+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r2>>16)&0xff);
				sav_fm_stat = 582+615;
			}
			break;
		case (582+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r2>>8)&0xff);
				sav_fm_stat = 583+615;
			}
			break;
		case (583+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r2&0xff);
				cksmspi1+=sav_addr1_xy4z3r2;
				sav_fm_stat = 584+615;
			}
			break;
		case (584+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r3>>24)&0xff);
				sav_fm_stat = 585+615;
			}
			break;
		case (585+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r3>>16)&0xff);
				sav_fm_stat = 586+615;
			}
			break;
		case (586+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r3>>8)&0xff);
				sav_fm_stat = 587+615;
			}
			break;
		case (587+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r3&0xff);
				cksmspi1+=sav_addr1_xy4z3r3;
				sav_fm_stat = 588+615;
			}
			break;
		case (588+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r4>>24)&0xff);
				sav_fm_stat = 589+615;
			}
			break;
		case (589+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r4>>16)&0xff);
				sav_fm_stat = 590+615;
			}
			break;
		case (590+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r4>>8)&0xff);
				sav_fm_stat = 591+615;
			}
			break;
		case (591+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r4&0xff);
				cksmspi1+=sav_addr1_xy4z3r4;
				sav_fm_stat = 592+615;
			}
			break;
		case (592+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r5>>24)&0xff);
				sav_fm_stat = 593+615;
			}
			break;
		case (593+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r5>>16)&0xff);
				sav_fm_stat = 594+615;
			}
			break;
		case (594+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z3r5>>8)&0xff);
				sav_fm_stat = 595+615;
			}
			break;
		case (595+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z3r5&0xff);
				cksmspi1+=sav_addr1_xy4z3r5;
				sav_fm_stat = 596+615;
			}
			break;
		case (596+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r0>>24)&0xff);
				sav_fm_stat = 597+615;
			}
			break;
		case (597+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r0>>16)&0xff);
				sav_fm_stat = 598+615;
			}
			break;
		case (598+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r0>>8)&0xff);
				sav_fm_stat = 599+615;
			}
			break;
		case (599+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r0&0xff);
				cksmspi1+=sav_addr1_xy4z4r0;
				sav_fm_stat = 600+615;
			}
			break;
		case (600+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r1>>24)&0xff);
				sav_fm_stat = 601+615;
			}
			break;
		case (601+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r1>>16)&0xff);
				sav_fm_stat = 602+615;
			}
			break;
		case (602+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r1>>8)&0xff);
				sav_fm_stat = 603+615;
			}
			break;
		case (603+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r1&0xff);
				cksmspi1+=sav_addr1_xy4z4r1;
				sav_fm_stat = 604+615;
			}
			break;
		case (604+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r2>>24)&0xff);
				sav_fm_stat = 605+615;
			}
			break;
		case (605+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r2>>16)&0xff);
				sav_fm_stat = 606+615;
			}
			break;
		case (606+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r2>>8)&0xff);
				sav_fm_stat = 607+615;
			}
			break;
		case (607+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r2&0xff);
				cksmspi1+=sav_addr1_xy4z4r2;
				sav_fm_stat = 608+615;
			}
			break;
		case (608+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r3>>24)&0xff);
				sav_fm_stat = 609+615;
			}
			break;
		case (609+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r3>>16)&0xff);
				sav_fm_stat = 610+615;
			}
			break;
		case (610+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r3>>8)&0xff);
				sav_fm_stat = 611+615;
			}
			break;
		case (611+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r3&0xff);
				cksmspi1+=sav_addr1_xy4z4r3;
				sav_fm_stat = 612+615;
			}
			break;
		case (612+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r4>>24)&0xff);
				sav_fm_stat = 613+615;
			}
			break;
		case (613+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r4>>16)&0xff);
				sav_fm_stat = 614+615;
			}
			break;
		case (614+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r4>>8)&0xff);
				sav_fm_stat = 615+615;
			}
			break;
		case (615+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r4&0xff);
				cksmspi1+=sav_addr1_xy4z4r4;
				sav_fm_stat = 616+615;
			}
			break;
		case (616+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r5>>24)&0xff);
				sav_fm_stat = 617+615;
			}
			break;
		case (617+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r5>>16)&0xff);
				sav_fm_stat = 618+615;
			}
			break;
		case (618+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((sav_addr1_xy4z4r5>>8)&0xff);
				sav_fm_stat = 619+615;
			}
			break;
		case (619+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(sav_addr1_xy4z4r5&0xff);
				cksmspi1+=sav_addr1_xy4z4r5;
				sav_fm_stat = 620+615;
			}
			break;
		case (620+615):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((cksmspi1>>24)&0xff);
				sav_fm_stat = 1236;
			}
			break;
		case (1236):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((cksmspi1>>16)&0xff);
				sav_fm_stat = 1237;
			}
			break;
		case (1237):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte((cksmspi1>>8)&0xff);
				sav_fm_stat = 1238;
			}
			break;
		case (1238):
			if((spiREG1->FLG & 0x00000100U) == 0x00000100U)  /* Wait */
			{
				tmp= (uint8)spiREG1->BUF;
				spi1sendByte(cksmspi1&0xff);
				cksmspi1=0;
				sav_fm_stat = 1239;
			}
			break;			
			
		case (1239):
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
void ReadStatus(int i)
{
	uint16 ReadStatusData[2] = { 0x05, 0x00};
	uint16 ReceiveStatusData[2] = { 0 };
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
	spiTransmitAndReceiveData(spiREG2, &dataconfig1_t, 2, ReadStatusData, ReceiveStatusData);
	MyGioSetPortA(0xff);
	gioSetPort(gioPORTB,0xff);
//    		gioSetBit(gioPORTA,2,1);
//        sciDisableNotification(scilinREG,SCI_RX_INT);
//        MysciSendByte(ReceiveStatusData[0]);
//        MysciSendByte(ReceiveStatusData[1]);
//	      gioSetBit(gioPORTA,2,0);
//	      sciEnableNotification(scilinREG,SCI_RX_INT);
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
	sav_addr1_xy0z0r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z0r0;
	cksmrecover+=sav_addr1_xy0z0r0;    //100e
	sav_addr1_xy0z0r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z0r1;    //1012
	sav_addr1_xy0z0r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z0r2;    //1016
	sav_addr1_xy0z0r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z0r3;    //101a
	sav_addr1_xy0z0r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z0r4;    //101e
	sav_addr1_xy0z0r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z0r5;    //1022
	sav_addr1_xy0z1r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z1r0;    //1026
	sav_addr1_xy0z1r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z1r1;    //
	sav_addr1_xy0z1r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z1r2;    //
	sav_addr1_xy0z1r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z1r3;
	sav_addr1_xy0z1r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z1r4;
	sav_addr1_xy0z1r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z1r5;
	sav_addr1_xy0z2r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z2r0;    //103e
	sav_addr1_xy0z2r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z2r1;    //
	sav_addr1_xy0z2r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z2r2;    //
	sav_addr1_xy0z2r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z2r3;
	sav_addr1_xy0z2r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z2r4;
	sav_addr1_xy0z2r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z2r5;
	sav_addr1_xy0z3r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z3r0;    //1056
	sav_addr1_xy0z3r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z3r1;    //
	sav_addr1_xy0z3r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z3r2;    //
	sav_addr1_xy0z3r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z3r3;
	sav_addr1_xy0z3r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z3r4;
	sav_addr1_xy0z3r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z3r5;
	sav_addr1_xy0z4r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z4r0;    //106e
	sav_addr1_xy0z4r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z4r1;    //
	sav_addr1_xy0z4r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z4r2;    //
	sav_addr1_xy0z4r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z4r3;
	sav_addr1_xy0z4r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z4r4;
	sav_addr1_xy0z4r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy0z4r5;
	sav_addr1_xy1z0r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z0r0;    //1086
	sav_addr1_xy1z0r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z0r1;    //
	sav_addr1_xy1z0r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z0r2;    //
	sav_addr1_xy1z0r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z0r3;
	sav_addr1_xy1z0r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z0r4;
	sav_addr1_xy1z0r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z0r5;
	sav_addr1_xy1z1r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z1r0;    //109e
	sav_addr1_xy1z1r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z1r1;    //
	sav_addr1_xy1z1r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z1r2;    //
	sav_addr1_xy1z1r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z1r3;
	sav_addr1_xy1z1r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z1r4;
	sav_addr1_xy1z1r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z1r5;
	sav_addr1_xy1z2r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z2r0;    //10b6
	sav_addr1_xy1z2r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z2r1;    //
	sav_addr1_xy1z2r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z2r2;    //
	sav_addr1_xy1z2r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z2r3;
	sav_addr1_xy1z2r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z2r4;
	sav_addr1_xy1z2r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z2r5;
	sav_addr1_xy1z3r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z3r0;    //10ce
	sav_addr1_xy1z3r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z3r1;    //
	sav_addr1_xy1z3r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z3r2;    //
	sav_addr1_xy1z3r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z3r3;
	sav_addr1_xy1z3r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z3r4;
	sav_addr1_xy1z3r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z3r5;
	sav_addr1_xy1z4r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z4r0;    //10e6
	sav_addr1_xy1z4r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z4r1;    //
	sav_addr1_xy1z4r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z4r2;    //
	sav_addr1_xy1z4r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z4r3;
	sav_addr1_xy1z4r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z4r4;
	sav_addr1_xy1z4r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy1z4r5;
	sav_addr1_xy2z0r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z0r0;    //10fe
	sav_addr1_xy2z0r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z0r1;    //
	sav_addr1_xy2z0r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z0r2;    //
	sav_addr1_xy2z0r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z0r3;
	sav_addr1_xy2z0r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z0r4;
	sav_addr1_xy2z0r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z0r5;
	sav_addr1_xy2z1r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z1r0;    //1116
	sav_addr1_xy2z1r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z1r1;    //
	sav_addr1_xy2z1r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z1r2;    //
	sav_addr1_xy2z1r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z1r3;
	sav_addr1_xy2z1r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z1r4;
	sav_addr1_xy2z1r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z1r5;
	sav_addr1_xy2z2r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z2r0;    //112e
	sav_addr1_xy2z2r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z2r1;    //
	sav_addr1_xy2z2r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z2r2;    //
	sav_addr1_xy2z2r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z2r3;
	sav_addr1_xy2z2r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z2r4;
	sav_addr1_xy2z2r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z2r5;
	sav_addr1_xy2z3r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z3r0;    //1146
	sav_addr1_xy2z3r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z3r1;    //
	sav_addr1_xy2z3r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z3r2;    //
	sav_addr1_xy2z3r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z3r3;
	sav_addr1_xy2z3r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z3r4;
	sav_addr1_xy2z3r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z3r5;
	sav_addr1_xy2z4r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z4r0;    //115e
	sav_addr1_xy2z4r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z4r1;    //
	sav_addr1_xy2z4r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z4r2;    //
	sav_addr1_xy2z4r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z4r3;
	sav_addr1_xy2z4r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z4r4;
	sav_addr1_xy2z4r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy2z4r5;
	sav_addr1_xy3z0r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z0r0;    //1176
	sav_addr1_xy3z0r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z0r1;    //
	sav_addr1_xy3z0r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z0r2;    //
	sav_addr1_xy3z0r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z0r3;
	sav_addr1_xy3z0r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z0r4;
	sav_addr1_xy3z0r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z0r5;
	sav_addr1_xy3z1r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z1r0;    //118e
	sav_addr1_xy3z1r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z1r1;    //
	sav_addr1_xy3z1r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z1r2;    //
	sav_addr1_xy3z1r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z1r3;
	sav_addr1_xy3z1r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z1r4;
	sav_addr1_xy3z1r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z1r5;
	sav_addr1_xy3z2r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z2r0;    //11a6
	sav_addr1_xy3z2r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z2r1;    //
	sav_addr1_xy3z2r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z2r2;    //
	sav_addr1_xy3z2r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z2r3;
	sav_addr1_xy3z2r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z2r4;
	sav_addr1_xy3z2r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z2r5;
	sav_addr1_xy3z3r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z3r0;    //11be
	sav_addr1_xy3z3r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z3r1;    //
	sav_addr1_xy3z3r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z3r2;    //
	sav_addr1_xy3z3r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z3r3;
	sav_addr1_xy3z3r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z3r4;
	sav_addr1_xy3z3r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z3r5;
	sav_addr1_xy3z4r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z4r0;    //11d6
	sav_addr1_xy3z4r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z4r1;    //
	sav_addr1_xy3z4r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z4r2;    //
	sav_addr1_xy3z4r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z4r3;
	sav_addr1_xy3z4r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z4r4;
	sav_addr1_xy3z4r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy3z4r5;
	sav_addr1_xy4z0r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z0r0;    //11ee
	sav_addr1_xy4z0r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z0r1;    //
	sav_addr1_xy4z0r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z0r2;    //
	sav_addr1_xy4z0r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z0r3;
	sav_addr1_xy4z0r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z0r4;
	sav_addr1_xy4z0r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z0r5;
	sav_addr1_xy4z1r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z1r0;    //1206
	sav_addr1_xy4z1r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z1r1;    //
	sav_addr1_xy4z1r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z1r2;    //
	sav_addr1_xy4z1r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z1r3;
	sav_addr1_xy4z1r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z1r4;
	sav_addr1_xy4z1r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z1r5;
	sav_addr1_xy4z2r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z2r0;    //121e
	sav_addr1_xy4z2r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z2r1;    //
	sav_addr1_xy4z2r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z2r2;    //
	sav_addr1_xy4z2r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z2r3;
	sav_addr1_xy4z2r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z2r4;
	sav_addr1_xy4z2r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z2r5;
	sav_addr1_xy4z3r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z3r0;    //1236
	sav_addr1_xy4z3r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z3r1;    //
	sav_addr1_xy4z3r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z3r2;    //
	sav_addr1_xy4z3r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z3r3;
	sav_addr1_xy4z3r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z3r4;
	sav_addr1_xy4z3r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z3r5;
	sav_addr1_xy4z4r0=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z4r0;    //124e
	sav_addr1_xy4z4r1=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z4r1;    //
	sav_addr1_xy4z4r2=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z4r2;    //
	sav_addr1_xy4z4r3=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z4r3;
	sav_addr1_xy4z4r4=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z4r4;
	sav_addr1_xy4z4r5=spiReadAddr();
	cksmrecover+=sav_addr1_xy4z4r5;
	rcvspicksm=spiReadAddr();

	if(rcvspicksm==cksmrecover)
	{
		cksmrecover=0;
		rcvspicksm=0;	       
		spi1transmitByte(0x00);        //1267
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
		cksmrecover+=(minute<<8)|(second);;
		sav_addr0=spiReadAddr();
		cksmrecover+=sav_addr0;
		sav_addr1_xy0z0r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z0r0;
		sav_addr1_xy0z0r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z0r1;
		sav_addr1_xy0z0r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z0r2;
		sav_addr1_xy0z0r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z0r3;
		sav_addr1_xy0z0r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z0r4;
		sav_addr1_xy0z0r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z0r5;
		sav_addr1_xy0z1r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z1r0;
		sav_addr1_xy0z1r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z1r1;
		sav_addr1_xy0z1r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z1r2;
		sav_addr1_xy0z1r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z1r3;
		sav_addr1_xy0z1r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z1r4;
		sav_addr1_xy0z1r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z1r5;
		sav_addr1_xy0z2r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z2r0;
		sav_addr1_xy0z2r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z2r1;
		sav_addr1_xy0z2r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z2r2;
		sav_addr1_xy0z2r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z2r3;
		sav_addr1_xy0z2r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z2r4;
		sav_addr1_xy0z2r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z2r5;
		sav_addr1_xy0z3r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z3r0;
		sav_addr1_xy0z3r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z3r1;
		sav_addr1_xy0z3r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z3r2;
		sav_addr1_xy0z3r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z3r3;
		sav_addr1_xy0z3r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z3r4;
		sav_addr1_xy0z3r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z3r5;
		sav_addr1_xy0z4r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z4r0;
		sav_addr1_xy0z4r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z4r1;
		sav_addr1_xy0z4r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z4r2;
		sav_addr1_xy0z4r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z4r3;
		sav_addr1_xy0z4r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z4r4;
		sav_addr1_xy0z4r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy0z4r5;
		sav_addr1_xy1z0r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z0r0;    //
		sav_addr1_xy1z0r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z0r1;    //
		sav_addr1_xy1z0r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z0r2;    //
		sav_addr1_xy1z0r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z0r3;
		sav_addr1_xy1z0r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z0r4;
		sav_addr1_xy1z0r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z0r5;
		sav_addr1_xy1z1r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z1r0;    //
		sav_addr1_xy1z1r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z1r1;    //
		sav_addr1_xy1z1r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z1r2;    //
		sav_addr1_xy1z1r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z1r3;
		sav_addr1_xy1z1r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z1r4;
		sav_addr1_xy1z1r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z1r5;
		sav_addr1_xy1z2r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z2r0;    //
		sav_addr1_xy1z2r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z2r1;    //
		sav_addr1_xy1z2r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z2r2;    //
		sav_addr1_xy1z2r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z2r3;
		sav_addr1_xy1z2r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z2r4;
		sav_addr1_xy1z2r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z2r5;
		sav_addr1_xy1z3r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z3r0;    //
		sav_addr1_xy1z3r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z3r1;    //
		sav_addr1_xy1z3r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z3r2;    //
		sav_addr1_xy1z3r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z3r3;
		sav_addr1_xy1z3r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z3r4;
		sav_addr1_xy1z3r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z3r5;
		sav_addr1_xy1z4r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z4r0;    //
		sav_addr1_xy1z4r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z4r1;    //
		sav_addr1_xy1z4r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z4r2;    //
		sav_addr1_xy1z4r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z4r3;
		sav_addr1_xy1z4r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z4r4;
		sav_addr1_xy1z4r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy1z4r5;
		sav_addr1_xy2z0r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z0r0;    //
		sav_addr1_xy2z0r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z0r1;    //
		sav_addr1_xy2z0r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z0r2;    //
		sav_addr1_xy2z0r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z0r3;
		sav_addr1_xy2z0r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z0r4;
		sav_addr1_xy2z0r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z0r5;
		sav_addr1_xy2z1r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z1r0;    //
		sav_addr1_xy2z1r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z1r1;    //
		sav_addr1_xy2z1r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z1r2;    //
		sav_addr1_xy2z1r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z1r3;
		sav_addr1_xy2z1r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z1r4;
		sav_addr1_xy2z1r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z1r5;
		sav_addr1_xy2z2r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z2r0;    //
		sav_addr1_xy2z2r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z2r1;    //
		sav_addr1_xy2z2r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z2r2;    //
		sav_addr1_xy2z2r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z2r3;
		sav_addr1_xy2z2r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z2r4;
		sav_addr1_xy2z2r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z2r5;
		sav_addr1_xy2z3r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z3r0;    //
		sav_addr1_xy2z3r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z3r1;    //
		sav_addr1_xy2z3r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z3r2;    //
		sav_addr1_xy2z3r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z3r3;
		sav_addr1_xy2z3r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z3r4;
		sav_addr1_xy2z3r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z3r5;
		sav_addr1_xy2z4r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z4r0;    //
		sav_addr1_xy2z4r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z4r1;    //
		sav_addr1_xy2z4r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z4r2;    //
		sav_addr1_xy2z4r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z4r3;
		sav_addr1_xy2z4r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z4r4;
		sav_addr1_xy2z4r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy2z4r5;
		sav_addr1_xy3z0r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z0r0;    //
		sav_addr1_xy3z0r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z0r1;    //
		sav_addr1_xy3z0r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z0r2;    //
		sav_addr1_xy3z0r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z0r3;
		sav_addr1_xy3z0r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z0r4;
		sav_addr1_xy3z0r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z0r5;
		sav_addr1_xy3z1r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z1r0;    //
		sav_addr1_xy3z1r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z1r1;    //
		sav_addr1_xy3z1r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z1r2;    //
		sav_addr1_xy3z1r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z1r3;
		sav_addr1_xy3z1r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z1r4;
		sav_addr1_xy3z1r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z1r5;
		sav_addr1_xy3z2r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z2r0;    //
		sav_addr1_xy3z2r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z2r1;    //
		sav_addr1_xy3z2r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z2r2;    //
		sav_addr1_xy3z2r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z2r3;
		sav_addr1_xy3z2r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z2r4;
		sav_addr1_xy3z2r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z2r5;
		sav_addr1_xy3z3r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z3r0;    //
		sav_addr1_xy3z3r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z3r1;    //
		sav_addr1_xy3z3r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z3r2;    //
		sav_addr1_xy3z3r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z3r3;
		sav_addr1_xy3z3r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z3r4;
		sav_addr1_xy3z3r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z3r5;
		sav_addr1_xy3z4r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z4r0;    //
		sav_addr1_xy3z4r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z4r1;    //
		sav_addr1_xy3z4r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z4r2;    //
		sav_addr1_xy3z4r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z4r3;
		sav_addr1_xy3z4r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z4r4;
		sav_addr1_xy3z4r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy3z4r5;
		sav_addr1_xy4z0r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z0r0;    //
		sav_addr1_xy4z0r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z0r1;    //
		sav_addr1_xy4z0r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z0r2;    //
		sav_addr1_xy4z0r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z0r3;
		sav_addr1_xy4z0r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z0r4;
		sav_addr1_xy4z0r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z0r5;
		sav_addr1_xy4z1r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z1r0;    //
		sav_addr1_xy4z1r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z1r1;    //
		sav_addr1_xy4z1r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z1r2;    //
		sav_addr1_xy4z1r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z1r3;
		sav_addr1_xy4z1r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z1r4;
		sav_addr1_xy4z1r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z1r5;
		sav_addr1_xy4z2r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z2r0;    //
		sav_addr1_xy4z2r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z2r1;    //
		sav_addr1_xy4z2r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z2r2;    //
		sav_addr1_xy4z2r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z2r3;
		sav_addr1_xy4z2r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z2r4;
		sav_addr1_xy4z2r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z2r5;
		sav_addr1_xy4z3r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z3r0;    //
		sav_addr1_xy4z3r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z3r1;    //
		sav_addr1_xy4z3r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z3r2;    //
		sav_addr1_xy4z3r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z3r3;
		sav_addr1_xy4z3r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z3r4;
		sav_addr1_xy4z3r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z3r5;
		sav_addr1_xy4z4r0=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z4r0;    //
		sav_addr1_xy4z4r1=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z4r1;    //
		sav_addr1_xy4z4r2=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z4r2;    //
		sav_addr1_xy4z4r3=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z4r3;
		sav_addr1_xy4z4r4=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z4r4;
		sav_addr1_xy4z4r5=spiReadAddr();
		cksmrecover+=sav_addr1_xy4z4r5;
		rcvspicksm=spiReadAddr();
	}
	StartAdd(0x14d4);
	ar=spiReadAddr();					//addr 0x14d4
	br=spiReadAddr();				 	//
	cr=spiReadAddr();           	
	dr=spiReadAddr();				   	
	xy1=spiReadAddr();				  	
	xy2=spiReadAddr();				  	
	xy3=spiReadAddr();				  	
	xy4=spiReadAddr();				  	
	z1=spiReadAddr();				    
	z2=spiReadAddr();				    
	z3=spiReadAddr();				    
	z4=spiReadAddr();				    
	gioSetBit(gioPORTA,0,1);
	wait(0x1000000);  //time for the initialization of i2c
	/*
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
	*/
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
	if ( iTick == 8 ) /* filter to alter the sample frequency to 1024Hz */
	{
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
		ia2k += 1;
		avg_sec_x_float += accx_send;
		avg_sec_y_float += accy_send;
		avg_sec_z_float += accz_send;
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
	if ( ia2k == 10240 )
	{
		avg_sec_cur_x	= avg_sec_x_float / 10240; /* normalization */
		avg_sec_cur_y	= avg_sec_y_float / 10240;
		avg_sec_cur_z	= avg_sec_z_float / 10240;
		avg_sec_cur_r	= avg_sec_r_float / 10240;
		avg_sec_cur_r_float = avg_sec_r_float /10240;
		peak_sec_cur_r_float	= peak_sec_r_float;
		bottom_sec_cur_r_float	= bottom_sec_r_float;
		peak_sec_cur_r		= (int)peak_sec_r_float;
		bottom_sec_cur_r	= (int)bottom_sec_r_float;
		avg_sec_x_float		= 0;
		avg_sec_y_float		= 0;
		avg_sec_z_float		= 0;
		avg_sec_r_float		= 0;
		peak_sec_r_float	= 0;
		bottom_sec_r_float	= 10000;
		ia2k	= 0;
		delay++;
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
	
	


	
	
	
	
	sav_addr1_xy0z0r0=0x00000000;
	sav_addr1_xy0z0r1=0x1A5500;
	sav_addr1_xy0z0r2=0x34AA00;
	sav_addr1_xy0z0r3=0x4EFF00;
	sav_addr1_xy0z0r4=0x695400;
	sav_addr1_xy0z0r5=0x83A900;
	sav_addr1_xy0z1r0=0x9DFE00;
	sav_addr1_xy0z1r1=0xB85300;
	sav_addr1_xy0z1r2=0xD2A800;
	sav_addr1_xy0z1r3=0xECFD00;
	sav_addr1_xy0z1r4=0x1075200;
	sav_addr1_xy0z1r5=0x121A700;
	sav_addr1_xy0z2r0=0x13BFC00;
	sav_addr1_xy0z2r1=0x1565100;
	sav_addr1_xy0z2r2=0x170A600;
	sav_addr1_xy0z2r3=0x18AFB00;
	sav_addr1_xy0z2r4=0x1A55000;
	sav_addr1_xy0z2r5=0x1BFA500;
	sav_addr1_xy0z3r0=0x1D9FA00;
	sav_addr1_xy0z3r1=0x1F44F00;
	sav_addr1_xy0z3r2=0x20EA400;
	sav_addr1_xy0z3r3=0x228F900;
	sav_addr1_xy0z3r4=0x2434E00;
	sav_addr1_xy0z3r5=0x25DA300;
	sav_addr1_xy0z4r0=0x277F800;
	sav_addr1_xy0z4r1=0x2924D00;
	sav_addr1_xy0z4r2=0x2ACA200;
	sav_addr1_xy0z4r3=0x2C6F700;
	sav_addr1_xy0z4r4=0x2E14C00;
	sav_addr1_xy0z4r5=0x2FBA100;
	sav_addr1_xy1z0r0=0x315F600;
	sav_addr1_xy1z0r1=0x3304B00;
	sav_addr1_xy1z0r2=0x34AA000;
	sav_addr1_xy1z0r3=0x364F500;
	sav_addr1_xy1z0r4=0x37F4A00;
	sav_addr1_xy1z0r5=0x3999F00;
	sav_addr1_xy1z1r0=0x3B3F400;
	sav_addr1_xy1z1r1=0x3CE4900;
	sav_addr1_xy1z1r2=0x3E89E00;
	sav_addr1_xy1z1r3=0x402F300;
	sav_addr1_xy1z1r4=0x41D4800;
	sav_addr1_xy1z1r5=0x4379D00;
	sav_addr1_xy1z2r0=0x451F200;
	sav_addr1_xy1z2r1=0x46C4700;
	sav_addr1_xy1z2r2=0x4869C00;
	sav_addr1_xy1z2r3=0x4A0F100;
	sav_addr1_xy1z2r4=0x4BB4600;
	sav_addr1_xy1z2r5=0x4D59B00;
	sav_addr1_xy1z3r0=0x4EFF000;
	sav_addr1_xy1z3r1=0x50A4500;
	sav_addr1_xy1z3r2=0x5249A00;
	sav_addr1_xy1z3r3=0x53EEF00;
	sav_addr1_xy1z3r4=0x5594400;
	sav_addr1_xy1z3r5=0x5739900;
	sav_addr1_xy1z4r0=0x58DEE00;
	sav_addr1_xy1z4r1=0x5A84300;
	sav_addr1_xy1z4r2=0x5C29800;
	sav_addr1_xy1z4r3=0x5DCED00;
	sav_addr1_xy1z4r4=0x5F74200;
	sav_addr1_xy1z4r5=0x6119700;
	sav_addr1_xy2z0r0=0x62BEC00;
	sav_addr1_xy2z0r1=0x6464100;
	sav_addr1_xy2z0r2=0x6609600;
	sav_addr1_xy2z0r3=0x67AEB00;
	sav_addr1_xy2z0r4=0x6954000;
	sav_addr1_xy2z0r5=0x6AF9500;
	sav_addr1_xy2z1r0=0x6C9EA00;
	sav_addr1_xy2z1r1=0x6E43F00;
	sav_addr1_xy2z1r2=0x6FE9400;
	sav_addr1_xy2z1r3=0x718E900;
	sav_addr1_xy2z1r4=0x7333E00;
	sav_addr1_xy2z1r5=0x74D9300;
	sav_addr1_xy2z2r0=0x767E800;
	sav_addr1_xy2z2r1=0x7823D00;
	sav_addr1_xy2z2r2=0x79C9200;
	sav_addr1_xy2z2r3=0x7B6E700;
	sav_addr1_xy2z2r4=0x7D13C00;
	sav_addr1_xy2z2r5=0x7EB9100;
	sav_addr1_xy2z3r0=0x805E600;
	sav_addr1_xy2z3r1=0x8203B00;
	sav_addr1_xy2z3r2=0x83A9000;
	sav_addr1_xy2z3r3=0x854E500;
	sav_addr1_xy2z3r4=0x86F3A00;
	sav_addr1_xy2z3r5=0x8898F00;
	sav_addr1_xy2z4r0=0x8A3E400;
	sav_addr1_xy2z4r1=0x8BE3900;
	sav_addr1_xy2z4r2=0x8D88E00;
	sav_addr1_xy2z4r3=0x8F2E300;
	sav_addr1_xy2z4r4=0x90D3800;
	sav_addr1_xy2z4r5=0x9278D00;
	sav_addr1_xy3z0r0=0x941E200;
	sav_addr1_xy3z0r1=0x95C3700;
	sav_addr1_xy3z0r2=0x9768C00;
	sav_addr1_xy3z0r3=0x990E100;
	sav_addr1_xy3z0r4=0x9AB3600;
	sav_addr1_xy3z0r5=0x9C58B00;
	sav_addr1_xy3z1r0=0x9DFE000;
	sav_addr1_xy3z1r1=0x9FA3500;
	sav_addr1_xy3z1r2=0xA148A00;
	sav_addr1_xy3z1r3=0xA2EDF00;
	sav_addr1_xy3z1r4=0xA493400;
	sav_addr1_xy3z1r5=0xA638900;
	sav_addr1_xy3z2r0=0xA7DDE00;
	sav_addr1_xy3z2r1=0xA983300;
	sav_addr1_xy3z2r2=0xAB28800;
	sav_addr1_xy3z2r3=0xACCDD00;
	sav_addr1_xy3z2r4=0xAE73200;
	sav_addr1_xy3z2r5=0xB018700;
	sav_addr1_xy3z3r0=0xB1BDC00;
	sav_addr1_xy3z3r1=0xB363100;
	sav_addr1_xy3z3r2=0xB508600;
	sav_addr1_xy3z3r3=0xB6ADB00;
	sav_addr1_xy3z3r4=0xB853000;
	sav_addr1_xy3z3r5=0xB9F8500;
	sav_addr1_xy3z4r0=0xBB9DA00;
	sav_addr1_xy3z4r1=0xBD42F00;
	sav_addr1_xy3z4r2=0xBEE8400;
	sav_addr1_xy3z4r3=0xC08D900;
	sav_addr1_xy3z4r4=0xC232E00;
	sav_addr1_xy3z4r5=0xC3D8300;
	sav_addr1_xy4z0r0=0xC57D800;
	sav_addr1_xy4z0r1=0xC722D00;
	sav_addr1_xy4z0r2=0xC8C8200;
	sav_addr1_xy4z0r3=0xCA6D700;
	sav_addr1_xy4z0r4=0xCC12C00;
	sav_addr1_xy4z0r5=0xCDB8100;
	sav_addr1_xy4z1r0=0xCF5D600;
	sav_addr1_xy4z1r1=0xD102B00;
	sav_addr1_xy4z1r2=0xD2A8000;
	sav_addr1_xy4z1r3=0xD44D500;
	sav_addr1_xy4z1r4=0xD5F2A00;
	sav_addr1_xy4z1r5=0xD797F00;
	sav_addr1_xy4z2r0=0xD93D400;
	sav_addr1_xy4z2r1=0xDAE2900;
	sav_addr1_xy4z2r2=0xDC87E00;
	sav_addr1_xy4z2r3=0xDE2D300;
	sav_addr1_xy4z2r4=0xDFD2800;
	sav_addr1_xy4z2r5=0xE177D00;
	sav_addr1_xy4z3r0=0xE31D200;
	sav_addr1_xy4z3r1=0xE4C2700;
	sav_addr1_xy4z3r2=0xE667C00;
	sav_addr1_xy4z3r3=0xE80D100;
	sav_addr1_xy4z3r4=0xE9B2600;
	sav_addr1_xy4z3r5=0xEB57B00;
	sav_addr1_xy4z4r0=0xECFD000;
	sav_addr1_xy4z4r1=0xEEA2500;
	sav_addr1_xy4z4r2=0xF047A00;
	sav_addr1_xy4z4r3=0xF1ECF00;
	sav_addr1_xy4z4r4=0xF392400;
	sav_addr1_xy4z4r5=0xF537900;
	sav_addr0 = 0xf6dce00;

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

/*
void CacuClockTime(uint32 TimeSec)
{
	int YearDelay = 0;
	int MonthDelay = 0;
	int DayDelay = 24 * 60* 60;
	int HourDelay = 60* 60;
	int MiniteDelay = 60;

	year=0;
	month=1;
	day=1;
	hour = 0;
	minute = 0;
	second = 0;
	realyear=1970;
	realmonth = 1;
	realday = 1;
	realhour = 0;
	realminute = 0;
	realsecond = 0;
	while(1)
	{
		if ((realyear % 4 == 0 && realyear % 100 != 0 )||realyear % 400 == 0)  //leap year
				YearDelay = 366*24*60*60;
		else
				YearDelay = 365*24*60*60;
		if (TimeSec >= YearDelay)                        
		{
				realyear++;
				TimeSec -= YearDelay;
		}
		else
		{
			if (realmonth == 1 || realmonth == 3 || realmonth == 5 || realmonth == 7 || realmonth == 8 || realmonth == 10 || realmonth == 12)	       
					MonthDelay = 31*24*60*60;	       
			else if (realmonth == 2)
			{
					if ((realyear % 4 == 0 && realyear % 100 != 0 )||(realyear % 400 == 0) ) //leap year	           
							MonthDelay = 29*24*60*60;	           
					else	         
							MonthDelay = 28*24*60*60;	          
			}
			else	       
					MonthDelay = 30*24*60*60;    
			if(TimeSec>=MonthDelay)
			{
					realmonth++;
					TimeSec -= MonthDelay;
			}	
			else
			{
					if(TimeSec>=DayDelay)
					{
							realday++;
							TimeSec-=DayDelay;
					}
					else
					{
							if(TimeSec>=HourDelay)
							{
									realhour++;
									TimeSec-=HourDelay;
							}
							else
							{
									if(TimeSec >= MiniteDelay)
									{
											realminute++;
											TimeSec -= MiniteDelay;
									}
									else
									{
											realsecond = TimeSec;
											break;
									}
							}
						}

				}
			}
		}		

	second = TimeSec;
	//realsecond=TimeSec;
	second=second/10*16+second%10;         // binary to decimal
	minute = realminute;
	minute = minute/10*16+minute%10;
	hour = realhour;                               //2019.6.16ȥ��+8Сʱ
	hour=hour/10*16+hour%10;
	
	day = realday;
	
	day=day/10*16+day%10;
	month = realmonth;
	month = month/10*16+month%10;
	year = realyear - 1970;
	year=year/10*16+year%10;
}
*/
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
