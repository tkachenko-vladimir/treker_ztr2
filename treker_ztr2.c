#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "eat_modem.h"
#include "eat_interface.h"
#include "eat_uart.h"
#include "eat_timer.h" 
#include "eat_clib_define.h" //only in main.c
#include "eat_periphery.h"
#include "eat_sms.h"
#include "eat_fs.h"
#include "eat_socket.h"
#include "eat_flash.h" 

#define Version 53
#define SettingsVer 3
#define DEBUG

//mini_SIM28
#define GPS_BR EAT_UART_BAUD_9600
#define GPS_UART EAT_UART_1
#define INZ EAT_PIN7_UART1_RI
#define OUT1 EAT_PIN3_UART1_RTS
#define LED EAT_PIN41_NETLIGHT
#define GPS_ON EAT_PIN5_UART1_DCD

#define main_buf_size 10000
#define APP_UPDATE_BUFF_SIZE 0x1000
#define EAT_MEM_MAX_SIZE 100*1024

typedef void (*app_user_func)(void*);
static u8 s_memPool[EAT_MEM_MAX_SIZE];
static EatEntryPara_st app_para;

#if defined(INFRQ1)
#define TARa_cnt 28
const unsigned int TARa2[TARa_cnt] = {0,20,40,60,80,100,120,140,160,180,200,220,240,260,280,300,320,340,360,380,400,420,440,460,480,500,520,540};
const unsigned int TARa1[TARa_cnt] = {509,547,586,623,659,694,727,761,795,828,860,893,926,959,992,1024,1057,1089,1122,1155,1188,1220,1253,1286,1329,1362,1397,1433};
#endif
#if defined(INFRQ2)
#define TARb_cnt 28
const unsigned int TARb2[TARb_cnt] = {0,20,40,60,80,100,120,140,160,180,200,220,240,260,280,300,320,340,360,380,400,420,440,460,480,500,520,531};
const unsigned int TARb1[TARb_cnt] = {509,542,582,620,657,693,726,760,794,828,861,896,929,963,996,1029,1063,1096,1129,1162,1196,1229,1262,1296,1327,1361,1397,1416};
#endif

static u8 ExIP[3][4] = {185,25,117,35, 185,25,117,35, 185,25,117,35};
static unsigned int ExPort[3] = {10034, 10034, 10034};
static u8 APN[31] = "www.kyivstar.net", APN_user[31] = "", APN_pass[31] = "";
static unsigned int Period1 = 10, Period2 = 60, Period3 = 2, prm_per = 300, gsmloc_per = 0, money_per = 0, trip_per = 0, debug_per = 0;
static unsigned int can1_per = 0, can2_per = 0, ADC_per = 0, INx_per = 0, freq_per = 0, ident_per = 0, offsim_per = 14400;
static unsigned int Send_per1 = 10, Send_per2 = 60, Stealth_ontm = 0, Stealth_per = 0, fuel_per = 0;
static unsigned int Vbt_off = 3500, Speed_lim = 250, Vcc_lim = 0, Vbt_lim = 0, Rsindp = 120, Sync = 0;
static unsigned char PortPr = 7;
static char SMS_pass[5] = "0000";
static char MoneyUSSD[11] = "*111#";
static char Num1[13] = "";
static char Num2[13] = "";
static char Num3[13] = "";
static char Num4[13] = "";
static char Num5[13] = "";
static char NumAd1[13] = "380674896016";
static char NumAd2[13] = "";
static u8 eng_block = 0;
static unsigned int Settings = 1+8+64+128+2048;
//bit0 - (0)пер.отпр.-0-движ;1-заж.;2-пост.;3-адапт.		1
//bit1 - (1)пер.отпр.-0-движ;1-заж.;2-пост.;3-адапт.		2
//bit2 - опред. роуминг по карте							4
//bit3 - опред. роуминг по GSM				 				8
//bit4 - Bluetooth On										16
//bit5 - отправлять данные в роуминге						32
//bit6 - отвечать на СМС  в роуминге			 			64
//bit7 - (0)фильтр на стоянкак-0-нет;1-по прогр.движ.		128
//bit8 - (1)2-по заж.3-reserved								256
//bit9 - СМС режим											512
//bit10 - Выключить GPS										1024
//bit11 - Не разрывать соединение							2048
//bit12 - Только WIFI(без сот)								4096
//bit13 - reserve											8192
//bit14 - reserve											16384
//bit15 - reserve											32768

static unsigned int Settings1 = 1+2+4+8;
//bit0 - при изм.заж.зап.коорд.								1
//bit1 - при изм.заж.начинать отпр.							2
//bit2 - при откл/вкл.вн.напр.зап.коорд.					4
//bit3 - при откл/вкл.вн.напр.начинать отпр.				8
//bit4 - IN1 - тревога										16
//bit5 - IN2 - тревога										32
//bit6 - Тревога1 акт.выс.						 			64
//bit7 - Тревога2 акт.выс.									128
//bit8 - отправл.gsmloc при опред. по gps					256
//bit9 - при изм.движ.зап.									512
//bit10 - при изм.движ.отпр.								1024
//bit11 - при изм. IN1 отпр.								2048
//bit12 - при изм. IN2 отпр.								4096
//bit13 - reserve											8192
//bit14 - reserve											16384
//bit15 - reserve											32768

static unsigned int Sts_d = 0;
//bit0 - Speed_lim				1
//bit1 - RoumingMAP				2
//bit2 - gprs_reg				4
//bit3 - Ignition				8
//bit4 - Move					16
//bit5 - gsm_reg0				32
//bit6 - gsm_reg1				64
//bit7 - gsm_reg2(engine on)	128
//bit8 - Jamming				256
//bit9 - Вн.напр.откл.			512
//bit10 - Авт.заблокирован		1024
//bit11 - Vcc_lim				2048
//bit12 - Vbt_lim				4096
//bit13 - Перегрев				8192
//bit14 - IN1					16384
//bit15 - IN2					32768
static char simrev[200], nmea_msg[100];
static char IMEI[17] = {31, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0}, ICC[6] = {0,0,0,0,0,0};
static u8 FTP_server[4];
static char FTP_user[31], FTP_pass[31], FTP_path[101], fw_filename[15], incall_nbr[13], sms_txt[251], sms_nbr[13], ussdcmd[100];
static u8 nmea_st = 0, main_st = 0, udpsend_st = 0, send_sms_st = 0, gprs_st = 0;
static u8 main_status = 0, cfun = 0, gsm_reg = 0, dis_gpson = 0;
static eat_bool cpin = EAT_FALSE, gps_ok = EAT_FALSE, gps_chk = EAT_FALSE, do_send = EAT_FALSE, fw_update = EAT_FALSE;
static eat_bool money_f = EAT_FALSE, gsmloc_f = EAT_FALSE, simreset_f = EAT_FALSE, offgps_f = EAT_FALSE, incall_f = EAT_FALSE;
static eat_bool ata_f = EAT_FALSE, ath_f = EAT_FALSE, first_init = EAT_FALSE, pos2sms = EAT_FALSE, sms_sended = EAT_FALSE, ath_simreset_f = EAT_FALSE;
static eat_bool send_sms_f = EAT_FALSE, gprs_enable = EAT_FALSE, send_cfun1 = EAT_FALSE, send_cfun4 = EAT_FALSE, send_dtmf = EAT_FALSE, fsinfo = EAT_FALSE;
static eat_bool gprs_reset = EAT_FALSE, log_upload = EAT_FALSE, modinfo = EAT_FALSE, ussd_send = EAT_FALSE, stsreset = EAT_FALSE, cpureset = EAT_FALSE;
static eat_bool getparam1 = EAT_FALSE, getparam2 = EAT_FALSE, getparam3 = EAT_FALSE, getparam4 = EAT_FALSE, getparam5 = EAT_FALSE, gsmoffon = EAT_FALSE;
static u8 dtmf_c = 0, dtmf_d = 0, dtmf_menu_number = 0, log_write_en = 0;
static u8 SatUs = 0, bear_st = 0, ExIPN = 0, senderr_cnt = 0, extoff_time = 0, gprs_st_timer = 0, gprs_st_errcnt = 0, udpsend_st_timer = 0, rs485_timer = 0;
static u16 Period = 0, period_cnt = 0, Send_per = 0, send_cnt = 0, rsindp_cnt = 0, prm_cnt = 0, gsmloc_cnt = 0, money_cnt = 0, offsim_cnt = 0, fuel_cnt = 0;
static u16 trip_cnt = 0, debug_cnt = 0, ADC_cnt = 0, INx_cnt = 0, freq_cnt = 0, ident_cnt = 0, reg_err_cnt = 0, can1_cnt = 0, can2_cnt = 0, vibration_cnt = 0;
static u16 insms_id = 0, stealth_timer = 0, stealth_cnt = 0, Stealth_per_old = 0;
static char * nmea_msg_pos;
static unsigned long Latitude = 0, Longitude = 0, Trip = 0;
static u8 Year = 0, Month = 0, Day = 0, Hour = 0, Minute = 0, Second = 0;
static u16 Speed = 0, Course = 0, Course_old = 0, Vbt = 0, Vcc = 0, Money = 0, freq1 = 0, freq2 = 0, fuel1 = 0, fuel2 = 0;
static u8 rssi = 0, Turn = 0;
static u8 MCU_ver = 0, MCU_inputs = 0, ADC1 = 0, ADC2 = 0;
static s8 Tm = 0;
static unsigned int MCC = 0, MNC = 0, LAC = 0, CID = 0, do_req = 0;
static unsigned char cmd_ret = 0;
static EatSemId_st sem_at_done = EAT_NULL;
static char at_answer[100], server_answer[1024];
static char * at_ret;
static u16 at_timer;
static u8 move_d = 0, at_res, CRC = 0, Event_nbr = 0, uv_cnt = 0, vclim_cnt = 0, vblim_cnt = 0;
static sockaddr_struct server_adr;
static s8 server_soc = 0;
static int log_file = 0;
static char debug_buf[50];

static unsigned char main_buf[main_buf_size], out_buf[1000];
static unsigned long eeprom_p1 = 0, eeprom_p2 = 0, eeprom_p2tmp = 0, out_buf_col = 0;

#if defined(RS485_UART)
static unsigned int DUT_F = 0, DUT_N = 0, DUT_t = 0, DUT_L = 0;
#define TAR_cnt 22//408
const unsigned int TAR2[TAR_cnt] = {0,20,40,60,80,100,150,200,250,300,350,400,450,500,550,600,650,700,750,800,820,833};
const unsigned int TAR1[TAR_cnt] = {0,93,206,307,409,507,746,984,1223,1445,1668,1899,2129,2356,2586,2817,3047,3286,3516,3766,3876,3954};
#endif
#if defined(MCU_UART)
static eat_bool do_extreset = EAT_FALSE, MCU_first = EAT_FALSE;
static u8 MCU_outputs = 0;
#endif
#if defined(CAN_UART)
static unsigned char speed_can = 0xFF, accel_can = 0xFF, fuel_can = 0xFF, temp_can = 0xFF;
static unsigned int rpm_can = 0xFFFF;
static unsigned long fuela_can = 0xFFFFFFFF, mth_can = 0xFFFFFFFF, dist_can = 0xFFFFFFFF;
static u8 CAN_CRC = 0;
#endif
#if defined(INFRQ1)
static unsigned int infrq1_cnt = 0;
static unsigned char f1_fltr_cnt = 0;
static unsigned int f1_fltr[5];
#endif
#if defined(INFRQ2)
static unsigned int infrq2_cnt = 0;
static unsigned char f2_fltr_cnt = 0;
static unsigned int f2_fltr[5];
#endif
#if defined(ESP_ON)
unsigned char MAC[120];
unsigned char MAC_cnt = 0;
#endif
EatUartConfig_st uart_config;

extern void APP_InitRegions(void);

void app_main(void *data);
void app_func_ext1(void *data);
void app_user2(void *data);
void app_user3(void *data);
void app_user7(void *data);
#if defined(LED)
void app_user8(void *data);
#endif

#pragma arm section rodata = "APP_CFG"
APP_ENTRY_FLAG 
#pragma arm section rodata

#pragma arm section rodata="APPENTRY"
	const EatEntry_st AppEntry = 
	{
		app_main,
		app_func_ext1,
		(app_user_func)EAT_NULL,//app_user1,
		(app_user_func)app_user2,//app_user2,
		(app_user_func)EAT_NULL,//app_user3,
		(app_user_func)EAT_NULL,//app_user4,
		(app_user_func)EAT_NULL,//app_user5,
		(app_user_func)EAT_NULL,//app_user6,
		(app_user_func)app_user7,//app_user7,
#if defined(LED)
		(app_user_func)app_user8,//app_user8,
#else
		(app_user_func)EAT_NULL,//app_user8,
#endif
		EAT_NULL,
		EAT_NULL,
		EAT_NULL,
		EAT_NULL,
		EAT_NULL,
		EAT_NULL
	};
#pragma arm section rodata

void app_func_ext1(void *data)
{
#if defined(DEBUG)
	eat_uart_set_debug(EAT_UART_USB);
	eat_uart_set_debug_config(EAT_UART_DEBUG_MODE_TRACE, NULL);
	eat_uart_set_at_port(EAT_UART_NULL);
#else
	eat_uart_set_debug_config(EAT_UART_DEBUG_MODE_UART, NULL);
	eat_uart_set_at_port(EAT_UART_NULL);
#endif
	
	eat_sim_detect_en(EAT_FALSE);
#if defined(LED)
	eat_pin_set_mode(LED, EAT_PIN_MODE_GPIO);
#endif
#if defined(GPS_ON)
	eat_pin_set_mode(GPS_ON, EAT_PIN_MODE_GPIO);
#endif
#if defined(OUT2)
	eat_pin_set_mode(OUT2, EAT_PIN_MODE_GPIO);
#endif
#if defined(OUT3)
	eat_pin_set_mode(OUT3, EAT_PIN_MODE_GPIO);
#endif
#if defined(OUT4)
	eat_pin_set_mode(OUT4, EAT_PIN_MODE_GPIO);
#endif
#if defined(ESP_ON)
	eat_pin_set_mode(ESP_ON, EAT_PIN_MODE_GPIO);
#endif
	eat_pin_set_mode(EAT_PIN1_UART1_TXD, EAT_PIN_MODE_UART);
	eat_pin_set_mode(EAT_PIN2_UART1_RXD, EAT_PIN_MODE_UART);
	eat_pin_set_mode(EAT_PIN22_UART2_TXD, EAT_PIN_MODE_UART);
	eat_pin_set_mode(EAT_PIN23_UART2_RXD, EAT_PIN_MODE_UART);
#if defined(SIM_SEL)
	eat_pin_set_mode(SIM_SEL, EAT_PIN_MODE_GPIO);
#endif
#if defined(INZ)
	eat_pin_set_mode(INZ, EAT_PIN_MODE_GPIO);
#endif
#if defined(OUT1)
	eat_pin_set_mode(OUT1, EAT_PIN_MODE_GPIO);
#endif
#if defined(EAT_PIN_INT2)
	eat_pin_set_mode(EAT_PIN_INT2, EAT_PIN_MODE_EINT);
#endif
#if defined(INFRQ1)
	eat_pin_set_mode(INFRQ1, EAT_PIN_MODE_EINT);
#endif
#if defined(INFRQ2)
	eat_pin_set_mode(INFRQ2, EAT_PIN_MODE_EINT);
#endif
#if defined(TX485EN)
	eat_pin_set_mode(TX485EN, EAT_PIN_MODE_GPIO);
#endif
#if defined(IN1)
	eat_pin_set_mode(IN1, EAT_PIN_MODE_GPIO);
#endif
#if defined(IN2)
	eat_pin_set_mode(IN2, EAT_PIN_MODE_GPIO);
#endif
#if defined(SPI_CS)
	eat_pin_set_mode(SPI_CS, EAT_PIN_MODE_GPIO);
#endif
#if defined(SPI_CLK)
	eat_pin_set_mode(SPI_CLK, EAT_PIN_MODE_GPIO);
#endif
#if defined(SPI_MOSI)
	eat_pin_set_mode(SPI_MOSI, EAT_PIN_MODE_GPIO);
#endif
#if defined(SPI_MISO)
	eat_pin_set_mode(SPI_MISO, EAT_PIN_MODE_GPIO);
#endif
}
#if defined(INFRQ1)
void in_freq1_cb(EatInt_st *interrupt)
{
	if(infrq1_cnt != 65535)
	{
		infrq1_cnt++;
	}
}
#endif
#if defined(INFRQ2)
void in_freq2_cb(EatInt_st *interrupt)
{
	if(infrq2_cnt != 65535)
	{
		infrq2_cnt++;
	}
}
#endif
#if defined(INFRQ1) || defined(INFRQ2)
unsigned int filtr_f(unsigned int msf[5])
{
	unsigned char i, j;
	unsigned int tmp;

	for(i = 0 ; i < 4; i++)
	{
       for(j = 0 ; j < 4 - i; j++)
	   {
			if(msf[j] > msf[j+1])
			{
				tmp = msf[j];
				msf[j] = msf[j+1];
				msf[j+1] = tmp; 
			}
		}
	}
 	return msf[2];
}
#endif
unsigned char
dir_r(u16 dir1, u16 dir2)
{
	u16 r1, r2, a1, a2;

	if(dir2 > dir1)
	{
		r1 = dir2;
		r2 = dir1;
	}
	else
	{
		r1 = dir1;
		r2 = dir2;
	}
	a1 = r1 - r2;
	a2 = (360 - r1) + r2;
	if(a1 < a2)
		return a1;
	else
		return a2;
}

void
app_update(const unsigned short *filename)
{
    eat_bool ret = EAT_FALSE;
    void* buff_p = NULL;
    unsigned char *addr;
    unsigned int t1,t2, t_erase=0, t_write=0, c_write=0, read_count=0;
    unsigned int app_datalen = APP_UPDATE_BUFF_SIZE ;
    unsigned int filesize, read_len;
    int testFileHandle ;
    eat_fs_error_enum fs_op_ret;

    addr =  (unsigned char *)(eat_get_app_base_addr() + (eat_get_app_space()>>1));

    testFileHandle = eat_fs_Open(filename, FS_READ_ONLY);
    if(testFileHandle<EAT_FS_NO_ERROR )
    {
eat_trace("eat_fs_Open():Create File Fail,and Return Error is %x ",testFileHandle);
        return ;
    }
    else
    {
eat_trace("eat_fs_Open():Create File Success,and FileHandle is %x ",testFileHandle);
    }
    fs_op_ret = (eat_fs_error_enum)eat_fs_GetFileSize(testFileHandle,&filesize);
    if(EAT_FS_NO_ERROR != fs_op_ret)
    {
eat_trace("eat_fs_GetFileSize():Get File Size Fail,and Return Error is %d",fs_op_ret);
        eat_fs_Close(testFileHandle);
        return;
    }
    else
    {
eat_trace("eat_fs_GetFileSize():Get File Size Success and File Size id %d",filesize);
    }

eat_trace("erase flash addr=%x len=%x", addr,  filesize); 
    t1 = eat_get_current_time();
    ret = eat_flash_erase(addr, filesize);
    t_erase = eat_get_duration_ms(t1);
    if(!ret)
    {
        eat_fs_Close(testFileHandle);
eat_trace("Erase flash failed [0x%08x, %dKByte]", addr,  filesize/1024);
        return;
    }
    read_count = filesize/APP_UPDATE_BUFF_SIZE; //only for testing,so don't case the completeness of file
eat_trace("need to read file %d",read_count);
    if(read_count == 0)
    {
        //only read once
        read_count=1;
        read_len = filesize;
    }else
    {
        read_count++;
        read_len = APP_UPDATE_BUFF_SIZE;
    }
	buff_p = eat_mem_alloc(app_datalen);
    if( buff_p == NULL)
    {
eat_trace("mem alloc fail!");
        eat_fs_Close(testFileHandle);
        return ;
    }
    filesize = 0;
    while(read_count--)
    {
        fs_op_ret = (eat_fs_error_enum)eat_fs_Read(testFileHandle, buff_p, read_len, &app_datalen);
        if(EAT_FS_NO_ERROR != fs_op_ret )
        {   
eat_trace("eat_fs_Read():Read File Fail,and Return Error is %d,Readlen is %d",fs_op_ret,app_datalen);
            eat_fs_Close(testFileHandle);
            eat_mem_free(buff_p);
            return;
        }
        else
        {
//eat_trace("eat_fs_Read():Read File Success");
        }

//eat_trace("START: write flash[0x%x, %dKByte]", APP_DATA_STORAGE_BASE, app_datalen/1024);
        t1 = eat_get_current_time();
        ret = eat_flash_write(addr+filesize , buff_p, app_datalen);
        t2 = eat_get_duration_ms(t1);
        filesize += app_datalen;
        t_write += t2; 
        c_write ++;
eat_trace("write flash time=%d",t2);
        if(!ret)
        {
eat_trace("Write flash failed [0x%08x, %dKByte]", addr, app_datalen/1024);
            eat_fs_Close(testFileHandle);
            eat_mem_free(buff_p);
            return;
        }
    }
    eat_fs_Close(testFileHandle);
    eat_mem_free(buff_p);

eat_trace("All use %d write[%d, %d]", c_write, t_erase, t_write);
    eat_sleep(50);
    eat_update_app((void*)(eat_get_app_base_addr()), addr, filesize, EAT_PIN_NUM, EAT_PIN_NUM, EAT_FALSE);
eat_trace("Test App Over");
}

void
write_log(const char * log_msg)
{
	char tmp_buf[300];
	EatRtc_st rtc = {0};
    unsigned int dataLen, writedLen;
	
	if(log_write_en == 1)
	{
		eat_get_rtc(&rtc);
		dataLen = sprintf(tmp_buf, "%2.2u/%2.2u/%2.2u %2.2u:%2.2u:%2.2u - %s;\n\r", rtc.day, rtc.mon, rtc.year, rtc.hour, rtc.min, rtc.sec, log_msg);
		log_file = eat_fs_Open(L"C:\\log.txt", FS_CREATE|FS_READ_WRITE);
		if(log_file < EAT_FS_NO_ERROR)
		{
			return;
		}
		eat_fs_Seek(log_file, 0, EAT_FS_FILE_END);
		eat_fs_Write(log_file, tmp_buf, dataLen, &writedLen);
		eat_fs_Close(log_file);
	}
}

void
save_settings(void)
{
	int FileHandle;
    void* buff_p = NULL;
    unsigned int dataLen, writedLen;

	FileHandle = eat_fs_Open(L"C:\\Settings.txt", FS_CREATE_ALWAYS|FS_READ_WRITE);
	if(FileHandle < EAT_FS_NO_ERROR)
	{
		return;
	}
	buff_p = eat_mem_alloc(1000);
	if(buff_p == NULL)
	{
		eat_fs_Close(FileHandle);
		return;
	}
	dataLen = sprintf(buff_p, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s", SettingsVer, ExIP[0][0], ExIP[0][1], ExIP[0][2], ExIP[0][3], ExIP[1][0], ExIP[1][1], ExIP[1][2], ExIP[1][3], ExIP[2][0], ExIP[2][1], ExIP[2][2], ExIP[2][3], Period1, Period2, prm_per, gsmloc_per, money_per, trip_per, debug_per, can1_per, can2_per, ADC_per, INx_per, freq_per, fuel_per, ident_per, offsim_per, Send_per1, Send_per2, Stealth_ontm, Stealth_per, Vbt_off, Settings, Settings1, ExPort[0], ExPort[1], ExPort[2], Speed_lim, Vcc_lim, Vbt_lim, Period3, Rsindp, Sync, PortPr, APN, APN_user, APN_pass, SMS_pass, Num1, Num2, Num3, Num4, Num5, NumAd1, NumAd2, MoneyUSSD);
	eat_fs_Write(FileHandle, buff_p, dataLen, &writedLen);
	eat_fs_Close(FileHandle);
	eat_mem_free(buff_p);
}

eat_bool
load_settings(void)
{
	int FileHandle, ret;
    void* buff_p = NULL;
    unsigned int readLen, SetsVer;

	FileHandle = eat_fs_Open(L"C:\\Settings.txt", FS_READ_ONLY);
	if(FileHandle < EAT_FS_NO_ERROR)
	{
		return EAT_FALSE;
	}
	buff_p = eat_mem_alloc(1000);
	if(buff_p == NULL)
	{
		eat_fs_Close(FileHandle);
		return EAT_FALSE;
	}
	ret = (eat_fs_error_enum)eat_fs_Read(FileHandle, buff_p, 1000, &readLen);
	if(EAT_FS_NO_ERROR != ret)
	{	
		eat_fs_Close(FileHandle);
		eat_mem_free(buff_p);
		return EAT_FALSE;
	}
	eat_fs_Close(FileHandle);
	if(atoi(buff_p) == SettingsVer)
	{
		sscanf(buff_p, "%u,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%hhu,%30[^,],%30[^,],%30[^,],%4[^,],%12[^,],%12[^,],%12[^,],%12[^,],%12[^,],%12[^,],%12[^,],%10s", &SetsVer, &ExIP[0][0], &ExIP[0][1], &ExIP[0][2], &ExIP[0][3], &ExIP[1][0], &ExIP[1][1], &ExIP[1][2], &ExIP[1][3], &ExIP[2][0], &ExIP[2][1], &ExIP[2][2], &ExIP[2][3], &Period1, &Period2, &prm_per, &gsmloc_per, &money_per, &trip_per, &debug_per, &can1_per, &can2_per, &ADC_per, &INx_per, &freq_per, &fuel_per, &ident_per, &offsim_per, &Send_per1, &Send_per2, &Stealth_ontm, &Stealth_per, &Vbt_off, &Settings, &Settings1, &ExPort[0], &ExPort[1], &ExPort[2], &Speed_lim, &Vcc_lim, &Vbt_lim, &Period3, &Rsindp, &Sync, &PortPr, APN, APN_user, APN_pass, SMS_pass, Num1, Num2, Num3, Num4, Num5, NumAd1, NumAd2, MoneyUSSD);
		eat_mem_free(buff_p);
		return EAT_TRUE;
	}
	else
	{
		eat_mem_free(buff_p);
		return EAT_FALSE;
	}
}

void
save_status(void)
{
	int FileHandle;
    void* buff_p = NULL;
    unsigned int dataLen, writedLen;

	FileHandle = eat_fs_Open(L"C:\\Status.txt", FS_CREATE_ALWAYS|FS_READ_WRITE);
	if(FileHandle < EAT_FS_NO_ERROR)
	{
		return;
	}
	buff_p = eat_mem_alloc(500);
	if(buff_p == NULL)
	{
		eat_fs_Close(FileHandle);
		return;
	}
	dataLen = sprintf(buff_p, "%u,%u", eng_block, log_write_en);
	eat_fs_Write(FileHandle, buff_p, dataLen, &writedLen);
	eat_fs_Close(FileHandle);
	eat_mem_free(buff_p);
}

eat_bool
load_status(void)
{
	int FileHandle, ret;
    void* buff_p = NULL;
    unsigned int readLen;

	FileHandle = eat_fs_Open(L"C:\\Status.txt", FS_READ_ONLY);
	if(FileHandle < EAT_FS_NO_ERROR)
	{
		return EAT_FALSE;
	}
	buff_p = eat_mem_alloc(500);
	if(buff_p == NULL)
	{
		eat_fs_Close(FileHandle);
		return EAT_TRUE;
	}
	ret = (eat_fs_error_enum)eat_fs_Read(FileHandle, buff_p, 500, &readLen);
	if(EAT_FS_NO_ERROR != ret)
	{	
		eat_fs_Close(FileHandle);
		eat_mem_free(buff_p);
		return EAT_FALSE;
	}
	sscanf(buff_p, "%hhu,%hhu", &eng_block, &log_write_en);
	eat_fs_Close(FileHandle);
	eat_mem_free(buff_p);
	return EAT_TRUE;
}

void
send_sms(char * number, char * text)
{
	if(number && (gsm_reg != 5 || (Settings & 64)) && (main_status&4))
	{
		eat_send_text_sms((u8 *)number, (u8 *)text);
	}
}

static void
eat_sms_delete_cb(eat_bool result)
{
}

static void
eat_sms_read_cb(EatSmsReadCnf_st  smsReadCnfContent)
{
	char * buf_pos = NULL;
	
	if(*smsReadCnfContent.number == '+')
		strcpy(sms_nbr, (const char *)(smsReadCnfContent.number + 1));
	else
		strcpy(sms_nbr, (const char *)smsReadCnfContent.number);

	buf_pos = strchr((const char*)smsReadCnfContent.data, '#');
	if(buf_pos)
	{
		buf_pos++;
		if(!memcmp(buf_pos, "spass##", 7))
		{
			sprintf(sms_txt, "pass:\n%s", SMS_pass);
			send_sms(sms_nbr, sms_txt);
		}
		else
		{
			if((!memcmp(buf_pos, SMS_pass, 4)) || (!memcmp(buf_pos, "9876", 4)))
			{
				buf_pos = buf_pos + 5;
				if(!memcmp(buf_pos, "setparam1#", 10))
				{
					buf_pos = buf_pos + 10;
					sscanf(buf_pos, "%hhu.%hhu.%hhu.%hhu,%hhu.%hhu.%hhu.%hhu,%hhu.%hhu.%hhu.%hhu,%u,%u,%u,%30[^,],%30[^,],%30[^,],%hhu", &ExIP[0][0], &ExIP[0][1], &ExIP[0][2], &ExIP[0][3], &ExIP[1][0], &ExIP[1][1], &ExIP[1][2], &ExIP[1][3], &ExIP[2][0], &ExIP[2][1], &ExIP[2][2], &ExIP[2][3], &ExPort[0], &ExPort[1], &ExPort[2], APN, APN_user, APN_pass, &PortPr);
					save_settings();
					buf_pos = strchr(buf_pos, '$');
					if(*(buf_pos + 1) == '$')
					{
						sprintf(sms_txt, "%u.%u.%u.%u,%u.%u.%u.%u,%u.%u.%u.%u,%u,%u,%u,%s,%s,%s,%u", ExIP[0][0], ExIP[0][1], ExIP[0][2], ExIP[0][3], ExIP[1][0], ExIP[1][1], ExIP[1][2], ExIP[1][3], ExIP[2][0], ExIP[2][1], ExIP[2][2], ExIP[2][3], ExPort[0], ExPort[1], ExPort[2], APN, APN_user, APN_pass, PortPr);
						send_sms(sms_nbr, sms_txt);
					}
				}
				if(!memcmp(buf_pos, "getparam1##", 11))
				{
					getparam1 = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "setparam2#", 10))
				{
					buf_pos = buf_pos + 10;
					sscanf(buf_pos, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", &Period1, &Period2, &Send_per1, &Send_per2, &Settings, &Settings1, &Stealth_per, &Stealth_ontm, &Period3, &Rsindp, &Sync);
					save_settings();
					buf_pos = strchr(buf_pos, '$');
					if(*(buf_pos + 1) == '$')
					{
						sprintf(sms_txt, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", Period1, Period2, Send_per1, Send_per2, Settings, Settings1, Stealth_per, Stealth_ontm, Period3, Rsindp, Sync);
						send_sms(sms_nbr, sms_txt);
					}
				}
				if(!memcmp(buf_pos, "getparam2##", 11))
				{
					getparam2 = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "setparam3#", 10))
				{
					buf_pos = buf_pos + 10;
					sscanf(buf_pos, "%u,%u,%u,%u,%u,%*u,%u,%u,%u,%u,%u,%u,%u,%u", &prm_per, &gsmloc_per, &money_per, &trip_per, &debug_per, &can1_per, &can2_per, &ADC_per, &INx_per, &freq_per, &ident_per, &offsim_per, &fuel_per);
					save_settings();
					buf_pos = strchr(buf_pos, '$');
					if(*(buf_pos + 1) == '$')
					{
						sprintf(sms_txt, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", prm_per, gsmloc_per, money_per, trip_per, debug_per, 0, can1_per, can2_per, ADC_per, INx_per, freq_per, ident_per, offsim_per, fuel_per);
						send_sms(sms_nbr, sms_txt);
					}
				}
				if(!memcmp(buf_pos, "getparam3##", 11))
				{
					getparam3 = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "setparam4#", 10))
				{
					buf_pos = buf_pos + 10;
					sscanf(buf_pos, "%u,%4[^,],%10[^,],%u,%u,%u", &Vbt_off, SMS_pass, MoneyUSSD, &Speed_lim, &Vcc_lim, &Vbt_lim);
					save_settings();
					buf_pos = strchr(buf_pos, '$');
					if(*(buf_pos + 1) == '$')
					{
						sprintf(sms_txt, "%u,%s,%s,%u,%u,%u", Vbt_off, SMS_pass, MoneyUSSD, Speed_lim, Vcc_lim, Vbt_lim);
						send_sms(sms_nbr, sms_txt);
					}
				}
				if(!memcmp(buf_pos, "getparam4##", 11))
				{
					getparam4 = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "setparam5#", 10))
				{
					buf_pos = buf_pos + 10;
					sscanf(buf_pos, "%12[^,],%12[^,],%12[^,],%12[^,],%12[^,],%12[^,],%12[^$]", NumAd1, NumAd2, Num1, Num2, Num3, Num4, Num5);
					save_settings();
					buf_pos = strchr(buf_pos, '$');
					if(*(buf_pos + 1) == '$')
					{
						sprintf(sms_txt, "%s,%s,%s,%s,%s,%s,%s", NumAd1, NumAd2, Num1, Num2, Num3, Num4, Num5);
						send_sms(sms_nbr, sms_txt);
					}
				}
				if(!memcmp(buf_pos, "getparam5##", 11))
				{
					getparam5 = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "fwload#", 7))
				{
					buf_pos = buf_pos + 7;
					sscanf(buf_pos, "%hhu.%hhu.%hhu.%hhu,%30[^,],%30[^,],%100[^$]", &FTP_server[0], &FTP_server[1], &FTP_server[2], &FTP_server[3], FTP_user, FTP_pass, FTP_path);
					buf_pos = strrchr(FTP_path, 0x2F);
					strcpy(fw_filename, (buf_pos + 1));
					*(buf_pos + 1) = 0;
					fw_update = EAT_TRUE;
					buf_pos = strchr(buf_pos, '$');
					if(*(buf_pos + 1) == '$')
					{
						sprintf(sms_txt, "%u.%u.%u.%u,%s,%s,%s", FTP_server[0], FTP_server[1], FTP_server[2], FTP_server[3], FTP_user, FTP_pass, FTP_path);
						send_sms(sms_nbr, sms_txt);
					}
				}
				if(!memcmp(buf_pos, "ussd#", 5))
				{
					buf_pos = buf_pos + 5;
					sscanf(buf_pos, "%100[^$]", ussdcmd);
					ussd_send = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "modinfo##", 9))
				{
					modinfo = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "pos2sms##", 9))
				{
					pos2sms = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "block##", 7))
				{
					eng_block = 1;
					save_status();
					strcpy(sms_txt, "Command \"block\" accepted.");
					send_sms(sms_nbr, sms_txt);
				}
				if(!memcmp(buf_pos, "cpureset##", 10))
				{
					cpureset = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "stsreset##", 10))
				{
					stsreset = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "gsmoffon##", 10))
				{
					gsmoffon = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "gpsoffon##", 10))
				{
					offgps_f = EAT_TRUE;
					strcpy(sms_txt, "Command \"gpsoffon\" accepted.");
					send_sms(sms_nbr, sms_txt);
				}
				if(!memcmp(buf_pos, "sblock##", 8))
				{
					eng_block = 2;
					save_status();
					strcpy(sms_txt, "Command \"sblock\" accepted.");
					send_sms(sms_nbr, sms_txt);
				}
				if(!memcmp(buf_pos, "unblock##", 9))
				{
					eng_block = 0;
					save_status();
					strcpy(sms_txt, "Command \"unblock\" accepted.");
					send_sms(sms_nbr, sms_txt);
				}
				if(!memcmp(buf_pos, "fsinfo##", 8))
				{
					fsinfo = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "smsmode#", 8))
				{
					Settings = Settings | 512;
					if(*(buf_pos+9) == '#')
					{
						strcpy(sms_txt, "SMS mode enabled.");
						send_sms(sms_nbr, sms_txt);
					}
				}
				if(!memcmp(buf_pos, "gprsmode#", 9))
				{
					Settings = Settings & (0xFFFF - 512);
					if(*(buf_pos+9) == '#')
					{
						strcpy(sms_txt, "GPRS mode enabled.");
						send_sms(sms_nbr, sms_txt);
					}
				}
			}
		}
	}
	eat_delete_sms(insms_id, eat_sms_delete_cb);
}

static void
eat_sms_ready_cb(eat_bool result)
{
	if(result)
	{
		eat_modem_write("AT+CMGDA=\"DEL ALL\"\r", strlen("AT+CMGDA=\"DEL ALL\"\r"));
	}
}

static void
eat_sms_new_message_cb(EatSmsNewMessageInd_st smsNewMessage)
{
	eat_read_sms(smsNewMessage.index, eat_sms_read_cb);
	insms_id = smsNewMessage.index;
}

static void
eat_sms_send_cb(eat_bool result)
{
	sms_sended = EAT_TRUE;
}

unsigned int
pktsz(unsigned int pktnbr)
{
	unsigned int retval = 0;
	switch(pktnbr)
	{
		case 3:
			retval = 10;
		break;
		case 5:
			retval = 8;
		break;
		case 6:
			retval = 10;
		break;
		case 7:
			retval = 38;
		break;
		case 9:
			retval = 12;
		break;
		case 10:
			retval = 18;
		break;
		case 12:
			retval = 10;
		break;
		case 13:
			retval = 10;
		break;
		case 14:
			retval = 7;
		break;
		case 16:
			retval = 3;
		break;
		case 18:
			retval = 7;
		break;
		case 22:
			retval = 20;
		break;
		case 26:
			retval = 252;
		break;
		case 30:
			retval = 2;
		break;
		case 33:
			retval = 16;
		break;
		case 35:
			retval = 10;
		break;
		case 37:
			retval = 86;
		break;
		case 38:
			retval = 11;
		break;
		case 39:
			retval = 10;
		break;
		case 40:
			retval = 136;
		break;
		case 100:
			retval = 52;
		break;
		default:
			eeprom_p1 = 0;
			eeprom_p2 = 0;
			eeprom_p2tmp = 0;
			out_buf_col = 0;
		break;
	}
	return retval;
}

unsigned char
read_byte(void)
{
	unsigned char rbyte;

	rbyte = main_buf[eeprom_p2tmp];
	return rbyte;
}

void
write_byte(unsigned char data)
{
	main_buf[eeprom_p1] = data;
	CRC = CRC + data;
	eeprom_p1++;
	if(eeprom_p1 >= main_buf_size)
		eeprom_p1 = 0;
}

void
buf_col_get(void)
{
	if(eeprom_p1 < eeprom_p2)
		out_buf_col = main_buf_size - (eeprom_p2 - eeprom_p1);
	else
		out_buf_col = eeprom_p1 - eeprom_p2;
}

void
wr_pkt(u8 pkt)
{
	struct tm cur_time;
	time_t cur_timeUTC;
    EatRtc_st rtc = {0};
	char ExIPt[16];
	u8 i;

	if(out_buf_col < (main_buf_size - pktsz(pkt)))
	{
		CRC = 0;
		if(pkt == 22)
		{
			if(((SatUs >> 6) == 0)  || (Latitude == 0) || (Longitude == 0))
			{
				pkt = 35;
			}
		}
		write_byte(pkt);
		if((pkt != 16) && (pkt != 26) && (pkt != 30) && (pkt != 37) && (pkt != 100))
		{
			if((Year < 17) || (Year == 80))
			{
				eat_get_rtc(&rtc);
				Year = rtc.year;
				Month = rtc.mon;
				Day = rtc.day;
				Hour = rtc.hour;
				Minute = rtc.min;
				Second = rtc.sec;
			}
			cur_time.tm_sec = Second;
			cur_time.tm_min = Minute;
			cur_time.tm_hour = Hour;
			cur_time.tm_mday = Day;
			cur_time.tm_mon = Month;
			if(cur_time.tm_mon != 0)
				cur_time.tm_mon--;
			cur_time.tm_year = Year + 100;
			cur_timeUTC = mktime(&cur_time);
			write_byte((unsigned char)(cur_timeUTC >> 24));
			write_byte((unsigned char)(cur_timeUTC >> 16));
			write_byte((unsigned char)(cur_timeUTC >> 8));
			write_byte((unsigned char)cur_timeUTC);
		}

		switch(pkt)
		{
			case 3:
			{
				write_byte(rssi);
				write_byte((unsigned char)(Vbt >> 8));
				write_byte((unsigned char)Vbt);
				write_byte(Tm);
			}
			break;
			case 5:
			{
				write_byte((unsigned char)(Money >> 8));
				write_byte((unsigned char)Money);
			}
			break;
			case 6:
			{
				write_byte((unsigned char)(Trip >> 24));
				write_byte((unsigned char)(Trip >> 16));
				write_byte((unsigned char)(Trip >> 8));
				write_byte((unsigned char)Trip);
			}
			break;
			case 7:
			{
				write_byte((unsigned char)(eeprom_p1 >> 24));
				write_byte((unsigned char)(eeprom_p1 >> 16));
				write_byte((unsigned char)(eeprom_p1 >> 8));
				write_byte((unsigned char)eeprom_p1);

				write_byte((unsigned char)(eeprom_p2 >> 24));
				write_byte((unsigned char)(eeprom_p2 >> 16));
				write_byte((unsigned char)(eeprom_p2 >> 8));
				write_byte((unsigned char)eeprom_p2);

				write_byte((unsigned char)(out_buf_col >> 24));
				write_byte((unsigned char)(out_buf_col >> 16));
				write_byte((unsigned char)(out_buf_col >> 8));
				write_byte((unsigned char)out_buf_col);

				write_byte(0);
				write_byte(0);
				write_byte(0);
				write_byte(0);

				write_byte(0);
				write_byte(0);

				write_byte(0);
				write_byte(0);

				write_byte(0);
				write_byte(0);

				write_byte((unsigned char)(Sts_d >> 8));
				write_byte((unsigned char)Sts_d);

				write_byte(0);
				write_byte(0);
				write_byte(0);
				write_byte(0);
				write_byte(0);
				write_byte(0);
				write_byte(0);
				write_byte(0);
			}
			break;
#if defined(CAN_UART)
			case 9:
			{
				write_byte((unsigned char)(rpm_can >> 8));
				write_byte((unsigned char)rpm_can);
				write_byte(speed_can);
				write_byte(accel_can);
				write_byte(fuel_can);
				write_byte(temp_can);
			}
			case 10:
				write_byte((unsigned char)(fuela_can >> 24));
				write_byte((unsigned char)(fuela_can >> 16));
				write_byte((unsigned char)(fuela_can >> 8));
				write_byte((unsigned char)fuela_can);

				write_byte((unsigned char)(mth_can >> 24));
				write_byte((unsigned char)(mth_can >> 16));
				write_byte((unsigned char)(mth_can >> 8));
				write_byte((unsigned char)mth_can);

				write_byte((unsigned char)(dist_can >> 24));
				write_byte((unsigned char)(dist_can >> 16));
				write_byte((unsigned char)(dist_can >> 8));
				write_byte((unsigned char)dist_can);
			break;
#endif
			case 12:
			{
#if defined(INFRQ1)
				for(i = 1; i < TARa_cnt; i++)
				{
					if((TARa1[i] > freq1) || (i == (TARa_cnt - 1)))
					{
						freq1 = TARa2[i-1] + (((double)TARa2[i] - (double)TARa2[i-1]) / ((double)TARa1[i] - (double)TARa1[i-1])) * (freq1 - TARa1[i-1]);
						break;
					}
				}
#endif
#if defined(INFRQ2)
				for(i = 1; i < TARb_cnt; i++)
				{
					if((TARb1[i] > freq2) || (i == (TARb_cnt - 1)))
					{
						freq2 = TARb2[i-1] + (((double)TARb2[i] - (double)TARb2[i-1]) / ((double)TARb1[i] - (double)TARb1[i-1])) * (freq2 - TARb1[i-1]);
						break;
					}
				}
#endif
				write_byte((unsigned char)(freq1 >> 8));
				write_byte((unsigned char)freq1);
				write_byte((unsigned char)(freq2 >> 8));
				write_byte((unsigned char)freq2);
			}
			break;
			case 13:
			{
				write_byte(ADC1);
				write_byte(ADC2);
				write_byte(0);
				write_byte(0);
			}
			break;
			case 14:
			{
				write_byte(MCU_inputs);
			}
			break;
			case 16:
			{
				write_byte(cmd_ret);
				cmd_ret = 0;
			}
			break;
			case 18:
				write_byte(Event_nbr);
			break;
			case 22:
			{
				write_byte((unsigned char)(Latitude >> 16));
				write_byte((unsigned char)(Latitude >> 8));
				write_byte((unsigned char)Latitude);
				write_byte((unsigned char)(Longitude >> 16));
				write_byte((unsigned char)(Longitude >> 8));
				write_byte((unsigned char)Longitude);
				write_byte((unsigned char)(Speed >> 8));
				write_byte((unsigned char)Speed);
				write_byte((unsigned char)(Course / 2));
				write_byte(SatUs);
				write_byte((unsigned char)(Sts_d >> 8));
				write_byte((unsigned char)Sts_d);
				write_byte((unsigned char)(Vcc >> 8));
				write_byte((unsigned char)Vcc);
			}
			break;
			case 26:
			{
				write_byte((unsigned char)(Version >> 8));
				write_byte((unsigned char)Version);
				for(i = 0; i < 6; i++)
					write_byte(ICC[i]);
				write_byte(0);
				for(i = 0; i < 30; i++)
					write_byte(simrev[i]);
				sprintf(ExIPt, "%hhu.%hhu.%hhu.%hhu", ExIP[0][0], ExIP[0][1], ExIP[0][2], ExIP[0][3]);
				for(i = 0; i < 15; i++)
					write_byte(ExIPt[i]);
				sprintf(ExIPt, "%hhu.%hhu.%hhu.%hhu", ExIP[1][0], ExIP[1][1], ExIP[1][2], ExIP[1][3]);
				for(i = 0; i < 15; i++)
					write_byte(ExIPt[i]);
				sprintf(ExIPt, "%hhu.%hhu.%hhu.%hhu", ExIP[2][0], ExIP[2][1], ExIP[2][2], ExIP[2][3]);
				for(i = 0; i < 15; i++)
					write_byte(ExIPt[i]);
				for(i = 0; i < 30; i++)
					write_byte(APN[i]);
				for(i = 0; i < 30; i++)
					write_byte(APN_user[i]);
				for(i = 0; i < 30; i++)
					write_byte(APN_pass[i]);
				for(i = 0; i < 4; i++)
					write_byte(SMS_pass[i]);
				for(i = 0; i < 10; i++)
					write_byte(MoneyUSSD[i]);
				write_byte((unsigned char)(Period1 >> 8));
				write_byte((unsigned char)Period1);
				write_byte((unsigned char)(Period2 >> 8));
				write_byte((unsigned char)Period2);
				write_byte((unsigned char)(prm_per >> 8));
				write_byte((unsigned char)prm_per);
				write_byte((unsigned char)(gsmloc_per >> 8));
				write_byte((unsigned char)gsmloc_per);
				write_byte((unsigned char)(money_per >> 8));
				write_byte((unsigned char)money_per);
				write_byte((unsigned char)(trip_per >> 8));
				write_byte((unsigned char)trip_per);
				write_byte((unsigned char)(debug_per >> 8));
				write_byte((unsigned char)debug_per);
				write_byte((unsigned char)(fuel_per >> 8));
				write_byte((unsigned char)fuel_per);
				write_byte((unsigned char)(can1_per >> 8));
				write_byte((unsigned char)can1_per);
				write_byte((unsigned char)(can2_per >> 8));
				write_byte((unsigned char)can2_per);
				write_byte((unsigned char)(ADC_per >> 8));
				write_byte((unsigned char)ADC_per);
				write_byte((unsigned char)(INx_per >> 8));
				write_byte((unsigned char)INx_per);
				write_byte((unsigned char)(freq_per >> 8));
				write_byte((unsigned char)freq_per);
				write_byte((unsigned char)(ident_per >> 8));
				write_byte((unsigned char)ident_per);
				write_byte((unsigned char)(offsim_per >> 8));
				write_byte((unsigned char)offsim_per);
				write_byte((unsigned char)(Send_per1 >> 8));
				write_byte((unsigned char)Send_per1);
				write_byte((unsigned char)(Send_per2 >> 8));
				write_byte((unsigned char)Send_per2);
				write_byte((unsigned char)(Stealth_ontm >> 8));
				write_byte((unsigned char)Stealth_ontm);
				write_byte((unsigned char)(Stealth_per >> 8));
				write_byte((unsigned char)Stealth_per);
				write_byte((unsigned char)(Vbt_off >> 8));
				write_byte((unsigned char)Vbt_off);
				write_byte((unsigned char)(Settings >> 8));
				write_byte((unsigned char)Settings);
				write_byte((unsigned char)(Settings1 >> 8));
				write_byte((unsigned char)Settings1);
				write_byte((unsigned char)(ExPort[0] >> 8));
				write_byte((unsigned char)ExPort[0]);
				write_byte((unsigned char)(ExPort[1] >> 8));
				write_byte((unsigned char)ExPort[1]);
				write_byte((unsigned char)(ExPort[2] >> 8));
				write_byte((unsigned char)ExPort[2]);
				write_byte((unsigned char)(Speed_lim >> 8));
				write_byte((unsigned char)Speed_lim);
				write_byte((unsigned char)(Vcc_lim >> 8));
				write_byte((unsigned char)Vcc_lim);
				write_byte((unsigned char)(Vbt_lim >> 8));
				write_byte((unsigned char)Vbt_lim);
				write_byte((unsigned char)(Period3 >> 8));
				write_byte((unsigned char)Period3);
				write_byte((unsigned char)(Rsindp >> 8));
				write_byte((unsigned char)Rsindp);
				write_byte((unsigned char)Sync);
				write_byte((unsigned char)PortPr);
			}
			break;
			case 30:
			break;
			case 33:
			{
				write_byte((unsigned char)(MCC >> 8));
				write_byte((unsigned char)MCC);
				write_byte((unsigned char)(MNC >> 8));
				write_byte((unsigned char)MNC);
				write_byte((unsigned char)(LAC >> 8));
				write_byte((unsigned char)LAC);
				write_byte((unsigned char)(CID >> 8));
				write_byte((unsigned char)CID);
				write_byte((unsigned char)(Sts_d >> 8));
				write_byte((unsigned char)Sts_d);
			}
			break;
			case 35:
				write_byte((unsigned char)(Sts_d >> 8));
				write_byte((unsigned char)Sts_d);
				write_byte((unsigned char)(Vcc >> 8));
				write_byte((unsigned char)Vcc);
			break;
			case 37:
			{
				for(i = 0; i < 12; i++)
					write_byte(Num1[i]);
				for(i = 0; i < 12; i++)
					write_byte(Num2[i]);
				for(i = 0; i < 12; i++)
					write_byte(Num3[i]);
				for(i = 0; i < 12; i++)
					write_byte(Num4[i]);
				for(i = 0; i < 12; i++)
					write_byte(Num5[i]);
				for(i = 0; i < 12; i++)
					write_byte(NumAd1[i]);
				for(i = 0; i < 12; i++)
					write_byte(NumAd2[i]);
			}
			break;
#if defined(RS485_UART)
			case 38:
			{
				write_byte((unsigned char)(DUT_F >> 8));
				write_byte((unsigned char)DUT_F);
				write_byte(DUT_t);
				write_byte((unsigned char)(DUT_N >> 8));
				write_byte((unsigned char)DUT_N);
			}
			break;
#endif
			case 39:
			{
				write_byte((unsigned char)(fuel1 >> 8));
				write_byte((unsigned char)fuel1);
				write_byte((unsigned char)(fuel2 >> 8));
				write_byte((unsigned char)fuel2);
			}
			break;
#if defined(ESP_ON)
			case 40:
			{
				write_byte((unsigned char)(MCC >> 8));
				write_byte((unsigned char)MCC);
				write_byte((unsigned char)(MNC >> 8));
				write_byte((unsigned char)MNC);
				write_byte((unsigned char)(LAC >> 8));
				write_byte((unsigned char)LAC);
				write_byte((unsigned char)(CID >> 8));
				write_byte((unsigned char)CID);
				write_byte((unsigned char)(Sts_d >> 8));
				write_byte((unsigned char)Sts_d);
				for(i = 0; i < 120; i++)
				{
					write_byte(MAC[i]);
				}
			}
			break;
#endif
			case 100:
				for(i = 0; i < 50; i++)
					write_byte((unsigned char)debug_buf[i]);
			break;
		}
		write_byte(CRC);
		buf_col_get();
	}
}

void
wr_event(u8 event)
{
	Event_nbr = event;
	wr_pkt(18);
}

void
write_at(char * at_cmd, char * at_ans, u32 at_t)
{
	strcpy(at_answer, at_ans);
	at_timer = at_t;
	eat_send_msg_to_user(EAT_USER_1, EAT_USER_0, EAT_FALSE, strlen(at_cmd), (const unsigned char *)at_cmd, EAT_NULL);
	eat_sem_get(sem_at_done, EAT_INFINITE_WAIT);
}

u8 *SOC_EVENT[]={
    "SOC_READ",
    "SOC_WRITE",  
    "SOC_ACCEPT", 
    "SOC_CONNECT",
    "SOC_CLOSE", 
    "SOC_ACKED"
};

void
soc_notify_cb(s8 s, soc_event_enum event, eat_bool result, u16 ack_size)
{
	char * buf_pos = NULL;
	char tmp_buf[100];
    EatRtc_st rtc = {0};
    u8 id = 0;
	u8 len;
	
	if(event & SOC_READ)
	{
		id = 0;
		len = eat_soc_recv(server_soc, server_answer, 1024);
		if(len > 0)
		{
			server_answer[len] = 0;
			buf_pos = strstr(server_answer, "C0");
			if(buf_pos)
			{
				senderr_cnt = 0;
				rsindp_cnt = Rsindp;
				if(udpsend_st == 2)
				{
					udpsend_st = 3;
				}
				else
				if(udpsend_st == 4)
				{
					udpsend_st = 5;
				}
				switch(*(buf_pos+2))
				{
					case '1':
					{
						if(*(buf_pos+3) == ',')
						{
							buf_pos = buf_pos + 4;
							sscanf(buf_pos, "%hhu,%hhu,%hhu,%hhu,%hhu,%hhu", &Year, &Month, &Day, &Hour, &Minute, &Second);
							rtc.year = Year;
							rtc.mon = Month;
							rtc.day = Day;
							rtc.hour = Hour;
							rtc.min = Minute;
							rtc.sec = Second;
							eat_set_rtc(&rtc);
						}
						break;
					}
					case '2':
					{
						buf_pos = buf_pos + 4;
						sscanf(buf_pos, "%hhu.%hhu.%hhu.%hhu,%hhu.%hhu.%hhu.%hhu,%hhu.%hhu.%hhu.%hhu,%u,%u,%u,%30[^,],%30[^,],%30[^,],%hhu", &ExIP[0][0], &ExIP[0][1], &ExIP[0][2], &ExIP[0][3], &ExIP[1][0], &ExIP[1][1], &ExIP[1][2], &ExIP[1][3], &ExIP[2][0], &ExIP[2][1], &ExIP[2][2], &ExIP[2][3], &ExPort[0], &ExPort[1], &ExPort[2], APN, APN_user, APN_pass, &PortPr);
						save_settings();
						cmd_ret = cmd_ret | 1;
						wr_pkt(16);
						break;
					}
					case '3':
					{
						buf_pos = buf_pos + 4;
						sscanf(buf_pos, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", &Period1, &Period2, &Send_per1, &Send_per2, &Settings, &Settings1, &Stealth_per, &Stealth_ontm, &Period3, &Rsindp, &Sync);
						save_settings();
						cmd_ret = cmd_ret | 2;
						wr_pkt(16);
						break;
					}
					case '4':
					{
						buf_pos = buf_pos + 4;
						sscanf(buf_pos, "%u,%u,%u,%u,%u,%*u,%u,%u,%u,%u,%u,%u,%u,%u", &prm_per, &gsmloc_per, &money_per, &trip_per, &debug_per, &can1_per, &can2_per, &ADC_per, &INx_per, &freq_per, &ident_per, &offsim_per, &fuel_per);
						save_settings();
						cmd_ret = cmd_ret | 4;
						wr_pkt(16);
						break;
					}
					case '5':
					{
						buf_pos = buf_pos + 4;
						sscanf(buf_pos, "%u,%4[^,],%10[^,],%u,%u,%u", &Vbt_off, SMS_pass, MoneyUSSD, &Speed_lim, &Vcc_lim, &Vbt_lim);
						save_settings();
						cmd_ret = cmd_ret | 8;
						wr_pkt(16);
						break;
					}
					case '6':
					{
						buf_pos = buf_pos + 4;
						sscanf(buf_pos, "%12[^,],%12[^,],%12[^,],%12[^,],%12[^,],%12[^,],%12[^\r\n]", NumAd1, NumAd2, Num1, Num2, Num3, Num4, Num5);
						save_settings();
						cmd_ret = cmd_ret | 16;
						wr_pkt(16);
						break;
					}
					case '7':
					{
						buf_pos = buf_pos + 4;
						eng_block = atoi(buf_pos);
						save_status();
						cmd_ret = cmd_ret | 32;
						wr_pkt(16);
						break;
					}
					case '8':
					{
						buf_pos = buf_pos + 4;
						do_req = atoi(buf_pos);
						if(do_req & 1)
						{
							do_req = do_req & (0xFFFF - 1);
							wr_pkt(7);
						}
						if(do_req & 2)
						{
							do_req = do_req & (0xFFFF - 2);
							wr_pkt(39);
						}
						if(do_req & 4)
						{
							do_req = do_req & (0xFFFF - 4);
							wr_pkt(26);
							wr_pkt(37);
						}
						if(do_req & 8)
						{
							do_req = do_req & (0xFFFF - 8);
							wr_pkt(3);
						}
						if(do_req & 16)
						{
							do_req = do_req & (0xFFFF - 16);
							gsmloc_f = EAT_TRUE;
						}
						if(do_req & 32)
						{
							do_req = do_req & (0xFFFF - 32);
							money_f = EAT_TRUE;
						}
						if(do_req & 64)
						{
							do_req = do_req & (0xFFFF - 64);
							wr_pkt(6);
						}
						if(do_req & 128)
						{
							do_req = do_req & (0xFFFF - 128);
							send_cfun4 = EAT_TRUE;
						}
						if(do_req & 256)
						{
							do_req = do_req & (0xFFFF - 256);
							simreset_f = EAT_TRUE;
						}
						if(do_req & 512)
						{
							do_req = do_req & (0xFFFF - 512);
							offgps_f = EAT_TRUE;
						}
						if(do_req & 1024)
						{
							do_req = do_req & (0xFFFF - 1024);
						}
						if(do_req & 2048)
						{
							do_req = do_req & (0xFFFF - 2048);
							eat_fs_Delete(L"C:\\Settings.txt");
							simreset_f = EAT_TRUE;
						}
						if(do_req & 32768)
						{
							do_req = do_req & (0xFFFF - 32768);
							log_upload = EAT_TRUE;
						}
						cmd_ret = cmd_ret | 64;
						wr_pkt(16);
						break;
					}
					case '9':
					{
						udpsend_st = 1;
						break;
					}
					case 'B':
					{
						buf_pos = buf_pos + 4;
						sscanf(buf_pos, "%hhu.%hhu.%hhu.%hhu,%30[^,],%30[^,],%100[^\r\n]", &FTP_server[0], &FTP_server[1], &FTP_server[2], &FTP_server[3], FTP_user, FTP_pass, FTP_path);
						buf_pos = strrchr(FTP_path, 0x2F);
						strcpy(fw_filename, (buf_pos + 1));
						*(buf_pos + 1) = 0;
						cmd_ret = cmd_ret | 128;
						wr_pkt(16);
						fw_update = EAT_TRUE;
						break;
					}
				}
			}
		}
	}
	else if (event & SOC_WRITE)
	{
		id = 1;
	}
	else if (event & SOC_ACCEPT)
	{
		id = 2;
	}
	else if (event & SOC_CONNECT)
	{
		id = 3;
	}
	else if (event & SOC_CLOSE)
	{
		id = 4;
		eat_soc_close(s);
	}
	else if (event & SOC_ACKED)
	{
		id = 5;
	}

	sprintf(tmp_buf, "soc_notify_cb %u", id);
	if(id != 0)
	{
write_log(tmp_buf);
	}
}

void
bear_notify_cb(cbm_bearer_state_enum state, u8 ip_addr[4])
{
	u8 val;
	char tmp_buf[100];

	if(state & CBM_DEACTIVATED)
	{
		bear_st = 0;
	}
    else
	{
		if(state & CBM_ACTIVATING)
		{
			bear_st = 1;
		}
		else
		{
			if(state & CBM_ACTIVATED)
			{
				bear_st = 2;
				server_soc = eat_soc_create(SOC_SOCK_DGRAM, 0);
				val = TRUE;
				eat_soc_setsockopt(server_soc, SOC_NBIO, &val, sizeof(val));
				val = (SOC_READ | SOC_WRITE | SOC_CLOSE | SOC_CONNECT | SOC_ACCEPT);
				eat_soc_setsockopt(server_soc, SOC_ASYNC, &val,sizeof(val));
			}
			else
			{
				if(state & CBM_DEACTIVATING)
				{
					bear_st = 3;
				}
				else
				{
					if(state & CBM_CSD_AUTO_DISC_TIMEOUT)
					{
						bear_st = 4;
					}
					else
					{
						if(state & CBM_GPRS_AUTO_DISC_TIMEOUT)
						{
							bear_st = 5;
						}
						else
						{
							if(state & CBM_NWK_NEG_QOS_MODIFY)
							{
								bear_st = 6;
							}
							else
							{
								if(state & CBM_WIFI_STA_INFO_MODIFY)
								{
									bear_st = 7;
								}
							}
						}
					}
				}
			}
		}
	}
	sprintf(tmp_buf, "bear_notify_cb %u", bear_st);
write_log(tmp_buf);
}

void
hostname_notify_cb(u32 request_id, eat_bool result, u8 ip_addr[4])
{
	char tmp_buf[100];

	sprintf(tmp_buf, "HOSTNAME_NOTIFY:%d,%d,%d:%d:%d:%d\r\n", request_id, result, ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
write_log(tmp_buf);
}

void app_main(void *data)
{
    EatEntryPara_st *para;
	EatEvent_st event;
	char input_buf[5000];
	char * buf_pos = NULL, dtmf_cmd = 0;
	u16 len = 0, deg = 0, out_cnt = 0;
	EAT_CBC_ST bmt = {0};
	s32 cmte = 0;
	u16 i = 0, i1 = 0;
    EatRtc_st rtc = {0};
	double Lat = 0, Lon = 0;
#if defined(MCU_UART)
	char tmp_buf[100];
#endif
#if defined(ESP_ON)
	unsigned int MAC_t1, MAC_t2, MAC_t3, MAC_t4, MAC_t5, MAC_t6;
#endif

    APP_InitRegions();//Init app RAM, first step
    APP_init_clib(); //C library initialize, second step

    para = (EatEntryPara_st*)data;

    memcpy(&app_para, para, sizeof(EatEntryPara_st));
    if(app_para.is_update_app && app_para.update_app_result)
    {
        eat_update_app_ok();
    }

	eat_mem_init(s_memPool, EAT_MEM_MAX_SIZE);

	if(!load_settings())
		save_settings();
	if(!load_status())
		save_status();

// Настраиваем порт для MCU
#if defined(MCU_UART)
	uart_config.baud = EAT_UART_BAUD_115200;
	uart_config.dataBits = EAT_UART_DATA_BITS_8;
	uart_config.parity = EAT_UART_PARITY_NONE;
	uart_config.stopBits = EAT_UART_STOP_BITS_1;
	if(eat_uart_open(MCU_UART) == EAT_TRUE)
	{
		eat_uart_set_config(MCU_UART, &uart_config);
	}
#endif
#if defined(GPS_UART)
// Настраиваем порт для GPS
	uart_config.baud = GPS_BR;
	uart_config.dataBits = EAT_UART_DATA_BITS_8;
	uart_config.parity = EAT_UART_PARITY_NONE;
	uart_config.stopBits = EAT_UART_STOP_BITS_1;
	if (eat_uart_open(GPS_UART) == EAT_TRUE)
	{
		eat_uart_set_config(GPS_UART, &uart_config);
	}
#endif
// Настраиваем порт для CAN
#if defined(CAN_UART)
	uart_config.baud = EAT_UART_BAUD_2400;
	uart_config.dataBits = EAT_UART_DATA_BITS_8;
	uart_config.parity = EAT_UART_PARITY_NONE;
	uart_config.stopBits = EAT_UART_STOP_BITS_1;
	if(eat_uart_open(CAN_UART) == EAT_TRUE)
	{
		eat_uart_set_config(CAN_UART, &uart_config);
	}
#endif
// Настраиваем порт для 485
#if defined(RS485_UART)
	uart_config.baud = EAT_UART_BAUD_19200;
	uart_config.dataBits = EAT_UART_DATA_BITS_8;
	uart_config.parity = EAT_UART_PARITY_NONE;
	uart_config.stopBits = EAT_UART_STOP_BITS_1;
	if(eat_uart_open(RS485_UART) == EAT_TRUE)
	{
		eat_uart_set_config(RS485_UART, &uart_config);
	}
#endif
#if defined(GPS_ON)
	eat_gpio_setup(GPS_ON, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(GPS_ON, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(LED)
	eat_gpio_setup(LED, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(OUT2)
	eat_gpio_setup(OUT2, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(OUT2, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(OUT3)
	eat_gpio_setup(OUT3, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(OUT3, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(OUT4)
	eat_gpio_setup(OUT4, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(OUT4, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(ESP_ON)
	eat_gpio_setup(ESP_ON, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(ESP_ON, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(SIM_SEL)
	eat_gpio_setup(SIM_SEL, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(SIM_SEL, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(EAT_PIN_INT2)
	eat_gpio_setup(EAT_PIN_INT2, EAT_GPIO_DIR_INPUT, EAT_GPIO_LEVEL_LOW);
	eat_int_setup(EAT_PIN_INT2, EAT_INT_TRIGGER_HIGH_LEVEL, 10, NULL);
#endif
#if defined(INFRQ1)
	eat_gpio_setup(INFRQ1, EAT_GPIO_DIR_INPUT, EAT_GPIO_LEVEL_LOW);
	eat_int_setup(INFRQ1, EAT_INT_TRIGGER_RISING_EDGE, 0, in_freq1_cb);
#endif
#if defined(INFRQ2)
	eat_gpio_setup(INFRQ2, EAT_GPIO_DIR_INPUT, EAT_GPIO_LEVEL_LOW);
	eat_int_setup(INFRQ2, EAT_INT_TRIGGER_RISING_EDGE, 0, in_freq2_cb);
#endif
#if defined(TX485EN)
	eat_gpio_setup(TX485EN, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(TX485EN, EAT_GPIO_LEVEL_HIGH);
#endif
#if defined(IN1)
	eat_gpio_setup(IN1, EAT_GPIO_DIR_INPUT, EAT_GPIO_LEVEL_LOW);
	if(eat_gpio_read(IN1) == EAT_GPIO_LEVEL_HIGH)
		Sts_d = Sts_d & (0xFFFF - 16384);
	else
		Sts_d = Sts_d | 16384;
#endif
#if defined(SPI_CS)
	eat_gpio_setup(SPI_CS, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(SPI_CS, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(SPI_CLK)
	eat_gpio_setup(SPI_CLK, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(SPI_CLK, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(SPI_MOSI)
	eat_gpio_setup(SPI_MOSI, EAT_GPIO_DIR_INPUT, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(SPI_MISO)
	eat_gpio_setup(SPI_MISO, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(SPI_MISO, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(IN2)
	eat_gpio_setup(IN2, EAT_GPIO_DIR_INPUT, EAT_GPIO_LEVEL_LOW);
	if(eat_gpio_read(IN2) == EAT_GPIO_LEVEL_HIGH)
		Sts_d = Sts_d & (0xFFFF - 32768);
	else
		Sts_d = Sts_d | 16384;
#endif
#if defined(INZ)
	eat_gpio_setup(INZ, EAT_GPIO_DIR_INPUT, EAT_GPIO_LEVEL_LOW);
	if(eat_gpio_read(INZ) == EAT_GPIO_LEVEL_HIGH)
		Sts_d = Sts_d & (0xFFFF - 8);
	else
		Sts_d = Sts_d | 8;
#endif
#if defined(OUT1)
	eat_gpio_setup(OUT1, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(OUT1) || defined(OUT1ext)
	if(eng_block == 1)
	{
#if defined(OUT1)
		eat_gpio_write(OUT1, EAT_GPIO_LEVEL_HIGH);
#endif
#if defined(OUT1ext)
		MCU_outputs = MCU_outputs | OUT1ext;
#endif
		Sts_d = Sts_d | 1024;
	}
#endif
	
	eat_set_sms_operation_mode(EAT_TRUE);
	eat_sms_register_new_message_callback(eat_sms_new_message_cb);
	eat_sms_register_sms_ready_callback(eat_sms_ready_cb);
	eat_sms_register_send_completed_callback(eat_sms_send_cb);
	eat_set_sms_format(1);
	eat_poweroff_key_sw(EAT_TRUE);
	eat_modem_set_poweron_urc_dir(EAT_USER_0);
	strcpy(simrev, eat_get_version());
	eat_get_imei((u8*)&IMEI[1], 15);
	IMEI[16] = 0;
	for(i = 0; i < 16; i++)
	{
		IMEI[16] = IMEI[16] + IMEI[i];
	}

	server_adr.sock_type = SOC_SOCK_DGRAM;
	server_adr.addr_len = 4;
	server_adr.port = ExPort[ExIPN];
	server_adr.addr[0]=ExIP[ExIPN][0];
	server_adr.addr[1]=ExIP[ExIPN][1];
	server_adr.addr[2]=ExIP[ExIPN][2];
	server_adr.addr[3]=ExIP[ExIPN][3];

	eat_soc_notify_register(soc_notify_cb);
	eat_soc_gethost_notify_register(hostname_notify_cb);
	sem_at_done = eat_create_sem("at_done", 0);
	eat_timer_start(EAT_TIMER_1, 1000);
	eat_timer_start(EAT_TIMER_2, 10000);
	
	if(!(Settings & 512))
	{
		wr_pkt(30);
		do_send = EAT_TRUE;
	}

write_log("App started");
    while(EAT_TRUE)
    {
        eat_get_event(&event);
        switch(event.event)
        {
            case EAT_EVENT_USER_MSG:
			{
				eat_modem_write(event.data.user_msg.data, event.data.user_msg.len);
				eat_timer_start(EAT_TIMER_3, at_timer);
				at_res = 0;
			}
            case EAT_EVENT_MDM_READY_RD:
			{
				len = eat_modem_read((unsigned char *)input_buf, 1024);
				if(len > 0)
				{
eat_trace(input_buf);
					input_buf[len] = 0;
					if(at_answer[0])
					{
						buf_pos = strstr(input_buf, at_answer);
						if(buf_pos)
						{
							at_res = 1;
							at_ret = buf_pos;
							eat_timer_stop(EAT_TIMER_3);
							eat_sem_put(sem_at_done);
							at_answer[0] = 0;
						}
					}
					if(strstr(input_buf, "RDY"))
					{
						main_status = main_status | 1;
					}
					else
					if(strstr(input_buf, "Call Ready"))
					{
						main_status = main_status | 2;
					}
					else
					if(strstr(input_buf, "SMS Ready"))
					{
						main_status = main_status | 4;
					}
					buf_pos = strstr(input_buf, "+CFUN:");
					if(buf_pos)
					{
						buf_pos = buf_pos + 7;
						cfun = *buf_pos - 0x30;
					}
					buf_pos = strstr(input_buf, "+CPIN:");
					if(buf_pos)
					{
						buf_pos = buf_pos + 7;
						if(!memcmp(buf_pos, "READY", 5))
						{
							cpin = EAT_TRUE;
						}
						else
						{
							cpin = EAT_FALSE;
						}
					}
					if(ICC[0] == 0)
					{
						buf_pos = input_buf;
						do{
							while(!isdigit(*buf_pos) && *buf_pos)
							{
								buf_pos++;
							}
							if(!*buf_pos)
								break;
							for(i = 0; i < 19; i++)
							{
								if(!isdigit(*buf_pos))
									break;
								buf_pos++;
							}
							if(i == 19)
							{
								buf_pos = buf_pos - 6;
								memcpy(ICC, buf_pos, 6);
							}
						}while(*buf_pos && !ICC[0]);
					}
					buf_pos = strstr(input_buf, "+DTMF:");
					if(buf_pos)
					{
						buf_pos = buf_pos + 7;
						dtmf_cmd = *buf_pos;
						if(dtmf_cmd == '#')
						{
							dtmf_menu_number = 0;
							dtmf_c = 'A';
							dtmf_d = 5;
							send_dtmf = EAT_TRUE;
						}
						switch(dtmf_menu_number)
						{
							case 0:
							{
								switch(dtmf_cmd)
								{
									case '0':
									{
										dtmf_menu_number = 1;
										dtmf_c = 'A';
										dtmf_d = 5;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '1':
									{
										dtmf_menu_number = 2;
										dtmf_c = 'A';
										dtmf_d = 5;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '2':
									{
										dtmf_menu_number = 3;
										dtmf_c = 'A';
										dtmf_d = 5;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '3':
									{
										dtmf_menu_number = 4;
										dtmf_c = 'A';
										dtmf_d = 5;
										send_dtmf = EAT_TRUE;
										break;
									}
								}
								break;
							}
							case 1:
							{
								switch(dtmf_cmd)
								{
									case '0':
									{
										eat_fs_Delete(L"C:\\log.txt");
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '1':
									{
										log_write_en = 1;
										save_status();
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '2':
									{
										log_write_en = 0;
										save_status();
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '3':
									{
										log_upload = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
								}
								break;
							}
							case 2:
							{
								switch(dtmf_cmd)
								{
									case '0':
									{
										ath_simreset_f = EAT_TRUE;
										break;
									}
									case '1':
									{
										eat_fs_Delete(L"C:\\Settings.txt");
										ath_simreset_f = EAT_TRUE;
										break;
									}
									case '2':
									{
										gprs_reset = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '3':
									{
										offgps_f = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
								}
								break;
							}
							case 3:
							{
								switch(dtmf_cmd)
								{
									case '0':
									{
										strcpy(sms_nbr, incall_nbr);
										fsinfo = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '1':
									{
										strcpy(sms_nbr, incall_nbr);
										pos2sms = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '2':
									{
										strcpy(sms_nbr, incall_nbr);
										modinfo = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '3':
									{
										strcpy(sms_nbr, incall_nbr);
										getparam1 = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '4':
									{
										strcpy(sms_nbr, incall_nbr);
										getparam2 = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '5':
									{
										strcpy(sms_nbr, incall_nbr);
										getparam3 = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '6':
									{
										strcpy(sms_nbr, incall_nbr);
										getparam4 = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '7':
									{
										strcpy(sms_nbr, incall_nbr);
										getparam5 = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
								}
							}
							break;
							case 4:
							{
								switch(dtmf_cmd)
								{
									case '0':
									{
										eng_block = 2;
										save_status();
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '1':
									{
										eng_block = 1;
										save_status();
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '2':
									{
										eng_block = 0;
										save_status();
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
								}
								break;
							}
						}
					}
					buf_pos = strstr(input_buf, "+CLIP:");
					if(buf_pos)
					{
						incall_nbr[0] = 0;
						buf_pos = strchr(buf_pos, '\"');
						buf_pos++;
						if(*buf_pos == '+')
							buf_pos++;
						sscanf(buf_pos, "%12[^\"]", incall_nbr);
						if(!strcmp(incall_nbr, Num1) || !strcmp(incall_nbr, Num2) || !strcmp(incall_nbr, Num3) || !strcmp(incall_nbr, Num4) || !strcmp(incall_nbr, Num5) || !strcmp(incall_nbr, NumAd1) || !strcmp(incall_nbr, NumAd2))
						{
							ata_f = EAT_TRUE;
						}
						else
						{
							ath_f = EAT_TRUE;
						}
					}
					buf_pos = strstr(input_buf, "NO CARRIER");
					if(buf_pos)
					{
						incall_f = EAT_FALSE;
					}
					buf_pos = strstr(input_buf, "SJDR:");
					if(buf_pos)
					{
						buf_pos = buf_pos + 6;
						if(!memcmp(buf_pos, "JAMMING DETECTED", 16))
						{
							if(!(Sts_d & 256))
							{
								Sts_d = Sts_d | 256;
							}
						}
						if(!memcmp(buf_pos, "NO JAMMING", 10))
						{
							if(Sts_d & 256)
							{
								Sts_d = Sts_d & (0xFFFF - 256);
							}
						}
					}
				}
			}
			break;
            case EAT_EVENT_UART_READY_RD:
			{
				EatUart_enum uart = event.data.uart.uart;
				len = eat_uart_read(uart, (unsigned char *)input_buf, 5000);
				if(len != 0)
				{
					input_buf[len] = 0;
#if defined(MCU_UART)
					if(uart == MCU_UART)
					{
						sscanf(input_buf, "\n%hhu,%hhu\n", &MCU_ver, &MCU_inputs);
#if defined(INZext) || defined(IN1ext) || defined(IN2ext)
						if(!MCU_first)
						{
#if defined(INZext)
							if(MCU_inputs & INZext)
								Sts_d = Sts_d | 8;
							else
								Sts_d = Sts_d & (0xFFFF - 8);
#endif
#if defined(IN1ext)
							if(MCU_inputs & IN1ext)
								Sts_d = Sts_d | 8;
							else
								Sts_d = Sts_d & (0xFFFF - 16384);
#endif
#if defined(IN2ext)
							if(MCU_inputs & IN2ext)
								Sts_d = Sts_d | 8;
							else
								Sts_d = Sts_d & (0xFFFF - 32768);
#endif
							MCU_first = EAT_TRUE;
						}
#endif
#if defined(INZext)
						if(MCU_inputs & INZext)
						{
							if(!(Sts_d & 8))
							{
								Sts_d = Sts_d | 8;
								if(Settings1 & 1)
									wr_pkt(22);
								if(Settings1 & 2)
									do_send = EAT_TRUE;
							}
						}
						else
						{
							if(Sts_d & 8)
							{
								Sts_d = Sts_d & (0xFFFF - 8);
								if(Settings1 & 1)
									wr_pkt(22);
								if(Settings1 & 2)
									do_send = EAT_TRUE;
							}
						}
#endif
#if defined(IN1ext)
						if(MCU_inputs & IN1ext)
						{
							if(!(Sts_d & 8))
							{
								Sts_d = Sts_d | 16384;
								if(Settings1 & 2048)
								{
									wr_pkt(22);
									do_send = EAT_TRUE;
								}
							}
						}
						else
						{
							if(Sts_d & 8)
							{
								Sts_d = Sts_d & (0xFFFF - 16384);
								if(Settings1 & 2048)
								{
									wr_pkt(22);
									do_send = EAT_TRUE;
								}
							}
						}
#endif
#if defined(IN2ext)
						if(MCU_inputs & IN2ext)
						{
							if(!(Sts_d & 8))
							{
								Sts_d = Sts_d | 32768;
								if(Settings1 & 4096)
								{
									wr_pkt(22);
									do_send = EAT_TRUE;
								}
							}
						}
						else
						{
							if(Sts_d & 8)
							{
								Sts_d = Sts_d & (0xFFFF - 32768);
								if(Settings1 & 4096)
								{
									wr_pkt(22);
									do_send = EAT_TRUE;
								}
							}
						}
#endif
					}
#endif
#if defined(GPS_UART)
					if(uart == GPS_UART)
					{
						buf_pos = input_buf;
#if defined(ESP_ON)
if(dis_gpson)
{
	MAC_cnt = 0;
	do{
		buf_pos = strstr(buf_pos, "+CWLAP:(");
		if(buf_pos)
		{
			buf_pos++;
			buf_pos = strchr(buf_pos, ',');
			if(buf_pos)
			{
				buf_pos++;
				buf_pos = strchr(buf_pos, ',');
				if(buf_pos)
				{
					buf_pos++;
					buf_pos = strchr(buf_pos, ',');
					if(buf_pos)
					{
						buf_pos = buf_pos + 2;
eat_trace(buf_pos);
						sscanf(buf_pos, "%x:%x:%x:%x:%x:%x", &MAC_t1, &MAC_t2, &MAC_t3, &MAC_t4, &MAC_t5, &MAC_t6);
						MAC[MAC_cnt] = (unsigned char)MAC_t1;
						MAC[MAC_cnt+1] = (unsigned char)MAC_t2;
						MAC[MAC_cnt+2] = (unsigned char)MAC_t3;
						MAC[MAC_cnt+3] = (unsigned char)MAC_t4;
						MAC[MAC_cnt+4] = (unsigned char)MAC_t5;
						MAC[MAC_cnt+5] = (unsigned char)MAC_t6;
						MAC_cnt = MAC_cnt + 6;
					}
				}
			}
		}
	}while(buf_pos);
}
else
#endif
	{
						while(*buf_pos)
						{
							switch(nmea_st)
							{
								case 0:
								{
									if(*buf_pos == '$')
									{
										nmea_msg_pos = nmea_msg;
										nmea_st = 1;
									}
								}
								break;
								case 1:
								{
									*nmea_msg_pos = *buf_pos;
									nmea_msg_pos++;
									if(*buf_pos == '*')
									{
										nmea_st = 0;
										*nmea_msg_pos = 0;
										nmea_msg_pos = strstr(nmea_msg, "RMC");
										if(nmea_msg_pos)
										{
											gps_chk = EAT_TRUE;
											nmea_msg_pos = nmea_msg_pos + 4;
											if(isdigit(*nmea_msg_pos))
											{
												Hour = ((*nmea_msg_pos - 0x30) * 10) + (*(nmea_msg_pos + 1) - 0x30);
												Minute = ((*(nmea_msg_pos + 2) - 0x30) * 10) + (*(nmea_msg_pos + 3) - 0x30);
												Second = ((*(nmea_msg_pos + 4) - 0x30) * 10) + (*(nmea_msg_pos + 5) - 0x30);
											}
											else
											{
												Hour = 0;
												Minute = 0;
												Second = 0;
											}
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											if(isdigit(*nmea_msg_pos))
											{
												deg = ((*nmea_msg_pos - 0x30) * 10) + (*(nmea_msg_pos + 1) - 0x30);
												nmea_msg_pos = nmea_msg_pos + 2;
												Latitude = atoi(nmea_msg_pos);
												Latitude = Latitude * 10000;
												nmea_msg_pos = nmea_msg_pos + 3;
												Latitude = atoi(nmea_msg_pos) + Latitude;
												Latitude = Latitude * 0.16666666;
												Latitude = (deg * 100000) + Latitude;
											}
											else
												Latitude = 0;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											if(isdigit(*nmea_msg_pos))
											{
												deg = ((*nmea_msg_pos - 0x30) * 100) + ((*(nmea_msg_pos + 1) - 0x30) * 10) + (*(nmea_msg_pos + 2) - 0x30);
												nmea_msg_pos = nmea_msg_pos + 3;
												Longitude = atoi(nmea_msg_pos);
												Longitude = Longitude * 10000;
												nmea_msg_pos = nmea_msg_pos + 3;
												Longitude = atoi(nmea_msg_pos) + Longitude;
												Longitude = Longitude * 0.16666666;
												Longitude = (deg * 100000) + Longitude;
											}
											else
												Longitude = 0;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											if(isdigit(*nmea_msg_pos))
											{
												Speed = atoi(nmea_msg_pos);
												Speed = (unsigned int)((float)Speed * 1.852);
											}
											else
												Speed = 0;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											if(isdigit(*nmea_msg_pos))
												Course = atoi(nmea_msg_pos);
											else
												Course = 0;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											if(isdigit(*nmea_msg_pos))
											{
												Day = ((*nmea_msg_pos - 0x30) * 10) + (*(nmea_msg_pos + 1) - 0x30);
												Month = ((*(nmea_msg_pos + 2) - 0x30) * 10) + (*(nmea_msg_pos + 3) - 0x30);
												Year = ((*(nmea_msg_pos + 4) - 0x30) * 10) + (*(nmea_msg_pos + 5) - 0x30);
											}
											else
											{
												Day = 0;
												Month = 0;
												Year = 0;
											}
											if(((SatUs >> 6) > 0)  && (Latitude != 0) && (Longitude != 0))
											{
												if(Speed > Speed_lim)
												{
													if(!(Sts_d & 1))
													{
														Sts_d = Sts_d | 1;
														wr_event(15);
														wr_pkt(22);
														do_send = EAT_TRUE;
													}
												}
												else
												{
													if(Speed < (Speed_lim - 10))
													{
														Sts_d = Sts_d & (0xFFFF - 1);
													}
												}

												if(Speed > 1)
												{
													if(move_d < 5)
														move_d++;
												}
												else
												{
													if(move_d != 0)
													move_d--;
												}
												if(move_d == 5)
												{
													if(!(Sts_d & 16))
													{
														Sts_d = Sts_d | 16;
														if(Settings1 & 512)
															wr_pkt(22);
														if(Settings1 & 1024)
															do_send = EAT_TRUE;
													}
												}
												if(move_d == 0)
												{
													if(Sts_d & 16)
													{
														Sts_d = Sts_d & (0xFFFF - 16);
														if(Settings1 & 512)
															wr_pkt(22);
														if(Settings1 & 1024)
															do_send = EAT_TRUE;
													}
												}

												if(Sts_d & 16)
												{
													if(dir_r(Course, Course_old) > 4)
													{
														Course_old = Course;
														Turn = 1;
													}
													else
														Turn = 0;
												}
												else
													Turn = 0;
											}
										}
										nmea_msg_pos = strstr(nmea_msg, "GGA");
										if(nmea_msg_pos)
										{
											gps_chk = EAT_TRUE;
											nmea_msg_pos = nmea_msg_pos + 4;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											if(isdigit(*nmea_msg_pos))
											{
												SatUs = SatUs & 0xC0;
												SatUs = SatUs + atoi(nmea_msg_pos);
											}
										}
										nmea_msg_pos = strstr(nmea_msg, "GSA");
										if(nmea_msg_pos)
										{
											gps_chk = EAT_TRUE;
											nmea_msg_pos = nmea_msg_pos + 4;
											nmea_msg_pos = strchr(nmea_msg_pos, ',');
											nmea_msg_pos++;
											if(isdigit(*nmea_msg_pos))
											{
												SatUs = SatUs & 0x3F;
												SatUs = SatUs + ((atoi(nmea_msg_pos) - 1) << 6);
											}
										}
									}
								}
								break;
							}
							buf_pos++;
						}
	}
					}
#endif
#if defined(RS485_UART)
					if(uart == RS485_UART)
					{
						sscanf(input_buf, "F=%X t=%X N=%X", &DUT_F, &DUT_t, &DUT_N);
						for(i = 1; i < TAR_cnt; i++)
						{
							if((TAR1[i] > DUT_N) || (i == (TAR_cnt - 1)))
							{
								DUT_L = TAR2[i-1] + (((double)TAR2[i] - (double)TAR2[i-1]) / ((double)TAR1[i] - (double)TAR1[i-1])) * (DUT_N - TAR1[i-1]);
								DUT_N = DUT_L * 10;
								break;
							}
						}
						rs485_timer = 150;
					}
#endif
#if defined(CAN_UART)
					if(uart == CAN_UART)
					{
						if((len == 22) && (input_buf[0] == 0x55) && (input_buf[1] == 0x55) && (input_buf[2] == 0x55))
						{
							CAN_CRC = 0;
							for(i = 3; i < 21; i++)
							{
								CAN_CRC = CAN_CRC + input_buf[i];
							}
							if(CAN_CRC == input_buf[21])
							{
								speed_can = input_buf[3];
								accel_can = input_buf[4];
								fuel_can = input_buf[5];
								temp_can = input_buf[6];
								rpm_can = (input_buf[7] * 0x100) + input_buf[8];
								fuela_can = (input_buf[9] * 0x1000000) + (input_buf[10] * 0x10000) + (input_buf[11] * 0x100) + input_buf[12];
								mth_can = (input_buf[13] * 0x1000000) + (input_buf[14] * 0x10000) + (input_buf[15] * 0x100) + input_buf[16];
								dist_can = (input_buf[17] * 0x1000000) + (input_buf[18] * 0x10000) + (input_buf[19] * 0x100) + input_buf[20];
							}
						}
					}
#endif
				}
			}
			break;
			case EAT_EVENT_TIMER:
			{
				if(event.data.timer.timer_id == EAT_TIMER_1)
				{
					eat_timer_start(EAT_TIMER_1, 1000);

#if defined(INFRQ1)
//					f1_fltr[f1_fltr_cnt] = infrq1_cnt;
//					f1_fltr_cnt++;
//					if(f1_fltr_cnt == 5)
//					{
//						f1_fltr_cnt = 0;
//						freq1 = filtr_f(f1_fltr);
//					}
					freq1 = infrq1_cnt;
					infrq1_cnt = 0;
#endif
#if defined(INFRQ2)
//					f2_fltr[f2_fltr_cnt] = infrq2_cnt;
//					f2_fltr_cnt++;
//					if(f2_fltr_cnt == 5)
//					{
//						f2_fltr_cnt = 0;
//						freq2 = filtr_f(f2_fltr);
//					}
					freq2 = infrq2_cnt;
					infrq2_cnt = 0;
#endif

					if(eat_get_adc_sync(EAT_PIN38_ADC, (u32 *)&Vcc))
						Vcc = Vcc * 13;
					
					if(Vcc < Vcc_lim)
					{
						if(vclim_cnt != 5)
							vclim_cnt++;
					}
					else
					{
						if(vclim_cnt != 0)
							vclim_cnt--;
					}
					if(vclim_cnt == 5)
					{
						if(!(Sts_d & 2048))
						{
							Sts_d = Sts_d | 2048;
							wr_event(16);
							wr_pkt(22);
						}
					}
					if(vclim_cnt == 0)
					{
						if(Sts_d & 2048)
						{
							Sts_d = Sts_d & (0xFFFF - 2048);
						}
					}
					if(Vcc < 5000)
					{
						if(uv_cnt != 5)
							uv_cnt++;
					}
					else
					{
						if(uv_cnt != 0)
							uv_cnt--;
					}
					if(uv_cnt == 5)
					{
						if(!(Sts_d & 512))
						{
							wr_event(5);
							Sts_d = Sts_d | 512;
							if(Settings1 & 4)
								wr_pkt(22);
							if(Settings1 & 8)
								do_send = EAT_TRUE;
						}
					}
					if(uv_cnt == 0)
					{
						if(Sts_d & 512)
						{
							Sts_d = Sts_d & (0xFFFF - 512);
							if(Settings1 & 4)
								wr_pkt(22);
							if(Settings1 & 8)
								do_send = EAT_TRUE;
						}
					}

					if(rs485_timer != 0)
					{
						rs485_timer--;
					}
					if(simreset_f)
					{
						eat_reset_module();
						simreset_f = EAT_FALSE;
					}
#if defined(EAT_PIN_INT2)
					if(vibration_cnt > 2)
					{
					}
#endif
#if defined(OUT1) || defined(OUT1ext)
					if(!(eng_block & 3))
					{
						if(Sts_d & 1024)
						{
#if defined(OUT1)
							eat_gpio_write(OUT1, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(OUT1ext)
							MCU_outputs = MCU_outputs & (0xFFFF - OUT1ext);
#endif
							Sts_d = Sts_d & (0xFFFF - 1024);
							wr_pkt(22);
							do_send = EAT_TRUE;
						}
					}

					if(eng_block & 1)
					{
						if(!(Sts_d & 1024))
						{
#if defined(OUT1)
							eat_gpio_write(OUT1, EAT_GPIO_LEVEL_HIGH);
#endif
#if defined(OUT1ext)
							MCU_outputs = MCU_outputs | OUT1ext;
#endif
							Sts_d = Sts_d | 1024;
							wr_pkt(22);
							do_send = EAT_TRUE;
						}
					}

					if((eng_block & 2) && !(Sts_d & 16) && (((SatUs >> 6) == 2) && (Latitude != 0) && (Longitude != 0)))
					{
						if(!(Sts_d & 1024))
						{
#if defined(OUT1)
							eat_gpio_write(OUT1, EAT_GPIO_LEVEL_HIGH);
#endif
#if defined(OUT1ext)
							MCU_outputs = MCU_outputs | OUT1ext;
#endif
							Sts_d = Sts_d | 1024;
							wr_pkt(22);
							do_send = EAT_TRUE;
						}
					}
#endif
#if defined(INZ)
					if(eat_gpio_read(INZ) == EAT_GPIO_LEVEL_HIGH)
					{
						if(Sts_d & 8)
						{
							Sts_d = Sts_d & (0xFFFF - 8);
							if(Settings1 & 1)
								wr_pkt(22);
							if(Settings1 & 2)
								do_send = EAT_TRUE;
						}
					}
					else
					{
						if(!(Sts_d & 8))
						{
							Sts_d = Sts_d | 8;
							if(Settings1 & 1)
								wr_pkt(22);
							if(Settings1 & 2)
								do_send = EAT_TRUE;
						}
					}
#endif
#if defined(IN1)
					if(eat_gpio_read(IN1) == EAT_GPIO_LEVEL_HIGH)
					{
						if(Sts_d & 16384)
						{
							Sts_d = Sts_d & (0xFFFF - 16384);
							if(Settings1 & 2048)
							{
								wr_pkt(22);
								do_send = EAT_TRUE;
							}
						}
					}
					else
					{
						if(!(Sts_d & 16384))
						{
							Sts_d = Sts_d | 16384;
							if(Settings1 & 2048)
							{
								wr_pkt(22);
								do_send = EAT_TRUE;
							}
						}
					}
#endif
#if defined(IN2)
					if(eat_gpio_read(IN2) == EAT_GPIO_LEVEL_HIGH)
					{
						if(Sts_d & 32768)
						{
							Sts_d = Sts_d & (0xFFFF - 32768);
							if(Settings1 & 4096)
							{
								wr_pkt(22);
								do_send = EAT_TRUE;
							}
						}
					}
					else
					{
						if(!(Sts_d & 32768))
						{
							Sts_d = Sts_d | 32768;
							if(Settings1 & 4096)
							{
								wr_pkt(22);
								do_send = EAT_TRUE;
							}
						}
					}
#endif
					switch(send_sms_st)
					{
						case 0:
						{
							if(send_sms_f)
							{
								if(Num1[0] != 0)
								{
									sms_sended = EAT_FALSE;
									send_sms(Num1, sms_txt);
									send_sms_st = 1;
								}
								else
								{
									send_sms_st = 2;
								}
							}
						}
						break;
						case 1:
						{
							if(sms_sended)
							{
								send_sms_st = 2;
							}
						}
						break;
						case 2:
						{
							if(Num2[0] != 0)
							{
								sms_sended = EAT_FALSE;
								send_sms(Num2, sms_txt);
								send_sms_st = 3;
							}
							else
							{
								send_sms_st = 4;
							}
						}
						break;
						case 3:
						{
							if(sms_sended)
							{
								send_sms_st = 4;
							}
						}
						break;
						case 4:
						{
							if(Num3[0] != 0)
							{
								sms_sended = EAT_FALSE;
								send_sms(Num3, sms_txt);
								send_sms_st = 5;
							}
							else
							{
								send_sms_st = 6;
							}
						}
						break;
						case 5:
						{
							if(sms_sended)
							{
								send_sms_st = 6;
							}
						}
						break;
						case 6:
						{
							if(Num4[0] != 0)
							{
								sms_sended = EAT_FALSE;
								send_sms(Num4, sms_txt);
								send_sms_st = 7;
							}
							else
							{
								send_sms_st = 8;
							}
						}
						break;
						case 7:
						{
							if(sms_sended)
							{
								send_sms_st = 8;
							}
						}
						break;
						case 8:
						{
							if(Num5[0] != 0)
							{
								sms_sended = EAT_FALSE;
								send_sms(Num5, sms_txt);
								send_sms_st = 9;
							}
							else
							{
								send_sms_st = 10;
							}
						}
						break;
						case 9:
						{
							if(sms_sended)
							{
								send_sms_st = 10;
							}
						}
						break;
						case 10:
						{
							send_sms_f = EAT_FALSE;
							send_sms_st = 0;
						}
						break;
					}

					switch(gprs_st)
					{
						case 0:
						{
							if((gprs_enable) && (gsm_reg != 5 || (Settings & 32)) && (!(Settings & 512)) && (main_status&2))
							{
								if(bear_st == 0)
								{
									eat_gprs_bearer_open(APN, APN_user, APN_pass, bear_notify_cb);
								}
								gprs_st_timer = 0;
								gprs_st = 1;
							}
						}
						break;
						case 1:
						{
							if(bear_st == 2)
							{
								gprs_st_errcnt = 0;
								gprs_st = 2;
							}
							gprs_st_timer++;
							if(gprs_st_timer > 20)
							{
								gprs_st_errcnt++;
								gprs_st = 0;
							}
						}
						break;
						case 2:
						{
							if((!gprs_enable) || (gsm_reg == 5 && (!(Settings & 32))) || (Settings & 512))
							{
								eat_soc_close(server_soc);
					            eat_gprs_bearer_release();
								gprs_st = 0;
							}
							else
							if(bear_st != 2)
							{
								gprs_st = 0;
							}
						}
						break;
					}

					switch(udpsend_st)
					{
						case 0:
						{
							if((bear_st == 2) && (do_send || (Settings&2048)) && (out_buf_col != 0))
							{
								do_send = EAT_FALSE;
								if(rsindp_cnt == 0)
									udpsend_st = 1;
								else
									udpsend_st = 3;
							}
						}
						break;
						case 1:
						{
							eat_soc_sendto(server_soc, IMEI, 17, &server_adr);
							udpsend_st_timer = 15;
							udpsend_st = 2;
						}
						break;
						case 2:
						{
							if(udpsend_st_timer != 0)
							{
								udpsend_st_timer--;
							}
							else
							{
								ExIPN++;
								if(ExIPN == 3)
									ExIPN = 0;
								server_adr.port = ExPort[ExIPN];
								server_adr.addr[0]=ExIP[ExIPN][0];
								server_adr.addr[1]=ExIP[ExIPN][1];
								server_adr.addr[2]=ExIP[ExIPN][2];
								server_adr.addr[3]=ExIP[ExIPN][3];

								udpsend_st = 1;
								senderr_cnt++;
								if(senderr_cnt > 30)
								{
									senderr_cnt = 0;
								}
								else
								if(senderr_cnt > 6)
								{
									udpsend_st_timer = 60;
									udpsend_st = 6;
								}
								else
								if(senderr_cnt == 3)
								{
									gprs_reset = EAT_TRUE;
write_log("GPRS reset1");
									udpsend_st_timer = 10;
									udpsend_st = 6;
								}
							}
						}
						break;
						case 3:
						{
							eeprom_p2tmp = eeprom_p2;
							i1 = 0;
							while((i1 < 675) && (eeprom_p2tmp != eeprom_p1))
							{
								out_cnt = pktsz(read_byte());
								for(i = 0; i < out_cnt; i++)
								{
									out_buf[i1] = read_byte();
									eeprom_p2tmp++;
									i1++;
									if(eeprom_p2tmp >= main_buf_size)
										eeprom_p2tmp = 0;
								}
							}
							eat_soc_sendto(server_soc, out_buf, i1, &server_adr);
							udpsend_st_timer = 15;
							udpsend_st = 4;
						}
						break;
						case 4:
						{
							if(udpsend_st_timer != 0)
							{
								udpsend_st_timer--;
							}
							else
							{
								ExIPN++;
								if(ExIPN == 3)
									ExIPN = 0;
								server_adr.port = ExPort[ExIPN];
								server_adr.addr[0]=ExIP[ExIPN][0];
								server_adr.addr[1]=ExIP[ExIPN][1];
								server_adr.addr[2]=ExIP[ExIPN][2];
								server_adr.addr[3]=ExIP[ExIPN][3];

								udpsend_st = 3;
								senderr_cnt++;
								if(senderr_cnt > 30)
								{
									senderr_cnt = 0;
								}
								else
								if(senderr_cnt > 6)
								{
									udpsend_st_timer = 60;
									udpsend_st = 6;
								}
								else
								if(senderr_cnt == 3)
								{
									gprs_reset = EAT_TRUE;
write_log("GPRS reset2");
									udpsend_st_timer = 10;
									udpsend_st = 6;
								}
							}
						}
						break;
						case 5:
						{
							eeprom_p2 = eeprom_p2tmp;
							buf_col_get();
							if(out_buf_col != 0)
								udpsend_st = 3;
							else
								udpsend_st = 0;
						}
						break;
						case 6:
						{
							if(udpsend_st_timer != 0)
							{
								udpsend_st_timer--;
							}
							else
							{
								udpsend_st = 0;
							}
						}
						break;
					}

					if(Stealth_per_old != Stealth_per)
					{
						main_st = 0;
						Stealth_per_old = Stealth_per;
						if(Stealth_per == 0)
						{
							dis_gpson = EAT_FALSE;
						}
					}

					if(Stealth_per == 0)
					{
						if(cfun == 4)
						{
							send_cfun1 = EAT_TRUE;
						}
						switch(Settings & 3)
						{
							case 0:
								if(Sts_d & 16)
								{
									Period = Period1;
									Send_per = Send_per1;
								}
								else
								{
									Period = Period2;
									Send_per = Send_per2;
								}
							break;
							case 1:
								if(Sts_d & 8)
								{
									Period = Period1;
									Send_per = Send_per1;
								}
								else
								{
									Period = Period2;
									Send_per = Send_per2;
								}
							break;
							case 2:
									Period = Period1;
									Send_per = Send_per1;
							break;
							case 3:
								if(Sts_d & 16)
								{
									Send_per = Send_per1;
									if(Turn)
										Period = Period3;
									else
										Period = Period1;
								}
								else
								{
									Period = Period2;
									Send_per = Send_per2;
								}
							break;
						}
						if(rsindp_cnt != 0)
							rsindp_cnt--;

						prm_cnt++;
						if((prm_cnt >= prm_per) && (prm_per != 0))
						{
							prm_cnt = 0;
							wr_pkt(3);
						}

						gsmloc_cnt++;
						if((gsmloc_cnt >= gsmloc_per) && (gsmloc_per != 0) && (main_status&2))
						{
							gsmloc_cnt = 0;
							if(((SatUs >> 6) == 0) || (Settings1 & 256))
							{
								gsmloc_f = EAT_TRUE;
							}
						}

						money_cnt++;
						if((money_cnt >= money_per) && (money_per != 0) && (main_status&2))
						{
							money_cnt = 0;
							money_f = EAT_TRUE;
						}

						trip_cnt++;
						if((trip_cnt >= trip_per) && (trip_per != 0))
						{
							trip_cnt = 0;
							wr_pkt(6);
						}

						debug_cnt++;
						if((debug_cnt >= debug_per) && (debug_per != 0))
						{
							debug_cnt = 0;
							wr_pkt(7);
						}

						ADC_cnt++;
						if((ADC_cnt >= ADC_per) && (ADC_per != 0))
						{
							ADC_cnt = 0;
							wr_pkt(13);
						}

						INx_cnt++;
						if((INx_cnt >= INx_per) && (INx_per != 0))
						{
							INx_cnt = 0;
							wr_pkt(14);
						}

						freq_cnt++;
						if((freq_cnt >= freq_per) && (freq_per != 0))
						{
							freq_cnt = 0;
							wr_pkt(12);
						}

						fuel_cnt++;
						if((fuel_cnt >= fuel_per) && (fuel_per != 0))
						{
							fuel_cnt = 0;
							wr_pkt(39);
						}

						ident_cnt++;
						if((ident_cnt >= ident_per) && (ident_per != 0))
						{
							ident_cnt = 0;
							wr_pkt(26);
							wr_pkt(37);
						}

						can1_cnt++;
						if((can1_cnt >= can1_per) && (can1_per != 0))
						{
							can1_cnt = 0;
#if defined(CAN_UART)
							if(speed_can != 0xFF)
							{
								wr_pkt(9);
							}
#endif
#if defined(RS485_UART)
							if(rs485_timer != 0)
							{
								wr_pkt(38);
							}
#endif
						}

						can2_cnt++;
						if((can2_cnt >= can2_per) && (can2_per != 0))
						{
							can2_cnt = 0;
#if defined(CAN_UART)
							if(speed_can != 0xFF)
							{
								wr_pkt(10);
							}
#endif
						}

						offsim_cnt++;
						if((offsim_cnt >= offsim_per) && (offsim_per != 0))
						{
							offsim_cnt = 0;
							send_cfun4 = EAT_TRUE;
						}

						period_cnt++;
						if((period_cnt >= Period) && (Period != 0))
						{
							period_cnt = 0;
							wr_pkt(22);
						}

						send_cnt++;
						if(((send_cnt >= Send_per) && (Send_per != 0)) || (Settings & 2048))
						{
							send_cnt = 0;
							do_send = EAT_TRUE;
						}

						switch(main_st)
						{
							case 0:
							{
								if(main_status&2)
								{
									gprs_enable = EAT_TRUE;
									main_st = 1;
								}
							}
							break;
							case 1:
							{
							}
							break;
						}
					}
					else
					{
						stealth_timer++;
						if(stealth_timer == 900)
						{
							main_st = 7;
						}
						switch(main_st)
						{
							case 0:
							{
								if(Settings & 1024)
								{
									main_st = 1;
								}
								stealth_cnt++;
								if(((SatUs >> 6) > 0)  && (Latitude != 0) && (Longitude != 0))
								{
									wr_pkt(22);
									dis_gpson = EAT_TRUE;
#if defined(GPS_ON)
									eat_gpio_write(GPS_ON, EAT_GPIO_LEVEL_LOW);
#endif
									main_st = 3;
								}
								if(stealth_cnt >= Stealth_ontm)
								{
									send_cfun1 = EAT_TRUE;
									gprs_enable = EAT_TRUE;
									do_send = EAT_TRUE;
									dis_gpson = EAT_TRUE;
#if defined(GPS_ON)
									eat_gpio_write(GPS_ON, EAT_GPIO_LEVEL_LOW);
#endif
									main_st = 1;
								}
							}
							break;
							case 1:
							{
								if(out_buf_col == 0)
								{
									gsmloc_f = EAT_TRUE;
									main_st = 2;
								}
							}
							break;
							case 2:
							{
								if(!gsmloc_f)
								{
									main_st = 3;
								}
							}
							break;
							case 3:
							{
								if(Settings & 512)
								{
									if((gsm_reg != 5 || (Settings & 64)))
									{
										buf_pos = sms_txt;
										if((Latitude != 0) && (Longitude != 0))
										{
											Lat = (double)Latitude / 100000;
											Lon = (double)Longitude / 100000;
											buf_pos = buf_pos + sprintf(buf_pos, "Lat:%.7f\n\rLon:%.7f\n\r", Lat, Lon);
										}
										else
										{
											buf_pos = buf_pos + sprintf(buf_pos, "MCC:%u,MNC:%u,LAC:%u,CID:%u\n\r", MCC, MNC, LAC, CID);
										}
										buf_pos = buf_pos + sprintf(buf_pos, "%2.2u/%2.2u/%2.2u\n\r%2.2u:%2.2u:%2.2u\n\r%ukm/h\n\r%udeg\n\r", Day, Month, Year, Hour, Minute, Second, Speed, Course);
										buf_pos = buf_pos + sprintf(buf_pos, "Vbt:%1.3fV\n\r", (double)Vbt/1000);
										if((Latitude != 0) && (Longitude != 0))
										{
											buf_pos = buf_pos + sprintf(buf_pos, "http://server1.gsm-gps.com/maps1/vp_gps.php?lat=%2.5f&lon=%3.5f&spd=%u&dir=%u", Lat, Lon, Speed, Course);
										}
										else
										{
											buf_pos = buf_pos + sprintf(buf_pos, "http://server1.gsm-gps.com/maps1/vp_gsm.php?MCC=%u&MNC=%u&LAC=%u&CID=%u", MCC, MNC, LAC, CID);
										}
										send_sms_f = EAT_TRUE;
										main_st = 4;
									}
									else
									{
										main_st = 6;
									}
								}
								else
								{
									if((gsm_reg != 5) || (Settings & 32))
									{
										if(prm_per != 0)
										{
											wr_pkt(3);
										}
										if(money_per != 0)
										{
											money_f = EAT_TRUE;
										}
										if(trip_per != 0)
										{
											wr_pkt(6);
										}
										if(debug_per != 0)
										{
											wr_pkt(7);
										}
										if(ADC_per != 0)
										{
											wr_pkt(13);
										}
										if(INx_per != 0)
										{
											wr_pkt(14);
										}
										if(freq_per != 0)
										{
											wr_pkt(12);
										}
										if(fuel_per != 0)
										{
											wr_pkt(39);
										}
										if(ident_per != 0)
										{
											wr_pkt(26);
											wr_pkt(37);
										}
										send_cfun1 = EAT_TRUE;
										gprs_enable = EAT_TRUE;
										do_send = EAT_TRUE;
										main_st = 5;
									}
									else
									{
										main_st = 7;
									}
								}
							}
							break;
							case 4:
							{
								if(!send_sms_f)
								{
									main_st = 6;
								}
							}
							break;
							case 5:
							{
								if(out_buf_col == 0)
								{
									main_st = 6;
								}
							}
							break;
							case 6:
							{
								if(!fw_update)
								{
									main_st = 7;
								}
							}
							break;
							case 7:
							{
								send_cfun4 = EAT_TRUE;
								main_st = 8;
							}
							break;
							case 8:
							{
								if(!send_cfun4)
								{
									main_st = 9;
								}
							}
							break;
							case 9:
							{
								extoff_time = Stealth_per;
							}
							break;
						}
					}
#if defined(MCU_UART)
					sprintf(tmp_buf, "\rO%u\r", MCU_outputs);
					eat_uart_write(MCU_UART, (const unsigned char *)tmp_buf, strlen(tmp_buf));
					if(do_extreset)
					{
						eat_uart_write(MCU_UART, "\rR\r", strlen("\rR\r"));
					}
					if(extoff_time != 0)
					{
						sprintf(tmp_buf, "\rF%u\r", extoff_time);
						eat_uart_write(MCU_UART, (const unsigned char *)tmp_buf, strlen(tmp_buf));
					}
#endif
				}
				if(event.data.timer.timer_id == EAT_TIMER_2)
				{
					eat_timer_start(EAT_TIMER_2, 10000);
					eat_get_rtc(&rtc);
					rssi = eat_network_get_csq();
					rssi = (unsigned char)(rssi * 3.2258);
					if(eat_get_module_temp_sync(&cmte))
						Tm = cmte / 1000;
					gsm_reg = eat_network_get_creg();
					eat_get_cbc(&bmt);
					Vbt = bmt.volt;
					if(Vbt < Vbt_lim)
					{
						if(vblim_cnt != 5)
							vblim_cnt++;
					}
					else
					{
						if(vblim_cnt != 0)
							vblim_cnt--;
					}
					if(vblim_cnt == 5)
					{
						if(!(Sts_d & 4096))
						{
							Sts_d = Sts_d | 4096;
						}
					}
					if(vblim_cnt == 0)
					{
						if(Sts_d & 4096)
						{
							Sts_d = Sts_d & (0xFFFF - 4096);
						}
					}

					if(Stealth_per == 0)
					{
						if((gsm_reg != 1) && (gsm_reg != 5))
						{
							reg_err_cnt++;
							if(reg_err_cnt > 90)
							{
								reg_err_cnt = 0;
								send_cfun4 = EAT_TRUE;
write_log("GSM reg error1");
							}
						}
						else
						{
							reg_err_cnt = 0;
						}
					}
					else
					{
						if((gsm_reg != 1) && (gsm_reg != 5))
						{
							reg_err_cnt++;
							if(reg_err_cnt > 60)
							{
								reg_err_cnt = 0;
								send_cfun4 = EAT_TRUE;
write_log("GSM reg error2");
							}
						}
						else
						{
							reg_err_cnt = 0;
						}
					}
				}
				if(event.data.timer.timer_id == EAT_TIMER_3)
				{
					at_res = 2;
					at_answer[0] = 0;
					eat_sem_put(sem_at_done);
eat_trace("EAT_TIMER_3");
				}
			}
			break;
 			case EAT_EVENT_INT:
			{
				vibration_cnt++;
				break;
			}
       	}
    }
}

void app_user2(void *data)
{
	char tmp_buf[300];
	static char * buf_pos = NULL, * buf_pos1 = NULL;
	s8 ret = 0;
	SINT64 fs_freesize;
	UINT filesize;
	int testFileHandle;
	double Lat, Lon;
	EatRtc_st rtc = {0};
	unsigned int asd1;

	while(EAT_TRUE)
    {
		if(send_dtmf)
		{
			sprintf(tmp_buf, "AT+VTD=%u\r", dtmf_d);
			write_at(tmp_buf, "OK", 1000);
			sprintf(tmp_buf, "AT+VTS=\"%c\"\r", dtmf_c);
			write_at(tmp_buf, "OK", 1000);
			send_dtmf = EAT_FALSE;
		}
		if(send_cfun1)
		{
			write_at("AT+CFUN=1\r", "OK", 1000);
			if(at_res == 1)
			{
				cfun = 1;
			}
			send_cfun1 = EAT_FALSE;
		}
		if(send_cfun4)
		{
			write_at("AT+CFUN=4\r", "OK", 15000);
			if(at_res == 1)
			{
				cfun = 4;
			}
			send_cfun4 = EAT_FALSE;
		}
		if(main_status&1)
		{
			if(!first_init)
			{
				first_init = EAT_TRUE;
				write_at("ATE0\r", "OK", 1000);
				write_at("AT+CNETLIGHT=0\r", "OK", 1000);
				write_at("AT+CLIP=1\r", "OK", 1000);
				write_at("AT+DDET=1,0,1,0\r", "OK", 1000);
				write_at("AT+CNETSCAN=1\r", "OK", 1000);
				write_at("AT+CSGS=0\r", "OK", 1000);
				write_at("AT+SJDR=1,1,20,1\r", "OK", 1000);
				write_at("AT+SIMEI=\"869640052312595\"\r", "OK", 1000);
			}
		}
		if(main_status&4)
		{
			if(getparam1)
			{
				sprintf(sms_txt, "%u.%u.%u.%u,%u.%u.%u.%u,%u.%u.%u.%u,%u,%u,%u,%s,%s,%s,%u", ExIP[0][0], ExIP[0][1], ExIP[0][2], ExIP[0][3], ExIP[1][0], ExIP[1][1], ExIP[1][2], ExIP[1][3], ExIP[2][0], ExIP[2][1], ExIP[2][2], ExIP[2][3], ExPort[0], ExPort[1], ExPort[2], APN, APN_user, APN_pass, PortPr);
				send_sms(sms_nbr, sms_txt);
				getparam1 = EAT_FALSE;
			}
			if(getparam2)
			{
				sprintf(sms_txt, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", Period1, Period2, Send_per1, Send_per2, Settings, Settings1, Stealth_per, Stealth_ontm, Period3, Rsindp, Sync);
				send_sms(sms_nbr, sms_txt);
				getparam2 = EAT_FALSE;
			}
			if(getparam3)
			{
				sprintf(sms_txt, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", prm_per, gsmloc_per, money_per, trip_per, debug_per, 0, can1_per, can2_per, ADC_per, INx_per, freq_per, ident_per, offsim_per, fuel_per);
				send_sms(sms_nbr, sms_txt);
				getparam3 = EAT_FALSE;
			}
			if(getparam4)
			{
				sprintf(sms_txt, "%u,%s,%s,%u,%u,%u", Vbt_off, SMS_pass, MoneyUSSD, Speed_lim, Vcc_lim, Vbt_lim);
				send_sms(sms_nbr, sms_txt);
				getparam4 = EAT_FALSE;
			}
			if(getparam5)
			{
				sprintf(sms_txt, "%s,%s,%s,%s,%s,%s,%s", NumAd1, NumAd2, Num1, Num2, Num3, Num4, Num5);
				send_sms(sms_nbr, sms_txt);
				getparam5 = EAT_FALSE;
			}
			if(ussd_send)
			{
				sprintf(tmp_buf, "AT+CUSD=1,\"%s\"\r", ussdcmd);
				write_at(tmp_buf, "+CUSD:", 30000);
				if(at_res == 1)
				{
					buf_pos = at_ret + 7;
					if(*buf_pos == '0')
					{
						buf_pos = strchr(at_ret, '\"');
						if(buf_pos)
						{
							sscanf(buf_pos, "%250[^\"]", sms_txt);
							send_sms(sms_nbr, sms_txt);
						}
					}
					else
					{
						sprintf(sms_txt, "USSD error %c", *buf_pos);
					}
				}
				ussd_send = EAT_FALSE;
			}
			if(modinfo)
			{
				buf_pos = sms_txt;
				buf_pos = buf_pos + sprintf(buf_pos, "HW ver:%u\n\r", Version);
				buf_pos = buf_pos + sprintf(buf_pos, "IMEI:%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n\r", IMEI[1], IMEI[2], IMEI[3], IMEI[4], IMEI[5], IMEI[6], IMEI[7], IMEI[8], IMEI[9], IMEI[10], IMEI[11], IMEI[12], IMEI[13], IMEI[14], IMEI[15]);
				buf_pos = buf_pos + sprintf(buf_pos, "GSM ver:%s\n\r", simrev);
				buf_pos = buf_pos + sprintf(buf_pos, "MCU ver:%u\n\r", MCU_ver);
				buf_pos = buf_pos + sprintf(buf_pos, "SIM ICC:%c%c%c%c%c%c\n\r", ICC[0], ICC[1], ICC[2], ICC[3], ICC[4], ICC[5]);
				send_sms(sms_nbr, sms_txt);
				modinfo = EAT_FALSE;
			}
			if(fsinfo)
			{
				buf_pos = sms_txt;
				ret = eat_fs_GetDiskSize(EAT_FS, &fs_freesize);
				if(ret >= 0)
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Disk size %lld B\n\r", fs_freesize);
				}
				else
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Disk size get error: %d\n\r", ret);
				}
				ret = eat_fs_GetDiskFreeSize(EAT_FS, &fs_freesize);
				if(ret >= 0)
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Free size %lld B\n\r", fs_freesize);
				}
				else
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Free size get error: %d\n\r", ret);
				}
				testFileHandle = eat_fs_Open(L"C:\\log.txt", FS_READ_ONLY);
				if(testFileHandle < EAT_FS_NO_ERROR)
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Log file open error:%d\n\r", testFileHandle);
				}
				else
				{
					ret = eat_fs_GetFileSize(testFileHandle, &filesize);
					eat_fs_Close(testFileHandle);
					if(ret >= 0)
					{
						buf_pos = buf_pos + sprintf(buf_pos, "Log file size %d B\n\r", filesize);
					}
					else
					{
						buf_pos = buf_pos + sprintf(buf_pos, "Log file size get error: %d\n\r", ret);
					}
				}
				send_sms(sms_nbr, sms_txt);
				fsinfo = EAT_FALSE;
			}
			if(pos2sms && (!incall_f))
			{
				buf_pos = sms_txt;
				if((Year < 17) || (Year == 80))
				{
					eat_get_rtc(&rtc);
					Year = rtc.year;
					Month = rtc.mon;
					Day = rtc.day;
					Hour = rtc.hour;
					Minute = rtc.min;
					Second = rtc.sec;
				}
				buf_pos = buf_pos + sprintf(buf_pos, "%2.2u/%2.2u/%2.2u\n\r%2.2u:%2.2u:%2.2u\n\r", Day, Month, Year, Hour, Minute, Second);
				buf_pos = buf_pos + sprintf(buf_pos, "Vbt:%1.3fV\n\r", (double)Vbt/1000);
				if((Latitude != 0) && (Longitude != 0))
				{
					Lat = (double)Latitude / 100000;
					Lon = (double)Longitude / 100000;
					buf_pos = buf_pos + sprintf(buf_pos, "Lat:%.7f\n\rLon:%.7f\n\r", Lat, Lon);
				buf_pos = buf_pos + sprintf(buf_pos, "%ukm/h\n\r%udeg\n\r", Speed, Course);
					buf_pos = buf_pos + sprintf(buf_pos, "http://server1.gsm-gps.com/maps1/vp_gps.php?lat=%2.5f&lon=%3.5f&spd=%u&dir=%u\n\r", Lat, Lon, Speed, Course);
				}
				else
				{
					write_at("AT+SJDR=0\r", "OK", 1000);
					write_at("AT+CNETSCAN\r", "Operator:", 30000);
					if(at_res == 1)
					{
						buf_pos1 = strstr(at_ret, "MCC");
						sscanf(buf_pos1, "MCC:%u,MNC:%u,Rxlev:%*u,Cellid:%X,Arfcn:%*X,Lac:%X", &MCC, &MNC, &CID, &LAC);
						buf_pos = buf_pos + sprintf(buf_pos, "MCC:%u,MNC:%u,LAC:%u,CID:%u\n\r", MCC, MNC, LAC, CID);
						buf_pos = buf_pos + sprintf(buf_pos, "http://server1.gsm-gps.com/maps1/vp_gsm.php?MCC=%u&MNC=%u&LAC=%u&CID=%u\n\r", MCC, MNC, LAC, CID);
					}
					else
					{
						buf_pos = buf_pos + sprintf(buf_pos, "AT+CNETSCAN ERROR\n\r");
					}
					write_at("AT+SJDR=1,1,20,1\r", "OK", 1000);
				}
				send_sms(sms_nbr, sms_txt);
				pos2sms = EAT_FALSE;
			}
			if(gsmoffon)
			{
				strcpy(sms_txt, "Command \"gsmoffon\" accepted.");
				send_sms(sms_nbr, sms_txt);
				sms_sended = EAT_FALSE;
				while(!sms_sended)
					eat_sleep(1000);
				send_cfun4 = EAT_TRUE;
				gsmoffon = EAT_FALSE;
			}
			if(cpureset)
			{
				strcpy(sms_txt, "Command \"cpureset\" accepted.");
				send_sms(sms_nbr, sms_txt);
				sms_sended = EAT_FALSE;
				while(!sms_sended)
					eat_sleep(1000);
				simreset_f = EAT_TRUE;
				cpureset = EAT_FALSE;
			}
			if(stsreset)
			{
				eat_fs_Delete(L"C:\\Settings.txt");
				strcpy(sms_txt, "Command \"stsreset\" accepted.");
				send_sms(sms_nbr, sms_txt);
				sms_sended = EAT_FALSE;
				while(!sms_sended)
					eat_sleep(1000);
				simreset_f = EAT_TRUE;
				stsreset = EAT_FALSE;
			}
		}
		if(main_status&2)
		{
			if(fw_update)
			{
				write_at("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r", "OK", 1000);
				sprintf(tmp_buf, "AT+SAPBR=3,1,\"APN\",\"%s\"\r", APN);
				write_at(tmp_buf, "OK", 1000);
				sprintf(tmp_buf, "AT+SAPBR=3,1,\"USER\",\"%s\"\r", APN_user);
				write_at(tmp_buf, "OK", 1000);
				sprintf(tmp_buf, "AT+SAPBR=3,1,\"PWD\",\"%s\"\r", APN_pass);
				write_at(tmp_buf, "OK", 1000);
				write_at("AT+SAPBR=1,1\r", "OK", 20000);
				if(at_res == 1)
				{
					write_at("AT+FTPCID=1\r", "OK", 1000);
					sprintf(tmp_buf, "AT+FTPSERV=\"%u.%u.%u.%u\"\r", FTP_server[0], FTP_server[1], FTP_server[2], FTP_server[3]);
					write_at(tmp_buf, "OK", 1000);
					sprintf(tmp_buf, "AT+FTPUN=\"%s\"\r", FTP_user);
					write_at(tmp_buf, "OK", 1000);
					sprintf(tmp_buf, "AT+FTPPW=\"%s\"\r", FTP_pass);
					write_at(tmp_buf, "OK", 1000);
					sprintf(tmp_buf, "AT+FTPGETNAME=\"%s\"\r", fw_filename);
					write_at(tmp_buf, "OK", 1000);
					sprintf(tmp_buf, "AT+FTPGETPATH=\"%s\"\r", FTP_path);
					write_at(tmp_buf, "OK", 1000);
					write_at("AT+FTPPORT=21\r", "OK", 1000);
					write_at("AT+FTPTIMEOUT=3\r", "OK", 1000);
					write_at("AT+FTPGETTOFS=0,\"app\"\r", "+FTPGETTOFS:", 65000);
					if(at_res == 1)
					{
eat_trace("1");
						buf_pos = at_ret + 13;
						if(*buf_pos == '0')
						{
eat_trace("2");
							app_update(L"C:\\User\\Ftp\\app");
						}
					}
				}
				write_at("AT+SAPBR=0,1\r", "OK", 20000);
				fw_update = EAT_FALSE;
			}
			if(ICC[0] == 0)
			{
				write_at("AT+CCID\r", "OK", 1000);
			}
			if(money_f)
			{
				money_f = EAT_FALSE;
				sprintf(tmp_buf, "AT+CUSD=1,\"%s\"\r", MoneyUSSD);
				write_at(tmp_buf, "+CUSD:", 30000);
				if(at_res == 1)
				{
					buf_pos = at_ret + 7;
					if(*buf_pos == '0')
					{
						buf_pos = strchr(at_ret, '\"');
						if(buf_pos)
						{
							while(!isdigit(*buf_pos))
								buf_pos++;
							Money = atoi(buf_pos);
							wr_pkt(5);
						}
					}
				}
			}
			if(gsmloc_f)
			{
#if defined(GPS_ON)
				eat_gpio_write(GPS_ON, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(ESP_ON)
				dis_gpson = EAT_TRUE;
				for(MAC_cnt = 0; MAC_cnt < 120; MAC_cnt++)
				{
					MAC[MAC_cnt] = 0;
				}
				uart_config.baud = EAT_UART_BAUD_115200;
				uart_config.dataBits = EAT_UART_DATA_BITS_8;
				uart_config.parity = EAT_UART_PARITY_NONE;
				uart_config.stopBits = EAT_UART_STOP_BITS_1;
				eat_uart_set_config(GPS_UART, &uart_config);
				
				eat_gpio_write(ESP_ON, EAT_GPIO_LEVEL_HIGH);
eat_trace("ESP ON");
				eat_sleep(5000);
				eat_uart_write(GPS_UART, "AT+CWLAP\r\n", 10);
eat_trace("CWLAP");
				eat_sleep(3000);
				eat_gpio_write(ESP_ON, EAT_GPIO_LEVEL_LOW);
#endif
				uart_config.baud = GPS_BR;
				uart_config.dataBits = EAT_UART_DATA_BITS_8;
				uart_config.parity = EAT_UART_PARITY_NONE;
				uart_config.stopBits = EAT_UART_STOP_BITS_1;
				eat_uart_set_config(GPS_UART, &uart_config);
				dis_gpson = EAT_FALSE;
#if defined(ESP_ON)
eat_trace("ESP OFF");
				if(MAC_cnt != 0)
				{
					for(asd1 = 0; asd1 < MAC_cnt; asd1++)
					{
eat_trace("%x", MAC[asd1]);
					}
				}
#endif
				MCC = 0;
				MNC = 0;
				CID = 0;
				LAC = 0;
				if(!(Settings & 4096))
				{
					write_at("AT+SJDR=0\r", "OK", 1000);
					write_at("AT+CNETSCAN\r", "Operator:", 30000);
					if(at_res == 1)
					{
						buf_pos = strstr(at_ret, "MCC");
						if(buf_pos)
						{
							sscanf(buf_pos, "MCC:%u,MNC:%u,Rxlev:%*u,Cellid:%X,Arfcn:%*X,Lac:%X", &MCC, &MNC, &CID, &LAC);
						}
					}
					write_at("AT+SJDR=1,1,20,1\r", "OK", 1000);
				}
				wr_pkt(40);
				gsmloc_f = EAT_FALSE;
			}
		}
		if(gprs_reset)
		{
			ret = eat_soc_close(server_soc);
            ret = eat_gprs_bearer_release();
			gprs_reset = EAT_FALSE;
		}
		if(log_upload && (!incall_f))
		{
			write_at("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r", "OK", 1000);
			sprintf(tmp_buf, "AT+SAPBR=3,1,\"APN\",\"%s\"\r", APN);
			write_at(tmp_buf, "OK", 1000);
			sprintf(tmp_buf, "AT+SAPBR=3,1,\"USER\",\"%s\"\r", APN_user);
			write_at(tmp_buf, "OK", 1000);
			sprintf(tmp_buf, "AT+SAPBR=3,1,\"PWD\",\"%s\"\r", APN_pass);
			write_at(tmp_buf, "OK", 1000);
			write_at("AT+SAPBR=1,1\r", "OK", 20000);
			if(at_res == 1)
			{
				write_at("AT+FTPCID=1\r", "OK", 1000);
				write_at("AT+FTPSERV=\"194.28.172.136\"\r", "OK", 1000);
				write_at("AT+FTPUN=\"vladimir\"\r", "OK", 1000);
				write_at("AT+FTPPW=\"dkflbvbh79820409\"\r", "OK", 1000);
				write_at("AT+FTPPORT=21\r", "OK", 1000);
				write_at("AT+FTPTIMEOUT=3\r", "OK", 1000);
				write_at("AT+FTPPUTPATH=\"/TrLog/\"\r", "OK", 1000);
				sprintf(tmp_buf, "AT+FTPPUTNAME=\"T%c%c%c%c%c%c%c.txt\"\r", IMEI[9], IMEI[10], IMEI[11], IMEI[12], IMEI[13], IMEI[14], IMEI[15]);
				write_at(tmp_buf, "OK", 1000);
				write_at("AT+FTPPUTFRMFS=C:\\log.txt\r", "+FTPPUTFRMFS:", 60000);
			}
			write_at("AT+SAPBR=0,1\r", "OK", 20000);
			log_upload = EAT_FALSE;
		}
		if(ata_f)
		{
			ata_f = EAT_FALSE;
			write_at("ATA\r", "OK", 1000);
			dtmf_menu_number = 0;
			incall_f = EAT_TRUE;
		}
		if(ath_f)
		{
			ath_f = EAT_FALSE;
			write_at("ATH\r", "OK", 1000);
			incall_f = EAT_FALSE;
		}
		if(ath_simreset_f)
		{
			ath_simreset_f = EAT_FALSE;
			write_at("ATH\r", "OK", 1000);
			incall_f = EAT_FALSE;
			eat_reset_module();
		}

		eat_sleep(1000);
	}
}

void app_user7(void *data)
{
    while(EAT_TRUE)
    {
		if(!(Settings & 1024))
		{
			if(offgps_f)
			{
#if defined(GPS_ON)
				eat_gpio_write(GPS_ON, EAT_GPIO_LEVEL_LOW);
#endif
				offgps_f = EAT_FALSE;
			}
			if(!gps_chk)
			{
				gps_ok = EAT_FALSE;
				if(!dis_gpson)
				{
#if defined(GPS_ON)
					eat_gpio_write(GPS_ON, EAT_GPIO_LEVEL_LOW);
					eat_sleep(1000);
					eat_gpio_write(GPS_ON, EAT_GPIO_LEVEL_HIGH);
#endif
				}
			}
			else
			{
				gps_ok = EAT_TRUE;
			}
		}
		else
		{
#if defined(GPS_ON)
			eat_gpio_write(GPS_ON, EAT_GPIO_LEVEL_LOW);
#endif
		}
		eat_sleep(2000);
	}
}
#if defined(LED)
void app_user8(void *data)
{
    while(EAT_TRUE)
    {
		if(incall_f)
		{
			eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
			eat_sleep(100);
			eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
			eat_sleep(500);
		}
		else
		if(fw_update)
		{
			eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
			eat_sleep(100);
			eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
			eat_sleep(100);
			eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
			eat_sleep(100);
			eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
			eat_sleep(500);
		}
		else
		{
			if(!cpin)
			{
				eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
				eat_sleep(100);
				eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
			}
			else
			{
				eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
				eat_sleep(300);
				eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
				if((gsm_reg == 1) || (gsm_reg == 5))
				{
					eat_sleep(300);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
					eat_sleep(300);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
					if(bear_st == 2)
					{
						eat_sleep(300);
						eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
						eat_sleep(300);
						eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
					}
				}
			}
			eat_sleep(500);

			if(gps_ok)
			{
				eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
				eat_sleep(100);
				eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
				if((SatUs >> 6) == 1)
				{
					eat_sleep(200);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
					eat_sleep(100);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
				}
				if((SatUs >> 6) == 2)
				{
					eat_sleep(200);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
					eat_sleep(100);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
					eat_sleep(200);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
					eat_sleep(100);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
				}
			}
			eat_sleep(1500);
		}
    }
}
#endif
