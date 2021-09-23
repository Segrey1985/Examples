#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "mbSlave.h"
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "stm32f1xx_ll_usart.h"
#include "port.h"
#include <string.h>
#include <stdlib.h>
#include "aes.h"
#include <math.h>

#define CrLf putchar(13); putchar(10)
#define CtrlZ putchar(26)
#define ESC putchar(27)
#define Cr putchar(13)
#define SMS_IN_PERIOD 240000 // 240 / 60 = 4 мин
#define CHECK_SIGNAL_PERIOD 600000 //
#define INIT_MODEM_PERIOD 21600000 // = 6 часов

static void usart3TxIsr(void);
static void usart3RxIsr(void);
void getTelNumber(uint8_t holdingA3BufPosition);
void checkEqupCrash(void);
RetStateType gsmSendAnserSms(int phoneNumberPosition);
void replyCheckEqupCrash(void);
void usart3IRQHandler(void);

static uint8_t alarmWordBuf[48];
static uint8_t tempAlarmWordBuf[48];
char numTel[5][12];
static char line[128];
static char smsLine[512];
static char sendBuf[128];
static char dataBuf[64];
static char alarmList[48][33] = {

/************************BEGIN************************************/

"TEMP. OV MIN\n",     // 1 == 1

		"SRABOTAL RASCEPITEL ZAGAZ.\n",     // 2 == 2

		"KLAPAN GAZA ZAKRYT\n",     //3 == 8

		"DAVL. VODY OV MIN\n",     // 4 == 16

		"OTSUTSTV. CIRKUL. VODY OV\n",     // 5 == 32

		"PRONIKNOVENIE\n",     // 6 == 64

		"AVARIA NASOSA K5\n",     // 7 == 128

		"AVARIA NASOSA K4\n",     // 8 == 256

		"AVARIA NASOSA K3.2\n",     // 9 == 512

		"AVARIA NASOSA K3.1\n",     // 10 == 1024

		"NET SVYAZI A3*A2\n",     // 11 == 2048

		"TERMOSTAT KOTLA K1.2\n",     // 12

		"AVARIA GORELKI K2.2\n",     // 13

		"NET SVYAZI A3*A1\n",     // 14

		"TERMOSTAT KOTLA K1.1\n",     // 15

		"AVARIA GORELKI K2.1\n",     // 16

		/************************BEGIN************************************/

		"VKL. RESERV. ELEKTR. VVOD\n",     // 17

		"OTSUTSTV. ELEKTROPITANIA\n",     // 18

		"DATCH. T KOTLA K1.2\n",     // 19

		"DATCH. T DIM. GAZ. K1.2\n",     // 20

		"DATCH. T KOTLA K1.1\n",     // 21

		"DATCH. T DIM. GAZ. K1.1\n",     // 22

		"DATCH. T OBR. OV\n",     // 23

		"DATCH. T OBR. KK\n",     // 24

		"DATCH. T PR. GVS\n",     // 25

		"DATCH. T RP. OV\n",     // 26

		"DATCH. T NAR. VOZD.\n",     // 27

		"DATCH. T PR. KK\n",     // 28

		"DATCH. P RP. GVS\n",     // 29

		"DATCH. P PODP.\n",     // 30

		"DATCH. P OBR. OV\n",     // 31

		"DATCH. P RP. OV\n",     // 32

		/************************BEGIN************************************/

		"RESERVE\n",     // 33

		"RESERVE\n",     // 34

		"TEMP. GVS MIN\n",     // 35

		"T. DIM. GAZ K1.1 PREVISHENA\n",     // 36

		"T. DIM. GAZ K1.2 PREVISHENA\n",     // 37

		"NET SVYAZI A3*MB110\n",     // 38

		"NET SVYAZI A3*MY110\n",     // 39

		"AVARIA POZH. SIGN.\n",     // 40

		"POZHAR\n",     // 41

		"AVARIA ESSA\n",     // 42

		"ZAGAZ. CO 2 POROG\n",     // 43

		"ZAGAZ. CO 1 POROG\n",     // 44

		"ZAGAZ. CH4 2 POROG\n",     // 45

		"ZAGAZ. CH4 1 POROG\n",     // 46

		"DAVL. PODPITKI MIN\n",     // 47

		"PEREPAD DAVL. OV MIN\n",     // 48

		};

QueueHandle_t gsmRxQueue;
QueueHandle_t gsmTxQueue;
// 2b 7e 15 16 28 ae d2 a6 ab f7 15 88 09 cf 4f 3c
const uint8_t key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
uint8_t in[160];
uint8_t iv[16];
extern osMutexId inputRegMutex;
extern osMutexId holdingRegMutex;
extern QueueHandle_t gsmQueue;
extern osTimerId periodSmsSendTimer;
extern osTimerId httpsSendTimer;
extern osTimerId smsCheckTimer;
extern osTimerId signalCheckTimer;
extern osTimerId modemInitTimer;
extern UART_HandleTypeDef huart3;

extern uint16_t firstWord;
extern uint16_t secondWord;
extern uint16_t thirdWord;
extern uint16_t thirdWord;
extern uint16_t fourthWord;

static float tempOutdoor;
static float tempHeat1;
static float tempHeat2;
static float tempGvs;
static float tempBoilUn;
static float tempBoilUnRev;
static float tempBoil1;
static float tempBoil2;
static float tempFlue1;
static float tempFlue2;
static float pressHeat1;
static float pressHeat2;
static float pressColdWater;
static float pressGvs;

typedef enum {
	GsmNotInit, GsmIdle, GsmOff
} GsmStateType;

static GsmStateType GsmState = GsmOff;

void StartGsmTask(void const *argument) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	static GSMMessageType message;
	uint32_t smsDelay;
	uint32_t httpsDelay;
	gsmRxQueue = xQueueCreate(256, sizeof(char));
	gsmTxQueue = xQueueCreate(512, sizeof(char));
	for (int a = 0; a < sizeof(tempAlarmWordBuf); a++) {
		tempAlarmWordBuf[a] = 0xFF;
	}
	MX_USART3_UART_Init();
	for (;;) {
		xQueueReceive(gsmQueue, &message, osWaitForever);
		osDelay(500);
		switch (message) {
		case SmsOn:
			osMutexWait(holdingRegMutex, osWaitForever);
			smsDelay = mbRegHoldingBuf.A3[25] * 60000;
			osMutexRelease(holdingRegMutex);
			if (smsDelay == 0)
				smsDelay = 20 * 60000;
			osTimerStart(periodSmsSendTimer, smsDelay);
			osTimerStart(smsCheckTimer, SMS_IN_PERIOD);
			osTimerStart(signalCheckTimer, CHECK_SIGNAL_PERIOD);
			osTimerStart(modemInitTimer, INIT_MODEM_PERIOD);
			if (GsmState != GsmIdle) {
				if (gsmInit() == Ret_OK && gsmNetworkConnect() == Ret_OK)
					GsmState = GsmIdle;
				else
					GsmState = GsmNotInit;
			}
			break;
		case SmsOff:
			if (GsmState == GsmOff)
				break;
			osTimerStop(periodSmsSendTimer);
			osTimerStop(smsCheckTimer);
			if (mbCoilsBuf.Settings.HttpsOption == 0) {
				osTimerStop(signalCheckTimer);
				osTimerStop(modemInitTimer);
				GsmState = GsmOff;
				mbDiscrInputsBuf.A3.GsmModemErr = 0;
				mbDiscrInputsBuf.A3.GsmModemNotInit = 0;
				mbDiscrInputsBuf.A3.GsmModemNotRegistered = 0;
				mbDiscrInputsBuf.A3.GsmModemSignalLow = 0;
				osMutexWait(inputRegMutex, osWaitForever);
				mbRegInputBuf.A3[39] = 0;
				osMutexRelease(inputRegMutex);
			}
			mbDiscrInputsBuf.A3.smsSendErr = 0;
			break;
		case HttpsOn:
			osMutexWait(holdingRegMutex, osWaitForever);
			httpsDelay = mbRegHoldingBuf.A3[78] * 60000;
			osMutexRelease(holdingRegMutex);
			if (httpsDelay == 0)
				httpsDelay = 15 * 60000;
			osTimerStart(httpsSendTimer, httpsDelay);
			osTimerStart(signalCheckTimer, CHECK_SIGNAL_PERIOD);
			osTimerStart(modemInitTimer, INIT_MODEM_PERIOD);
			if (GsmState != GsmIdle) {
				if (gsmInit() == Ret_OK && gsmNetworkConnect() == Ret_OK) {
					GsmState = GsmIdle;
				} else
					GsmState = GsmNotInit;
			}
			break;
		case HttpsOff:
			if (GsmState == GsmOff)
				break;
			osTimerStop(httpsSendTimer);
			if (mbCoilsBuf.A3.SmsOn_Off == 0) {
				osTimerStop(signalCheckTimer);
				osTimerStop(modemInitTimer);
				GsmState = GsmOff;
				mbDiscrInputsBuf.A3.GsmModemErr = 0;
				mbDiscrInputsBuf.A3.GsmModemNotInit = 0;
				mbDiscrInputsBuf.A3.GsmModemNotRegistered = 0;
				mbDiscrInputsBuf.A3.GsmModemSignalLow = 0;
				osMutexWait(inputRegMutex, osWaitForever);
				mbRegInputBuf.A3[39] = 0;
				osMutexRelease(inputRegMutex);
			}
			mbDiscrInputsBuf.A3.HttpsSendError = 0;
			break;
		case PeriodSmsCha:
			osMutexWait(holdingRegMutex, osWaitForever);
			smsDelay = mbRegHoldingBuf.A3[25] * 60000;
			osMutexRelease(holdingRegMutex);
			if (GsmState != GsmOff)
				osTimerStart(periodSmsSendTimer, smsDelay);
			break;
		case PeriodHttpsCha:
			httpsDelay = mbRegHoldingBuf.A3[78] * 60000;
			osMutexWait(holdingRegMutex, osWaitForever);
			osMutexRelease(holdingRegMutex);
			if (GsmState != GsmOff)
				osTimerStart(httpsSendTimer, httpsDelay);
			break;
		case EquipCrash:
			if (GsmState != GsmOff) {
				if (mbCoilsBuf.Settings.HttpsOption == 1)
					httpsSendDataToServer();
				if (mbCoilsBuf.A3.SmsOn_Off == 1)
					checkEqupCrash();
			}
			break;
		case httpsSendData:
			if (GsmState != GsmOff)
				httpsSendDataToServer();
			break;
		case Sms_Temp1:
			mbDiscrInputsBuf.A3.smsSendErr = (gsmSendAnserSms(0) == Ret_ERROR) ? 1 : 0;
			break;
		case Sms_Temp2:
			if (gsmSendAnserSms(1) == Ret_ERROR)
				mbDiscrInputsBuf.A3.smsSendErr = 1;
			else
				mbDiscrInputsBuf.A3.smsSendErr = 0;
			break;
		case Sms_Temp3:
			if (gsmSendAnserSms(2) == Ret_ERROR)
				mbDiscrInputsBuf.A3.smsSendErr = 1;
			else
				mbDiscrInputsBuf.A3.smsSendErr = 0;
			break;
		case Sms_Temp4:
			if (gsmSendAnserSms(3) == Ret_ERROR)
				mbDiscrInputsBuf.A3.smsSendErr = 1;
			else
				mbDiscrInputsBuf.A3.smsSendErr = 0;
			break;
		case Sms_Temp5:
			if (gsmSendAnserSms(4) == Ret_ERROR)
				mbDiscrInputsBuf.A3.smsSendErr = 1;
			else
				mbDiscrInputsBuf.A3.smsSendErr = 0;
			break;
		case PeriodSend:
			if (GsmState != GsmOff)
				replyCheckEqupCrash();
			break;
		case CheckIncom:
			if (GsmState != GsmOff) {
				if (gsmSMSParse() == Ret_ERROR)
					mbDiscrInputsBuf.A3.GsmModemNotInit = 1;
				else
					mbDiscrInputsBuf.A3.GsmModemNotInit = 0;
			}
			break;
		case CheckSignal:
			if (GsmState != GsmOff) {
				if (executeRequest("AT\r", "OK", 8) == Ret_ERROR)
					mbDiscrInputsBuf.A3.GsmModemErr = 1;
				else
					mbDiscrInputsBuf.A3.GsmModemErr = 0;
				if (gsmNetworkConnect() == Ret_OK)
					GsmState = GsmIdle;
				else
					GsmState = GsmNotInit;
			} else {
				mbDiscrInputsBuf.A3.GsmModemErr = 0;
				mbDiscrInputsBuf.A3.GsmModemNotInit = 0;
				mbDiscrInputsBuf.A3.HttpsSendError = 0;
				mbDiscrInputsBuf.A3.GsmModemNotRegistered = 0;
				mbDiscrInputsBuf.A3.GsmModemSignalLow = 0;
				mbDiscrInputsBuf.A3.smsSendErr = 0;
				osMutexWait(inputRegMutex, osWaitForever);
				mbRegInputBuf.A3[39] = 0;
				osMutexRelease(inputRegMutex);
			}
			break;
		case ModemInit:
			if (GsmState != GsmOff) {
				if (gsmInit() == Ret_OK && gsmNetworkConnect() == Ret_OK) {
					GsmState = GsmIdle;
				} else
					GsmState = GsmNotInit;
			}
			break;
		default:
			break;
		}
	}
}

RetStateType gsmInit(void) {
	uint8_t errorCounter = 0;
	if (executeRequest("AT#REBOOT\r", "OK", 10) == Ret_ERROR)
		errorCounter++;
	osDelay(15000);
	if (executeRequest("ATE0\r", "OK", 4) == Ret_ERROR)     // turn off echo
		errorCounter++;
	if (executeRequest("AT+CMGF=1\r", "OK", 4) == Ret_ERROR)     // set text mode for SMS messages
		errorCounter++;
	if (executeRequest("AT+IFC=1,1\r", "OK", 4) == Ret_ERROR)     // software control of data flow
		errorCounter++;
	if (executeRequest("ATS0=0\r", "OK", 4) == Ret_ERROR)     // set manual call response
		errorCounter++;
	if (executeRequest("AT+CSDH=0\r", "OK", 4) == Ret_ERROR)     // allow to display SMS message headers
		errorCounter++;
	if (executeRequest("AT+CMGD=1,3\r", "OK", 4) == Ret_ERROR)     // delete messages
		errorCounter++;
	mbDiscrInputsBuf.A3.GsmModemNotRegistered = (errorCounter > 0) ? 1 : 0;
	return (errorCounter > 0) ? Ret_ERROR : Ret_OK;
}

RetStateType gsmSMSSend(char *ptr, int phoneNumberPosition) {
	uint8_t errorCounter = 0;
	getTelNumber(phoneNumberPosition * 10);
	if (numTel[phoneNumberPosition][0] != '0' && GsmState != GsmOff && mbCoilsBuf.A3.SmsOn_Off == 1) {
		snprintf(dataBuf, sizeof(dataBuf), "AT+CMGS=\"+%s\"\r", numTel[phoneNumberPosition]);
		if (executeRequest(dataBuf, ">", 10) == Ret_OK) {     // 2 сек с паузой 0,25 сек
			printf("%s", ptr);
			if (putCharRequest(26, "OK", 50) == Ret_ERROR)     // 20 сек с задержкой 0,25 сек
				errorCounter++;
		} else {
			ESC;
			errorCounter++;
		}
	}
	return (errorCounter > 0) ? Ret_ERROR : Ret_OK;
}

RetStateType gsmSMSParse(void) {
	char tempStr[14];
	static GSMMessageType message;
	memset(tempStr, NULL, sizeof tempStr);
	for (int i = 0; i < 5; i++) {
		getTelNumber(i * 10);
		if (numTel[i][0] == '0')
			continue;
		for (int a = 1; a <= 5; a++) {
			snprintf(tempStr, sizeof(tempStr), "AT+CMGR=%u\r", a);
			if (checkIncomSms(tempStr, numTel[i], 4) == Ret_OK) {
				switch (i) {
				case 0:
					message = Sms_Temp1;
					break;
				case 1:
					message = Sms_Temp2;
					break;
				case 2:
					message = Sms_Temp3;
					break;
				case 3:
					message = Sms_Temp4;
					break;
				case 4:
					message = Sms_Temp5;
					break;
				}
				xQueueSend(gsmQueue, &message, 0);
			}
		}
	}
	return (executeRequest("AT+CMGD=1,2\r", "OK", 20) == Ret_ERROR) ? Ret_ERROR : Ret_OK;
}

void getTelNumber(uint8_t holdingA3BufPosition) {
	uint16_t temp[8];
	memset(temp, NULL, sizeof temp);
	osMutexWait(holdingRegMutex, osWaitForever);
	for (int i = 0; i < 8; i++)
		temp[i] = mbRegHoldingBuf.PhoneA3[holdingA3BufPosition + i];
	osMutexRelease(holdingRegMutex);
	if (temp[0] != 0) {
		snprintf(&numTel[holdingA3BufPosition / 10][0], sizeof(numTel), "%u", temp[0]);
		for (int i = 0; i < 7; i++) {
			snprintf(&numTel[holdingA3BufPosition / 10][i + 4], sizeof(numTel[holdingA3BufPosition / 10]), "%u",
					temp[i + 1]);
		}
		numTel[holdingA3BufPosition / 10][11] = '\0';
	} else {
		for (int i = 0; i < 11; i++) {
			numTel[holdingA3BufPosition / 10][i] = '0';
		}
		numTel[holdingA3BufPosition / 10][11] = '\0';
	}
}

void usart3RxIsr(void) {
	static char message;
	message = (char) LL_USART_ReceiveData8(USART3);
	xQueueSendFromISR(gsmRxQueue, &message, NULL);
}

GETCHAR_PROTO {
	int32_t getCharTimeOut = 250;
	static char message;
	if (xQueueReceive(gsmRxQueue, &message, getCharTimeOut) != pdPASS)
		return EOF;
	return (int32_t) message;
}

char* readLine(char *buf, int size) {
	int i, ch;
	for (i = 0; i < size - 1; i++) {
		ch = getchar();
		if (ch == EOF)
			return NULL;
		*buf++ = (char) ch;
	}
	*buf = '\0';
	return buf - i;
}

char* readPartOfLine(char *buf, int size) {
	int i, ch;
	for (i = 0; i < size - 1; i++) {
		ch = getchar();
		if (ch == EOF)
			return NULL;
		*buf++ = (char) ch;
		if (ch == '\n')
			break;
	}
	*buf = '\0';
	return buf - i;
}

size_t __read(int Handle, unsigned char *Buf, size_t BufSize) {
	int nChars = 0;
	for (/*Empty*/; BufSize > 0; --BufSize) {
		int c = getchar();
		if (c < 0)
			break;
		*Buf++ = c;
		++nChars;
	}
	return nChars;
}

void usart3TxIsr(void) {
	static char message;
	if (xQueueReceiveFromISR(gsmTxQueue, &message, NULL) != pdPASS) {
		__HAL_UART_DISABLE_IT(&huart3, UART_IT_TXE);
	} else
		LL_USART_TransmitData8(USART3, message);
}

PUTCHAR_PROTO {
	static char message;
	message = ch;
	xQueueSend(gsmTxQueue, &message, 0);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);
	return ch;
}

void checkEqupCrash(void) {
	uint8_t acceptCounter = 0;
	uint8_t passCounter = 0;
	uint8_t errorCounter = 0;
	intToBin(firstWord, 16, alarmWordBuf, 0);
	intToBin(thirdWord, 16, alarmWordBuf, 16);
	intToBin(fourthWord, 16, alarmWordBuf, 32);
	memset(smsLine, NULL, sizeof smsLine);
	for (int i = 0; i < 48; i++) {
		if (alarmWordBuf[i] == 1 && tempAlarmWordBuf[i] != alarmWordBuf[i])
			strcat(smsLine, alarmList[i]);
		if (tempAlarmWordBuf[i] == 1)
			acceptCounter++;
		tempAlarmWordBuf[i] = alarmWordBuf[i];
		if (tempAlarmWordBuf[i] == 1)
			passCounter++;
	}
	if (acceptCounter > passCounter && passCounter == 0) {
		memset(smsLine, NULL, sizeof smsLine);
		strcat(smsLine, "OBORUDOVANIE V NORME");
	}
	if (strlen(smsLine) > 0) {
		if (mbCoilsBuf.Settings.HttpsOption == 1)
			executeRequest("AT#SGACT=1,0\r", "OK", 20);
		for (int a = 0; a < 5; a++) {
			osDelay(1000);
			if (gsmSMSSend(smsLine, a) == Ret_ERROR)
				errorCounter++;
		}
	}
	mbDiscrInputsBuf.A3.smsSendErr = (errorCounter > 0) ? 1 : 0;
}

void replyCheckEqupCrash(void) {
	uint8_t errorCounter = 0;
	intToBin(firstWord, 16, alarmWordBuf, 0);
	intToBin(thirdWord, 16, alarmWordBuf, 16);
	intToBin(fourthWord, 16, alarmWordBuf, 32);
	memset(smsLine, NULL, sizeof smsLine);
	for (int i = 0; i < 48; i++) {
		if (alarmWordBuf[i] == 1)
			strcat(smsLine, alarmList[i]);
	}
	if (strlen(smsLine) > 0) {
		if (mbCoilsBuf.Settings.HttpsOption == 1)
			executeRequest("AT#SGACT=1,0\r", "OK", 20);
		for (int a = 0; a < 5; a++) {
			osDelay(1000);
			if (gsmSMSSend(smsLine, a) == Ret_ERROR)
				errorCounter++;
		}
	}
	mbDiscrInputsBuf.A3.smsSendErr = (errorCounter > 0) ? 1 : 0;
}

RetStateType httpsSendDataToServer(void) {
	struct AES_ctx ctx;
	static int randInt;
	uint8_t errorCounter = 0;
	uint16_t idPtr, tguKey, ip1, ip2, ip3, ip4, port, bnr1w, bnr2w;
	static uint8_t ping = 0;
	uint8_t sendPer;
	float setTempBoilUn;
	float setTempHeat1;
	float setTempGvs;
	float setTempBoil1Casc;
	float setTempBoil2Casc;
	float setTempBoil1Autonom;
	float setTempBoil2Autonom;
	char convCharToStr[3];
	char wordEncodeString[321];
	char vectorEncodeString[33];
	char tempBuf[64];
	osMutexWait(inputRegMutex, osWaitForever);
	pressHeat1 = *((float*) &mbRegInputBuf.A3[0]);
	pressHeat2 = *((float*) &mbRegInputBuf.A3[2]);
	pressColdWater = *((float*) &mbRegInputBuf.A3[4]);
	pressGvs = *((float*) &mbRegInputBuf.A3[6]);
	tempBoilUn = *((float*) &mbRegInputBuf.A3[8]);     // Температура котлового контура
	tempOutdoor = *((float*) &mbRegInputBuf.A3[10]);     // Температура наружного воздуха
	tempHeat1 = *((float*) &mbRegInputBuf.A3[12]);     // Температура контура отопления подача
	tempGvs = *((float*) &mbRegInputBuf.A3[14]);     // Температура контура ГВС
	tempBoilUnRev = *((float*) &mbRegInputBuf.A3[16]);
	tempHeat2 = *((float*) &mbRegInputBuf.A3[18]);     // Температура контура отопления обратка
	tempBoil1 = *((float*) &mbRegInputBuf.A1[2]);
	tempFlue1 = *((float*) &mbRegInputBuf.A1[0]);
	bnr1w = *((uint16_t*) &mbRegInputBuf.A1[10]);
	tempBoil2 = *((float*) &mbRegInputBuf.A2[2]);
	tempFlue2 = *((float*) &mbRegInputBuf.A2[0]);
	bnr2w = *((uint16_t*) &mbRegInputBuf.A2[10]);
	setTempBoilUn = *((float*) &mbRegInputBuf.A3[32]);
	setTempHeat1 = *((float*) &mbRegInputBuf.A3[34]);
	setTempBoil1Casc = *((float*) &mbRegInputBuf.A1[8]);
	setTempBoil2Casc = *((float*) &mbRegInputBuf.A2[8]);
	osMutexRelease(inputRegMutex);

	osMutexWait(holdingRegMutex, osWaitForever);
	idPtr = mbRegHoldingBuf.A3[71];
	tguKey = mbRegHoldingBuf.A3[72];
	ip1 = mbRegHoldingBuf.A3[73];
	ip2 = mbRegHoldingBuf.A3[74];
	ip3 = mbRegHoldingBuf.A3[75];
	ip4 = mbRegHoldingBuf.A3[76];
	port = mbRegHoldingBuf.A3[77];
	sendPer = mbRegHoldingBuf.A3[78];
	setTempGvs = *((float*) &mbRegHoldingBuf.A3[58]);
	setTempBoil1Autonom = *((float*) &mbRegHoldingBuf.A1[2]);
	setTempBoil2Autonom = *((float*) &mbRegHoldingBuf.A2[2]);
	osMutexRelease(holdingRegMutex);
	if (mbDiscrInputsBuf.A3.ModbusErrA1 == 0) {     // вычисление уставки воды в котле
		if (mbDiscrInputsBuf.A1.Cascaded == 1)
			setTempBoil1Autonom = setTempBoil1Casc;
		else if (mbDiscrInputsBuf.A1.Autonom == 0 && mbDiscrInputsBuf.A1.Cascaded == 0) {
			setTempBoil1Autonom = 0;
		}
	} else
		setTempBoil1Autonom = 0;
	if (mbDiscrInputsBuf.A3.ModbusErrA2 == 0) {     // вычисление уставки воды в котле
		if (mbDiscrInputsBuf.A2.Cascaded == 1)
			setTempBoil2Autonom = setTempBoil2Casc;
		else if (mbDiscrInputsBuf.A2.Autonom == 0 && mbDiscrInputsBuf.A2.Cascaded == 0) {
			setTempBoil2Autonom = 0;
		}
	} else
		setTempBoil2Autonom = 0;

	if (idPtr - 1234 != tguKey) {
		mbDiscrInputsBuf.A3.HttpsSendError = 1;
		return Ret_ERROR;
	}

	randInt += 1;
	srand(randInt);
	for (int i = 0; i < sizeof(iv); i++) {
		iv[i] = rand();
	}

	snprintf(smsLine, sizeof(smsLine),
			"%u,%u,%u,%u,%u,%u,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f,%u,%3.1f,%3.1f,%u,%3.1f,%3.1f,%3.1f,%3.1f,%3.1f",
			idPtr, firstWord, secondWord, thirdWord, sendPer, ping, pressHeat1, pressHeat2, pressColdWater, pressGvs,
			tempBoilUn, tempOutdoor, tempHeat1, tempGvs, tempBoilUnRev, tempHeat2, tempBoil1, tempFlue1, bnr1w,
			tempBoil2, tempFlue2, bnr2w, setTempBoil1Autonom, setTempBoil2Autonom, setTempBoilUn, setTempHeat1,
			setTempGvs);
	int size = strlen(smsLine);
	smsLine[size] = ',';
	for (int u = size + 1; u < sizeof(in); u++) {
		smsLine[u] = '0';
	}
	for (int i = 0; i < sizeof(in); i++) {
		in[i] = (uint8_t) smsLine[i];
	}
	AES_init_ctx_iv(&ctx, key, iv);
	for (uint8_t i = 0; i < sizeof(iv); i++) {
		snprintf(convCharToStr, sizeof(convCharToStr), "%02x", iv[i]);
		strcat(vectorEncodeString, convCharToStr);
	}
	AES_CBC_encrypt_buffer(&ctx, in, sizeof(in));
	for (int i = 0; i < sizeof(in); i++) {
		snprintf(convCharToStr, sizeof(convCharToStr), "%02x", in[i]);
		strcat(wordEncodeString, convCharToStr);
	}
	snprintf(smsLine, sizeof(smsLine), "{H:[\"%s\"],S:[\"%s\"]}\r", vectorEncodeString, wordEncodeString);
	snprintf(sendBuf, sizeof(sendBuf), "AT#HTTPSND=0,0,\"/api/tgu\",%u,\"application/json;charset=utf-8\"\r",
			strlen(smsLine));
	snprintf(tempBuf, sizeof(tempBuf), "AT+CGDCONT=1,\"IP\",%u.%u.%u.%u\r", ip1, ip2, ip3, ip4 /*217, 79, 3 , 204*/);
	if (executeRequest(tempBuf, "OK", 20) == Ret_ERROR)     // 6*0,25=1,5сек
		errorCounter++;
	if (executeRequest("AT#SGACT?\r", "#SGACT: 1,1", 20) == Ret_ERROR)     // 4*0,25=1сек
		if (executeRequest("AT#SGACT=1,1\r", "OK", 30) == Ret_ERROR)     // 30*0,25=7,5сек
			errorCounter++;
	snprintf(tempBuf, sizeof(tempBuf), "AT#HTTPCFG=0,\"%u.%u.%u.%u\",%u\r", ip1, ip2, ip3, ip4,
			port /*217, 79, 3 , 204, 3939*/);
	if (executeRequest(tempBuf, "OK", 20) == Ret_ERROR)     // 20*0,25=5сек
		errorCounter++;
	if (executeRequest(sendBuf, ">>>", 20) == Ret_OK) {     // 20*0,25=5сек
		if (executeRequest(smsLine, "OK", 20) == Ret_ERROR)     // 20*0,25=5сек
			errorCounter++;
		else if (readLineAfterRequest("#HTTPRING", 40) == Ret_ERROR)     // 40*0,25=10сек
			errorCounter++;
		else if (executeRequest("AT#HTTPRCV=0\r", "OpcWriteSuccess", 6)     // 6*0,25=1,5сек
		== Ret_ERROR)
			errorCounter++;
	} else
		errorCounter++;
// Итого: 1+2,5+1,5+1,5+1,5+2,5+5+10+1,5=25,5сек
	memset(wordEncodeString, NULL, sizeof wordEncodeString);
	memset(vectorEncodeString, NULL, sizeof vectorEncodeString);
	xQueueReset(gsmRxQueue);
	ping = (ping == 12) ? 0 : 12;
	if (errorCounter > 0) {
		mbDiscrInputsBuf.A3.HttpsSendError = 1;
		return Ret_ERROR;
	} else {
		mbDiscrInputsBuf.A3.HttpsSendError = 0;
		return Ret_OK;
	}
}

RetStateType gsmSendAnserSms(int phoneNumberPosition) {
	char gvsString[36];
	xQueueReset(gsmRxQueue);
	osMutexWait(inputRegMutex, osWaitForever);
	tempOutdoor = *((float*) &mbRegInputBuf.A3[10]);     // Температура наружного воздуха
	tempBoilUn = *((float*) &mbRegInputBuf.A3[8]);     // Температура котлового контура
	tempHeat1 = *((float*) &mbRegInputBuf.A3[12]);     // Температура контура отопления подача
	tempHeat2 = *((float*) &mbRegInputBuf.A3[18]);     // Температура контура отопления обратка
	pressHeat1 = *((float*) &mbRegInputBuf.A3[0]);
	pressHeat2 = *((float*) &mbRegInputBuf.A3[2]);
	pressColdWater = *((float*) &mbRegInputBuf.A3[4]);
	tempGvs = *((float*) &mbRegInputBuf.A3[14]);     // Температура контура ГВС
	pressGvs = *((float*) &mbRegInputBuf.A3[6]);     // Давление контура ГВС
	osMutexRelease(inputRegMutex);
	snprintf(smsLine, sizeof(smsLine),
			"Tnr.v:%3.1f*C\nTov.pr:%3.1f*C\nTov.obr:%3.1f*C\nTk.pr:%3.1f*C\nPov.pr:%2.1fBar\nPov.obr:%2.1fBar\nPpodp:%2.1fBar",
			tempOutdoor, tempHeat1, tempHeat2, tempBoilUn, pressHeat1, pressHeat2, pressColdWater);
	if (mbCoilsBuf.A3.DhwOption == 1) {
		snprintf(gvsString, sizeof(gvsString), "\nTgvs:%3.1f*C\nPgvs:%2.1fBar", tempGvs, pressGvs);
		strcat(smsLine, gvsString);
	}
	if (mbCoilsBuf.Settings.HttpsOption == 1)
		executeRequest("AT#SGACT=1,0\r", "OK", 20);
	return (gsmSMSSend(smsLine, phoneNumberPosition) == Ret_ERROR) ? Ret_ERROR : Ret_OK;
}

RetStateType gsmNetworkConnect(void) {
	uint8_t errorCounter = 0;
	if (executeRequest("AT+CNMI=0,0,0,0,0\r", "OK", 4) == Ret_ERROR)
		errorCounter++;
	if (executeRequest("AT+CREG?\r", "+CREG: 0,1", 8) == Ret_OK) {
		if (executeRequest("AT+COPS=2\r", "OK", 8) == Ret_ERROR)
			errorCounter++;
		if (executeRequest("AT+COPS=0\r", "OK", 8) == Ret_ERROR)
			errorCounter++;
	}
	if (executeRequest("AT+CMEE=1\r", "OK", 6) == Ret_ERROR)
		errorCounter++;
	if (executeRequest("AT+CPIN?\r", "+CPIN: READY", 6) == Ret_ERROR)
		errorCounter++;
	if (executeDataRequest("AT+CSQ\r", "+CSQ: ", 39, 6) == Ret_ERROR)
		errorCounter++;
	osMutexWait(inputRegMutex, osWaitForever);
	if (mbRegInputBuf.A3[39] < 12 || mbRegInputBuf.A3[39] >= 33) {
		mbRegInputBuf.A3[39] = 0;
		mbDiscrInputsBuf.A3.GsmModemSignalLow = 1;
	} else {
		mbDiscrInputsBuf.A3.GsmModemSignalLow = 0;
		mbRegInputBuf.A3[39] = mbRegInputBuf.A3[39] * 3;
	}
	osMutexRelease(inputRegMutex);
	if (errorCounter > 0) {
		gsmInit();
		mbDiscrInputsBuf.A3.GsmModemNotRegistered = 1;
		return Ret_ERROR;
	} else {
		mbDiscrInputsBuf.A3.GsmModemNotRegistered = 0;
		return Ret_OK;
	}
}

RetStateType executeRequest(char *executeCommand, char *anserString, uint16_t delay) {
	xQueueReset(gsmRxQueue);
	memset(line, NULL, sizeof line);
	printf(executeCommand);
	osDelay(250);
	for (int i = 0; i < delay; i++) {
		readLine(line, sizeof(line));
		if (strstr(line, anserString) != NULL) {
			xQueueReset(gsmRxQueue);
			return Ret_OK;
		} else
			osDelay(250);
	}
	xQueueReset(gsmRxQueue);
	return Ret_ERROR;
}

RetStateType checkIncomSms(char *executeCommand, char *phoneNumber, uint16_t delay) {
	xQueueReset(gsmRxQueue);
	memset(line, NULL, sizeof line);
	printf(executeCommand);
	osDelay(250);
	for (int i = 0; i < delay; i++) {
		readLine(line, sizeof(line));
		if (strstr(line, phoneNumber) != NULL) {
			if (strstr(line, "INFO") != NULL) {
				xQueueReset(gsmRxQueue);
				return Ret_OK;
			}
		} else
			osDelay(250);
	}
	xQueueReset(gsmRxQueue);
	return Ret_ERROR;
}

RetStateType readLineAfterRequest(char *anserString, uint16_t delay) {
	xQueueReset(gsmRxQueue);
	memset(line, NULL, sizeof line);
	osDelay(250);
	for (int i = 0; i < delay; i++) {
		readLine(line, sizeof(line));
		if (strstr(line, anserString) != NULL) {
			xQueueReset(gsmRxQueue);
			return Ret_OK;
		} else
			osDelay(250);
	}
	xQueueReset(gsmRxQueue);
	return Ret_ERROR;
}

RetStateType putCharRequest(uint8_t charNumber, char *anserString, uint16_t delay) {
	xQueueReset(gsmRxQueue);
	memset(line, NULL, sizeof line);
	putchar(charNumber);
	osDelay(250);
	for (int i = 0; i < delay; i++) {
		readLine(line, sizeof(line));
		if (strstr(line, anserString) != NULL) {
			xQueueReset(gsmRxQueue);
			return Ret_OK;
		} else
			osDelay(250);
	}
	xQueueReset(gsmRxQueue);
	return Ret_ERROR;
}

RetStateType executeDataRequest(char *executeCommand, char *anserString, uint16_t bufNumber, uint16_t delay) {
	xQueueReset(gsmRxQueue);
	memset(line, NULL, sizeof line);
	printf(executeCommand);
	osDelay(250);
	for (int i = 0; i < delay; i++) {
		readLine(line, sizeof(line));
		if (strstr(line, anserString) != NULL) {
			for (int i = 0; sizeof(smsLine); i++) {
				if ((line[i] >= '0') && (line[i] <= '9')) {
					osMutexWait(inputRegMutex, osWaitForever);
					mbRegInputBuf.A3[bufNumber] = atoi(&line[i]);
					osMutexRelease(inputRegMutex);
					break;
				}
			}
			xQueueReset(gsmRxQueue);
			memset(line, NULL, sizeof line);
			return Ret_OK;
		} else
			osDelay(250);
	}
	xQueueReset(gsmRxQueue);
	memset(line, NULL, sizeof line);
	return Ret_ERROR;
}

void intToBin(uint16_t word, int count, uint8_t *out, uint8_t shift) {
	unsigned int mask = 1U << (count - 1);
	for (int i = 0; i < count; i++) {
		out[i + shift] = (word & mask) ? 1 : 0;
		word <<= 1;
	}
}

void periodSmsSendTimerCallback(void const *argument) {
	static GSMMessageType message;
	message = PeriodSend;
	xQueueSend(gsmQueue, &message, 0);
}

void httpsSendTimerCallback(void const *argument) {
	static GSMMessageType message;
	message = httpsSendData;
	xQueueSend(gsmQueue, &message, 0);
}

void smsCheckTimerCallback(void const *argument) {
	static GSMMessageType message;
	message = CheckIncom;
	xQueueSend(gsmQueue, &message, 0);
}

void signalCheckTimerCallback(void const *argument) {
	static GSMMessageType message;
	message = CheckSignal;
	xQueueSend(gsmQueue, &message, 0);
}

void modemInitTimerCallback(void const *argument) {
	static GSMMessageType message;
	message = ModemInit;
	xQueueSend(gsmQueue, &message, 0);
}

void usart3IRQHandler(void) {
	if (LL_USART_IsEnabledIT_ERROR(USART3) && LL_USART_IsActiveFlag_ORE(USART3)) {
		/* Call Error function */
		LL_USART_ClearFlag_PE(USART3);
	}
	if (LL_USART_IsEnabledIT_ERROR(USART3) && LL_USART_IsActiveFlag_FE(USART3)) {
		/* Call Error function */
		LL_USART_ClearFlag_PE(USART3);
	}
	if (LL_USART_IsEnabledIT_ERROR(USART3) && LL_USART_IsActiveFlag_NE(USART3)) {
		/* Call Error function */
		LL_USART_ClearFlag_PE(USART3);
	}
	/* Приём ---------------------------------------------------*/
	if (LL_USART_IsActiveFlag_RXNE(USART3) && LL_USART_IsEnabledIT_RXNE(USART3)) {
		usart3RxIsr();
	}
	/* Передача ------------------------------------------------*/
	if (LL_USART_IsEnabledIT_TXE(USART3) && LL_USART_IsActiveFlag_TXE(USART3)) {
		usart3TxIsr();
	}
}
