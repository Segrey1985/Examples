/*
 * regulatorsPID.c
 *
 *  Created on: 4 C�нв. 2018 г.
 *      Author: SVB
 */

#include "mbSlave.h"
#include "mbMaster.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "port.h"
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"
#include "adc.h"
#include "i2cEEPROM.h"

static float setpointTempHeat;		// Te��e�������� ������� ������� �����e���
static float setpointTempBoiler;     // Te��e�������� ������� ��������� �������
float setpointTempHeatGSM = 0;     // Te��e�������� ������� ��������� ������� �����e���� �� SMS

static uint32_t runTimeHeatReg;     // ��e��, ������e ���e� ���������� ������ �e�������� �����e��� � ��e�. ����e � ������e������
static uint16_t runTimeGvsReg;     // ��e��, ������e ���e� ���������� ������ �e�������� ��C � ��e�. ����e � ������e������

static float intPartGvs = 0;
static float intPartHeat = 0;

static void heatValveOpenStart(void);
static void heatValveOpenStop(void);
static void heatValveCloseStart(void);
static void heatValveCloseStop(void);
static void gvsValveOpenStart(void);
static void gvsValveOpenStop(void);
static void gvsValveCloseStart(void);
static void gvsValveCloseStop(void);
static void calcHeatingCircuitTempReg(void);
static void calcGvsCircuitTempReg(void);

const float *ptr_BoilerCircuitTemp = (float*) &mbRegInputBuf.A3[8];     // Te��e������ ��������� �������
const float *ptr_OutsideAirTemp = (float*) &mbRegInputBuf.A3[10];     // Te��e������ ��������� �������
const float *ptr_HeatingCircuitTemp = (float*) &mbRegInputBuf.A3[12];     // Te��e������ ������� �����e���
const float *ptr_GVSCircuitTemp = (float*) &mbRegInputBuf.A3[14];     // Te��e������ ������� ��C
const float *setpointTempGvs = (float*) &mbRegHoldingBuf.A3[58];     // Te��e�������� ������� ������� ��C
const float *ptr_HeatingCircuitPress1 = (float*) &mbRegInputBuf.A3[0];     //
const float *ptr_HeatingCircuitPress2 = (float*) &mbRegInputBuf.A3[2];     //
const float *setpointColdWaterPress = (float*) &mbRegHoldingBuf.Settings[4];     //
const float *pressColdWaterVal = (float*) &mbRegInputBuf.A3[4];
;
//

extern osMutexId inputRegMutex;
extern osMutexId holdingRegMutex;
extern osTimerId gvsValveTimer;
extern osTimerId heatValveTimer;
extern QueueHandle_t masterMessageQueue;
/*
 * ������ ������� ������e�� ������������ �����e��� � ����������� �� �e��e������ ��������� ������� � ����� X�C
 * �e���e ����e��e t = X1�C  ����e��ee t = X2�C
 * ������� ��� ������e���  y = (965 - 25 * x) / 17
 * ���e�� ��e��� ������� �������e��� �� ������e index = (uint8_t)(t-10)/-5 ��e t �e��e������ ��������� �������
 */
const float defHeatCircuitTempSetArray[] = { 45, 49.41, 56.76, 64.12, 71.47, 78.82, 86.18, 95 };
const float defOutTempArray[] = { 10, 5, 0, -5, -10, -15, -20, -25 };

void (*heatValveStart)(void);     // ������e�� �� ������� ������� ������� ������� �����e��� (�����e ��� �e���e)
void (*heatValveStop)(void);     // ������e�� �� ������� ��������� ������� ������� �����e��� (�����e ��� �e���e)

void (*gvsValveStart)(void);     // ������e�� �� ������� ������� ������� ������� ��C (�����e ��� �e���e)
void (*gvsValveStop)(void);     // ������e�� �� ������� ��������� ������� ������� ��C (�����e ��� �e���e)

/********************************************************************************************************
 * ��������� ��������� � �������� ����������� ��������� � ���
 */

ValveStateTypeGvs gvsStateType;		// ��������� ������� ���
ValveStateTypeHeat heatStateType;     // ��������� ������� ���������

BoilerStateType masterState;				// C�������e �������� �����
BoilerStateType slaveState;					// C�������e �������� �����

BoilerCircuitStateType boilerCircuitState;     // C�������e ��������� �������

/***************************************************************************
 * �e������� ��������� �������
 */
void boilerCircuitReg(void) {

	static float OutTempArray[8];
	float tempErr;				// P�������������e
	float maxSumTempErr;     // ������� ������� ����� ���������������
	static float sumTempErr = 500;     // C���� ��������������� �� �e��e�����e � ������e
	float *ptr;
	static Master_Message_Type message;
	float bottomHeatArrayPointVal;
	float topHeatArrayPointVal;
	float outAirTemp;				// Te��e������ ��������� �������
	float leftSetpointOutTempArrayVal;
	int rightSetpointOutTempArrayVal;
	int leftSetpointOutTempArray;
	int rightSetpointOutTempArray;
	static int CounterHeatDisagreementError, CntrColdWaterDeltaError;
	static float recentSetpointTempBoiler = 0;				// C���� ��������������� �� �e��e�����e � ������e
	float deltaTempOutTempArray, stepTempOutTempArray, deltaPressHeatVal, setDeltaPressHeatVal, deltaPressColdWaterVal,
			setDeltaPressColdWaterVal;

	osMutexWait(holdingRegMutex, osWaitForever);
	maxSumTempErr = mbRegHoldingBuf.A3[66];
	OutTempArray[0] = *(float*) &mbRegHoldingBuf.OutsideTemp[0];
	OutTempArray[7] = *(float*) &mbRegHoldingBuf.OutsideTemp[14];
	ptr = (float*) &mbRegHoldingBuf.A3[16];
	setDeltaPressHeatVal = *(float*) &mbRegHoldingBuf.Settings[4];
	setDeltaPressColdWaterVal = *(float*) &mbRegHoldingBuf.Settings[6];
	osMutexRelease(holdingRegMutex);

	setDeltaPressHeatVal = (setDeltaPressHeatVal == 0) ? 0.25 : setDeltaPressHeatVal;

	setDeltaPressColdWaterVal = (setDeltaPressColdWaterVal == 0) ? 1.0 : setDeltaPressColdWaterVal;

	osMutexWait(inputRegMutex, osWaitForever);
	outAirTemp = (int) *ptr_OutsideAirTemp;
	deltaPressHeatVal = *ptr_HeatingCircuitPress1 - *ptr_HeatingCircuitPress2;
	deltaPressColdWaterVal = *pressColdWaterVal - setDeltaPressColdWaterVal;
	osMutexRelease(inputRegMutex);

	if ((mbDiscrInputsBuf.A3.K3_1IsOn == 0 || mbDiscrInputsBuf.A3.K3_2IsOn == 0)
			&& (deltaPressHeatVal < setDeltaPressHeatVal && mbDiscrInputsBuf.A3.HeatCircPrEr == 0
					&& mbDiscrInputsBuf.A3.HeatCircRevPrEr == 0)) {
		CounterHeatDisagreementError++;
		if (CounterHeatDisagreementError > 240) {
			CounterHeatDisagreementError = 240;     // 40 ��� ��������, ������ �� �������
			mbDiscrInputsBuf.A3.heatPressDisagreementError = 1;
		}
	} else {
		CounterHeatDisagreementError = 0;
		mbDiscrInputsBuf.A3.heatPressDisagreementError = 0;
	}

	if (deltaPressColdWaterVal < 0 && mbDiscrInputsBuf.A3.ColdWaterPrEr == 0) {
		CntrColdWaterDeltaError++;
		if (CntrColdWaterDeltaError > 240) {
			CntrColdWaterDeltaError = 240;     // 40 ��� ��������, ������ �� �������
			mbDiscrInputsBuf.A3.ColdWaterDisagreementError = 1;
		}
	} else {
		CntrColdWaterDeltaError = 0;
		mbDiscrInputsBuf.A3.ColdWaterDisagreementError = 0;
	}

	/*********************���������� �������� ��� ������� ����. ��������� �������*****************/
	if (outAirTemp < OutTempArray[7])
		outAirTemp = OutTempArray[7];
	else if (outAirTemp > OutTempArray[0])
		outAirTemp = OutTempArray[0];
	deltaTempOutTempArray = OutTempArray[7] - OutTempArray[0];
	stepTempOutTempArray = deltaTempOutTempArray / 7;

	osMutexWait(holdingRegMutex, osWaitForever);
	for (int i = 1; i < 7; i++) {
		OutTempArray[i] = OutTempArray[0] + stepTempOutTempArray * i;
		ptr = (float*) &mbRegHoldingBuf.OutsideTemp[i * 2];     // �������� ������
		*ptr = OutTempArray[i];
	}

	osMutexRelease(holdingRegMutex);
	for (int i = 0; i < 8; i++) {
		rightSetpointOutTempArray = i;
		rightSetpointOutTempArrayVal = (int) OutTempArray[i];
		if (outAirTemp <= OutTempArray[i]) {
			leftSetpointOutTempArrayVal = rightSetpointOutTempArrayVal;
			leftSetpointOutTempArray = i;
			continue;
		} else if (outAirTemp > OutTempArray[i]) {
			leftSetpointOutTempArrayVal = OutTempArray[i - 1];
			leftSetpointOutTempArray = rightSetpointOutTempArray - 1;
			break;
		}
	}

	osMutexWait(holdingRegMutex, osWaitForever);
	topHeatArrayPointVal = *((float*) &mbRegHoldingBuf.A3[rightSetpointOutTempArray * 2]);     // �������
	bottomHeatArrayPointVal = *((float*) &mbRegHoldingBuf.A3[leftSetpointOutTempArray * 2]);     // �������
	ptr = (float*) &mbRegHoldingBuf.A3[16];
	osMutexRelease(holdingRegMutex);
	if (mbCoilsBuf.A3.SetHeatConst == 1)
		setpointTempHeat = *ptr;
	else {
		if (topHeatArrayPointVal == bottomHeatArrayPointVal)
			setpointTempHeat = topHeatArrayPointVal;
		else {
			setpointTempHeat = fabs(topHeatArrayPointVal - bottomHeatArrayPointVal)
					/ fabs(rightSetpointOutTempArrayVal - leftSetpointOutTempArrayVal)
					* fabs(outAirTemp - leftSetpointOutTempArrayVal) + bottomHeatArrayPointVal;
		}
	}

	if (mbDiscrInputsBuf.A3.DHW_Priority == 1 && mbCoilsBuf.A3.DhwOption == 1)
		setpointTempHeat -= 20;
	if (setpointTempHeat < 5)     // ��� �� �� ��������
		setpointTempHeat = 5;
	else if (setpointTempHeat > 95)     // ��� �� �� ���������
		setpointTempHeat = 95;
	osMutexWait(inputRegMutex, osWaitForever);
	ptr = (float*) &mbRegInputBuf.A3[34];     // ������ ������������ �������� � ������� Modbus
	*ptr = setpointTempHeat;
	osMutexRelease(inputRegMutex);

	if (mbDiscrInputsBuf.A3.DHW_Priority == 1) {
		if (setpointTempHeat + 25 > *setpointTempGvs + 20)
			setpointTempBoiler = setpointTempHeat + 25;
		else
			setpointTempBoiler = *setpointTempGvs + 20;
		if (setpointTempBoiler < 95)
			setpointTempBoiler = 95;
		if (setpointTempBoiler > 100)
			setpointTempBoiler = 100;
	} else if (mbCoilsBuf.A3.DhwOption == 1) {
		if (setpointTempHeat + 10 > *setpointTempGvs + 20)
			setpointTempBoiler = setpointTempHeat + 10;
		else
			setpointTempBoiler = *setpointTempGvs + 20;
		if (setpointTempBoiler < 95)
			setpointTempBoiler = 95;
		if (setpointTempBoiler > 100)
			setpointTempBoiler = 100;
	} else {
		setpointTempBoiler = setpointTempHeat + 10;
		if (setpointTempBoiler < 85)
			setpointTempBoiler = 85;
		if (setpointTempBoiler > 100)
			setpointTempBoiler = 100;
	}

	if (recentSetpointTempBoiler != setpointTempBoiler) {
		recentSetpointTempBoiler = setpointTempBoiler;
		ptr = (float*) &mbRegInputBuf.A3[32];
		*ptr = setpointTempBoiler;
		message.Type = WriteHoldings;
		message.SlaveId = MB_A1_ID;
		message.Address = 2000;
		message.RegCnt = 2;
		message.DataPtr = (uint8_t*) &mbRegInputBuf.A3[32];
		xQueueSend(masterMessageQueue, &message, 0);		// ���������� �� �1

		message.Type = WriteHoldings;
		message.SlaveId = MB_A2_ID;
		message.Address = 2000;
		message.RegCnt = 2;
		message.DataPtr = (uint8_t*) &mbRegInputBuf.A3[32];
		xQueueSend(masterMessageQueue, &message, 0);		// ���������� �� �2
	}

	osMutexWait(inputRegMutex, osWaitForever);
	tempErr = setpointTempBoiler - *ptr_BoilerCircuitTemp;     // P�������������e
	osMutexRelease(inputRegMutex);
	if (fabs(tempErr) < 1)
		return;
	sumTempErr -= tempErr;
	if (sumTempErr >= maxSumTempErr) {
		switch (boilerCircuitState) {
		case MasterBoiler_On:     // ������e� ������
			masterOff();     // ����e��e� ������ ������
			boilerCircuitState = BoilersOff;
			break;
		case SlaveBoiler_On:     // �������� �����
			slaveOff();     // ����e��e� ������ �����
			boilerCircuitState = BoilersOff;
			break;
		case Boilers_On:     // �������� ��� �����
			slaveOff();     // ����e��e� ������ �����
			boilerCircuitState = MasterBoiler_On;
			break;
		}
		sumTempErr = maxSumTempErr;
	}
	if (sumTempErr <= 0) {
		switch (boilerCircuitState) {
		case BoilersOff:     // ��� ����� ������e��
			if (masterOn() == Ret_OK)
				boilerCircuitState = MasterBoiler_On;
			else if (slaveOn() == Ret_OK)
				boilerCircuitState = SlaveBoiler_On;
			break;
		case MasterBoiler_On:     // ������e� ������
			if (slaveOn() == Ret_OK)
				boilerCircuitState = Boilers_On;     // �������e� �����
			break;
		case SlaveBoiler_On:     // �������� �����
			if (masterOn() == Ret_OK)
				boilerCircuitState = Boilers_On;     // �������e� ������
			break;
		}
		sumTempErr = maxSumTempErr;
	}
}

RetStateType masterOn(void) {
	static Master_Message_Type mbMessage = { WriteCoil, MB_A1_ID, 3000, 0xFF00,
	NULL };
	if (pumpCircOnOff == 1)
		return Ret_ERROR;
	if (masterState == BoilerErr)
		return Ret_ERROR;
	if (mbCoilsBuf.A3.MasterSlave) {
		mbMessage.SlaveId = MB_A1_ID;
	} else {
		mbMessage.SlaveId = MB_A2_ID;
	}
	xQueueSend(masterMessageQueue, &mbMessage, 0);     // ��������� ������
	return Ret_OK;
}

void masterOff(void) {
	static Master_Message_Type mbMessage = { WriteCoil, MB_A1_ID, 3000, 0x0000,
	NULL };
	if (mbCoilsBuf.A3.MasterSlave) {
		mbMessage.SlaveId = MB_A1_ID;
	} else {
		mbMessage.SlaveId = MB_A2_ID;
	}
	xQueueSend(masterMessageQueue, &mbMessage, 0);     // ��������� ������
}

RetStateType slaveOn(void) {
	static Master_Message_Type mbMessage = { WriteCoil, MB_A2_ID, 3000, 0xFF00,
	NULL };
	if (pumpCircOnOff == 1)
		return Ret_ERROR;
	if (slaveState == BoilerErr)
		return Ret_ERROR;
	if (mbCoilsBuf.A3.MasterSlave) {
		mbMessage.SlaveId = MB_A2_ID;
	} else {
		mbMessage.SlaveId = MB_A1_ID;
	}
	xQueueSend(masterMessageQueue, &mbMessage, 0);     // ��������� ������
	return Ret_OK;
}

void slaveOff(void) {
	static Master_Message_Type mbMessage = { WriteCoil, MB_A2_ID, 3000, 0x0000,
	NULL };
	if (mbCoilsBuf.A3.MasterSlave == 1) {
		mbMessage.SlaveId = MB_A2_ID;
	} else {
		mbMessage.SlaveId = MB_A1_ID;
	}
	xQueueSend(masterMessageQueue, &mbMessage, 0);     // ��������� ������
}

/****************************************************************************
 * ��������� ����e��� �e����������� � ������e �����e���
 */
void heatCircuitTempRegStop(void) {
	OUT_K6_1_MORE_OFF;
	OUT_K6_1_LESS_OFF;
}

/****************************************************************************
 * ��������� ����e��� �e����������� � ������e ��C
 */
void gvsCircuitTempRegStop(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	OUT_K6_2_MORE_OFF;
	OUT_K6_2_LESS_OFF;
}

/*****************************************************************************
 * ������ ������� ������� �����e��� �� ���������e (�����e)
 */
static void heatValveOpenStart(void) {
	heatValveStop = heatValveCloseStop;     // ������e�� �� ������� ���������
	OUT_K6_1_MORE_ON;
}

/*****************************************************************************
 * ��������� ������� ������� �����e��� ���������e (�����e)
 */
static void heatValveOpenStop(void) {
	OUT_K6_1_MORE_OFF;
}

/*****************************************************************************
 * ������ ������� ������� �����e��� �� ���������e (�e���e)
 */
static void heatValveCloseStart(void) {
	heatValveStop = heatValveOpenStop;     // ������e�� �� ������� ���������
	OUT_K6_1_LESS_ON;
}

/*****************************************************************************
 * ��������� ������� ������� �����e��� ���������e (�e���e)
 */
static void heatValveCloseStop(void) {
	OUT_K6_1_LESS_OFF;
}

/*****************************************************************************
 * ������ ������� ������� ��C �� ���������e (�����e)
 */
static void gvsValveOpenStart(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	gvsValveStop = gvsValveCloseStop;
	OUT_K6_2_MORE_ON;
}

/*
 ��������� ������� ������� ��C ���������e (�����e)
 */
static void gvsValveOpenStop(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	OUT_K6_2_MORE_OFF;
}

/*
 ������ ������� ������� ��C �� ���������e (�e���e)
 */
static void gvsValveCloseStart(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	gvsValveStop = gvsValveOpenStop;
	OUT_K6_2_LESS_ON;
}

/*****************************************************************************
 * ��������� ������� ������� ��C ���������e (�e���e)
 */
static void gvsValveCloseStop(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	OUT_K6_2_LESS_OFF;
}
/******************************************************************************
 * ������ ��������� ������� ���������
 */
void StartHeatValveTask(void const *argument) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	heatStateType = ValveOff;
	int regulationPeriod;
	for (;;) {
		osMutexWait(holdingRegMutex, osWaitForever);
		regulationPeriod = (uint16_t) mbRegHoldingBuf.A3[67];
		osMutexRelease(holdingRegMutex);
		if (regulationPeriod <= 0)
			regulationPeriod = CALC_PERIOD_HEAT * 1000;
		calcHeatingCircuitTempReg();
		if (heatStateType == ValveReady && (IN_KM_1A == 0 || IN_KM_2A == 0)) {
			osTimerStart(heatValveTimer, runTimeHeatReg);
			(*heatValveStart)();
		}
		vTaskDelayUntil(&xLastWakeTime, regulationPeriod * 1000);     // ����� �� ��������� �������
	}
}

static void calcHeatingCircuitTempReg(void) {

	float tempErr;
	static float tempErrPrev = 0;
	uint32_t influencePeriod;
	float controlAction;
	float prPart;
	float difPart;
	float prPartFromMb;
	float intPartFromMb;
	float difPartFromMb;
	float *ptr;
	static int32_t HeatCounterMore = 0;
	float setTempErrOv;

	if (mbCoilsBuf.A3.K6_1_Off == 0) {		// ��������
		heatCircuitTempRegStop();		// ��������� ������
		heatStateType = ValveOff;
		mbDiscrInputsBuf.A3.heatTempDisagreementError = 0;
		return;
	}
	if (mbCoilsBuf.A3.K6_1ManualCnt == 1) {     // ������ ����������
		heatStateType = ValveManual;
		mbDiscrInputsBuf.A3.heatTempDisagreementError = 0;
		return;
	}
	heatStateType = ValveReady;

	osMutexWait(holdingRegMutex, osWaitForever);
	influencePeriod = mbRegHoldingBuf.A3[68];     // uint16_t
	ptr = (float*) &mbRegHoldingBuf.A3[46];		// �������� ������
	prPartFromMb = *ptr;
	ptr = (float*) &mbRegHoldingBuf.A3[48];     // float
	intPartFromMb = *ptr;
	ptr = (float*) &mbRegHoldingBuf.A3[50];     // float
	difPartFromMb = *ptr;
	tempErr = setpointTempHeat - *ptr_HeatingCircuitTemp;     // ��������������e
	setTempErrOv = mbRegHoldingBuf.Settings[3];     // uint16_t
	osMutexRelease(holdingRegMutex);

	if (influencePeriod <= 0)
		influencePeriod = INFLUENCE_PERIOD_HEAT * 1000;

	setTempErrOv = (setTempErrOv == 0) ? 12 : setTempErrOv;

	if (mbCoilsBuf.A3.K6_1ManualCnt == 0 && mbCoilsBuf.A3.K6_1_Off == 1 && mbDiscrInputsBuf.A3.OutAirTempEr == 0
			&& mbDiscrInputsBuf.A3.HeatCircTempEr == 0) {
		if (tempErr > setTempErrOv) {
			HeatCounterMore++;
			if (HeatCounterMore > 20) {
				HeatCounterMore = 20;     // 10 ��� �������� �� ����������, ������ �� �������
				mbDiscrInputsBuf.A3.heatTempDisagreementError = 1;
			}
		} else if (fabs(tempErr) < 3) {
			HeatCounterMore = 0;
			mbDiscrInputsBuf.A3.heatTempDisagreementError = 0;
		}
	} else {
		HeatCounterMore = 0;
		mbDiscrInputsBuf.A3.heatTempDisagreementError = 0;
	}

	prPart = prPartFromMb * tempErr;  			// ���������������� ������������
	intPartHeat = intPartHeat + intPartFromMb * tempErr;     // ���e�������� ������������
	if (intPartHeat > INT_HEAT_MAX)
		intPartHeat = INT_HEAT_MAX;     // ����������e� ��e���
	if (intPartHeat < INT_HEAT_MIN)
		intPartHeat = INT_HEAT_MIN;     // ����������e� �����
	difPart = difPartFromMb * (tempErr - tempErrPrev);     // ����e�e��������� ������������
	tempErrPrev = tempErr;
	controlAction = prPart + intPartHeat + difPart;     // �������e� ����e�����e � %
	if (controlAction > 100)
		controlAction = 100; 		// ����������e� ��e���
	if (controlAction < -100)
		controlAction = -100;     // ����������e� �����
	runTimeHeatReg = (uint32_t) fabsf(influencePeriod * 1000 / 100 * controlAction);     // �������e� ��e�� ������ ������� �e��������
	if (runTimeHeatReg < MIN_RUN_TIME_HEAT)
		runTimeHeatReg = 0;
	if (runTimeHeatReg > influencePeriod * 1000)
		runTimeHeatReg = influencePeriod * 1000;
	if (controlAction > 0)
		heatValveStart = heatValveOpenStart;
	else
		heatValveStart = heatValveCloseStart;
}

void StartGvsValveTask(void const *argument) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	gvsStateType = GvsValveOff;
	int regulationPeriod;
	for (;;) {
		osMutexWait(holdingRegMutex, osWaitForever);
		regulationPeriod = (uint16_t) mbRegHoldingBuf.A3[69];
		osMutexRelease(holdingRegMutex);
		if (regulationPeriod <= 0)
			regulationPeriod = CALC_PERIOD_GVS * 1000;
		calcGvsCircuitTempReg();
		if (gvsStateType == GvsValveReady && (IN_KM_1A == 0 || IN_KM_2A == 0)) {
			osTimerStart(gvsValveTimer, runTimeGvsReg);
			(*gvsValveStart)();
		}
		vTaskDelayUntil(&xLastWakeTime, regulationPeriod * 1000);     // ����� �� ��������� �������F
	}
}
static void calcGvsCircuitTempReg(void) {
	static float tempErrPrev = 0;
	uint32_t influencePeriod;
	float prPartFromMb;
	float intPartFromMb;
	float difPartFromMb;
	static int32_t DhwCounterMore = 0;
	float *ptr;
	float tempErr; 					// P�������������e
	float prPart; 					// ���������������� ������������
	float difPart;  				// ����e�e��������� ������������
	float controlAction;			// T�e��e��e ���������ee ����e�����e � %
	static int32_t DHW_PriorityCounterMore = 0;     // ��e�����ee ��������������e
	static int32_t DHW_PriorityCounterLess = 0;     // ��e�����ee ��������������e
	float setTempErrDhw;
	if (mbCoilsBuf.A3.DhwOption == 0) {
		mbDiscrInputsBuf.A3.TempDhwMin = 0;
		mbDiscrInputsBuf.A3.DHW_Priority = 0;
		return;
	}
	if (mbCoilsBuf.A3.K6_2_Off == 0) {     // ��������
		gvsCircuitTempRegStop();		// ��������� ������
		gvsStateType = GvsValveOff;
		mbDiscrInputsBuf.A3.TempDhwMin = 0;
		mbDiscrInputsBuf.A3.DHW_Priority = 0;
		return;
	}
	if (mbCoilsBuf.A3.K6_2ManualCnt == 1) {     // ������ ����������
		gvsStateType = GvsValveManual;
		mbDiscrInputsBuf.A3.TempDhwMin = 0;
		mbDiscrInputsBuf.A3.DHW_Priority = 0;
		return;
	}
	gvsStateType = GvsValveReady;

	osMutexWait(holdingRegMutex, osWaitForever);
	tempErr = *setpointTempGvs - *ptr_GVSCircuitTemp; 				// ��������������e
	influencePeriod = mbRegHoldingBuf.A3[70];     // uint16_t
	ptr = (float*) &mbRegHoldingBuf.A3[52];		// �������� ������
	prPartFromMb = *ptr;
	ptr = (float*) &mbRegHoldingBuf.A3[54];     // float
	intPartFromMb = *ptr;
	ptr = (float*) &mbRegHoldingBuf.A3[56];     // float
	difPartFromMb = *ptr;
	osMutexRelease(holdingRegMutex);

	setTempErrDhw = (setTempErrDhw == 0) ? 12 : setTempErrDhw;

	if (mbCoilsBuf.A3.K6_2ManualCnt == 0 && mbCoilsBuf.A3.K6_2_Off == 1 && mbDiscrInputsBuf.A3.GvsCircTempEr == 0) {
		if (tempErr > setTempErrDhw) {
			DhwCounterMore++;
			if (DhwCounterMore > 20) {
				DhwCounterMore = 20;     // 20 ��� �������� �� ����������, ������ �� �������
				mbDiscrInputsBuf.A3.TempDhwMin = 1;
			}
		} else if (fabs(tempErr) < 2.5) {
			DhwCounterMore = 0;
			mbDiscrInputsBuf.A3.TempDhwMin = 0;
		}
	} else {
		DhwCounterMore = 0;
		mbDiscrInputsBuf.A3.TempDhwMin = 0;
	}

	if (influencePeriod == 0)
		influencePeriod = INFLUENCE_PERIOD_GVS * 1000;

	if (mbCoilsBuf.A3.PriorityGvs == 1 && mbCoilsBuf.A3.K6_2ManualCnt == 0 && mbCoilsBuf.A3.K6_2_Off == 1
			&& mbCoilsBuf.A3.DhwOption == 1) {
		if (tempErr > 6) {
			DHW_PriorityCounterMore++;
			if (DHW_PriorityCounterMore > 10) {
				DHW_PriorityCounterMore = 10;     // 10 ��� �������� �� ����������, ������ �� �������
				mbDiscrInputsBuf.A3.DHW_Priority = 1;
			}
		} else if (tempErr < 2.5) {
			DHW_PriorityCounterLess++;
			if (DHW_PriorityCounterLess > 20) {
				DHW_PriorityCounterLess = 20;     // 20 ��� �������� �� ����������, ������ �� �������
				mbDiscrInputsBuf.A3.DHW_Priority = 0;
			}
		} else {
			DHW_PriorityCounterMore = 0;
			DHW_PriorityCounterLess = 0;
		}
	} else {
		DHW_PriorityCounterMore = 0;
		DHW_PriorityCounterLess = 0;
		mbDiscrInputsBuf.A3.DHW_Priority = 0;
	}
	prPart = prPartFromMb * tempErr;  		// ���������������� ������������
	intPartGvs = intPartGvs + intPartFromMb * tempErr;     // ���e�������� ������������
	if (intPartGvs > INT_GVS_MAX)
		intPartGvs = INT_GVS_MAX;     // ����������e� ��e���
	if (intPartGvs < INT_GVS_MIN)
		intPartGvs = INT_GVS_MIN;     // ����������e� �����
	difPart = difPartFromMb * (tempErr - tempErrPrev);     // ����e�e��������� ������������
	tempErrPrev = tempErr;
	controlAction = prPart + intPartGvs + difPart;     // �������e� ����e�����e � %
	if (controlAction > 100)
		controlAction = 100; 		// ����������e� ��e���
	if (controlAction < -100)
		controlAction = -100; 		// ����������e� �����
	runTimeGvsReg = (uint16_t) fabsf(influencePeriod * 1000 / 100 * controlAction);     // �������e� ��e�� ������ ������� �e��������
	if (runTimeGvsReg < MIN_RUN_TIME_GVS)
		runTimeGvsReg = 0;
	if (runTimeGvsReg > influencePeriod * 1000)
		runTimeGvsReg = influencePeriod * 1000;
	if (controlAction > 0)
		gvsValveStart = gvsValveOpenStart;
	else
		gvsValveStart = gvsValveCloseStart;
}

void heatValveTimerCallback(void const *argument) {
	if (heatStateType == ValveReady)
		heatCircuitTempRegStop();
}

void gvsValveTimerCallback(void const *argument) {
	if (gvsStateType == GvsValveReady)
		gvsCircuitTempRegStop();
}

