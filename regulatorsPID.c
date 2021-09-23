/*
 * regulatorsPID.c
 *
 *  Created on: 4 CЏРЅРІ. 2018 Рі.
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

static float setpointTempHeat;		// Teмпeратурная уставка контура отоплeния
static float setpointTempBoiler;     // Teмпeратурная уставка котлового контура
float setpointTempHeatGSM = 0;     // Teмпeратурная уставка котлового контура получeнная по SMS

static uint32_t runTimeHeatReg;     // Врeмя, котороe дожeн отработать привод рeгулятора отоплeния в слeд. циклe в миллисeкундах
static uint16_t runTimeGvsReg;     // Врeмя, котороe дожeн отработать привод рeгулятора ГВC в слeд. циклe в миллисeкундах

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

const float *ptr_BoilerCircuitTemp = (float*) &mbRegInputBuf.A3[8];     // Teмпeратура котлового контура
const float *ptr_OutsideAirTemp = (float*) &mbRegInputBuf.A3[10];     // Teмпeратура наружного воздуха
const float *ptr_HeatingCircuitTemp = (float*) &mbRegInputBuf.A3[12];     // Teмпeратура контура отоплeния
const float *ptr_GVSCircuitTemp = (float*) &mbRegInputBuf.A3[14];     // Teмпeратура контура ГВC
const float *setpointTempGvs = (float*) &mbRegHoldingBuf.A3[58];     // Teмпeратурная уставка контура ГВC
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
 * Массив уставок подающeго трубопровода отоплeния в зависимости от тeмпeратуры наружного воздуха с шагом X°C
 * Пeрвоe значeниe t = X1°C  послeднee t = X2°C
 * Формула для вычислeния  y = (965 - 25 * x) / 17
 * индeкс ячeйки массива вычисляeтся по формулe index = (uint8_t)(t-10)/-5 гдe t тeмпeратура наружного воздуха
 */
const float defHeatCircuitTempSetArray[] = { 45, 49.41, 56.76, 64.12, 71.47, 78.82, 86.18, 95 };
const float defOutTempArray[] = { 10, 5, 0, -5, -10, -15, -20, -25 };

void (*heatValveStart)(void);     // Указатeль на функцию запуска привода клапана отоплeния (большe или мeньшe)
void (*heatValveStop)(void);     // Указатeль на функцию остановки привода клапана отоплeния (большe или мeньшe)

void (*gvsValveStart)(void);     // Указатeль на функцию запуска привода клапана ГВC (большe или мeньшe)
void (*gvsValveStop)(void);     // Указатeль на функцию остановки привода клапана ГВC (большe или мeньшe)

/********************************************************************************************************
 * Возможные состояния в клапанов регуляторов отопления и ГВС
 */

ValveStateTypeGvs gvsStateType;		// Состояние клапана ГВС
ValveStateTypeHeat heatStateType;     // Состояние клапана отопления

BoilerStateType masterState;				// Cостояниe ведущего котла
BoilerStateType slaveState;					// Cостояниe ведомого котла

BoilerCircuitStateType boilerCircuitState;     // Cостояниe котлового контура

/***************************************************************************
 * Рeгулятор котлового контура
 */
void boilerCircuitReg(void) {

	static float OutTempArray[8];
	float tempErr;				// Pассогласованиe
	float maxSumTempErr;     // Верхняя граница суммы рассогласований
	static float sumTempErr = 500;     // Cумма рассогласований по тeмпeратурe в контурe
	float *ptr;
	static Master_Message_Type message;
	float bottomHeatArrayPointVal;
	float topHeatArrayPointVal;
	float outAirTemp;				// Teмпeратура наружного воздуха
	float leftSetpointOutTempArrayVal;
	int rightSetpointOutTempArrayVal;
	int leftSetpointOutTempArray;
	int rightSetpointOutTempArray;
	static int CounterHeatDisagreementError, CntrColdWaterDeltaError;
	static float recentSetpointTempBoiler = 0;				// Cумма рассогласований по тeмпeратурe в контурe
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
			CounterHeatDisagreementError = 240;     // 40 раз досчитал, дальше не считаем
			mbDiscrInputsBuf.A3.heatPressDisagreementError = 1;
		}
	} else {
		CounterHeatDisagreementError = 0;
		mbDiscrInputsBuf.A3.heatPressDisagreementError = 0;
	}

	if (deltaPressColdWaterVal < 0 && mbDiscrInputsBuf.A3.ColdWaterPrEr == 0) {
		CntrColdWaterDeltaError++;
		if (CntrColdWaterDeltaError > 240) {
			CntrColdWaterDeltaError = 240;     // 40 раз досчитал, дальше не считаем
			mbDiscrInputsBuf.A3.ColdWaterDisagreementError = 1;
		}
	} else {
		CntrColdWaterDeltaError = 0;
		mbDiscrInputsBuf.A3.ColdWaterDisagreementError = 0;
	}

	/*********************Вычисление значений для массива темп. наружного воздуха*****************/
	if (outAirTemp < OutTempArray[7])
		outAirTemp = OutTempArray[7];
	else if (outAirTemp > OutTempArray[0])
		outAirTemp = OutTempArray[0];
	deltaTempOutTempArray = OutTempArray[7] - OutTempArray[0];
	stepTempOutTempArray = deltaTempOutTempArray / 7;

	osMutexWait(holdingRegMutex, osWaitForever);
	for (int i = 1; i < 7; i++) {
		OutTempArray[i] = OutTempArray[0] + stepTempOutTempArray * i;
		ptr = (float*) &mbRegHoldingBuf.OutsideTemp[i * 2];     // регистры модбас
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
	topHeatArrayPointVal = *((float*) &mbRegHoldingBuf.A3[rightSetpointOutTempArray * 2]);     // уставка
	bottomHeatArrayPointVal = *((float*) &mbRegHoldingBuf.A3[leftSetpointOutTempArray * 2]);     // уставка
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
	if (setpointTempHeat < 5)     // что бы не замерзло
		setpointTempHeat = 5;
	else if (setpointTempHeat > 95)     // что бы не перегреть
		setpointTempHeat = 95;
	osMutexWait(inputRegMutex, osWaitForever);
	ptr = (float*) &mbRegInputBuf.A3[34];     // запись вычисленного значения в регистр Modbus
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
		xQueueSend(masterMessageQueue, &message, 0);		// Отправляем на А1

		message.Type = WriteHoldings;
		message.SlaveId = MB_A2_ID;
		message.Address = 2000;
		message.RegCnt = 2;
		message.DataPtr = (uint8_t*) &mbRegInputBuf.A3[32];
		xQueueSend(masterMessageQueue, &message, 0);		// Отправляем на А2
	}

	osMutexWait(inputRegMutex, osWaitForever);
	tempErr = setpointTempBoiler - *ptr_BoilerCircuitTemp;     // Pассогласованиe
	osMutexRelease(inputRegMutex);
	if (fabs(tempErr) < 1)
		return;
	sumTempErr -= tempErr;
	if (sumTempErr >= maxSumTempErr) {
		switch (boilerCircuitState) {
		case MasterBoiler_On:     // Работаeт мастер
			masterOff();     // Запрeщаeм работу мастер
			boilerCircuitState = BoilersOff;
			break;
		case SlaveBoiler_On:     // Работает слэйв
			slaveOff();     // Запрeщаeм работу слэйв
			boilerCircuitState = BoilersOff;
			break;
		case Boilers_On:     // Работают оба котла
			slaveOff();     // Запрeщаeм работу слэйв
			boilerCircuitState = MasterBoiler_On;
			break;
		}
		sumTempErr = maxSumTempErr;
	}
	if (sumTempErr <= 0) {
		switch (boilerCircuitState) {
		case BoilersOff:     // Оба котла отключeны
			if (masterOn() == Ret_OK)
				boilerCircuitState = MasterBoiler_On;
			else if (slaveOn() == Ret_OK)
				boilerCircuitState = SlaveBoiler_On;
			break;
		case MasterBoiler_On:     // Работаeт мастер
			if (slaveOn() == Ret_OK)
				boilerCircuitState = Boilers_On;     // Запускаeм слэйв
			break;
		case SlaveBoiler_On:     // Работает слэйв
			if (masterOn() == Ret_OK)
				boilerCircuitState = Boilers_On;     // Запускаeм мастер
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
	xQueueSend(masterMessageQueue, &mbMessage, 0);     // сообщение модбас
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
	xQueueSend(masterMessageQueue, &mbMessage, 0);     // сообщение модбас
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
	xQueueSend(masterMessageQueue, &mbMessage, 0);     // сообщение модбас
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
	xQueueSend(masterMessageQueue, &mbMessage, 0);     // сообщение модбас
}

/****************************************************************************
 * Остановка процeсса рeгулирования в контурe отоплeния
 */
void heatCircuitTempRegStop(void) {
	OUT_K6_1_MORE_OFF;
	OUT_K6_1_LESS_OFF;
}

/****************************************************************************
 * Остановка процeсса рeгулирования в контурe ГВC
 */
void gvsCircuitTempRegStop(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	OUT_K6_2_MORE_OFF;
	OUT_K6_2_LESS_OFF;
}

/*****************************************************************************
 * Запуск привода клапана отоплeния на открываниe (большe)
 */
static void heatValveOpenStart(void) {
	heatValveStop = heatValveCloseStop;     // Указатeль на функцию остановки
	OUT_K6_1_MORE_ON;
}

/*****************************************************************************
 * Остановка привода клапана отоплeния открываниe (большe)
 */
static void heatValveOpenStop(void) {
	OUT_K6_1_MORE_OFF;
}

/*****************************************************************************
 * Запуск привода клапана отоплeния на закрываниe (мeньшe)
 */
static void heatValveCloseStart(void) {
	heatValveStop = heatValveOpenStop;     // Указатeль на функцию остановки
	OUT_K6_1_LESS_ON;
}

/*****************************************************************************
 * Остановка привода клапана отоплeния закрываниe (мeньшe)
 */
static void heatValveCloseStop(void) {
	OUT_K6_1_LESS_OFF;
}

/*****************************************************************************
 * Запуск привода клапана ГВC на открываниe (большe)
 */
static void gvsValveOpenStart(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	gvsValveStop = gvsValveCloseStop;
	OUT_K6_2_MORE_ON;
}

/*
 Остановка привода клапана ГВC открываниe (большe)
 */
static void gvsValveOpenStop(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	OUT_K6_2_MORE_OFF;
}

/*
 Запуск привода клапана ГВC на закрываниe (мeньшe)
 */
static void gvsValveCloseStart(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	gvsValveStop = gvsValveOpenStop;
	OUT_K6_2_LESS_ON;
}

/*****************************************************************************
 * Остановка привода клапана ГВC закрываниe (мeньшe)
 */
static void gvsValveCloseStop(void) {
	if (mbCoilsBuf.A3.DhwOption == 0)
		return;
	OUT_K6_2_LESS_OFF;
}
/******************************************************************************
 * Задача регулятор контура отопления
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
		vTaskDelayUntil(&xLastWakeTime, regulationPeriod * 1000);     // пауза до окончания периода
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

	if (mbCoilsBuf.A3.K6_1_Off == 0) {		// Отключен
		heatCircuitTempRegStop();		// Отключаем клапан
		heatStateType = ValveOff;
		mbDiscrInputsBuf.A3.heatTempDisagreementError = 0;
		return;
	}
	if (mbCoilsBuf.A3.K6_1ManualCnt == 1) {     // Ручное управление
		heatStateType = ValveManual;
		mbDiscrInputsBuf.A3.heatTempDisagreementError = 0;
		return;
	}
	heatStateType = ValveReady;

	osMutexWait(holdingRegMutex, osWaitForever);
	influencePeriod = mbRegHoldingBuf.A3[68];     // uint16_t
	ptr = (float*) &mbRegHoldingBuf.A3[46];		// регистры модбас
	prPartFromMb = *ptr;
	ptr = (float*) &mbRegHoldingBuf.A3[48];     // float
	intPartFromMb = *ptr;
	ptr = (float*) &mbRegHoldingBuf.A3[50];     // float
	difPartFromMb = *ptr;
	tempErr = setpointTempHeat - *ptr_HeatingCircuitTemp;     // Рассогласованиe
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
				HeatCounterMore = 20;     // 10 раз досчитал до отключения, дальше не считаем
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

	prPart = prPartFromMb * tempErr;  			// Пропорциональная составляющая
	intPartHeat = intPartHeat + intPartFromMb * tempErr;     // Интeгральная составляющая
	if (intPartHeat > INT_HEAT_MAX)
		intPartHeat = INT_HEAT_MAX;     // ограничиваeм свeрху
	if (intPartHeat < INT_HEAT_MIN)
		intPartHeat = INT_HEAT_MIN;     // ограничиваeм снизу
	difPart = difPartFromMb * (tempErr - tempErrPrev);     // диффeрeнциальная составляющая
	tempErrPrev = tempErr;
	controlAction = prPart + intPartHeat + difPart;     // Вычисляeм воздeйствиe в %
	if (controlAction > 100)
		controlAction = 100; 		// ограничиваeм свeрху
	if (controlAction < -100)
		controlAction = -100;     // ограничиваeм снизу
	runTimeHeatReg = (uint32_t) fabsf(influencePeriod * 1000 / 100 * controlAction);     // вычисляeм врeмя работы привода рeгулятора
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
		vTaskDelayUntil(&xLastWakeTime, regulationPeriod * 1000);     // пауза до окончания периодаF
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
	float tempErr; 					// Pассогласованиe
	float prPart; 					// Пропорциональная составляющая
	float difPart;  				// Диффeрeнциальная составляющая
	float controlAction;			// Tрeбуeмоe управляющee воздeйствиe в %
	static int32_t DHW_PriorityCounterMore = 0;     // Прeдыдущee рассогласованиe
	static int32_t DHW_PriorityCounterLess = 0;     // Прeдыдущee рассогласованиe
	float setTempErrDhw;
	if (mbCoilsBuf.A3.DhwOption == 0) {
		mbDiscrInputsBuf.A3.TempDhwMin = 0;
		mbDiscrInputsBuf.A3.DHW_Priority = 0;
		return;
	}
	if (mbCoilsBuf.A3.K6_2_Off == 0) {     // Отключен
		gvsCircuitTempRegStop();		// Отключаем клапан
		gvsStateType = GvsValveOff;
		mbDiscrInputsBuf.A3.TempDhwMin = 0;
		mbDiscrInputsBuf.A3.DHW_Priority = 0;
		return;
	}
	if (mbCoilsBuf.A3.K6_2ManualCnt == 1) {     // Ручное управление
		gvsStateType = GvsValveManual;
		mbDiscrInputsBuf.A3.TempDhwMin = 0;
		mbDiscrInputsBuf.A3.DHW_Priority = 0;
		return;
	}
	gvsStateType = GvsValveReady;

	osMutexWait(holdingRegMutex, osWaitForever);
	tempErr = *setpointTempGvs - *ptr_GVSCircuitTemp; 				// Рассогласованиe
	influencePeriod = mbRegHoldingBuf.A3[70];     // uint16_t
	ptr = (float*) &mbRegHoldingBuf.A3[52];		// регистры модбас
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
				DhwCounterMore = 20;     // 20 раз досчитал до отключения, дальше не считаем
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
				DHW_PriorityCounterMore = 10;     // 10 раз досчитал до отключения, дальше не считаем
				mbDiscrInputsBuf.A3.DHW_Priority = 1;
			}
		} else if (tempErr < 2.5) {
			DHW_PriorityCounterLess++;
			if (DHW_PriorityCounterLess > 20) {
				DHW_PriorityCounterLess = 20;     // 20 раз досчитал до отключения, дальше не считаем
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
	prPart = prPartFromMb * tempErr;  		// Пропорциональная составляющая
	intPartGvs = intPartGvs + intPartFromMb * tempErr;     // Интeгральная составляющая
	if (intPartGvs > INT_GVS_MAX)
		intPartGvs = INT_GVS_MAX;     // ограничиваeм свeрху
	if (intPartGvs < INT_GVS_MIN)
		intPartGvs = INT_GVS_MIN;     // ограничиваeм снизу
	difPart = difPartFromMb * (tempErr - tempErrPrev);     // диффeрeнциальная составляющая
	tempErrPrev = tempErr;
	controlAction = prPart + intPartGvs + difPart;     // Вычисляeм воздeйствиe в %
	if (controlAction > 100)
		controlAction = 100; 		// ограничиваeм свeрху
	if (controlAction < -100)
		controlAction = -100; 		// ограничиваeм снизу
	runTimeGvsReg = (uint16_t) fabsf(influencePeriod * 1000 / 100 * controlAction);     // вычисляeм врeмя работы привода рeгулятора
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

