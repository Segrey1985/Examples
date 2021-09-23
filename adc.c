/**
 ******************************************************************************
 * File Name          : ADC.c
 * Description        : This file provides code for the configuration
 *                      of the ADC instances.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "gpio.h"
#include "dma.h"
#include "main.h"
#include "mbSlave.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

volatile DevStateType stateADC = Idle;     // ���������� ��� ��������� ���
volatile uint16_t dataADC[NUMBER_OF_MEASUREMENTS][NUMBER_OF_CHANNELS];     //������ ���������� �� ���(9 �������, NUMBER_OF_MEASUREMENTS ���������)

volatile float biasFactorPt_1000[] = { 0, 0, 0, 0, 0, 0 };     // ������������� ������������ ��� �������� �����������

volatile float currPrSensors[4];        // ���� �� �������� ��������
volatile float biasFactorPrSensors[] = { 0, 0, 0, 0 };     // ������������� ������������ ��� �������� ��������

extern xSemaphoreHandle adcDataSemaphore;
extern osMutexId inputRegMutex;
extern osMutexId holdingRegMutex;

/*****************************************************************************************************************
 * ������ ���, ���������� size ��������������, ������ ����������� �� ������ addres
 */
void ADC_Start(uint32_t addres, uint16_t size) {
	__IO uint32_t wait_loop_index = 0;
	if (stateADC != Idle)
		return;     //�����  ���� �� ��������� ��������������
	stateADC = Busy;
	/* ������������� ������ ���������(���) � ������(�����) ��� DMA */
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &(ADC1->DR), addres, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, size);     // ���������� ������(uint16_t) ��� �������� �� DMA
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_ADC_Enable(ADC1);
	wait_loop_index = (ADC_DELAY_ENABLE_CALIB_CPU_CYCLES >> 1);
	while (wait_loop_index != 0) {
		wait_loop_index--;
	}

	LL_ADC_REG_StartConversionSWStart(ADC1);
}

//-------------------------------------------------------------------------------------

/********************************************************************************
 * ���������� DMA ��� �������������� ���������
 */
void AdcDmaTransferComplete_Callback(void) {

	LL_ADC_Disable(ADC1);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);

	xSemaphoreGiveFromISR(adcDataSemaphore, NULL);
}

/****************************************************
 * ���������� DMA ��� "������"
 */
void AdcDmaTransferError_Callback(void) {
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_ADC_Disable(ADC1);
	errorHandler(ErrADC);
}

/***********************************************************************************
 * ��������� ������ �� ���
 */
void StartHandlerADCDataTask(void const *argument) {
//	static SysEventStrType message;
//	message.Type = ErrSensor;
	uint32_t tmp[NUMBER_OF_CHANNELS];
	float *pointerData;
	float *pointerBias;
	float *pointerRes;
	float tmpData;
	float tmpDataToSend;
//	uint16_t errorCnt;	// ������� �������� � ����� ����� ���� - ����� �������

	for (;;) {
		xSemaphoreTake(adcDataSemaphore, osWaitForever);

//		errorCnt = 0;

		// ��������� ������� �������� ��������� ���
		for (int i = 0; i < NUMBER_OF_CHANNELS; i++) {
			tmp[i] = 0;
			for (int j = 0; j < NUMBER_OF_MEASUREMENTS; j++) {
				tmp[i] += dataADC[j][i];
			}
			tmp[i] = tmp[i] / NUMBER_OF_MEASUREMENTS;     // ���������� ������ ������� ���
		}

		stateADC = Idle;

		pointerData = (float*) &mbRegInputBuf.A3[0];     // ����� ������� �� 8 ��������� Modbus ���������� ������ � �������� � 4 ��������
		pointerBias = (float*) &mbRegHoldingBuf.A3[26];     // ����� ������� ������������ ������������ ��� �������� ��������
		// ��������� ���� �������� ��������:
		// ���������� �� ����� ��� ����� X*3/4095, ���������� �� ������������� ��������� - (X*3/4095)/100, ��� X - ������ � ���
		// ��� ����� ������ ����� ((X*3/4095)/100)/1.5 � = X*20/4095 mA
		for (int i = 0; i < 4; i++) {
			currPrSensors[i] = (float) tmp[i] * 20 / 4095;
			if (currPrSensors[i] > 22 || currPrSensors[i] < 2) {     // ��� ������� ��������� ��� �������
				currPrSensors[i] = 0;
				switch (i) {
				case 0:
					mbDiscrInputsBuf.A3.HeatCircPrEr = 1;
					break;
				case 1:
					mbDiscrInputsBuf.A3.HeatCircRevPrEr = 1;
					break;
				case 2:
					mbDiscrInputsBuf.A3.ColdWaterPrEr = 1;
					break;
				case 3:
					if (mbCoilsBuf.A3.DhwOption == 1)
						mbDiscrInputsBuf.A3.GvsCircPrEr = 1;
					else
						mbDiscrInputsBuf.A3.GvsCircPrEr = 0;
					break;
				}
			} else {
				switch (i) {
				case 0:
					mbDiscrInputsBuf.A3.HeatCircPrEr = 0;
					break;
				case 1:
					mbDiscrInputsBuf.A3.HeatCircRevPrEr = 0;
					break;
				case 2:
					mbDiscrInputsBuf.A3.ColdWaterPrEr = 0;
					break;
				case 3:
					if (mbCoilsBuf.A3.DhwOption == 1)
						mbDiscrInputsBuf.A3.GvsCircPrEr = 0;
					break;
				}
			}
			osMutexWait(holdingRegMutex, osWaitForever);
			tmpData = *pointerBias++;				//����������� �����������
			osMutexRelease(holdingRegMutex);
			if (currPrSensors[i] != 0)
				tmpData += pressureCalculation(currPrSensors[i], i);
			else
				tmpData = 0;
			osMutexWait(inputRegMutex, osWaitForever);
			*pointerData++ = tmpData;
			osMutexRelease(inputRegMutex);
		}

		pointerBias = (float*) &mbRegHoldingBuf.A3[34];     // ����� ������� ������������ ������������ ��� �������� �����������
		pointerData = (float*) &mbRegInputBuf.A3[8];     // ����� ������� �� 12 ��������� Modbus ���������� ������ � ����������� � 6 ��������
		pointerRes = (float*) &mbRegInputBuf.A3[20];     // ����� ������� �� 12 ��������� Modbus ���������� ������ � ������������� 6 ��������
		// ��������� ������������� �������� pt-1000:
		// ���������� �� ����� ��� ����� X*3/4095, ���������� �� ������� - (X*3/4095)/5, ��� X - ������ � ���
		// ��� ����� ������ �� ����� ����� 1.25/5000, ������ ��� ������������� ����� ((X*3/4095)/5)/(125/500000) = X*2400/4095
		for (int i = 4; i < 10; i++) {

			osMutexWait(holdingRegMutex, osWaitForever);
			tmpData = *(pointerBias++);				//����������� �����������
			osMutexRelease(holdingRegMutex);

			osMutexWait(inputRegMutex, osWaitForever);
			*pointerRes = (float) tmp[i] * 2400 / 4095;			// ��������� ������������� � ������� � ������� Modbus
			tmpData = tmpData + temperatureCalculation(*pointerRes++);     // ��������� ����������� � ������� � ������� Modbus
			if (tmpData > 250 || tmpData < -50)
				tmpDataToSend = 0;
			else
				tmpDataToSend = tmpData;
			*pointerData++ = tmpDataToSend;     // ��������� ����������� � ������� � ������� Modbus
			osMutexRelease(inputRegMutex);
			if (tmpData > 250 || tmpData < -50) {     // ����������� ������� ������� ��� ������
				switch (i) {
				case 4:
					mbDiscrInputsBuf.A3.BoilCircTempEr = 1;
					break;
				case 5:
					mbDiscrInputsBuf.A3.OutAirTempEr = 1;
					break;
				case 6:
					mbDiscrInputsBuf.A3.HeatCircTempEr = 1;
					break;
				case 7:
					if (mbCoilsBuf.A3.DhwOption == 1)
						mbDiscrInputsBuf.A3.GvsCircTempEr = 1;
					else
						mbDiscrInputsBuf.A3.GvsCircTempEr = 0;
					break;
				case 8:
					mbDiscrInputsBuf.A3.BoilCircRevTempEr = 1;
					break;
				case 9:
					mbDiscrInputsBuf.A3.HeatCircRevTempEr = 1;
					break;
				}
			} else {
				switch (i) {
				case 4:
					mbDiscrInputsBuf.A3.BoilCircTempEr = 0;
					break;
				case 5:
					mbDiscrInputsBuf.A3.OutAirTempEr = 0;
					break;
				case 6:
					mbDiscrInputsBuf.A3.HeatCircTempEr = 0;
					break;
				case 7:
					if (mbCoilsBuf.A3.DhwOption == 1)
						mbDiscrInputsBuf.A3.GvsCircTempEr = 0;
					break;
				case 8:
					mbDiscrInputsBuf.A3.BoilCircRevTempEr = 0;
					break;
				case 9:
					mbDiscrInputsBuf.A3.HeatCircRevTempEr = 0;
					break;
				}
			}
		}
	}
}

/********************************************************************************************************
 * ������ ����������� �� ������������� �������
 */
float temperatureCalculation(float res) {
	if (res < RT0) {
		return D1 * (res / RT0 - 1) + D2 * powf((res / RT0 - 1), 2) + D3 * powf((res / RT0 - 1), 3)
				+ D4 * powf((res / RT0 - 1), 4);
	}
	return (sqrtf(A * A - 4 * B * (1 - res / RT0)) - A) / (2 * B);
}

/********************************************************************************************************
 * ������ �������� �� ���� �������
 * curr - ���
 * num - ����� �������
 */
float pressureCalculation(float curr, uint8_t num) {
	uint16_t pUp;
	if (curr <= 4)
		return 0;
	pUp = mbRegHoldingBuf.A3[num + 62];     // num + 62 - ����� ������ � ������� �������� ������� * 10
	return (curr - 4) * pUp / 160;
}

//*************************************************************************************************************

/*
 * ADC1 init function
 */
void MX_ADC1_Init(void) {
	LL_ADC_InitTypeDef ADC_InitStruct;
	LL_ADC_CommonInitTypeDef ADC_CommonInitStruct;
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;
	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

	/**ADC1 GPIO Configuration
	 PA0   ------> ADC1_IN0 -  ������ X10A
	 PA3   ------> ADC1_IN3 -  ������ X10B
	 PA4   ------> ADC1_IN4 -  ������ X10C
	 PA5   ------> ADC1_IN5 -  ������ X10D
	 PA6   ------> ADC1_IN6 -  ������ X9A
	 PA7   ------> ADC1_IN7 -  ������ X9B
	 PC4   ------> ADC1_IN14-  ������ X9C
	 PC5   ------> ADC1_IN15-  ������ X9D
	 PB0   ------> ADC1_IN8 -  ������ X9E
	 PB1   ------> ADC1_IN9 -  ������ X9F
	 */

	/* ADC1 DMA Init */
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1,
	LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1,
	LL_DMA_PRIORITY_MEDIUM);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_NORMAL);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1,
	LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1,
	LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1,
	LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1,
	LL_DMA_MDATAALIGN_HALFWORD);

	/* ADC2 interrupt Init */
	NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(ADC1_2_IRQn);

	/**Common config */
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);

	ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
	LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

//��������� ����������
	LL_ADC_DisableIT_EOS(ADC1);

	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_10RANKS;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

	/**Configure Regular Channels */
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_3);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_4);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_5);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_6);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_6, LL_ADC_CHANNEL_7);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_7,
	LL_ADC_CHANNEL_14);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_14,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_8,
	LL_ADC_CHANNEL_15);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_15,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_9, LL_ADC_CHANNEL_8);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_10,
	LL_ADC_CHANNEL_9);
	LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9,
	LL_ADC_SAMPLINGTIME_239CYCLES_5);

// ���������� ���1
	__IO uint32_t wait_loop_index = 0;

	if (LL_ADC_IsEnabled(ADC1) == 0) {
		/* Enable ADC */
		LL_ADC_Enable(ADC1);

		wait_loop_index = (ADC_DELAY_ENABLE_CALIB_CPU_CYCLES >> 1);
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}

		/* Run ADC self calibration */
		LL_ADC_StartCalibration(ADC1);

		while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0) {
		}
		LL_ADC_Disable(ADC1);
	}
	stateADC = Idle;
}

/* ADC2 init function */
void MX_ADC2_Init(void) {
	LL_ADC_InitTypeDef ADC_InitStruct;
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;

//LL_GPIO_InitTypeDef GPIO_InitStruct;
	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

	/**ADC2 GPIO Configuration	������ � ��� 1
	 PC3   ------> ADC2_IN13
	 */

	/* ADC2 interrupt Init */
	NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
	NVIC_EnableIRQ(ADC1_2_IRQn);

	/**Common config*/
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
	LL_ADC_Init(ADC2, &ADC_InitStruct);

	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
	LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

	/**Configure Analog WatchDog */
	LL_ADC_SetAnalogWDMonitChannels(ADC2, LL_ADC_AWD_CHANNEL_13_REG);
	LL_ADC_SetAnalogWDThresholds(ADC2, LL_ADC_AWD_THRESHOLD_HIGH,
	ALARM_ADC_THRESHOLD_HIGH);     // ������� �������
	LL_ADC_SetAnalogWDThresholds(ADC2, LL_ADC_AWD_THRESHOLD_LOW,
	ALARM_ADC_THRESHOLD_LOW);     // ������ �������
	LL_ADC_EnableIT_AWD1(ADC2);

	/**Configure Regular Channel */
	LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1,
	LL_ADC_CHANNEL_13);
	LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_13,
	LL_ADC_SAMPLINGTIME_55CYCLES_5);

// ���������� ���1
	__IO uint32_t wait_loop_index = 0;     // �������

	if (LL_ADC_IsEnabled(ADC2) == 0) {
		/* Enable ADC */
		LL_ADC_Enable(ADC2);

		wait_loop_index = (ADC_DELAY_ENABLE_CALIB_CPU_CYCLES >> 1);
		while (wait_loop_index != 0) {
			wait_loop_index--;
		}
		/* Run ADC self calibration */
		LL_ADC_StartCalibration(ADC2);

		while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0) {
		}
		LL_ADC_Disable(ADC2);
	}
}

/*
 * ������ ��� �� �������
 */
void adcStartCallback(void const *argument) {
	ADC_Start((uint32_t) dataADC,
	NUMBER_OF_MEASUREMENTS * NUMBER_OF_CHANNELS);     // ��������� ���
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
