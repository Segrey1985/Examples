#include "mbMaster.h"
#include "mb.h"
#include "mb_m.h"
#include "mbutils.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/*-----------------------Master mode use these variables----------------------*/
#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0

Oven_Coils_Type mbOvenCoils;
extern osMutexId inputRegMutex;
extern osMutexId holdingRegMutex;

extern QueueHandle_t masterMessageQueue;

uint8_t mbErrorCountA1 = 0;
uint8_t mbErrorCountA2 = 0;
uint8_t mbErrorCountMB110 = 0;
uint8_t mbErrorCountOven = 0;

/**
 * Modbus input register callback function.
 * Отдаём usNRegs входных регистров начиная с адреса usAddress в буфер по ссылке pucRegBuffer
 */
eMBErrorCode eMBMasterRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs) {
	eMBErrorCode eStatus = MB_ENOERR;
	uint8_t iRegIndex;
	uint8_t destId;
	uint16_t *bufPtr;

	usAddress--;

	destId = ucMBMasterGetDestAddress();     // ID текущего slave
	if (destId == MB_A1_ID)
		bufPtr = (uint16_t*) &mbRegInputBuf.A1;
	else if (destId == MB_A2_ID)
		bufPtr = (uint16_t*) &mbRegInputBuf.A2;
	else if (destId == MB_OVEN_IN_ID)
		bufPtr = (uint16_t*) &mbRegInputBufMB110.MB110;
	if ((usAddress >= MB_M_REG_INPUT_START) && (usAddress + usNRegs <= MB_M_REG_INPUT_START + MB_M_REG_INPUT_CNT)) {
		iRegIndex = (int) (usAddress - MB_M_REG_INPUT_START);

		osMutexWait(inputRegMutex, osWaitForever);
		while (usNRegs > 0) {
			*(bufPtr + iRegIndex) = *pucRegBuffer++ << 8;
			*(bufPtr + iRegIndex) |= *pucRegBuffer++;
			iRegIndex++;
			usNRegs--;
		}
		osMutexRelease(inputRegMutex);
	} else if ((usAddress >= MB_M_REG_INPUT_MB110_START)
			&& (usAddress + usNRegs <= MB_M_REG_INPUT_MB110_START + MB_M_REG_INPUT_MB110_CNT)) {
		iRegIndex = (int) (usAddress - MB_M_REG_INPUT_MB110_START);
		osMutexWait(inputRegMutex, osWaitForever);
		while (usNRegs > 0) {
			*(bufPtr + iRegIndex) = *pucRegBuffer++ << 8;
			*(bufPtr + iRegIndex) |= *pucRegBuffer++;
			iRegIndex++;
			usNRegs--;
		}
		osMutexRelease(inputRegMutex);
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

/**
 * Modbus holding register callback function.
 */
eMBErrorCode eMBMasterRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode) {
	eMBErrorCode eStatus = MB_ENOERR;
	uint8_t iRegIndex;
	uint8_t destId;
	uint16_t *bufPtr;

	usAddress--;

	destId = ucMBMasterGetDestAddress();     // ID текущего slave
	if (destId == MB_A1_ID)
		bufPtr = (uint16_t*) &mbRegHoldingBuf.A1;
	else if (destId == MB_A2_ID)
		bufPtr = (uint16_t*) &mbRegHoldingBuf.A2;

	if ((usAddress >= MB_M_REG_HOLDING_START)
			&& (usAddress + usNRegs <= MB_M_REG_HOLDING_START + MB_M_REG_HOLDING_CNT)) {
		iRegIndex = usAddress - MB_M_REG_HOLDING_START;
		/* write current register values with new values from the protocol stack. */
		osMutexWait(holdingRegMutex, osWaitForever);
		while (usNRegs > 0) {
			*(bufPtr + iRegIndex) = *pucRegBuffer++ << 8;
			*(bufPtr + iRegIndex) |= *pucRegBuffer++;
			iRegIndex++;
			usNRegs--;
		}
		osMutexRelease(holdingRegMutex);
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

/**
 * Modbus coils callback function.
 */
eMBErrorCode eMBMasterRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode) {
	eMBErrorCode eStatus = MB_ENOERR;
	unsigned short usBitOffset;
	short iNCoils = (short) usNCoils;
	UCHAR destId;
	USHORT startAdr;
	UCHAR *bufPtr;
	USHORT regCnt;

	usAddress--;

	destId = ucMBMasterGetDestAddress();     // ID текущего slave
	switch (destId) {
	case MB_A1_ID:
		startAdr = MB_M_COILS_START_A1_2;
		bufPtr = (UCHAR*) &mbCoilsBuf.A1;
		regCnt = MB_M_COILS_A1_2_CNT;
		break;
	case MB_A2_ID:
		startAdr = MB_M_COILS_START_A1_2;
		bufPtr = (UCHAR*) &mbCoilsBuf.A2;
		regCnt = MB_M_COILS_A1_2_CNT;
		break;
	case MB_OVEN_OUT_ID:
		startAdr = MB_M_COILS_START_OVEN;
		bufPtr = (UCHAR*) &mbOvenCoils;
		regCnt = MB_M_COILS_OVEN_CNT;
		break;
	}

	if ((usAddress >= startAdr) && (usAddress + usNCoils <= startAdr + regCnt)) {
		usBitOffset = (USHORT) (usAddress - startAdr);

		while (iNCoils > 0) {
			xMBUtilSetBits(bufPtr, usBitOffset, (UCHAR) (iNCoils > 8 ? 8 : iNCoils), *pucRegBuffer++);
			iNCoils -= 8;
			usBitOffset += 8;
		}
	} else {
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

/**
 * Modbus master discrete callback function.
 *
 * @param pucRegBuffer discrete buffer
 * @param usAddress discrete address
 * @param usNDiscrete discrete number
 *
 * @return result
 */
eMBErrorCode eMBMasterRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete) {
	eMBErrorCode eStatus = MB_ENOERR;
	unsigned short usBitOffset;
	short iNDiscrete = (short) usNDiscrete;
	uint8_t destId;
	UCHAR *bufPtr;

	usAddress--;

	destId = ucMBMasterGetDestAddress();     // ID текущего slave
	if (destId == MB_A1_ID)
		bufPtr = (UCHAR*) &mbDiscrInputsBuf.A1;
	else if (destId == MB_A2_ID)
		bufPtr = (UCHAR*) &mbDiscrInputsBuf.A2;

	if ((usAddress >= MB_M_DISCRETEINPUTS_START)
			&& (usAddress + usNDiscrete <= MB_M_DISCRETEINPUTS_START + MB_M_DISCRETEINPUTS_CNT)) {
		usBitOffset = (USHORT) (usAddress - MB_M_DISCRETEINPUTS_START);
		while (iNDiscrete > 0) {
			xMBUtilSetBits(bufPtr, usBitOffset, (unsigned char) (iNDiscrete > 8 ? 8 : iNDiscrete), *pucRegBuffer++);
			iNDiscrete -= 8;
			usBitOffset += 8;
		}
	} else {
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

/**
 * Работа с Modbus master
 */
void StartModbusMasterTask(void const *argument) {
	eMBMasterReqErrCode errorCode;
	static Master_Message_Type message;
	for (;;) {

		xQueueReceive(masterMessageQueue, &message, osWaitForever);
		switch (message.Type) {

		case ReadAll:

			errorCode = eMBMasterReqReadCoils(MB_A1_ID, MB_M_COILS_START_A1_2,
			MB_M_COILS_A1_2_CNT, MB_TIMEOUT);
			osDelay(10);
			if (errorCode != MB_MRE_NO_ERR) {
				mbErrorCountA1++;
			} else
				mbErrorCountA1 = 0;
			osDelay(55);
			errorCode = eMBMasterReqReadCoils(MB_A2_ID, MB_M_COILS_START_A1_2,
			MB_M_COILS_A1_2_CNT, MB_TIMEOUT);
			osDelay(10);
			if (errorCode != MB_MRE_NO_ERR) {
				mbErrorCountA2++;
			} else
				mbErrorCountA2 = 0;
			break;

		case ReadAllInp:     // Считать Input, Holding и Discrete Input со слэйвов

			errorCode = eMBMasterReqReadInputRegister(MB_OVEN_IN_ID,
			MB_M_REG_INPUT_MB110_START, MB_M_REG_INPUT_MB110_CNT, MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				mbErrorCountMB110++;
			} else
				mbErrorCountMB110 = 0;
			osDelay(55);

			errorCode = eMBMasterReqReadInputRegister(MB_A1_ID,
			MB_M_REG_INPUT_START, MB_M_REG_INPUT_CNT, MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				mbErrorCountA1++;
			} else
				mbErrorCountA1 = 0;
			osDelay(55);
			errorCode = eMBMasterReqReadInputRegister(MB_A2_ID,
			MB_M_REG_INPUT_START, MB_M_REG_INPUT_CNT, MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				mbErrorCountA2++;
			} else
				mbErrorCountA2 = 0;
			osDelay(55);
			errorCode = eMBMasterReqReadDiscreteInputs(MB_A1_ID,
			MB_M_DISCRETEINPUTS_START, MB_M_DISCRETEINPUTS_CNT,
			MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				mbErrorCountA1++;
			} else
				mbErrorCountA1 = 0;
			osDelay(55);
			errorCode = eMBMasterReqReadDiscreteInputs(MB_A2_ID,
			MB_M_DISCRETEINPUTS_START, MB_M_DISCRETEINPUTS_CNT,
			MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				mbErrorCountA2++;
			} else
				mbErrorCountA2 = 0;
			osDelay(55);
			errorCode = eMBMasterReqReadHoldingRegister(MB_A1_ID,
			MB_M_REG_HOLDING_START, MB_M_REG_HOLDING_CNT, MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				mbErrorCountA1++;
			} else
				mbErrorCountA1 = 0;
			osDelay(55);

			errorCode = eMBMasterReqReadHoldingRegister(MB_A2_ID,
			MB_M_REG_HOLDING_START, MB_M_REG_HOLDING_CNT, MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				mbErrorCountA2++;
			} else
				mbErrorCountA2 = 0;
			break;

		case WriteCoil:		// записать 1 coil (значение в message.RegCnt)
			osDelay(100);
			errorCode = eMBMasterReqWriteCoil(message.SlaveId, message.Address, message.RegCnt, 100);
			if (errorCode != MB_MRE_NO_ERR) {
				if (message.SlaveId == MB_OVEN_OUT_ID)
					mbErrorCountOven++;
				else if (message.SlaveId == MB_A1_ID)
					mbErrorCountA1++;
				else if (message.SlaveId == MB_A2_ID)
					mbErrorCountA2++;
			} else if (message.SlaveId == MB_OVEN_OUT_ID)
				mbErrorCountOven = 0;
			osDelay(100);
			break;

		case WriteAllCoil:     // записать несколько coil на А1 или А2

			errorCode = eMBMasterReqWriteMultipleCoils(message.SlaveId, message.Address, message.RegCnt,
					message.DataPtr,
					MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				if (message.SlaveId == MB_A1_ID)
					mbErrorCountA1++;
				if (message.SlaveId == MB_A2_ID)
					mbErrorCountA2++;
			}
			break;

		case Write1Holding:     // записать 1 holding

			errorCode = eMBMasterReqWriteHoldingRegister(message.SlaveId, message.Address, message.RegCnt, MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				if (message.SlaveId == MB_A1_ID)
					mbErrorCountA1++;
				if (message.SlaveId == MB_A2_ID)
					mbErrorCountA2++;
			}
			break;

		case WriteHoldings:
			errorCode = eMBMasterReqWriteMultipleHoldingRegister(message.SlaveId, message.Address, message.RegCnt,
					(uint16_t*) message.DataPtr, MB_TIMEOUT);
			if (errorCode != MB_MRE_NO_ERR) {
				if (message.SlaveId == MB_A1_ID)
					mbErrorCountA1++;
				if (message.SlaveId == MB_A2_ID)
					mbErrorCountA2++;
			}
			break;
		}

		if (mbErrorCountA1 > 3) {
			mbDiscrInputsBuf.A3.ModbusErrA1 = 1;
			errorHandler(ErrModbusA1);
			mbErrorCountA1 = 3;
		} else if (mbDiscrInputsBuf.A3.ModbusErrA1 == 1) {
			if (mbErrorCountA1 == 0)
				mbDiscrInputsBuf.A3.ModbusErrA1 = 0;
		}

		if (mbErrorCountA2 > 3) {
			mbDiscrInputsBuf.A3.ModbusErrA2 = 1;
			errorHandler(ErrModbusA2);
			mbErrorCountA2 = 3;
		} else if (mbDiscrInputsBuf.A3.ModbusErrA2 == 1) {
			if (mbErrorCountA2 == 0)
				mbDiscrInputsBuf.A3.ModbusErrA2 = 0;
		}

		if (mbCoilsBuf.Settings.OvenInOption == 0)
			mbDiscrInputsBuf.A3.ModbusErrMB110 = 0;
		else if (mbErrorCountMB110 > 3) {
			mbDiscrInputsBuf.A3.ModbusErrMB110 = 1;
//			errorHandler(ErrModbusA2);
			mbErrorCountMB110 = 3;
		} else if (mbDiscrInputsBuf.A3.ModbusErrMB110 == 1) {
			if (mbErrorCountMB110 == 0)
				mbDiscrInputsBuf.A3.ModbusErrMB110 = 0;
		}

		if (mbErrorCountOven > 0 && mbCoilsBuf.Settings.OvenOutOption == 1) {
			mbDiscrInputsBuf.A3.ModbusErrOven = 1;
			errorHandler(ErrModbusOven);
		} else
			mbDiscrInputsBuf.A3.ModbusErrOven = 0;
	}
}

/**
 * Function implementing the modbusMasterPollTask thread.

 */
void StartModbusMasterPollTask(void const *argument) {
	for (;;) {
		if (eMBMasterPoll() == MB_EILLSTATE)
			osDelay(10);
	}
}

/*
 * Периодический опрос slave по таймеру
 */
void mbMasterCallback(void const *argument) {
	static Master_Message_Type message = { ReadAllInp, 0, 0, 0, NULL };
	xQueueSend(masterMessageQueue, &message, 0);
}

#endif
