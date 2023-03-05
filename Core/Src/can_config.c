/*
 * can_config.c
 *
 *   Created on: 27.02.2022
 *  Modified on: 30.07.2022
 *       Author: Krystian Sosin
 *      Version: 1.0.2
 *  Last change: Fix minor bugs and add CAN_AcknowledgeWriteMessage() function.
 */

#include "main.h"
#include "can_config.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include "stdbool.h"


/* Variables and arrays needed to proper work of CAN */
CAN_FilterTypeDef    sFilterConfig;
CAN_TxHeaderTypeDef  TxHeader;
CAN_RxHeaderTypeDef  RxHeader;
uint8_t				 TxData[8];
uint8_t				 RxData[8];
uint32_t			 TxMailbox;

/* USER CODE BEGIN Externs */
/* Externs used in configs */
extern bool var;
extern data_temp_label;
extern data_size;
extern I2C_is_receiving_temp;
extern hi2c1;

extern bool print;
extern bool confirm_button;

bool Write_Green_LED_State;
bool Print_Numbers_State;
extern bool PLUS_button_State;



extern uint8_t CAN_temp1_value;
extern uint8_t CAN_temp2_value;
extern uint8_t CAN_temp3_value;
extern uint8_t CAN_temp4_value;
extern uint8_t CAN_temp5_value;





uint8_t can_map_value = 0;
uint8_t can_tc_value = 0;
uint8_t can_speed_value = 0;
uint8_t can_diff_value = 0;
uint8_t can_ts_value = 0;
uint8_t can_leng_value = 0;
uint8_t can_linv_value = 0;
uint8_t can_bat_value = 0;
uint8_t can_rinv_value = 0;
uint8_t can_reng_value = 0;
uint8_t can_err_value = 0;
uint8_t can_hv_value = 0;
uint8_t can_low_value = 0;

//extern uint8_t can_map_value;
//extern uint8_t can_tc_value;
//extern uint8_t can_speed_value;
//extern uint8_t can_diff_value;
//extern uint8_t can_ts_value;
//extern uint8_t can_leng_value;
//extern uint8_t can_linv_value;
//extern uint8_t can_bat_value;

extern bool map_value_active;
extern bool tc_value_active;
extern bool speed_value_active;
extern bool diff_value_active;
extern bool ts_value_active;

uint8_t CANdata[2] = {0,0};
uint16_t CANsize = 2;
#define slave_addr 0x01






extern bool Write_State2;
extern bool Write_State3;
extern uint8_t Write_Data1;

extern uint8_t Var_1;


extern uint8_t Read_Data2;
extern uint8_t Read_Data3;
extern uint8_t Read_Data4;
/* USER CODE END Externs */

/** 
 * CAN READ MESSAGE FRAME
 *
 * | RxID  |  2  | 0x3D | ADDR  |
 * | StdID | DLC | READ | RegID |
 *
 *
 */
// Enum of ReadRegs defined in can_config.h. Nothing needs to be configured here.

/* USER CODE BEGIN ResponseMessage */

/**
 *  CAN RESPONSE MESSAGE FRAME
 *
 * | TxID  | DLC | ADDR  | VALUE  | ... | VALUE  |
 * | StdID | DLC | RegID | DATA_1 | ... | DATA_N |
 *
 **/
ResponseMessageFrame ResponseMessage[NUMBER_OF_READ_REGS] =
{
	{
		.Response_DLC        = 2u,                         // Data length of response message
		.Read_ReactionHandler = Read_Var_1_Handler,       // Handler of reaction to read request from MCU
		//.Response_RegID      = Read_MeaningfulRegName1_ID, // Address of regs which response refers			zakomentowałem na czas tesów odbioru
		//.Response_Data1      = &Var_1                 // Returned data to MCU									to też
	}
};
/* USER CODE END ResponseMessage */

/* USER CODE BEGIN WriteMessage */

/**
 *  CAN WRITE MESSAGE FRAME
 *
 * | RxID  | DLC | ADDR  | VALUE  | ... | VALUE  |
 * | StdID | DLC | WRITE | DATA_1 | ... | DATA_N |
 *
 **/
WriteMessageFrame WriteMessage[NUMBER_OF_WRITE_REGS] =
{
    // Replace by some more meaningful name of reg
	{
		.Write_RegID           = Write_Toggle_Green_LED_ID, // Reg which should be written by MCU command
		.Write_ReactionHandler = Write_Toggle_Green_LED_Handler,    // Handler of reaction to write request from MCU
		.Write_State           = &Write_Green_LED_State             // If this MCU command should change state of sth this pointer should point to variable which regards this state eg. if MCU want to light up brake light, this structure element should point to variable which contain the state of brake lights
	},

	{
		.Write_RegID 		   = Print_Numbers_ID,
		.Write_ReactionHandler = Print_Numbers_Handler,
		.Write_State           = &Print_Numbers_State
	},

	{
		.Write_RegID 		   = MAP_VALUE_ID,
		.Write_ReactionHandler = map_value_Handler,
//		.Write_State           = &PLUS_button_State,
		.Write_Data1		   = &can_map_value
//		.Write_Data2		   = &CAN_temp1_value,
//		.Write_Data3		   = &CAN_temp2_value,
//		.Write_Data4		   = &CAN_temp3_value,
//		.Write_Data5		   = &CAN_temp4_value,
//		.Write_Data6		   = &CAN_temp5_value
	},

	{
		.Write_RegID 		   = TC_VALUE_ID,
		.Write_ReactionHandler = tc_value_Handler,
		.Write_Data1		   = &can_tc_value
	},

	{
		.Write_RegID 		   = SPEED_VALUE_ID,
		.Write_ReactionHandler = speed_value_Handler,
		.Write_Data1		   = &can_speed_value
	},

	{
		.Write_RegID 		   = DIFF_VALUE_ID,
		.Write_ReactionHandler = diff_value_Handler,
		.Write_Data1		   = &can_diff_value
	},

	{
		.Write_RegID 		   = TS_VALUE_ID,
		.Write_ReactionHandler = ts_value_Handler,
		.Write_Data1		   = &can_ts_value
	},

	{
		.Write_RegID 		   = LENG_VALUE_ID,
		.Write_ReactionHandler = leng_value_Handler,
		.Write_Data1		   = &can_leng_value
	},

	{
		.Write_RegID 		   = LINV_VALUE_ID,
		.Write_ReactionHandler = linv_value_Handler,
		.Write_Data1		   = &can_linv_value
	},

	{
		.Write_RegID 		   = BAT_VALUE_ID,
		.Write_ReactionHandler = bat_value_Handler,
		.Write_Data1		   = &can_bat_value
	},

	{
		.Write_RegID 		   = RINV_VALUE_ID,
		.Write_ReactionHandler = rinv_value_Handler,
		.Write_Data1		   = &can_rinv_value
	},

	{
		.Write_RegID 		   = RENG_VALUE_ID,
		.Write_ReactionHandler = reng_value_Handler,
		.Write_Data1		   = &can_reng_value
	},

	{
		.Write_RegID 		   = ERR_VALUE_ID,
		.Write_ReactionHandler = err_value_Handler,
		.Write_Data1		   = &can_err_value
	},

	{
		.Write_RegID 		   = HV_VALUE_ID,
		.Write_ReactionHandler = hv_value_Handler,
		.Write_Data1		   = &can_hv_value
	},

	{
		.Write_RegID 		   = LOW_VALUE_ID,
		.Write_ReactionHandler = low_value_Handler,
		.Write_Data1		   = &can_low_value
	}
};
/* USER CODE END WriteMessage */
/**
 *  CAN ERROR MESSAGE FRAME
 *
 * | TxID  |  2  | 0x1D  |    ID   |
 * | StdID | DLC | ERROR | ErrorID |
 *
 */
// Enum of ErrorRegs defined in can_config.h. Nothing needs to be configured here.

/** CAN_Init
 * @brief Function to ensure proper work of CAN interface
          - configuration of filter and calling essantial functions of CAN initialization
          Filter configured in accordance with E&S Team Project Guidlines.
 *
 * @retval None.
 **/
void CAN_Init(void)
{
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = Rx_ID << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0xFFFF << 5;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	TxHeader.StdId = Tx_ID;
	TxHeader.ExtId = 0x0000;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
}

/** HAL_CAN_RxFifo0MsgPendingCallback
 * @brief HAL Callback to handle interuption from CAN new message
 * 
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 *
 * @retval None 
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Receive(hcan, &RxHeader, RxData);
	CAN_On_Receive(RxData);
}

/** CAN_On_Receive
 * @brief Function to procces received message - checking if message is read or write command and call specific handler
 * 
 * @param RxData pointer to uint8_t array which stores received data
 * 
 * @retval None
 **/
void CAN_On_Receive(uint8_t *RxData)
{
	if(Read_RequestMessage == RxData[ReadMessage])
	{
		CAN_Respond();
	}
	else
	{
		CAN_ProcessWriteCommand();
	}
}

/** CAN_Receive
 * @brief Function to receive data via CAN and, if neccessary, report error of CAN connection
 * 
 * @param CANPointerpointer to a CAN_HandleTypeDef structure that contains
 *                          the configuration information for the specified CAN.
 * @param RxHeader CAN_RxHeaderTypeDef pointer to structure that contains RxHeader configuration.
 * @param RxData uint8_t pointer to array that will contain received data.
 * 
 * @retval None.
 **/
void CAN_Receive(CAN_HandleTypeDef *CANPointer, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
	if(HAL_CAN_GetRxMessage(CANPointer, CAN_RX_FIFO0, RxHeader, RxData) != HAL_OK)
	{
		CANBUS_Error_Handler();
	}
};

/** CAN_Transmit
 * @brief Function to transmit data via CAN and, if neccessary, report error of CAN connection
 * 
 * @param TxHeader CAN_TxHeaderTypeDef pointer to structure that contains TxHeader configuration.
 * @param TxDLC uint8_t variable which contains Date Length of CAN message.
 * @param TxData uint8_t pointer to array that contains data to transmit.
 * @param TxMailbox uint32_t pointer to array that contains whole CAN message to transmit.
 * 
 * @retval None.
 **/
void CAN_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t TxDLC, uint8_t *TxData, uint32_t *TxMailbox)
{
	TxHeader->DLC = TxDLC;
	if(HAL_CAN_AddTxMessage(&hcan, TxHeader, TxData, TxMailbox) != HAL_OK)
	{
		CANBUS_Error_Handler();
	}
}

/** CAN_Respond
 * @brief Function to respond in connection with read request from MCU
 * 
 * @retval None.
 **/
void CAN_Respond(void)
{
	for (int i = FIRST_ARRAY_ELEMENT; i < NUMBER_OF_READ_REGS; i++)
	{
		if (ResponseMessage[i].Response_RegID == RxData[ReadRegID])
		{
			ResponseMessage[i].Read_ReactionHandler();
		}
	}
}

/** CAN_ProcessWriteCommand
 * @brief Function to process write command
 * 
 * @retval None.
 **/
void CAN_ProcessWriteCommand(void)
{
	for (int i = FIRST_ARRAY_ELEMENT; i < NUMBER_OF_WRITE_REGS; i++)
	{
		if (WriteMessage[i].Write_RegID == RxData[WriteMessage_reg])
		{
			CAN_AcknowledgeWriteMessage(WriteMessage[i].Write_RegID);
			WriteMessage[i].Write_ReactionHandler();
		}
	}
}

/** CAN_AcknowledgeWriteMessage
 * @brief Function to send acknowledment received write instruction via CAN
 * 
 * @param WriteReqID ID of received write instruction
 * 
 * @retval None.
 **/
void CAN_AcknowledgeWriteMessage(WriteRegsID WriteReqID)
{
	TxData[AcknowledgmentMessage_reg] = Write_AcknowledgmentMessage; // 1st Data Byte: Standard Write Acknowledgment instruction 
	TxData[WriteRegID] = WriteReqID;                                 // 2nd Data Byte: Acknowledged Received Write Command ReqID
	CAN_Transmit(&TxHeader, ACKNOWLEDMENT_DLC, TxData, &TxMailbox);  // Transmit Data
}

/** CAN_ReportError
 * @brief Function to report error via CAN
 * 
 * @param ErrorID ID of reported Error Register
 * 
 * @retval None.
 **/
void CAN_ReportError(ErrorRegsID ErrorID)
{
	TxData[ErrorMessage_reg] = Error_ReportMessage;         // 1st Data Byte: Standard Error Report instruction 
	TxData[ErrorRegID] = ErrorID;                           // 2nd Data Byte: Reported Error ID
	CAN_Transmit(&TxHeader, ERROR_DLC, TxData, &TxMailbox); // Transmit Data
}

/* USER CODE BEGIN CANBUS_Error_Handler */

/** CANBUS_Error_Handler
 * @brief General error handler of CAN connection and communication
 * 
 * @retval None.
 * */
void CANBUS_Error_Handler(void)
{
	__disable_irq();
	/*
	Put here behaviour of ECU when error will be occured.
	*/
}
/* USER CODE END ReadReactionHandlers */

/* USER CODE BEGIN ReadReactionHandlers */

/** Add function name
 * Add brief
 **/
void Read_Var_1_Handler(void)
{
	//Var_1 = 222;
	//TxData[Res];						zakomentowałem na czas tesów odbioru
	//CAN_Transmit(&TxHeader, TxDLC, TxData, TxMailbox)
	//*(ResponseMessage[0].Response_Data1);
}
/* USER CODE END ReadReactionHandlers */

/* USER CODE BEGIN WriteReactionHandlers */

/** Add function name
 * Add brief
 **/

void Write_Toggle_Green_LED_Handler(void)
{
	*(WriteMessage[0].Write_State) = RxData[1];
	if(*(WriteMessage[0].Write_State) == true)
	{
		//HAL_I2C_Master_Transmit_IT(&hi2c1, 0x01, &data_temp_label, data_size);
		//I2C_is_receiving_temp = 1;
		var = true;
		//*(WriteMessage[0].Write_State) = false;
	}
}


void Print_Numbers_Handler(void)
{
	*(WriteMessage[1].Write_State) = RxData[1];
	if(*(WriteMessage[1].Write_State) == true)
	{
		//HAL_I2C_Master_Transmit_IT(&hi2c1, 0x01, &data_temp_label, data_size);
		//I2C_is_receiving_temp = 1;
		print = true;
		//*(WriteMessage[1].Write_State) = false;
	}
}

void map_value_Handler(void)
{
/* USER CODE END WriteReactionHandlers */

	*(WriteMessage[2].Write_Data1) = RxData[1];

//	if(RxData[0] == 0xC3)
//	{
//		map_value_active = true;
//	}

//	if(RxData[0] == 0xD3)
//	{
//		tc_value_active = true;
//	}
//	*(WriteMessage[2].Write_Data2) = RxData[3];
//	*(WriteMessage[2].Write_Data3) = RxData[4];
//	*(WriteMessage[2].Write_Data4) = RxData[5];
//	*(WriteMessage[2].Write_Data5) = RxData[6];
//	*(WriteMessage[2].Write_Data5) = RxData[7];
	/* if(*(WriteMessage[2].Write_State) == true)
	 {
		 confirm_button = true;
	 }*/

	CANdata[0] = 0x05;
	CANdata[1] = can_map_value;
	HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void tc_value_Handler(void)
{
	*(WriteMessage[3].Write_Data1) = RxData[1];

//	if(RxData[0] == 0xD3)
//	{
//		tc_value_active = true;
//	}

	CANdata[0] = 0x04;
    CANdata[1] = can_tc_value;
    HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
    HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void speed_value_Handler(void)
{
	*(WriteMessage[4].Write_Data1) = RxData[1];

//	if(RxData[0] == 0xE3)
//	{
//		speed_value_active = true;
//	}

	CANdata[0] = 0x03;
	CANdata[1] = can_speed_value;
	HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void diff_value_Handler(void)
{
	*(WriteMessage[5].Write_Data1) = RxData[1];

//	if(RxData[0] == 0xF3)
//	{
//		diff_value_active = true;
//	}

	  CANdata[0] = 0x06;
	  CANdata[1] = can_diff_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void ts_value_Handler(void)
{
	*(WriteMessage[6].Write_Data1) = RxData[1];

	  CANdata[0] = 0x07;
	  CANdata[1] = can_ts_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void leng_value_Handler(void)
{
	*(WriteMessage[7].Write_Data1) = RxData[1];

	  CANdata[0] = 0x08;
	  CANdata[1] = can_leng_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void linv_value_Handler(void)
{
	*(WriteMessage[8].Write_Data1) = RxData[1];

	  CANdata[0] = 0x09;
	  CANdata[1] = can_linv_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void bat_value_Handler(void)
{
	*(WriteMessage[9].Write_Data1) = RxData[1];

	  CANdata[0] = 0xA;
	  CANdata[1] = can_bat_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void rinv_value_Handler(void)
{
	*(WriteMessage[10].Write_Data1) = RxData[1];

	  CANdata[0] = 0xB;
	  CANdata[1] = can_rinv_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void reng_value_Handler(void)
{
	*(WriteMessage[11].Write_Data1) = RxData[1];

	  CANdata[0] = 0xC;
	  CANdata[1] = can_reng_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void err_value_Handler(void)
{
	*(WriteMessage[12].Write_Data1) = RxData[1];

	  CANdata[0] = 0xD;
	  CANdata[1] = can_err_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void hv_value_Handler(void)
{
	*(WriteMessage[13].Write_Data1) = RxData[1];

	  CANdata[0] = 0xE;
	  CANdata[1] = can_hv_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}

void low_value_Handler(void)
{
	*(WriteMessage[14].Write_Data1) = RxData[1];

	  CANdata[0] = 0xF;
	  CANdata[1] = can_low_value;
	  HAL_I2C_Master_Transmit_IT(&hi2c1, slave_addr, CANdata, CANsize);
	  HAL_GPIO_TogglePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin);
}
