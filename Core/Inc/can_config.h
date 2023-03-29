/*
 * can_config.h
 *
 *  Created on: 27.02.2022
 * Modified on: 30.07.2022
 *      Author: Krystian Sosin
 *     Version: 1.0.1
 * Last change: Fix minor bugs and add CAN_AcknowledgeWriteMessage() function.
 */

#ifndef INC_CAN_CONFIG_H_
#define INC_CAN_CONFIG_H_

#include "main.h"
#include "can.h"
#include "stdbool.h"

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------START OF DEFINES----------------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* USER CODE BEGIN DEFINES */
#define NUMBER_OF_READ_REGS  (3U)
#define NUMBER_OF_WRITE_REGS (15U) // oryginalnie było (3U) zmieniłem na (4U)
#define NUMBER_OF_ERROR_REGS (3U)
/* USER CODE END DEFINES */

#define ERROR_DLC            (2U)
#define ACKNOWLEDMENT_DLC    (2U)
#define FIRST_ARRAY_ELEMENT  (0U)

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------END OF DEFINES------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------START OF ENUMS DEFINITIONS------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* USER CODE BEGIN Registers */
/* Registers */
// All of Reg1 - RegN should be replaced by meaningful name of reg (don't delete prefixs)
typedef enum
{
    Read_Var_1_ID = 0xDFu,
} ReadRegsID;

typedef enum
{
	Write_Toggle_Green_LED_ID = 0xA3u,
	Print_Numbers_ID = 0xB3u,
	MAP_VALUE_ID = 0xC3u,
	TC_VALUE_ID = 0xD3u,
	SPEED_VALUE_ID = 0xE3u,
	DIFF_VALUE_ID = 0xF3u,
	TS_VALUE_ID = 0xF4u,
	LENG_VALUE_ID = 0xF5u,
	LINV_VALUE_ID = 0xF6u,
	BAT_VALUE_ID = 0xF7u,
	RINV_VALUE_ID = 0xF8u,
	RENG_VALUE_ID = 0xF9u,
	ERR_VALUE_ID = 0xFAu,
	HV_VALUE_ID = 0xFBu,
	LOW_VALUE_ID = 0xFCu
} WriteRegsID;

typedef enum
{
	Error_MeaningfulRegName1_ID = 0xAAu,
    Error_MeaningfulRegName2_ID = 0xBBu,
    Error_MeaningfulRegNameN_ID = 0xCCu
} ErrorRegsID;

enum CAN_MSG_FRAME
{
	MSG_ID = 0u,
	MSG_VALUE = 1u
};
/* USER CODE END Registers */

/* Enums to avoid magic numbers */
typedef enum
{
	ReadMessage,
	ReadRegID
} ReadFrameID;

typedef enum
{
	ResponseRegID,
	ResponseData1,
	ResponseData2
} ResponseFrame;

typedef enum
{
	WriteMessage_reg,
	WriteData1,
	WriteData2
} WriteFrame;

typedef enum
{
	ErrorMessage_reg,
	ErrorRegID
} ErrorFrame;

typedef enum
{
	AcknowledgmentMessage_reg,
	WriteRegID
} AcknowledgmentFrame;

typedef enum
{
	Error_ReportMessage = 0x1Du,
	Read_RequestMessage = 0x3Du,
	Write_AcknowledgmentMessage = 0x5Du
} CANStadardMessage;

/* USER CODE BEGIN NodeAddress */
typedef enum
{
	Rx_ID = 0x004Au, //tutaj zmiana adresu mcs_scrn //domyślnie 0x4Fu
	Tx_ID = 0x004Fu //0x4A
} NodeAddress;
/* USER CODE END NodeAddress */

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------END OF ENUMS DEFINITIONS--------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------START OF VARIABLES DEFINITIONS------------------------------ */
/* ---------------------------------------------------------------------------------------- */

typedef void (*ReadReactionHandlerFuncPtr)(void);
typedef void (*WriteReactionHandlerFuncPtr)(void);

typedef struct
{
	WriteRegsID  Write_RegID;
	WriteReactionHandlerFuncPtr Write_ReactionHandler;
	bool*        Write_State;
	uint8_t*     Write_Data1;
//	uint8_t*     Write_Data2;//te linie została dodana bo potrzeba odebrac wiecej danych w jednej wiadomości
//	uint8_t*     Write_Data3;
//	uint8_t*     Write_Data4;
//	uint8_t*     Write_Data5;
//	uint8_t*     Write_Data6;
} WriteMessageFrame;

typedef struct
{
	uint8_t  Response_DLC;
	ReadRegsID  Response_RegID;
	ReadReactionHandlerFuncPtr Read_ReactionHandler;
	uint8_t* Response_Data1;
	uint8_t* Response_Data2;
} ResponseMessageFrame;

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------END OF STRUCTURES DEFINITIONS-------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------START OF FUNCTIONS DECLARATIONS------------------------------ */
/* ---------------------------------------------------------------------------------------- */

void CAN_Init(void);
void CAN_On_Receive(uint8_t *RxData);
void CAN_Receive(CAN_HandleTypeDef *CANPointer, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);
void CAN_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t TxDLC, uint8_t *TxData, uint32_t *TxMailbox);
void CAN_Respond(void);
void CAN_ProcessWriteCommand(void);
void CAN_AcknowledgeWriteMessage(WriteRegsID WriteReqID);
void CAN_ReportError(ErrorRegsID ErrorID);
void CANBUS_Error_Handler(void);
void ReadReactionHandler1(void);

void Write_Toggle_Green_LED_Handler(void);
void Print_Numbers_Handler(void);

void map_value_Handler(void);
void tc_value_Handler(void);
void speed_value_Handler(void);
void diff_value_Handler(void);
void ts_value_Handler(void);
void leng_value_Handler(void);
void linv_value_Handler(void);
void bat_value_Handler(void);
void rinv_value_Handler(void);
void reng_value_Handler(void);
void err_value_Handler(void);
void hv_value_Handler(void);
void low_value_Handler(void);

void Read_Var_1_Handler(void);

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------END OF FUNCTIONS DECLARATIONS-------------------------------- */
/* ---------------------------------------------------------------------------------------- */

#endif /* INC_CAN_CONFIG_H_ */
