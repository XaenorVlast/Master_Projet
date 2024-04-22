/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* envoi_rep_definition */
  uint8_t               Movement_record_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* envoi_rep_definition */
static void Custom_Movement_record_Update_Char(void);
static void Custom_Movement_record_Send_Notification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */
	// copy data update buffer +BCH
	memcpy(UpdateCharData,pNotification->DataTransfered.pPayload,pNotification->DataTransfered.Length);
  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* envoi_rep_definition */
    case CUSTOM_STM_MOVEMENT_RECORD_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MOVEMENT_RECORD_READ_EVT */
    	// +BCH
    	Custom_App_Context.Movement_record_Notification_Status = 1;
    	Custom_Movement_record_Send_Notification();
      /* USER CODE END CUSTOM_STM_MOVEMENT_RECORD_READ_EVT */
      break;

    case CUSTOM_STM_MOVEMENT_RECORD_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MOVEMENT_RECORD_WRITE_NO_RESP_EVT */
    	Custom_App_Context.Movement_record_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_MOVEMENT_RECORD_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_MOVEMENT_RECORD_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MOVEMENT_RECORD_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_MOVEMENT_RECORD_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_MOVEMENT_RECORD_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MOVEMENT_RECORD_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_MOVEMENT_RECORD_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	  /* USER CODE BEGIN CUSTOM_APP_Init */
	Custom_App_Context.Movement_record_Notification_Status = 0;
	UTIL_SEQ_RegTask(1 << CFG_TASK_ACC_rep_ref_ID, UTIL_SEQ_RFU,
			Custom_Movement_record_Send_Notification);
	Custom_Movement_record_Update_Char();

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void BLE_MVT_REF(void) {
	UTIL_SEQ_SetTask(1 << CFG_TASK_ACC_rep_ref_ID, CFG_SCH_PRIO_0);

	//UTIL_SEQ_SetTask( 1<<CFG_TASK_B1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

	Custom_App_Context.Movement_record_Notification_Status = 1;
	printf("on est dans le FSVC_APP_B1_Button_Action");

	return;
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* envoi_rep_definition */
void Custom_Movement_record_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Movement_record_UC_1*/

  /* USER CODE END Movement_record_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MOVEMENT_RECORD, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Movement_record_UC_Last*/

  /* USER CODE END Movement_record_UC_Last*/
  return;
}

void Custom_Movement_record_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Movement_record_NS_1*/
  if (Custom_App_Context.Movement_record_Notification_Status == 1) {
  		updateflag = 1;
  		char rep_valide[] = "salut";
  		memcpy(&NotifyCharData[0], &rep_valide, sizeof(rep_valide));
  		APP_DBG_MSG("-- CUSTOM APPLICATION SERVER  : INFORM CLIENT rep good \n");

  	} else {
  		APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n");
  	}
  /* USER CODE END Movement_record_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_MOVEMENT_RECORD, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Movement_record_NS_Last*/

  /* USER CODE END Movement_record_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
