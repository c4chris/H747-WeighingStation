/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_host.c
  * @author  MCD Application Team
  * @brief   USBX host applicative file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_usbx_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Define HID Touchscreen Class structure.  */

typedef struct HOST_CLASS_HID_TOUCHSCREEN_STRUCT
{
    ULONG             host_class_hid_touchscreen_state;
    UX_HOST_CLASS_HID *host_class_hid_touchscreen_hid;
    USHORT            host_class_hid_touchscreen_id;
    ULONG             Tip;
    ULONG             X;
    ULONG             Y;
    ULONG             other;
    ULONG             event;
} HOST_CLASS_HID_TOUCHSCREEN;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USBX_APP_STACK_SIZE                  (2 * 1024)
#define USBX_MEMORY_SIZE                     (64 * 1024)
#define APP_QUEUE_SIZE                       5

#define HOST_CLASS_HID_DIGITIZER_TOUCHSCREEN 0x04

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern HCD_HandleTypeDef                 hhcd_USB_OTG_HS;
TX_THREAD                                ux_app_thread;
TX_THREAD                                mouse_app_thread;
TX_THREAD                                touchscreen_app_thread;
TX_QUEUE                                 ux_app_MsgQueue;
UX_HOST_CLASS_HID                        *hid;
UX_HOST_CLASS_HID_CLIENT                 *hid_client;
UX_HOST_CLASS_HID_MOUSE                  *mouse;
HOST_CLASS_HID_TOUCHSCREEN               *touchscreen;

__ALIGN_BEGIN ux_app_devInfotypeDef       ux_dev_info  __ALIGN_END;

UCHAR system_host_class_hid_client_touchscreen_name[] = "host_class_hid_client_touchscreen";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

extern void Error_Handler(void);
UINT host_class_hid_touchscreen_entry(UX_HOST_CLASS_HID_CLIENT_COMMAND *);

/* USER CODE END PFP */
/**
  * @brief  Application USBX Host Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_USBX_Host_Init(VOID *memory_ptr)
{
  UINT ret = UX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_USBX_Host_MEM_POOL */
    (void)byte_pool;
  /* USER CODE END MX_USBX_Host_MEM_POOL */

  /* USER CODE BEGIN MX_USBX_Host_Init */
  CHAR *pointer;
 
  /* Allocate the stack for thread 0. */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       USBX_MEMORY_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Initialize USBX memory. */
  if (ux_system_initialize(pointer, USBX_MEMORY_SIZE, UX_NULL, 0) != UX_SUCCESS)

  {
    ret = UX_ERROR;
  }

  /* register a callback error function */
  _ux_utility_error_callback_register(&ux_host_error_callback);

  /* Allocate the stack for thread 0. */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create the main App thread. */
  if (tx_thread_create(&ux_app_thread, "thread 0", usbx_app_thread_entry, 0,
                       pointer, USBX_APP_STACK_SIZE, 25, 25, 1,
                       TX_AUTO_START) != TX_SUCCESS)
  {
    ret = TX_THREAD_ERROR;
  }

  /* Allocate the stack for thread 1. */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create the HID mouse App thread. */
  if (tx_thread_create(&mouse_app_thread, "thread 1", hid_mouse_thread_entry, 0,
                       pointer, USBX_APP_STACK_SIZE, 30, 30, 1,
                       TX_AUTO_START) != TX_SUCCESS)
  {
    ret = TX_THREAD_ERROR;
  }

  /* Allocate the stack for touchscreen thread. */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       USBX_APP_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }

  /* Create the HID touchscreen App thread. */
  if (tx_thread_create(&touchscreen_app_thread, "thread 2", hid_touchscreen_thread_entry, 0,
                       pointer, USBX_APP_STACK_SIZE, 30, 30, 1,
                       TX_AUTO_START) != TX_SUCCESS)
  {
    ret = TX_THREAD_ERROR;
  }

  /* Allocate Memory for the Queue */
  if (tx_byte_allocate(byte_pool, (VOID **) &pointer,
                       APP_QUEUE_SIZE * sizeof(ux_app_devInfotypeDef), TX_NO_WAIT) != TX_SUCCESS)
  {
    ret = TX_POOL_ERROR;
  }
 
  /* Create the MsgQueue */
  if (tx_queue_create(&ux_app_MsgQueue, "Message Queue app", sizeof(ux_app_devInfotypeDef),
                      pointer, APP_QUEUE_SIZE * sizeof(ux_app_devInfotypeDef)) != TX_SUCCESS)
  {
    ret = TX_QUEUE_ERROR;
  }

  /* USER CODE END MX_USBX_Host_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
/**
  * @brief App_USBX_Host_Init
  *        Initialization of USB device.
  * Init USB Host Library, add supported class and start the library
  * @retval None
  */
UINT App_USBX_Host_Init(void)
{
  UINT ret = UX_SUCCESS;

  /* The code below is required for installing the host portion of USBX. */
  if (ux_host_stack_initialize(ux_host_event_callback) != UX_SUCCESS)
  {
    ret = UX_ERROR;
  }

  /* Register hid class. */
  if (ux_host_stack_class_register(_ux_system_host_class_hid_name,
                                   _ux_host_class_hid_entry) != UX_SUCCESS)
  {
    ret = UX_ERROR;
  }

  /* https://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/ */
  /* Register HID TouchScreen client */
  if (ux_host_class_hid_client_register(system_host_class_hid_client_touchscreen_name,
                                        host_class_hid_touchscreen_entry) != UX_SUCCESS)
  {
    ret = UX_ERROR;
  }

  /* Register HID Mouse client */
  if (ux_host_class_hid_client_register(_ux_system_host_class_hid_client_mouse_name,
                                        ux_host_class_hid_mouse_entry) != UX_SUCCESS)
  {
    ret = UX_ERROR;
  }

  /* Register all the USB host controllers available in this system.  */
  if (ux_host_stack_hcd_register(_ux_system_host_hcd_stm32_name,
                                 _ux_hcd_stm32_initialize, USB_OTG_HS_PERIPH_BASE,
                                 (ULONG)&hhcd_USB_OTG_HS) != UX_SUCCESS)
  {
    ret = UX_ERROR;
  }

  /* Enable USB Global Interrupt*/
  HAL_HCD_Start(&hhcd_USB_OTG_HS);

  return ret;
}

/**
  * @brief  Application_thread_entry .
  * @param  ULONG arg
  * @retval Void
  */
void  usbx_app_thread_entry(ULONG arg)
{
  printf("Starting CM7 Run on %s and %s\n", _tx_version_id, _ux_version_id);

  /* Initialize USBX_Host */
  App_USBX_Host_Init();

  /* Start Application Message */
  USBH_UsrLog(" **** USB OTG HS in FS HID Host **** \n");
  USBH_UsrLog("USB Host library started.\n");

  /* Wait for Device to be attached */
  USBH_UsrLog("Starting HID Application");
  USBH_UsrLog("Connect your HID Device\n");

  while (1)
  {
    /* Wait for a hid device to be connected */
    if (tx_queue_receive(&ux_app_MsgQueue, &ux_dev_info, TX_WAIT_FOREVER)!= TX_SUCCESS)
    {
     /*Error*/
     Error_Handler();
    }

    if (ux_dev_info.Dev_state == Device_connected)
    {
      switch (ux_dev_info.Device_Type)
      {
        case Mouse_Device :
          mouse = hid_client->ux_host_class_hid_client_local_instance;
          USBH_UsrLog("HID_Mouse_Device");
          USBH_UsrLog("PID: %#x ", (UINT)mouse->ux_host_class_hid_mouse_hid->ux_host_class_hid_device->ux_device_descriptor.idProduct);
          USBH_UsrLog("VID: %#x ", (UINT)mouse->ux_host_class_hid_mouse_hid->ux_host_class_hid_device->ux_device_descriptor.idVendor);
          USBH_UsrLog("USB HID Host Mouse App...");
          USBH_UsrLog("Mouse is ready...\n");
          break;

        case Touchscreen_Device :
          touchscreen = hid_client->ux_host_class_hid_client_local_instance;
          USBH_UsrLog("HID_Touchscreen_Device - VID: %#x - PID: %#x",
											(UINT)touchscreen->host_class_hid_touchscreen_hid->ux_host_class_hid_device->ux_device_descriptor.idVendor,
											(UINT)touchscreen->host_class_hid_touchscreen_hid->ux_host_class_hid_device->ux_device_descriptor.idProduct);
          break;

        case Unknown_Device :
          USBH_ErrLog("Unsupported USB device");
          break;

        default :
          break;
      }
    }
    else
    {
      /* clear hid_client local instance */
      mouse = NULL;
      touchscreen = NULL;
    }
  }
}

/**
* @brief ux_host_event_callback
* @param ULONG event
           This parameter can be one of the these values:
             1 : UX_DEVICE_INSERTION
             2 : UX_DEVICE_REMOVAL
             3 : UX_HID_CLIENT_INSERTION
             4 : UX_HID_CLIENT_REMOVAL
         UX_HOST_CLASS * Current_class
         VOID * Current_instance
* @retval Status
*/
UINT ux_host_event_callback(ULONG event, UX_HOST_CLASS *Current_class, VOID *Current_instance)
{
  UINT status;
  UX_HOST_CLASS *hid_class;

  switch (event)
  {
    case UX_DEVICE_INSERTION :
      /* Get current Hid Class */
      status = ux_host_stack_class_get(_ux_system_host_class_hid_name, &hid_class);

      if (status == UX_SUCCESS)
      {
        if ((hid_class == Current_class) && (hid == NULL))
        {
          /* Get current Hid Instance */
          hid = Current_instance;
          /* Get the HID Client */
          hid_client = hid->ux_host_class_hid_client;

          if (hid->ux_host_class_hid_client->ux_host_class_hid_client_status != (ULONG) UX_HOST_CLASS_INSTANCE_LIVE)
          {
            ux_dev_info.Device_Type = Unknown_Device;
          }
          /* Check the HID_client if this is a HID mouse or touchscreen device. */
          if (ux_utility_memory_compare(hid_client -> ux_host_class_hid_client_name,
                                        _ux_system_host_class_hid_client_mouse_name,
                                        ux_utility_string_length_get(_ux_system_host_class_hid_client_mouse_name)) == UX_SUCCESS)
          {
            /* update HID device Type */
            ux_dev_info.Device_Type = Mouse_Device;

            /* put a message queue to usbx_app_thread_entry */
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          } else if (ux_utility_memory_compare(hid_client->ux_host_class_hid_client_name,
																							 system_host_class_hid_client_touchscreen_name,
																							 ux_utility_string_length_get(system_host_class_hid_client_touchscreen_name)) == UX_SUCCESS)
	          {
	            /* update HID device Type */
	            ux_dev_info.Device_Type = Touchscreen_Device;

	            /* put a message queue to usbx_app_thread_entry */
	            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
	          }
          else
          {
            ux_dev_info.Device_Type = Unknown_Device;
            ux_dev_info.Dev_state = Device_connected;
            tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
          }
        }
      }
      else
      {
        /* No HID class found */
        USBH_ErrLog("NO HID Class found");
      }
      break;

    case UX_DEVICE_REMOVAL :

      if (Current_instance == hid)
      {
        /* Free Instance */
        hid = NULL;
        USBH_UsrLog("USB Device Unplugged");
        ux_dev_info.Dev_state   = No_Device;
        ux_dev_info.Device_Type = Unknown_Device;
      }
      break;

    case UX_HID_CLIENT_INSERTION :
      USBH_UsrLog("HID Client Plugged");
      ux_dev_info.Dev_state = Device_connected;
      break;

    case UX_HID_CLIENT_REMOVAL:
      USBH_UsrLog("HID Client Unplugged");
      ux_dev_info.Dev_state   =  Device_disconnected;
      ux_dev_info.Device_Type =  Unknown_Device;
      tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);

      break;

    default:
      break;

  }

  return (UINT) UX_SUCCESS;
}

/**
* @brief ux_host_error_callback
* @param ULONG event
         UINT system_context
         UINT error_code
* @retval Status
*/
VOID ux_host_error_callback(UINT system_level, UINT system_context, UINT error_code)
{
  switch (error_code)
  {
    case UX_DEVICE_ENUMERATION_FAILURE :

      ux_dev_info.Device_Type = Unknown_Device;
      ux_dev_info.Dev_state   = Device_connected;
      tx_queue_send(&ux_app_MsgQueue, &ux_dev_info, TX_NO_WAIT);
      break;

    case  UX_NO_DEVICE_CONNECTED :
      USBH_UsrLog("USB Device disconnected");
      break;

    default:
      break;
  }
}

VOID host_class_hid_touchscreen_callback(UX_HOST_CLASS_HID_REPORT_CALLBACK *callback)
{
	UX_HOST_CLASS_HID_CLIENT   *hid_client;
	HOST_CLASS_HID_TOUCHSCREEN *touchscreen_instance;
	UCHAR                      *rb;

	/* Get the HID client instance that issued the callback.  */
	hid_client = callback->ux_host_class_hid_report_callback_client;

	/* Get the touchscreen local instance */
	touchscreen_instance = (HOST_CLASS_HID_TOUCHSCREEN *) hid_client->ux_host_class_hid_client_local_instance;

	/* Get the report buffer.  */
	rb = (UCHAR *) callback->ux_host_class_hid_report_callback_buffer;
	if (rb[0] == 0x4) {
		touchscreen_instance->Tip = rb[1];
		touchscreen_instance->X = rb[2] | (rb[3] << 8);
		touchscreen_instance->Y = rb[4] | (rb[5] << 8);
		touchscreen_instance->other = rb[6] | (rb[7] << 8) | (rb[8] << 16) | (rb[9] << 24);
	}
	touchscreen_instance->event += 1;

	/* Return to caller.  */
	return;
}

UINT  host_class_hid_touchscreen_activate(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{
	UX_HOST_CLASS_HID_REPORT_GET_ID   report_id;
	UX_HOST_CLASS_HID                *hid;
	UX_HOST_CLASS_HID_CLIENT         *hid_client;
	HOST_CLASS_HID_TOUCHSCREEN       *touchscreen_instance;
	UINT                              status;

	/* Get the instance to the HID class.  */
	hid = command->ux_host_class_hid_client_command_instance;

	/* And of the HID client.  */
	hid_client = hid->ux_host_class_hid_client;

	/* Get some memory for both the HID class instance of this client
     and for the callback.  */
	touchscreen_instance = (HOST_CLASS_HID_TOUCHSCREEN *) _ux_utility_memory_allocate(UX_NO_ALIGN, UX_REGULAR_MEMORY,
																																										 sizeof(HOST_CLASS_HID_TOUCHSCREEN));
	if(touchscreen_instance == UX_NULL)
		return(UX_MEMORY_INSUFFICIENT);

	/* Attach the touchscreen instance to the client instance.  */
	hid_client->ux_host_class_hid_client_local_instance = (VOID *) touchscreen_instance;

	/* Save the HID instance in the client instance.  */
	touchscreen_instance->host_class_hid_touchscreen_hid = hid;

	/* The instance is live now.  */
	touchscreen_instance->host_class_hid_touchscreen_state = UX_HOST_CLASS_INSTANCE_LIVE;

	/* Get the report ID for the touchscreen. The touchscreen is a INPUT report. */
	report_id.ux_host_class_hid_report_get_report = UX_NULL;
	report_id.ux_host_class_hid_report_get_type = UX_HOST_CLASS_HID_REPORT_TYPE_INPUT;
	status = _ux_host_class_hid_report_id_get(hid, &report_id);

	/* The report ID should exist.  */
	if (status == UX_SUCCESS)
	{
		/* I want report 4 */
		while (status == UX_SUCCESS && report_id.ux_host_class_hid_report_get_id != 4)
			status = _ux_host_class_hid_report_id_get(hid, &report_id);
		if (status == UX_SUCCESS)
		{
			/* Save the touchscreen report ID. */
			touchscreen_instance->host_class_hid_touchscreen_id = (USHORT)report_id.ux_host_class_hid_report_get_id;

			/* Set the idle rate of the touchscreen to 0.  */
			status = _ux_host_class_hid_idle_set(hid, 0, touchscreen_instance->host_class_hid_touchscreen_id);
			//printf("idle_set status = %u, id = %u\n", status, touchscreen_instance->host_class_hid_touchscreen_id);
		}
	}

	/* If we are OK, go on.  */
	if (status == UX_SUCCESS)
	{
		UX_HOST_CLASS_HID_REPORT_CALLBACK call_back;
		/* Initialize the report callback.  */
		call_back.ux_host_class_hid_report_callback_function = host_class_hid_touchscreen_callback;
		call_back.ux_host_class_hid_report_callback_flags =    UX_HOST_CLASS_HID_REPORT_RAW;
		call_back.ux_host_class_hid_report_callback_id =       touchscreen_instance->host_class_hid_touchscreen_id;
		call_back.ux_host_class_hid_report_callback_length =   0;
		call_back.ux_host_class_hid_report_callback_buffer =   UX_NULL;

		/* Register the report call back when data comes in on this report.  */
		status =  _ux_host_class_hid_report_callback_register(hid, &call_back);
	}

	/* If we are OK, go on.  */
	if (status == UX_SUCCESS)
	{
		/* Start the periodic report.  */
		status =  _ux_host_class_hid_periodic_report_start(hid);

		if (status == UX_SUCCESS)
		{
			/* If all is fine and the device is mounted, we may need to inform the application
         if a function has been programmed in the system structure.  */
			if (_ux_system_host->ux_system_host_change_function != UX_NULL)
			{
				/* Call system change function.  */
				_ux_system_host->ux_system_host_change_function(UX_HID_CLIENT_INSERTION, hid->ux_host_class_hid_class, (VOID *) hid_client);
			}

			/* Return completion status.  */
			return(status);
		}
	}

	/* We are here if there is error.  */

	/* Detach the client instance.  */
	hid_client->ux_host_class_hid_client_local_instance = UX_NULL;

	/* Free mouse client instance.  */
	_ux_utility_memory_free(touchscreen_instance);

	/* Return completion status.  */
	return(status);
}

UINT host_class_hid_touchscreen_deactivate(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{
	UX_HOST_CLASS_HID        *hid;
	UX_HOST_CLASS_HID_CLIENT *hid_client;
	UINT                     status;

	/* Get the instance to the HID class.  */
	hid = command->ux_host_class_hid_client_command_instance;

	/* Stop the periodic report.  */
	status = _ux_host_class_hid_periodic_report_stop(hid);

	/* Get the HID client pointer.  */
	hid_client = hid->ux_host_class_hid_client;

	/* Now free the instance memory.  */
	_ux_utility_memory_free(hid_client->ux_host_class_hid_client_local_instance);

	/* We may need to inform the application
     if a function has been programmed in the system structure.  */
	if (_ux_system_host->ux_system_host_change_function != UX_NULL)
	{
		/* Call system change function.  */
		_ux_system_host->ux_system_host_change_function(UX_HID_CLIENT_REMOVAL, hid->ux_host_class_hid_class, (VOID *) hid_client);
	}

	/* Return completion status.  */
	return(status);
}

UINT host_class_hid_touchscreen_entry(UX_HOST_CLASS_HID_CLIENT_COMMAND *command)
{
	UINT status;

	/* The command request will tell us we need to do here, either a enumeration
     query, an activation or a deactivation.  */
	switch (command -> ux_host_class_hid_client_command_request)
	{
		case UX_HOST_CLASS_COMMAND_QUERY:
			/* The query command is used to let the HID class know if we want to own
         this device or not.  */
			if ((command->ux_host_class_hid_client_command_page == UX_HOST_CLASS_HID_PAGE_DIGITIZER)
					&& (command->ux_host_class_hid_client_command_usage == HOST_CLASS_HID_DIGITIZER_TOUCHSCREEN))
			{
				/* printf("Accepting Digitizer\n"); */
				return(UX_SUCCESS);
			}
			else
				return(UX_NO_CLASS_MATCH);

		case UX_HOST_CLASS_COMMAND_ACTIVATE:
			/* The activate command is used by the HID class to start the HID client.  */
			status = host_class_hid_touchscreen_activate(command);
			/* Return completion status.  */
			return(status);

		case UX_HOST_CLASS_COMMAND_DEACTIVATE:
			/* The deactivate command is used by the HID class when it received a deactivate
         command from the USBX stack and there was a HID client attached to the HID instance.  */
			status = host_class_hid_touchscreen_deactivate(command);
			/* Return completion status.  */
			return(status);
	}

	/* Return error status.  */
	return(UX_ERROR);
}

void  hid_touchscreen_thread_entry(ULONG arg)
{
	ULONG value = 0;
	ULONG old_Pos_x = 0;
	ULONG old_Pos_y = 0;
	ULONG Pos_x = 0;
	ULONG Pos_y = 0;
	//ULONG lastEvent = 0;

	while (1)
	{
		/* start if the hid client is a touchscreen and connected */
		if ((ux_dev_info.Device_Type == Touchscreen_Device) && (ux_dev_info.Dev_state == Device_connected))
		{
			Pos_x = touchscreen->X * 720 / 16384;
			Pos_y = touchscreen->Y * 576 / 9600;

			if ((Pos_x != old_Pos_x) || (Pos_y != old_Pos_y) || value != touchscreen->Tip)
			{
				GX_EVENT e = {0};
				//e.gx_event_display_handle = LCD_FRAME_BUFFER;
				e.gx_event_payload.gx_event_pointdata.gx_point_x = Pos_x;
				e.gx_event_payload.gx_event_pointdata.gx_point_y = Pos_y;
				if (touchscreen->Tip == 0)
				{
					e.gx_event_type = GX_EVENT_PEN_UP; // pen UP
				}
				else if (value == 0)
				{
					e.gx_event_type = GX_EVENT_PEN_DOWN; // pen DOWN
				}
				else
					e.gx_event_type = GX_EVENT_PEN_DRAG; // pen DRAG
				/* Push the event to event pool. */
				if (e.gx_event_type == GX_EVENT_PEN_DRAG)
					gx_system_event_fold(&e);
				else
					gx_system_event_send(&e);
				/* update (x,y)old position */
				old_Pos_x = Pos_x;
				old_Pos_y = Pos_y;
				value = touchscreen->Tip;
			}
		}
		tx_thread_sleep(10);
	}
}

/* USER CODE END 1 */
