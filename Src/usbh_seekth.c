#include <usbh_seekth.h>

#define SEEKTH_FRAME_WIDTH  208
#define SEEKTH_FRAME_HEIGHT 156

#define DATA_BUF_SIZE (SEEKTH_FRAME_WIDTH * SEEKTH_FRAME_HEIGHT * 2 * 2)

uint8_t DataBuf[DATA_BUF_SIZE];

static uint8_t	DataInPipe;
static int DataRecvState = 0;
static int DataRecvSize = 0;
static int IsHalfBufFull = 0;
static int IsBufFull = 0;
static int IsBufOverrun = 0;

static USBH_StatusTypeDef USBH_SEEKTH_InterfaceInit  (USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef USBH_SEEKTH_InterfaceDeInit  (USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef USBH_SEEKTH_ClassRequest (USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef USBH_SEEKTH_Process(USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef USBH_SEEKTH_SOFProcess(USBH_HandleTypeDef *phost);

USBH_ClassTypeDef  SEEKTH_Class =
{
  "SEEKTHERMAL",
  USB_SEEKT_CLASS,
  USBH_SEEKTH_InterfaceInit,
  USBH_SEEKTH_InterfaceDeInit,
  USBH_SEEKTH_ClassRequest,
  USBH_SEEKTH_Process,
  USBH_SEEKTH_SOFProcess,
  NULL,
};

typedef enum {
	 BEGIN_MEMORY_WRITE              = 82,
	 COMPLETE_MEMORY_WRITE           = 81,
	 GET_BIT_DATA                    = 59,
	 GET_CURRENT_COMMAND_ARRAY       = 68,
	 GET_DATA_PAGE                   = 65,
	 GET_DEFAULT_COMMAND_ARRAY       = 71,
	 GET_ERROR_CODE                  = 53,
	 GET_FACTORY_SETTINGS            = 88,
	 GET_FIRMWARE_INFO               = 78,
	 GET_IMAGE_PROCESSING_MODE       = 63,
	 GET_OPERATION_MODE              = 61,
	 GET_RDAC_ARRAY                  = 77,
	 GET_SHUTTER_POLARITY            = 57,
	 GET_VDAC_ARRAY                  = 74,
	 READ_CHIP_ID                    = 54,
	 RESET_DEVICE                    = 89,
	 SET_BIT_DATA_OFFSET             = 58,
	 SET_CURRENT_COMMAND_ARRAY       = 67,
	 SET_CURRENT_COMMAND_ARRAY_SIZE  = 66,
	 SET_DATA_PAGE                   = 64,
	 SET_DEFAULT_COMMAND_ARRAY       = 70,
	 SET_DEFAULT_COMMAND_ARRAY_SIZE  = 69,
	 SET_FACTORY_SETTINGS            = 87,
	 SET_FACTORY_SETTINGS_FEATURES   = 86,
	 SET_FIRMWARE_INFO_FEATURES      = 85,
	 SET_IMAGE_PROCESSING_MODE       = 62,
	 SET_OPERATION_MODE              = 60,
	 SET_RDAC_ARRAY                  = 76,
	 SET_RDAC_ARRAY_OFFSET_AND_ITEMS = 75,
	 SET_SHUTTER_POLARITY            = 56,
	 SET_VDAC_ARRAY                  = 73,
	 SET_VDAC_ARRAY_OFFSET_AND_ITEMS = 72,
	 START_GET_IMAGE_TRANSFER        = 83,
	 TARGET_PLATFORM                 = 84,
	 TOGGLE_SHUTTER                  = 55,
	 UPLOAD_FIRMWARE_ROW_SIZE        = 79,
	 WRITE_MEMORY_DATA               = 80,
} SeekThRequest;

static USBH_StatusTypeDef VendorControlTransfer(USBH_HandleTypeDef *phost, int direction, uint8_t req, uint16_t value, uint16_t index, uint8_t* data, int dataSize) {
	phost->Control.setup.b.bmRequestType = (direction ? USB_D2H : USB_H2D) | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE;
	phost->Control.setup.b.bRequest = req;
	phost->Control.setup.b.wValue.w = value;
	phost->Control.setup.b.wIndex.w = index;
	phost->Control.setup.b.wLength.w = dataSize;
	USBH_StatusTypeDef st;
	do {
	   st = USBH_CtlReq(phost, data , dataSize);
	} while (st == USBH_BUSY);
	return st;
}

static USBH_StatusTypeDef USBH_SEEKTH_InterfaceInit  (USBH_HandleTypeDef *phost) {
	uint8_t interface = USBH_FindInterface(phost, 255, 240, 0); //"iAP Interface"
	if(interface != 0xFF) {
		USBH_SelectInterface (phost, interface);
		DataInPipe = USBH_AllocPipe(phost, 0x81);
		USBH_OpenPipe(phost, DataInPipe, 0x81, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[1].wMaxPacketSize);
		USBH_LL_SetToggle (phost, DataInPipe, 0);
		return USBH_OK;
	}
	return USBH_FAIL;
}

static USBH_StatusTypeDef USBH_SEEKTH_InterfaceDeInit  (USBH_HandleTypeDef *phost) {
	if(DataInPipe) {
		USBH_ClosePipe(phost, DataInPipe);
		USBH_FreePipe  (phost, DataInPipe);
		DataInPipe = 0;
	}
	return USBH_OK;
}

static USBH_StatusTypeDef USBH_SEEKTH_ClassRequest (USBH_HandleTypeDef *phost) {

	HAL_Delay(1);

	{
		uint8_t data[2] = { 0x00, 0x00 };
		VendorControlTransfer(phost, 0, SET_OPERATION_MODE, 0, 0, data, sizeof(data));
		VendorControlTransfer(phost, 0, SET_OPERATION_MODE, 0, 0, data, sizeof(data));
		VendorControlTransfer(phost, 0, SET_OPERATION_MODE, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[1] = { 0x00 };
		VendorControlTransfer(phost, 0, TARGET_PLATFORM, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[2] = {0x00, 0x00};
		VendorControlTransfer(phost, 0, SET_OPERATION_MODE, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[4];
		VendorControlTransfer(phost, 1, GET_FIRMWARE_INFO, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[12];
		VendorControlTransfer(phost, 1, READ_CHIP_ID, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[6] = { 0x20, 0x00, 0x30, 0x00, 0x00, 0x00 };
		VendorControlTransfer(phost, 0, SET_FACTORY_SETTINGS_FEATURES, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[64];
		VendorControlTransfer(phost, 1, GET_FACTORY_SETTINGS, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[6] = { 0x20, 0x00, 0x50, 0x00, 0x00, 0x00 };
		VendorControlTransfer(phost, 0, SET_FACTORY_SETTINGS_FEATURES, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[64];
		VendorControlTransfer(phost, 1, GET_FACTORY_SETTINGS, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[6] = { 0x0c, 0x00, 0x70, 0x00, 0x00, 0x00 };
		VendorControlTransfer(phost, 0, SET_FACTORY_SETTINGS_FEATURES, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[24];
		VendorControlTransfer(phost, 1, GET_FACTORY_SETTINGS, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[6] = { 0x06, 0x00, 0x08, 0x00, 0x00, 0x00 };
		VendorControlTransfer(phost, 0, SET_FACTORY_SETTINGS_FEATURES, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[12];
		VendorControlTransfer(phost, 1, GET_FACTORY_SETTINGS, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[2] = { 0x08, 0x00 };
		VendorControlTransfer(phost, 0, SET_IMAGE_PROCESSING_MODE, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[2];
		VendorControlTransfer(phost, 1, GET_OPERATION_MODE, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[2] = { 0x08, 0x00 };
		VendorControlTransfer(phost, 0, SET_IMAGE_PROCESSING_MODE, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[2] = { 0x01, 0x00 };
		VendorControlTransfer(phost, 0, SET_OPERATION_MODE, 0, 0, data, sizeof(data));
	}
	HAL_Delay(1);
	{
		uint8_t data[2];
		VendorControlTransfer(phost, 1, GET_OPERATION_MODE, 0, 0, data, sizeof(data));
	}

	IsHalfBufFull = 0;
	IsBufFull = 0;
	IsBufOverrun = 0;
	DataRecvSize = 0;
	DataRecvState = 0;

	phost->pUser(phost, HOST_USER_CLASS_ACTIVE);

	return USBH_OK;
}

static USBH_StatusTypeDef USBH_SEEKTH_Process(USBH_HandleTypeDef *phost) {

	//non-blocking version

	switch(DataRecvState) {
	case 0: {
		uint8_t data[4] = {0xc0, 0x7e, 0, 0};
		VendorControlTransfer(phost, 0, START_GET_IMAGE_TRANSFER, 0, 0, data, sizeof(data));
		DataRecvState = 1;
	}
		break;
	case 1: {
		USBH_LL_SetToggle (phost, DataInPipe, 0);
		USBH_BulkReceiveData(phost, &DataBuf[DataRecvSize], 8192, DataInPipe);
		DataRecvState = 2;
	}
	break;
	case 2: {
		if(USBH_LL_GetURBState(phost, DataInPipe) == USBH_URB_DONE) {
			int xferLen = USBH_LL_GetLastXferSize(phost, DataInPipe);
			DataRecvSize += xferLen;
			if(DataRecvSize == DATA_BUF_SIZE / 2) {
				if(IsHalfBufFull == 1) {
					IsBufOverrun = 1;
				}
				IsHalfBufFull = 1;
				DataRecvState = 0;
			} else if(DataRecvSize >= DATA_BUF_SIZE) {
				if(IsBufFull == 1) {
					IsBufOverrun = 1;
				}
				IsBufFull = 1;
				DataRecvSize = 0;
				DataRecvState = 0;
			} else {
				DataRecvState = 1;
			}
		}
	}
		break;
	}

//	//blocking version
//
//	uint8_t data[4] = {0xc0, 0x7e, 0, 0};
//	VendorControlTransfer(phost, 0, START_GET_IMAGE_TRANSFER, 0, 0, data, 4);
//	_IsBufFull = 0;
//	_DataRecvSize = 0;
//
//	while(1) {
//		int xferLen = 8192;
//
//		USBH_LL_SetToggle (phost, _DataInPipe, 0);
//		USBH_BulkReceiveData(phost, &DataBuf[_DataRecvSize], xferLen, _DataInPipe);
//
//		while(USBH_LL_GetURBState(phost, _DataInPipe) != USBH_URB_DONE);
//		xferLen = USBH_LL_GetLastXferSize(phost, _DataInPipe);
//
//		_DataRecvSize += xferLen;
//
//		if(_DataRecvSize >= ((208*156*2))) {
//			_IsBufFull = 1;
//			break;
//		}
//	}

	return USBH_OK;
}

static USBH_StatusTypeDef USBH_SEEKTH_SOFProcess(USBH_HandleTypeDef *phost) {
	return USBH_OK;
}

uint16_t* SeekT_getFrame() {
	if(IsBufOverrun) {
		IsBufOverrun = 0;
		IsHalfBufFull = 0;
		IsBufFull = 0;
	} else if(IsHalfBufFull) {
		IsHalfBufFull = 0;
		return (uint16_t*)&DataBuf[0];
	} else if(IsBufFull) {
		IsBufFull = 0;
		return (uint16_t*)&DataBuf[DATA_BUF_SIZE / 2];
	}
	return NULL;
}
