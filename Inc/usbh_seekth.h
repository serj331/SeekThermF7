#ifndef INC_USBH_SEEKTH_H_
#define INC_USBH_SEEKTH_H_

#include "usbh_core.h"

#define USB_SEEKT_CLASS 255

extern USBH_ClassTypeDef  SEEKTH_Class;
#define USBH_SEEKTH_CLASS    &SEEKTH_Class

uint16_t* SeekT_getFrame();

#endif /* INC_USBH_SEEKTH_H_ */
