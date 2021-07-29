
#ifndef _DESC_H_
#define _DESC_H_

#include <linux/types.h>

#define DEVICE_DESCRIPTOR               0x01
#define CONFIGURATION_DESCRIPTOR        0x02
#define STRING_DESCRIPTOR               0x03
#define INTERFACE_DESCRIPTOR            0x04  //½Ó¿ÚÃèÊö·û
#define ENDPOINT_DESCRIPTOR             0x05
#define HID_DESCRIPTOR                  0x21  //HIDÃèÊö·û
#define REPORT_DESCRIPTOR               0x22
#define PHYSICAL_DESCRIPTOR             0x23  //ÎïÀíÃèÊö·û

#define GET_STATUS                      0x00
#define CLEAR_FEATURE                   0x01
#define SET_FEATURE                     0x03
#define SET_ADDRESS                     0x05
#define GET_DESCRIPTOR                  0x06
#define SET_DESCRIPTOR                  0x07
#define GET_CONFIGURATION               0x08
#define SET_CONFIGURATION               0x09
#define GET_INTERFACE                   0x0A  //»ñÈ¡½Ó¿ÚÇëÇó
#define SET_INTERFACE                   0x0B
#define SYNCH_FRAME                     0x0C

#define GET_REPORT                      0x01  //»ñÈ¡±¨¸æ
#define GET_IDLE                        0x02

#define GET_PROTOCOL                    0x03  //»ñÈ¡Ð­Òé
#define SET_REPORT                      0x09
#define SET_IDLE                        0x0A  //ÉèÖÃ¿ÕÏÐ

#define SET_PROTOCOL                    0x0B
#define ENDPOINT_HALT                   0x00
#define DEVICE_REMOTE_WAKEUP            0x01

#define ENDPOINT_TYPE_CONTROL           0x00
#define ENDPOINT_TYPE_ISOCHRONOUS       0x01
#define ENDPOINT_TYPE_BULK              0x02
#define ENDPOINT_TYPE_INTERRUPT         0x03

typedef struct _DEVICE_DCESCRIPTOR_STRUCT
{
 uint8_t blength;
 uint8_t bDescriptorType;	                      //Éè±¸ÃèÊö·ûÀàÐÍ±àºÅ
 uint16_t bcdUSB;
 uint8_t bDeviceClass;                          //USB·ÖÅäµÄÉè±¸Àà´úÂë
 uint8_t bDeviceSubClass;
 uint8_t bDeviceProtocol;                       //USB·ÖÅäµÄÉè±¸Ð­Òé´úÂë
 uint8_t bMaxPacketSize0;
 uint16_t idVendor;                              //³§ÉÌ±àºÅ
 uint16_t idProduct;
 uint16_t bcdDevice;
 uint8_t iManufacturer;
 uint8_t iProduct;                              //ÃèÊö²úÆ·×Ö·û´®µÄË÷Òý
 uint8_t iSerialNumber;
 uint8_t bNumConfigurations;
}
DEVICE_DESCRIPTOR_STRUCT, * pDEVICE_DESCRIPTOR_STRUCT;


//¶¨Òå±ê×¼µÄÅäÖÃÃèÊö·û½á¹¹

typedef struct _CONFIGURATION_DESCRIPTOR_STRUCT
{
 uint8_t bLength;                               //ÅäÖÃÃèÊö·ûµÄ×Ö½ÚÊý´óÐ¡
 uint8_t bDescriptorType;
 uint16_t wTotalLength;
 uint8_t bNumInterfaces;                        //´ËÅäÖÃËùÖ§³ÖµÄ½Ó¿ÚÊýÁ¿
 uint8_t bConfigurationValue;
 uint8_t iConfiguration;                        //ÃèÊö¸ÃÅäÖÃµÄ×Ö·û´®µÄË÷ÒýÖµ
 uint8_t bmAttributes;
 uint8_t MaxPower;                              //Éè±¸´Ó×ÜÏßÌáÈ¡µÄ×î´óµçÁ÷
}
CONFIGURATION_DESCRIPTOR_STRUCT, * pCONFIGURATION_DESCRIPTOR_STRUCT;

typedef struct _INTERFACE_DESCRIPTOR_STRUCT
{
 uint8_t bLength;                               //½Ó¿ÚÃèÊö·ûµÄ×Ö½ÚÊý´óÐ¡
 uint8_t bDescriptorType;
 uint8_t bInterfaceNumber;                      //¸Ã½Ó¿ÚµÄ±àºÅ
 uint8_t bAlternateSetting;
 uint8_t bNumEndpoints;                         //¸Ã½Ó¿ÚÊ¹ÓÃµÄ¶ËµãÊý£¬²»°üÀ¨¶Ëµã0
 uint8_t bInterfaceClass;
 uint8_t bInterfaceSubClass;                    //½Ó¿Ú×ÓÀàÐÍ
 uint8_t bInterfaceProtocol;
 uint8_t iInterface;
}
INTERFACE_DESCRIPTOR_STRUCT, * pINTERFACE_DESCRIPTOR_STRUCT;

typedef struct _ENDPOINT_DESCRIPTOR_STRUCT
{
 uint8_t bLength;
 uint8_t bDescriptorType;
 uint8_t bEndpointAddress;
 uint8_t bmAttributes;                          //¶ËµãµÄ´«ÊäÀàÐÍÊôÐÔ
 uint16_t wMaxPacketSize;
 uint8_t bInterval;
}
ENDPOINT_DESCRIPTOR_STRUCT, * pENDPOINT_DESCRIPTOR_STRUCT;

typedef struct _HID_SUB_DESCRIPTOR_STRUCT
{
 uint8_t bDescriptorType;
 uint16_t wDescriptorLength;
}
HID_SUB_DESCRIPTOR_STRUCT,*pHID_SUB_DESCRIPTOR_STRUCT;

#define NUM_SUB_DESCRIPTORS 1

typedef struct _HID_DESCRIPTOR_STRUCT
{
 uint8_t bLength;
 uint8_t bDescriptorType;
 uint16_t bcdHID;
 uint8_t bCountryCode;
 uint8_t bNumDescriptors;
 HID_SUB_DESCRIPTOR_STRUCT HidSubDescriptors[NUM_SUB_DESCRIPTORS];
}
HID_DESCRIPTOR_STRUCT,*pHID_DESCRIPTOR_STRUCT;

typedef struct _STRING_DESCRIPTOR_STRUCT
{
 uint8_t bLength;                                //×Ö·û´®ÃèÊö·ûµÄ×Ö½ÚÊý´óÐ¡
 uint8_t bDescriptorType;
 uint8_t SomeDesriptor[36];                      //UNICODE±àÂëµÄ×Ö·û´®
}
STRING_DESCRIPTOR_STRUCT, * pSTRING_DESCRIPTOR_STRUCT;

typedef struct _REQUESTCMD
{
 uint8_t bmRequestType;                         //ÇëÇóÃüÁîÀàÐÍ
 uint8_t bRequest;
 uint16_t wValue;                                //ÃüÁîÐÅÏ¢
 uint16_t wIndex;
 uint16_t wLength;                               //´«ÊäÊý¾Ý´óÐ¡
}
REQUESTCMD, * pREQUESTCMD;

#define MAX_CONTROL_DATA_SIZE 16

typedef struct _control_data_buff
{
	REQUESTCMD DeviceRequest;
 	uint16_t wLength;
 	uint16_t wCount;
	uint8_t * pData;
	uint8_t dataBuffer[MAX_CONTROL_DATA_SIZE];
}CONTROL_DATA_BUFF;

#define ENDPOINT_NUMBER           2
#define MAX_CONTROL_DATA_SIZE     16

#define SWAP16(x)  ((((uint16_t)(x))<<8)|(((uint16_t)(x))>>8))

#define LSB(x) ((uint8)(x))

#define MSB(x) ((uint8)(((uint16)(x))>>8))

typedef struct _CON_INT_ENDP_DESCRIPTOR_STRUCT
{
	CONFIGURATION_DESCRIPTOR_STRUCT configuration_descriptor;
	INTERFACE_DESCRIPTOR_STRUCT  interface_descriptor;
 	HID_DESCRIPTOR_STRUCT hid_descriptor;
 	ENDPOINT_DESCRIPTOR_STRUCT  endpoint_descriptor[ENDPOINT_NUMBER];
}CON_INT_ENDP_DESCRIPTOR_STRUCT;

#endif

