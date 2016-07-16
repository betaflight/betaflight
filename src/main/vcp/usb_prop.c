/**
 ******************************************************************************
 * @file    usb_prop.c
 * @author  MCD Application Team
 * @version V4.0.0
 * @date    21-January-2013
 * @brief   All processing related to Virtual Com Port Demo
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t Request = 0;

LINE_CODING linecoding = { 115200, /* baud rate*/
0x00, /* stop bits-1*/
0x00, /* parity - none*/
0x08 /* no. of bits 8*/
};

/* -------------------------------------------------------------------------- */
/*  Structures initializations */
/* -------------------------------------------------------------------------- */

DEVICE Device_Table = {
EP_NUM, 1 };

DEVICE_PROP Device_Property = { Virtual_Com_Port_init, Virtual_Com_Port_Reset, Virtual_Com_Port_Status_In, Virtual_Com_Port_Status_Out, Virtual_Com_Port_Data_Setup, Virtual_Com_Port_NoData_Setup, Virtual_Com_Port_Get_Interface_Setting, Virtual_Com_Port_GetDeviceDescriptor,
        Virtual_Com_Port_GetConfigDescriptor, Virtual_Com_Port_GetStringDescriptor, 0, 0x40 /*MAX PACKET SIZE*/
};

USER_STANDARD_REQUESTS User_Standard_Requests = {
Virtual_Com_Port_GetConfiguration, Virtual_Com_Port_SetConfiguration,
Virtual_Com_Port_GetInterface,
Virtual_Com_Port_SetInterface,
Virtual_Com_Port_GetStatus,
Virtual_Com_Port_ClearFeature,
Virtual_Com_Port_SetEndPointFeature,
Virtual_Com_Port_SetDeviceFeature, Virtual_Com_Port_SetDeviceAddress };

ONE_DESCRIPTOR Device_Descriptor = { (uint8_t*)Virtual_Com_Port_DeviceDescriptor,
VIRTUAL_COM_PORT_SIZ_DEVICE_DESC };

ONE_DESCRIPTOR Config_Descriptor = { (uint8_t*)Virtual_Com_Port_ConfigDescriptor,
VIRTUAL_COM_PORT_SIZ_CONFIG_DESC };

ONE_DESCRIPTOR String_Descriptor[4] = { { (uint8_t*)Virtual_Com_Port_StringLangID, VIRTUAL_COM_PORT_SIZ_STRING_LANGID }, { (uint8_t*)Virtual_Com_Port_StringVendor, VIRTUAL_COM_PORT_SIZ_STRING_VENDOR }, { (uint8_t*)Virtual_Com_Port_StringProduct,
        VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT }, { (uint8_t*)Virtual_Com_Port_StringSerial, VIRTUAL_COM_PORT_SIZ_STRING_SERIAL } };

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 * Function Name  : Virtual_Com_Port_init.
 * Description    : Virtual COM Port Mouse init routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Virtual_Com_Port_init(void)
{

    /* Update the serial number string descriptor with the data from the unique
     ID*/
    Get_SerialNum();

    pInformation->Current_Configuration = 0;

    /* Connect the device */
    PowerOn();

    /* Perform basic device initialization operations */
    USB_SIL_Init();

    bDeviceState = UNCONNECTED;
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_Reset
 * Description    : Virtual_Com_Port Mouse reset routine
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Virtual_Com_Port_Reset(void)
{
    /* Set Virtual_Com_Port DEVICE as not configured */
    pInformation->Current_Configuration = 0;

    /* Current Feature initialization */
    pInformation->Current_Feature = Virtual_Com_Port_ConfigDescriptor[7];

    /* Set Virtual_Com_Port DEVICE with the default Interface*/
    pInformation->Current_Interface = 0;

    SetBTABLE(BTABLE_ADDRESS);

    /* Initialize Endpoint 0 */
    SetEPType(ENDP0, EP_CONTROL);
    SetEPTxStatus(ENDP0, EP_TX_STALL);
    SetEPRxAddr(ENDP0, ENDP0_RXADDR);
    SetEPTxAddr(ENDP0, ENDP0_TXADDR);
    Clear_Status_Out(ENDP0);
    SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);
    SetEPRxValid(ENDP0);

    /* Initialize Endpoint 1 */
    SetEPType(ENDP1, EP_BULK);
    SetEPTxAddr(ENDP1, ENDP1_TXADDR);
    SetEPTxStatus(ENDP1, EP_TX_NAK);
    SetEPRxStatus(ENDP1, EP_RX_DIS);

    /* Initialize Endpoint 2 */
    SetEPType(ENDP2, EP_INTERRUPT);
    SetEPTxAddr(ENDP2, ENDP2_TXADDR);
    SetEPRxStatus(ENDP2, EP_RX_DIS);
    SetEPTxStatus(ENDP2, EP_TX_NAK);

    /* Initialize Endpoint 3 */
    SetEPType(ENDP3, EP_BULK);
    SetEPRxAddr(ENDP3, ENDP3_RXADDR);
    SetEPRxCount(ENDP3, VIRTUAL_COM_PORT_DATA_SIZE);
    SetEPRxStatus(ENDP3, EP_RX_VALID);
    SetEPTxStatus(ENDP3, EP_TX_DIS);

    /* Set this device to response on default address */
    SetDeviceAddress(0);

    bDeviceState = ATTACHED;
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_SetConfiguration.
 * Description    : Update the device state to configured.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Virtual_Com_Port_SetConfiguration(void)
{
    DEVICE_INFO *pInfo = &Device_Info;

    if (pInfo->Current_Configuration != 0) {
        /* Device configured */
        bDeviceState = CONFIGURED;
    }
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_SetConfiguration.
 * Description    : Update the device state to addressed.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Virtual_Com_Port_SetDeviceAddress(void)
{
    bDeviceState = ADDRESSED;
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_Status_In.
 * Description    : Virtual COM Port Status In Routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Virtual_Com_Port_Status_In(void)
{
    if (Request == SET_LINE_CODING) {
        Request = 0;
    }
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_Status_Out
 * Description    : Virtual COM Port Status OUT Routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Virtual_Com_Port_Status_Out(void)
{
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_Data_Setup
 * Description    : handle the data class specific requests
 * Input          : Request Nb.
 * Output         : None.
 * Return         : USB_UNSUPPORT or USB_SUCCESS.
 *******************************************************************************/
RESULT Virtual_Com_Port_Data_Setup(uint8_t RequestNo)
{
    uint8_t *(*CopyRoutine)( uint16_t);

    CopyRoutine = NULL;

    if (RequestNo == GET_LINE_CODING) {
        if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
            CopyRoutine = Virtual_Com_Port_GetLineCoding;
        }
    } else if (RequestNo == SET_LINE_CODING) {
        if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
            CopyRoutine = Virtual_Com_Port_SetLineCoding;
        }
        Request = SET_LINE_CODING;
    }

    if (CopyRoutine == NULL) {
        return USB_UNSUPPORT;
    }

    pInformation->Ctrl_Info.CopyData = CopyRoutine;
    pInformation->Ctrl_Info.Usb_wOffset = 0;
    (*CopyRoutine)(0);
    return USB_SUCCESS;
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_NoData_Setup.
 * Description    : handle the no data class specific requests.
 * Input          : Request Nb.
 * Output         : None.
 * Return         : USB_UNSUPPORT or USB_SUCCESS.
 *******************************************************************************/
RESULT Virtual_Com_Port_NoData_Setup(uint8_t RequestNo)
{

    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
        if (RequestNo == SET_COMM_FEATURE) {
            return USB_SUCCESS;
        } else if (RequestNo == SET_CONTROL_LINE_STATE) {
            return USB_SUCCESS;
        }
    }

    return USB_UNSUPPORT;
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_GetDeviceDescriptor.
 * Description    : Gets the device descriptor.
 * Input          : Length.
 * Output         : None.
 * Return         : The address of the device descriptor.
 *******************************************************************************/
uint8_t *Virtual_Com_Port_GetDeviceDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData(Length, &Device_Descriptor);
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_GetConfigDescriptor.
 * Description    : get the configuration descriptor.
 * Input          : Length.
 * Output         : None.
 * Return         : The address of the configuration descriptor.
 *******************************************************************************/
uint8_t *Virtual_Com_Port_GetConfigDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData(Length, &Config_Descriptor);
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_GetStringDescriptor
 * Description    : Gets the string descriptors according to the needed index
 * Input          : Length.
 * Output         : None.
 * Return         : The address of the string descriptors.
 *******************************************************************************/
uint8_t *Virtual_Com_Port_GetStringDescriptor(uint16_t Length)
{
    uint8_t wValue0 = pInformation->USBwValue0;
    if (wValue0 > 4) {
        return NULL;
    } else {
        return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
    }
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_Get_Interface_Setting.
 * Description    : test the interface and the alternate setting according to the
 *                  supported one.
 * Input1         : uint8_t: Interface : interface number.
 * Input2         : uint8_t: AlternateSetting : Alternate Setting number.
 * Output         : None.
 * Return         : The address of the string descriptors.
 *******************************************************************************/
RESULT Virtual_Com_Port_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting)
{
    if (AlternateSetting > 0) {
        return USB_UNSUPPORT;
    } else if (Interface > 1) {
        return USB_UNSUPPORT;
    }
    return USB_SUCCESS;
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_GetLineCoding.
 * Description    : send the linecoding structure to the PC host.
 * Input          : Length.
 * Output         : None.
 * Return         : Linecoding structure base address.
 *******************************************************************************/
uint8_t *Virtual_Com_Port_GetLineCoding(uint16_t Length)
{
    if (Length == 0) {
        pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
        return NULL;
    }
    return (uint8_t *)&linecoding;
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_SetLineCoding.
 * Description    : Set the linecoding structure fields.
 * Input          : Length.
 * Output         : None.
 * Return         : Linecoding structure base address.
 *******************************************************************************/
uint8_t *Virtual_Com_Port_SetLineCoding(uint16_t Length)
{
    if (Length == 0) {
        pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
        return NULL;
    }
    return (uint8_t *)&linecoding;
}

/*******************************************************************************
 * Function Name  : Virtual_Com_Port_GetBaudRate.
 * Description    : Get the current baudrate
 * Input          : None.
 * Output         : None.
 * Return         : baudrate in bps
 *******************************************************************************/
uint32_t Virtual_Com_Port_GetBaudRate(void)
{
    return linecoding.bitrate;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

