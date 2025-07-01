#ifndef RC522_H
#define RC522_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define CommandReg          0x01
#define ComIrqReg           0x04
#define ErrorReg            0x06
#define Status1Reg          0x07
#define Status2Reg          0x08
#define FIFODataReg         0x09
#define FIFOLevelReg        0x0A
#define ControlReg          0x0C
#define BitFramingReg       0x0D
#define CRCResultReg1       0x21
#define CRCResultReg2       0x22
#define TModeReg            0x2A
#define TPrescalerReg       0x2B
#define TReloadRegH         0x2C
#define TReloadRegL         0x2D
#define TxControlReg        0x14
#define MfTxReg             0x1C
#define MfRxReg             0x1D
#define TxASKReg            0x15

#define SoftReset           0x0F
#define Transceive          0x0C
#define Idle                0x00
#define RxEnd               0x30
#define REQA                0x26
#define CalcCRC             0x03
#define AuthCmdA            0x60
#define AuthCmdB            0x61

#define WriteCard           0xA0
#define ReadCard            0x30
#define HALT                0x50

#define MAX_BLOCK_ADDR      63
#define MIFARE_BLOCK_SIZE   16

typedef struct RC522{
    SPI_HandleTypeDef * _SPI;
    GPIO_TypeDef * _CSGPIOx;
    uint16_t _CSPIN;
    bool initialized;
}RC522_InitTypeDef;

typedef enum{
    STATUS_ERROR=0,
    STATUS_OK
}RC522_STATUS_TypeDef;

RC522_STATUS_TypeDef RC522_Init(RC522_InitTypeDef * RC, SPI_HandleTypeDef * _SPI, GPIO_TypeDef * _CSGPIOx, uint16_t _CSPIN);
RC522_STATUS_TypeDef RC522_DeInit(RC522_InitTypeDef * RC);
void RC522_Write_Reg(RC522_InitTypeDef * RC, uint8_t addr, uint8_t data);
uint8_t RC522_Read_Reg(RC522_InitTypeDef * RC, uint8_t addr);
RC522_STATUS_TypeDef RC522_Transceive(RC522_InitTypeDef * RC, uint8_t * sendData, uint8_t sendLen, uint8_t * receiveData, uint16_t * receiveLen);
RC522_STATUS_TypeDef RC522_ReqA(RC522_InitTypeDef * RC, uint8_t *atqa);
RC522_STATUS_TypeDef RC522_AntiCol(RC522_InitTypeDef * RC, uint8_t * uid);
void RC522_CRC(RC522_InitTypeDef * RC, uint8_t * data, uint8_t dataLen, uint8_t * msb, uint8_t * lsb);
void RC522_BCC(uint8_t * uid, uint8_t uidLen, uint8_t * bcc);
RC522_STATUS_TypeDef RC522_SelectCard(RC522_InitTypeDef * RC, uint8_t * uid, uint8_t * SAK);
RC522_STATUS_TypeDef RC522_Auth(RC522_InitTypeDef * RC, uint8_t * uid, uint8_t * key, uint8_t keyType, uint8_t blockAddr);
RC522_STATUS_TypeDef RC522_Read_Card(RC522_InitTypeDef * RC, uint8_t *uid, uint8_t *key, uint8_t keyType, uint8_t blockAddr, uint8_t *data_out);
RC522_STATUS_TypeDef RC522_Write_Card(RC522_InitTypeDef * RC, uint8_t *uid, uint8_t *key, uint8_t keyType, uint8_t blockAddr, uint8_t * data_in);
RC522_STATUS_TypeDef RC522_CheckForCard(RC522_InitTypeDef * RC, uint8_t * uid);
RC522_STATUS_TypeDef RC522_ReadCardBlock(RC522_InitTypeDef * RC, uint8_t * key, uint8_t keyType, uint8_t blockAddr, uint8_t * data_out, uint8_t * uid_out);
RC522_STATUS_TypeDef RC522_WriteCardBlock(RC522_InitTypeDef * RC, uint8_t * key, uint8_t keyType, uint8_t blockAddr, uint8_t * data_in, uint8_t * uid_out);
void RC522_HALT(RC522_InitTypeDef * RC);

#endif
