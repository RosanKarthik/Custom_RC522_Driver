/*
* RC522.c
*
*  Created on: Jun 30, 2025
*      Author: Rosan Karthik
*/

#include "RC522.h"

RC522_STATUS_TypeDef RC522_Init(RC522_InitTypeDef * RC,SPI_HandleTypeDef * _SPI,GPIO_TypeDef * _CSGPIOx, uint16_t _CSPIN){
    if (!RC || !_SPI || !_CSGPIOx) {
        return STATUS_ERROR;
    }
    RC->_SPI=_SPI;
    RC->_CSGPIOx=_CSGPIOx;
    RC->_CSPIN=_CSPIN;
    RC->initialized = false;
    RC522_Write_Reg(RC,CommandReg,SoftReset);
    HAL_Delay(50); // Wait for RC522 to complete reset
    RC522_Write_Reg(RC,TModeReg,0x8D);//Set timer to AUTO,NOTGATED,AUTORELOAD(1 11 1 1101)
    RC522_Write_Reg(RC,TPrescalerReg,0x3E);//Set timer prescaler to desired value
    RC522_Write_Reg(RC,TReloadRegL, 30);//timer count low
    RC522_Write_Reg(RC,TReloadRegH, 0);//timer count high
    RC522_Write_Reg(RC,TxControlReg,0x83);//Turn Antenna ON
    RC522_Write_Reg(RC,TxASKReg, 0x40);//set 100% ASK
    RC->initialized = true;
    return STATUS_OK;
}

uint8_t RC522_Read_Reg(RC522_InitTypeDef * RC,uint8_t addr){
uint8_t data;
//PULL CS PIN TO LOW WHEN WRITING
HAL_GPIO_WritePin(RC->_CSGPIOx,RC->_CSPIN,GPIO_PIN_RESET);

//The Frame Format Is "MSB,6bitRegAddr,LSB"
//The LSB must Always be RESET
//To achieve this We Left Shit It To Move it in Place And RESET LSB
//To READ, we have to SET the MSB
//We OR with 1000 0000 (80) to Set the MSB
uint8_t read_addr_frmt=((addr<<1)| 0x80);

//Transmitting RegAdd to Read in
HAL_SPI_Transmit(RC->_SPI,&read_addr_frmt,1,500);

//Receiving Data
HAL_SPI_Receive(RC->_SPI,&data,1,500);

//Pulling back the CS to stop end transfer
HAL_GPIO_WritePin(RC->_CSGPIOx,RC->_CSPIN,GPIO_PIN_SET);

return data;
}

RC522_STATUS_TypeDef RC522_DeInit(RC522_InitTypeDef * RC) {
    // Input validation
    if (!RC) {
        return STATUS_ERROR;
    }
    RC522_HALT(RC);
    RC522_Write_Reg(RC, TxControlReg, 0x00); // Turn antenna OFF
    RC522_Write_Reg(RC, TModeReg, 0x00); // Disable timer
    RC522_Write_Reg(RC, ComIrqReg, 0x7F); // Clear all interrupt flags
    RC522_Write_Reg(RC, FIFOLevelReg, 0x80); // Flush FIFO
    RC522_Write_Reg(RC, CommandReg, Idle);
    RC522_Write_Reg(RC, CommandReg, SoftReset);
    HAL_Delay(50);
    RC->_SPI = NULL;
    RC->_CSGPIOx = NULL;
    RC->_CSPIN = 0;
    
    return STATUS_OK;
}

void RC522_Write_Reg(RC522_InitTypeDef * RC,uint8_t addr,uint8_t data){
//PULL CS PIN TO LOW WHEN WRITING
HAL_GPIO_WritePin(RC->_CSGPIOx,RC->_CSPIN,GPIO_PIN_RESET);

//The Frame Format Is "MSB,6bitRegAddr,LSB"
//The LSB must Always be RESET
//To achieve this We Left Shit It To Move it in Place And RESET LSB
//To Write, we have to RESET the MSB
//We AND with 0111 1110 (0x7E) to clear the MSB and LSB
uint8_t wrt_addr_frmt=((addr<<1)& 0x7E);

//Transmitting RegAdd to Write in
HAL_SPI_Transmit(RC->_SPI,&wrt_addr_frmt,1,500);

//Transmitting Data
HAL_SPI_Transmit(RC->_SPI,&data,1,500);

//Pulling back the CS to stop end transfer
HAL_GPIO_WritePin(RC->_CSGPIOx,RC->_CSPIN,GPIO_PIN_SET);
}

RC522_STATUS_TypeDef RC522_Transceive(
RC522_InitTypeDef * RC,
// uint8_t Cmd,
uint8_t * sendData,
uint8_t sendLen,
uint8_t * receiveData,
uint16_t * receiveLen
) {
    RC522_STATUS_TypeDef status=STATUS_ERROR;
    uint8_t n;
    uint8_t i;
    uint8_t irq_val;
    uint8_t num_lastbits;

    RC522_Write_Reg(RC,ComIrqReg,0x71);//Clear all intflags by setting MSB to 0 and other bits to 1
    RC522_Write_Reg(RC,FIFOLevelReg,0x80);//Flush all FIFO Data Buffer by SET-ting FIFOLevelReg MSB
    RC522_Write_Reg(RC,CommandReg,Idle);//Set command to switch the module to be idle, cancel any and all current operations

    //Write sendData to FIFO Buffer
    for(i=0;i<sendLen;i++){
        RC522_Write_Reg(RC,FIFODataReg,sendData[i]);
    }

    //Send Command to start module to transceive
    RC522_Write_Reg(RC,CommandReg,Transceive);
    //SET BitFramingReg MSB to start transmission
    //First checks the current status of the register and then only SETS the MSB without modifying the other flags
    RC522_Write_Reg(RC, BitFramingReg, RC522_Read_Reg(RC, BitFramingReg) | 0x80);

    //wait for reply
    uint16_t timeout=2000;
    // do{
    //     n=RC522_Read_Reg(RC,ComIrqReg);
    //     timeout--;
    // }
    // while(
    //     timeout!=0 && //Until it timesout
    //     !(n&0x01) && //Until it is idle
    //     !(n&RxEnd) //until the end of receive flag is set
    // );

    while(1) {
        irq_val = RC522_Read_Reg(RC, ComIrqReg);
        if (irq_val & 0x33) { // RxIRq, IdleIRq, ErrIRq, or TimerIRq
            break;
        }
        if (--timeout == 0) {
            return STATUS_ERROR; // Software timeout
        }
    }

    RC522_Write_Reg(RC, BitFramingReg, RC522_Read_Reg(RC, BitFramingReg) & ~0x80); // Stop transmission
    
    if (irq_val & 0x02) { // ErrIRq bit
        return STATUS_ERROR;
    }

    //check the last 3 bits of the control register to get the remaining last bits of the last byte
    num_lastbits =(RC522_Read_Reg(RC,ControlReg) & 0x07);
    //find how many bytes are in the FIFO buffer
    n=RC522_Read_Reg(RC,FIFOLevelReg);

    //store the total number of bits to get the recieved amt of bits
    if (receiveLen) {
        if (n == 0) {
            *receiveLen = 0;
        } else if (num_lastbits) {
            *receiveLen = ((n - 1) * 8) + num_lastbits;
        } else {
            *receiveLen = n * 8;
        }
    }

    if (receiveData) {
        for (i = 0; i < n; i++) {
            receiveData[i] = RC522_Read_Reg(RC, FIFODataReg);
        }
    }
    status=STATUS_OK;
    return status;
}

RC522_STATUS_TypeDef RC522_ReqA(RC522_InitTypeDef * RC,uint8_t *atqa){
        RC522_STATUS_TypeDef status = STATUS_ERROR;
        //Detecct a card if present by sending REQA
        //If present it will return ATQA, which is of 2 bytes so we pass a array(buffer) to receive it.
        uint8_t reqa_cmd=REQA;
        uint16_t atqaLen=0;

        // the REQ is only 7 bits so we,transmit only 7 bits by sending only the last 7 bits
        RC522_Write_Reg(RC,BitFramingReg,0x07);
        
        //Send ReqA and check if we recieved ATQA (2bytes)
        if(RC522_Transceive(RC,&reqa_cmd,1,atqa,&atqaLen)== STATUS_OK){
            if(atqaLen==16){ // 2 bytes * 8 bits = 16 bits
            status = STATUS_OK;
            }
        }
        //Set BitFrammingReg to normal
        RC522_Write_Reg(RC,BitFramingReg,0x00);
        return status;
}

RC522_STATUS_TypeDef RC522_AntiCol(RC522_InitTypeDef * RC,uint8_t * uid){        
        RC522_STATUS_TypeDef status = STATUS_ERROR;
        uint8_t antiCol_cmd[] = {0x93,0x20};//anti coll command, cascade1
        uint16_t uidLen=0;

        RC522_Write_Reg(RC,BitFramingReg,0x00);

        //Send AntiCol Cmd and recieve its UID (5bytes)
        if(RC522_Transceive(RC,antiCol_cmd,2,uid,&uidLen)== STATUS_OK){
            if(uidLen==40){ // 5 bytes * 8 bits = 40 bits
                status = STATUS_OK;
            }
        }

        return status;
}

void RC522_CRC(RC522_InitTypeDef * RC,uint8_t * data,uint8_t dataLen,uint8_t * msb, uint8_t * lsb){
    //send command to calculate crc
    uint8_t MSB=0;
    uint8_t LSB=0;

    RC522_Write_Reg(RC,CommandReg,Idle);//Set Idle
    RC522_Write_Reg(RC,FIFOLevelReg,0x80);//Flush

    //write the data to the buffer
    for(uint8_t i=0;i<dataLen;i++){
        RC522_Write_Reg(RC,FIFODataReg,data[i]);
    }
    
    //command to calcCRC
    RC522_Write_Reg(RC,CommandReg,CalcCRC);

    uint16_t timeout = 5000;
    while(--timeout && !(RC522_Read_Reg(RC,Status1Reg) & 0x20));//wait till it finishes by checking crc ready bit

    //fetch crc from CRCReg1(MSB) and CRCReg2(LSB)
    MSB = RC522_Read_Reg(RC,CRCResultReg1);
    LSB = RC522_Read_Reg(RC,CRCResultReg2);

    //combine into crc with bitmanuplation 
    *msb=MSB;
    *lsb=LSB;
}

void RC522_BCC(uint8_t * uid,uint8_t uidLen,uint8_t * bcc){
    uint8_t result=0;
    for(uint8_t i=0;i<uidLen;i++){
        result^=uid[i];
    }
    *bcc=result;
}

RC522_STATUS_TypeDef RC522_SelectCard(RC522_InitTypeDef * RC,uint8_t * uid,uint8_t * SAK){
    uint8_t crc_lsb,crc_msb,bcc;
    uint8_t select_cmd[9];
    uint16_t SAKLen=0;
    RC522_STATUS_TypeDef status=STATUS_ERROR;

    select_cmd[0] = 0x93; //Cascade lvl 1
    select_cmd[1] = 0x70; //select command
    select_cmd[2] = uid[0];
    select_cmd[3] = uid[1];
    select_cmd[4] = uid[2];
    select_cmd[5] = uid[3];

    RC522_BCC(uid, 4, &bcc);
    select_cmd[6] = bcc;
    
    RC522_CRC(RC, select_cmd, 7, &crc_msb,&crc_lsb); //crc calculated for first 7 frames of select_cmd
    
    select_cmd[7] = crc_msb; // MSB of crc
    select_cmd[8] = crc_lsb; // LSB of crc
    
    if(RC522_Transceive(RC,select_cmd,9,SAK,&SAKLen)==STATUS_OK){
        if(SAKLen==8 && SAK[0]==0x08){
            status=STATUS_OK;
        }
    }
    return status;
}

RC522_STATUS_TypeDef RC522_Auth(RC522_InitTypeDef * RC,uint8_t * uid, uint8_t * key,uint8_t keyType, uint8_t blockAddr)
{
    RC522_STATUS_TypeDef status=STATUS_ERROR;
    uint8_t auth_cmd[12];
    uint8_t rxData;
    uint16_t rxLen;

    if (!RC || !uid || !key) 
        return STATUS_ERROR;
    
    if (blockAddr > MAX_BLOCK_ADDR)
        return STATUS_ERROR;
        
    if (keyType != 'A' && keyType != 'B')
        return STATUS_ERROR;

    if (keyType == 'A') {
        auth_cmd[0] = AuthCmdA;
    } else if (keyType == 'B') {
        auth_cmd[0] = AuthCmdB;
    } else {
        return STATUS_ERROR; // Invalid key type
    }
    auth_cmd[1]=blockAddr;
    auth_cmd[2]=key[0];
    auth_cmd[3]=key[1];
    auth_cmd[4]=key[2];
    auth_cmd[5]=key[3];
    auth_cmd[6]=key[4];
    auth_cmd[7]=key[5];
    auth_cmd[8]=uid[0];
    auth_cmd[9]=uid[1];
    auth_cmd[10]=uid[2];
    auth_cmd[11]=uid[3];

    if (RC522_Transceive(RC,auth_cmd,12,&rxData,&rxLen) == STATUS_ERROR)
        return status;
    
    uint16_t timeout=5000;
    while(--timeout && !(RC522_Read_Reg(RC,Status2Reg) & 0x08));// wait till timeout or succesfull auth

    if(timeout){
        status=STATUS_OK;
    }
    return status;
}

RC522_STATUS_TypeDef RC522_Read_Card(
    RC522_InitTypeDef * RC,
    uint8_t *uid,
    uint8_t *key,
    uint8_t keyType,
    uint8_t blockAddr,
    uint8_t *data_out // buffer to store 16 bytes read from card
) {    
    RC522_STATUS_TypeDef status=STATUS_ERROR;
    uint8_t read_cmd[4],crc_msb,crc_lsb;

    // Input validation
    if (!RC || !uid || !key || !data_out) 
        return STATUS_ERROR;
    
    if (blockAddr > MAX_BLOCK_ADDR)
        return STATUS_ERROR;
        
    if (keyType != 'A' && keyType != 'B')
        return STATUS_ERROR;

    read_cmd[0]=ReadCard;
    read_cmd[1]=blockAddr;
    RC522_CRC(RC,read_cmd,2,&crc_msb,&crc_lsb);
    read_cmd[2]=crc_msb;
    read_cmd[3]=crc_lsb;

    //buffer for 16 bytes
    uint8_t rxData[16]={0};
    uint16_t rxLen=0;

    if(RC522_Transceive(RC,read_cmd,4,rxData,&rxLen)==STATUS_ERROR)
        return status;
    if(rxLen==128){ //16bytes of data, 8*16=128
        // Ensure we don't overflow the output buffer
        uint8_t copyLen = (rxLen/8 > MIFARE_BLOCK_SIZE) ? MIFARE_BLOCK_SIZE : rxLen/8;
        for(uint8_t i=0; i<copyLen; i++){
            data_out[i]=rxData[i];
        }
        status=STATUS_OK;
    }
    return status;
}

RC522_STATUS_TypeDef RC522_Write_Card(
    RC522_InitTypeDef * RC,
    uint8_t *uid,
    uint8_t *key,
    uint8_t keyType,
    uint8_t blockAddr,
    uint8_t * data_in
) {
    RC522_STATUS_TypeDef status=STATUS_ERROR;
    uint8_t write_cmd[4],crc_msb,crc_lsb;

    // Input validation
    if (!RC || !uid || !key || !data_in) 
        return STATUS_ERROR;
    
    if (blockAddr > MAX_BLOCK_ADDR)
        return STATUS_ERROR;
        
    if (keyType != 'A' && keyType != 'B')
        return STATUS_ERROR;
    
    //authenticate
    if(RC522_Auth(RC,uid,key,keyType,blockAddr)==STATUS_ERROR) 
        return status;

    //construct the write frame
    write_cmd[0]=WriteCard;
    write_cmd[1]=blockAddr;
    //create crc
    RC522_CRC(RC,write_cmd,2,&crc_msb,&crc_lsb);

    write_cmd[2]=crc_msb;
    write_cmd[3]=crc_lsb;

    //buffer for 4bit ACK
    uint8_t ACK=0;//must be equal to 0x0A
    uint16_t rxLen=0;

    //send the command frame and check for ACK
    if(RC522_Transceive(RC,write_cmd,4,&ACK,&rxLen)==STATUS_ERROR)
        return status;
    if(rxLen!=4 || (ACK & 0x0F)!= 0x0A)//check if the value matches 0x04 after converting it into lower half 4 bits
        return status;
    
    //create transmitting/sending frame
    uint8_t tx_cmd[18];
    for (uint8_t i = 0; i < MIFARE_BLOCK_SIZE; i++) {
        tx_cmd[i] = data_in[i];
    }

    //create crc for the data in the frame
    RC522_CRC(RC,tx_cmd,16,&crc_msb,&crc_lsb);

    tx_cmd[16] = crc_msb;
    tx_cmd[17] = crc_lsb;

    ACK = 0;
    rxLen = 0;
    //send the data frame check for ACK
    if(RC522_Transceive(RC,tx_cmd,18,&ACK,&rxLen)==STATUS_ERROR)
        return status;
    if(rxLen!=4 || (ACK & 0x0F)!= 0x0A)//check if the value matches 0x04 after converting it into lower half 4 bits
        return status;

    status=STATUS_OK;
    return status;
}

RC522_STATUS_TypeDef RC522_CheckForCard(RC522_InitTypeDef * RC,uint8_t * uid){
    RC522_STATUS_TypeDef status = STATUS_ERROR;
    uint8_t atqa[2]={0};
    if(RC522_ReqA(RC,atqa)==STATUS_ERROR){
        return status;
    }
    if(RC522_AntiCol(RC,uid)==STATUS_ERROR){
        return status;
    }
    status=STATUS_OK;
    return status;
}

RC522_STATUS_TypeDef RC522_ReadCardBlock(
    RC522_InitTypeDef * RC,
    uint8_t * key, 
    uint8_t keyType,
    uint8_t blockAddr, 
    uint8_t * data_out,
    uint8_t * uid_out
) {
    uint8_t uid[5];
    RC522_STATUS_TypeDef status;
    
    // Step 1: Detect card
    status = RC522_CheckForCard(RC, uid);
    if (status != STATUS_OK) {
        return STATUS_ERROR;
    }
    
    // Step 2: Authenticate
    status = RC522_Auth(RC, uid, key, keyType, blockAddr);
    if (status != STATUS_OK) {
        return STATUS_ERROR;
    }
    
    // Step 3: Read data
    status = RC522_Read_Card(RC, uid, key, keyType, blockAddr, data_out);
    
    // Copy UID if requested
    if (uid_out) {
        for (uint8_t i = 0; i < 5; i++) {
            uid_out[i] = uid[i];
        }
    }
    
    return status;
}

RC522_STATUS_TypeDef RC522_WriteCardBlock(
    RC522_InitTypeDef * RC, 
    uint8_t * key, 
    uint8_t keyType,
    uint8_t blockAddr, 
    uint8_t * data_in,
    uint8_t * uid_out
) {
    uint8_t uid[5];
    RC522_STATUS_TypeDef status;
    
    status = RC522_CheckForCard(RC, uid);
    if (status != STATUS_OK) {
        return STATUS_ERROR;
    }
    
    status = RC522_Write_Card(RC, uid, key, keyType, blockAddr, data_in);
    
    if (uid_out) {
        for (uint8_t i = 0; i < 5; i++) {
            uid_out[i] = uid[i];
        }
    }
    
    return status;
}

void RC522_HALT(RC522_InitTypeDef * RC){
    uint8_t halt_cmd[4],crc_msb,crc_lsb;
    halt_cmd[0]=HALT;
    halt_cmd[1]=0x00;

    RC522_CRC(RC,halt_cmd,2,&crc_msb,&crc_lsb);

    halt_cmd[2]=crc_msb;
    halt_cmd[3]=crc_lsb;
    
    RC522_Transceive(RC,halt_cmd,4,NULL,NULL);
}