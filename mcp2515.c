/*
 * mcp2515.c
 *
 *  Created on: Dec 13, 2020
 *      Author: matt
 */

#include "mcp2515.h"
#include "mcp2515_consts.h"
#include <stm32f4xx_hal.h>
#include <string.h> //Todo: this is yucky just for the memset function

/* Modify below items for your SPI configurations */
extern SPI_HandleTypeDef        hspi4;
#define SPI_CAN                 &hspi4
#define SPI_TIMEOUT             10
//#define MCP2515_CS_HIGH()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET)
//#define MCP2515_CS_LOW()    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET)

void startSPI() {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
}

void endSPI() {
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t SPI_transfer(uint8_t txByte){
	uint8_t rxByte;
	HAL_SPI_TransmitReceive(SPI_CAN, &txByte, &rxByte, 1, SPI_TIMEOUT);
	return rxByte;
}

void setRegister(uint8_t reg, uint8_t value)
{
    startSPI();
    SPI_transfer(INSTRUCTION_WRITE);
    SPI_transfer(reg);
    SPI_transfer(value);
    endSPI();
}

void setRegisters(uint8_t reg, uint8_t values[], uint8_t n)
{
    startSPI();
    SPI_transfer(INSTRUCTION_WRITE);
    SPI_transfer(reg);
    for (uint8_t i=0; i<n; i++) {
        SPI_transfer(values[i]);
    }

  //  HAL_SPI_Transmit(SPI_CAN, values, n, SPI_TIMEOUT);
    endSPI();
}

void loadTx(uint8_t reg, uint8_t values[], uint8_t n){
    startSPI();
    //SPI_transfer(INSTRUCTION_WRITE);
    SPI_transfer(reg);
    for (uint8_t i=0; i<n; i++) {
        SPI_transfer(values[i]);
    }

  //  HAL_SPI_Transmit(SPI_CAN, values, n, SPI_TIMEOUT);
    endSPI();
}

void modifyRegister(uint8_t reg, uint8_t mask, uint8_t data)
{
    startSPI();
    SPI_transfer(INSTRUCTION_BITMOD);
    SPI_transfer(reg);
    SPI_transfer(mask);
    SPI_transfer(data);
    endSPI();
}




uint8_t readRegister(REGISTER reg)
{
    startSPI();
    SPI_transfer(INSTRUCTION_READ);
    SPI_transfer(reg);
    uint8_t ret = SPI_transfer(0x00);
    endSPI();

    return ret;
}

void readRegisters(REGISTER reg, uint8_t values[], uint8_t n)
{
    startSPI();
    SPI_transfer(INSTRUCTION_READ);
    SPI_transfer(reg);
    // mcp2515 has auto-increment of address-pointer
    for (uint8_t i=0; i<n; i++) {
        values[i] = SPI_transfer(0x00);
    	//HAL_SPI_Receive(&hspi4, values, n, SPI_TIMEOUT);  //Todo, check if the 0x00 from above is needed
    }
    	endSPI();
}

void readRx(REGISTER reg, uint8_t values[], uint8_t n){
    startSPI();
    SPI_transfer(reg);
    // mcp2515 has auto-increment of address-pointer
    for (uint8_t i=0; i<n; i++) {
        values[i] = SPI_transfer(0x00);
    	//HAL_SPI_Receive(&hspi4, values, n, SPI_TIMEOUT);  //Todo, check if the 0x00 from above is needed
    }
    endSPI();
}

CAN_Error setMode(CANCTRL_REQOP_MODE mode)
{

    unsigned long endTime = HAL_GetTick() + 10;
    uint8_t modeMatch = 0;
    while (HAL_GetTick() < endTime) {
    	modifyRegister(MCP_CANCTRL, CANCTRL_REQOP, mode);
        uint8_t newmode = readRegister(MCP_CANSTAT);
        newmode &= CANSTAT_OPMOD;

        modeMatch = newmode == mode;

        if (modeMatch) {
            break;
        }
    }

    return modeMatch ? ERROR_OK : ERROR_FAIL;

}

CAN_Error setConfigMode()
{
    return setMode(CANCTRL_REQOP_CONFIG);
}

void prepareId(uint8_t *buffer, uint8_t ext, uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if(ext) {
        buffer[MCP_EID0] = (uint8_t) (canid & 0xFF);
        buffer[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[MCP_SIDL] = (uint8_t) (canid & 0x03);
        buffer[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        buffer[MCP_SIDL] |= TXB_EXIDE_MASK;
        buffer[MCP_SIDH] = (uint8_t) (canid >> 5);
    } else {
        buffer[MCP_SIDH] = (uint8_t) (canid >> 3);
        buffer[MCP_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[MCP_EID0] = 0;
        buffer[MCP_EID8] = 0;
    }
}

uint8_t getStatus(void)
{
    startSPI();
    SPI_transfer(INSTRUCTION_READ_STATUS);
    uint8_t i = SPI_transfer(0x00);
    endSPI();
    return i;
}

CAN_Error MCP_setFilterMask(MASK mask, uint8_t ext, uint32_t ulData)
{
	CAN_Error res = setConfigMode();
    if (res != ERROR_OK) {
        return res;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return ERROR_FAIL;
    }

    setRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}

CAN_Error MCP_setFilter(RXF num, uint8_t ext, uint32_t ulData)
{
    CAN_Error res = setConfigMode();
    if (res != ERROR_OK) {
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return ERROR_FAIL;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);
    setRegisters(reg, tbufdata, 4);

    return ERROR_OK;
}

CAN_Error MCP2515_reset(void)
{
    startSPI();
    SPI_transfer(INSTRUCTION_RESET);
    endSPI();


    HAL_Delay(10);

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    setRegisters(MCP_TXB0CTRL, zeros, 14);
    setRegisters(MCP_TXB1CTRL, zeros, 14);
    setRegisters(MCP_TXB2CTRL, zeros, 14);

    setRegister(MCP_RXB0CTRL, 0);
    setRegister(MCP_RXB1CTRL, 0);

    setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (uint8_t i=0; i<6; i++) {
        uint8_t ext = (i == 1);
        CAN_Error result = MCP_setFilter(filters[i], ext, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i=0; i<2; i++) {
    	CAN_Error result = MCP_setFilterMask(masks[i], 1, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    return ERROR_OK;
}

CAN_Error MCP_reset(void)
{
    startSPI();
    SPI_transfer(INSTRUCTION_RESET);
    endSPI();

    HAL_Delay(10);

    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    setRegisters(MCP_TXB0CTRL, zeros, 14);
    setRegisters(MCP_TXB1CTRL, zeros, 14);
    setRegisters(MCP_TXB2CTRL, zeros, 14);

    setRegister(MCP_RXB0CTRL, 0);
    setRegister(MCP_RXB1CTRL, 0);

    setRegister(MCP_CANINTE, CANINTF_RX0IF | CANINTF_RX1IF | CANINTF_ERRIF | CANINTF_MERRF);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i=0; i<6; i++) {
        uint8_t ext = (i == 1);
        CAN_Error result = MCP_setFilter(filters[i], ext, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i=0; i<2; i++) {
    	CAN_Error result = MCP_setFilterMask(masks[i], 1, 0);
        if (result != ERROR_OK) {
            return result;
        }
    }

    return ERROR_OK;
}

CAN_Error MCP_setListenOnlyMode()
{
    return setMode(CANCTRL_REQOP_LISTENONLY);
}

CAN_Error MCP_setSleepMode()
{
    return setMode(CANCTRL_REQOP_SLEEP);
}

CAN_Error MCP_setLoopbackMode()
{
    return setMode(CANCTRL_REQOP_LOOPBACK);
}

CAN_Error MCP_setNormalMode()
{
    return setMode(CANCTRL_REQOP_NORMAL);
}

CAN_Error MCP_setBitrateClock(CAN_SPEED canSpeed, CAN_CLOCK canClock)
{
	CAN_Error error = setConfigMode();
    if (error != ERROR_OK) {
        return error;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
        case (MCP_8MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5KBPS
            cfg1 = MCP_8MHz_5kBPS_CFG1;
            cfg2 = MCP_8MHz_5kBPS_CFG2;
            cfg3 = MCP_8MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10KBPS
            cfg1 = MCP_8MHz_10kBPS_CFG1;
            cfg2 = MCP_8MHz_10kBPS_CFG2;
            cfg3 = MCP_8MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20KBPS
            cfg1 = MCP_8MHz_20kBPS_CFG1;
            cfg2 = MCP_8MHz_20kBPS_CFG2;
            cfg3 = MCP_8MHz_20kBPS_CFG3;
            break;

            case (CAN_31K25BPS):                                            //  31.25KBPS
            cfg1 = MCP_8MHz_31k25BPS_CFG1;
            cfg2 = MCP_8MHz_31k25BPS_CFG2;
            cfg3 = MCP_8MHz_31k25BPS_CFG3;
            break;

            case (CAN_33KBPS):                                              //  33.333KBPS
            cfg1 = MCP_8MHz_33k3BPS_CFG1;
            cfg2 = MCP_8MHz_33k3BPS_CFG2;
            cfg3 = MCP_8MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_8MHz_40kBPS_CFG1;
            cfg2 = MCP_8MHz_40kBPS_CFG2;
            cfg3 = MCP_8MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_8MHz_50kBPS_CFG1;
            cfg2 = MCP_8MHz_50kBPS_CFG2;
            cfg3 = MCP_8MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_8MHz_80kBPS_CFG1;
            cfg2 = MCP_8MHz_80kBPS_CFG2;
            cfg3 = MCP_8MHz_80kBPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_8MHz_100kBPS_CFG1;
            cfg2 = MCP_8MHz_100kBPS_CFG2;
            cfg3 = MCP_8MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_8MHz_125kBPS_CFG1;
            cfg2 = MCP_8MHz_125kBPS_CFG2;
            cfg3 = MCP_8MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_8MHz_200kBPS_CFG1;
            cfg2 = MCP_8MHz_200kBPS_CFG2;
            cfg3 = MCP_8MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_8MHz_250kBPS_CFG1;
            cfg2 = MCP_8MHz_250kBPS_CFG2;
            cfg3 = MCP_8MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_8MHz_500kBPS_CFG1;
            cfg2 = MCP_8MHz_500kBPS_CFG2;
            cfg3 = MCP_8MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_8MHz_1000kBPS_CFG1;
            cfg2 = MCP_8MHz_1000kBPS_CFG2;
            cfg3 = MCP_8MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_16MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5Kbps
            cfg1 = MCP_16MHz_5kBPS_CFG1;
            cfg2 = MCP_16MHz_5kBPS_CFG2;
            cfg3 = MCP_16MHz_5kBPS_CFG3;
            break;

            case (CAN_10KBPS):                                              //  10Kbps
            cfg1 = MCP_16MHz_10kBPS_CFG1;
            cfg2 = MCP_16MHz_10kBPS_CFG2;
            cfg3 = MCP_16MHz_10kBPS_CFG3;
            break;

            case (CAN_20KBPS):                                              //  20Kbps
            cfg1 = MCP_16MHz_20kBPS_CFG1;
            cfg2 = MCP_16MHz_20kBPS_CFG2;
            cfg3 = MCP_16MHz_20kBPS_CFG3;
            break;

            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_16MHz_33k3BPS_CFG1;
            cfg2 = MCP_16MHz_33k3BPS_CFG2;
            cfg3 = MCP_16MHz_33k3BPS_CFG3;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_16MHz_40kBPS_CFG1;
            cfg2 = MCP_16MHz_40kBPS_CFG2;
            cfg3 = MCP_16MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_16MHz_50kBPS_CFG1;
            cfg2 = MCP_16MHz_50kBPS_CFG2;
            cfg3 = MCP_16MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_16MHz_80kBPS_CFG1;
            cfg2 = MCP_16MHz_80kBPS_CFG2;
            cfg3 = MCP_16MHz_80kBPS_CFG3;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_16MHz_83k3BPS_CFG1;
            cfg2 = MCP_16MHz_83k3BPS_CFG2;
            cfg3 = MCP_16MHz_83k3BPS_CFG3;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_16MHz_100kBPS_CFG1;
            cfg2 = MCP_16MHz_100kBPS_CFG2;
            cfg3 = MCP_16MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_16MHz_125kBPS_CFG1;
            cfg2 = MCP_16MHz_125kBPS_CFG2;
            cfg3 = MCP_16MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_16MHz_200kBPS_CFG1;
            cfg2 = MCP_16MHz_200kBPS_CFG2;
            cfg3 = MCP_16MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_16MHz_250kBPS_CFG1;
            cfg2 = MCP_16MHz_250kBPS_CFG2;
            cfg3 = MCP_16MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_16MHz_500kBPS_CFG1;
            cfg2 = MCP_16MHz_500kBPS_CFG2;
            cfg3 = MCP_16MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_16MHz_1000kBPS_CFG1;
            cfg2 = MCP_16MHz_1000kBPS_CFG2;
            cfg3 = MCP_16MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (MCP_20MHZ):
        switch (canSpeed)
        {
            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = MCP_20MHz_33k3BPS_CFG1;
            cfg2 = MCP_20MHz_33k3BPS_CFG2;
            cfg3 = MCP_20MHz_33k3BPS_CFG3;
	    break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = MCP_20MHz_40kBPS_CFG1;
            cfg2 = MCP_20MHz_40kBPS_CFG2;
            cfg3 = MCP_20MHz_40kBPS_CFG3;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = MCP_20MHz_50kBPS_CFG1;
            cfg2 = MCP_20MHz_50kBPS_CFG2;
            cfg3 = MCP_20MHz_50kBPS_CFG3;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = MCP_20MHz_80kBPS_CFG1;
            cfg2 = MCP_20MHz_80kBPS_CFG2;
            cfg3 = MCP_20MHz_80kBPS_CFG3;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = MCP_20MHz_83k3BPS_CFG1;
            cfg2 = MCP_20MHz_83k3BPS_CFG2;
            cfg3 = MCP_20MHz_83k3BPS_CFG3;
	    break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = MCP_20MHz_100kBPS_CFG1;
            cfg2 = MCP_20MHz_100kBPS_CFG2;
            cfg3 = MCP_20MHz_100kBPS_CFG3;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = MCP_20MHz_125kBPS_CFG1;
            cfg2 = MCP_20MHz_125kBPS_CFG2;
            cfg3 = MCP_20MHz_125kBPS_CFG3;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = MCP_20MHz_200kBPS_CFG1;
            cfg2 = MCP_20MHz_200kBPS_CFG2;
            cfg3 = MCP_20MHz_200kBPS_CFG3;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = MCP_20MHz_250kBPS_CFG1;
            cfg2 = MCP_20MHz_250kBPS_CFG2;
            cfg3 = MCP_20MHz_250kBPS_CFG3;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = MCP_20MHz_500kBPS_CFG1;
            cfg2 = MCP_20MHz_500kBPS_CFG2;
            cfg3 = MCP_20MHz_500kBPS_CFG3;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = MCP_20MHz_1000kBPS_CFG1;
            cfg2 = MCP_20MHz_1000kBPS_CFG2;
            cfg3 = MCP_20MHz_1000kBPS_CFG3;
            break;

            default:
            set = 0;
            break;
        }
        break;

        default:
        set = 0;
        break;
    }

    if (set) {
        setRegister(MCP_CNF1, cfg1);
        setRegister(MCP_CNF2, cfg2);
        setRegister(MCP_CNF3, cfg3);
        return ERROR_OK;
    }
    else {
        return ERROR_FAIL;
    }
}

CAN_Error MCP_setBitrate( CAN_SPEED canSpeed)
{
    return MCP_setBitrateClock(canSpeed, MCP_16MHZ);
}


void MCP_RequestToSend(uint8_t instruction)
{
    startSPI();
    SPI_transfer(instruction);
    endSPI();
}

CAN_Error MCP_sendMessageTo(TXBn txbn, can_frame *frame)
//TXBm is just 0,1,2 for txbox number
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }

    //Todo, fix these magic numbers, but not with something as awful as the og arduino library
    uint8_t load_addr = (2 * txbn) | 0x40;

    uint8_t rts_addr = (1 << txbn) | 0x80;

    uint8_t data[13];

    uint8_t ext = !!(frame->can_id & CAN_EFF_FLAG);
    uint8_t rtr = !!(frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    prepareId(data, ext, id);

    data[MCP_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    for(int i = 0; i < frame->can_dlc; i++){
    	data[MCP_DATA+i]=frame->data[i];
    }

   // memcpy(&data[MCP_DATA], frame->data, frame->can_dlc);

    loadTx(load_addr, data, 5 + frame->can_dlc);
    //setRegisters(load_addr, data, 5 + frame->can_dlc);

    //modifyRegister(txbuf->CTRL, TXB_TXREQ, TXB_TXREQ);
    //modifyRegister(rts_addr, TXB_TXREQ, TXB_TXREQ);
    MCP_RequestToSend(rts_addr);
    //setRegister(rts_addr, TXB_TXREQ);

    uint8_t ctrl = readRegister(rts_addr);
    if ((ctrl & (TXB_ABTF | TXB_MLOA | TXB_TXERR)) != 0) {
        return ERROR_FAILTX;
    }
    return ERROR_OK;
}

CAN_Error MCP_sendMessage(can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return ERROR_FAILTX;
    }


    for (uint8_t i=0; i<N_TXBUFFERS; i++) {
        uint8_t ctrlval = readRegister((i+3)<<4);
        if ( (ctrlval & TXB_TXREQ) == 0 ) {
            return MCP_sendMessageTo(i, frame);
        }
    }

    return ERROR_ALLTXBUSY;
}

CAN_Error MCP_readMessageFrom(RXBn rxbn, can_frame *frame)
{


    uint8_t readCommand = (rxbn << 2) | 0x90;

    rx_reg_t rxReg;

    readRx(readCommand, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));

    uint32_t id = (rxReg.rx_reg_array[MCP_SIDH]<<3) + (rxReg.rx_reg_array[MCP_SIDL]>>5);

    if ( (rxReg.rx_reg_array[MCP_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
        id = (id<<2) + (rxReg.rx_reg_array[MCP_SIDL] & 0x03);
        id = (id<<8) + rxReg.rx_reg_array[MCP_EID8];
        id = (id<<8) + rxReg.rx_reg_array[MCP_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (rxReg.rx_reg_array[MCP_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return ERROR_FAIL;
    }


    //0x60 or 0x70
    uint8_t ctrl = readRegister((rxbn + 6) << 4);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    frame->data[0] = rxReg.RXBnD0;
    frame->data[1] = rxReg.RXBnD1;
    frame->data[2] = rxReg.RXBnD2;
    frame->data[3] = rxReg.RXBnD3;
    frame->data[4] = rxReg.RXBnD4;
    frame->data[5] = rxReg.RXBnD5;
    frame->data[6] = rxReg.RXBnD6;
    frame->data[7] = rxReg.RXBnD7;


    //Clear the inbox interrupt, 0x1 or 0x2
    modifyRegister(MCP_CANINTF, rxbn + 1, 0);

    return ERROR_OK;
}

CAN_Error MCP_readMessage(can_frame *frame)
{
	CAN_Error rc;
    uint8_t stat = getStatus();

    if ( stat & STAT_RX0IF ) {
        rc = MCP_readMessageFrom(RXB0, frame);
    } else if ( stat & STAT_RX1IF ) {
        rc = MCP_readMessageFrom(RXB1, frame);
    } else {
        rc = ERROR_NOMSG;
    }

    return rc;
}

uint8_t MCP_checkReceive(void)
{
    uint8_t res = getStatus();
    if ( res & STAT_RXIF_MASK ) {
        return 1;
    } else {
        return 0;
    }
}

uint8_t MCP_getErrorFlags(void)
{
    return readRegister(MCP_EFLG);
}

uint8_t MCP_checkError(void)
{
    uint8_t eflg = MCP_getErrorFlags();

    if ( eflg & EFLG_ERRORMASK ) {
        return 1;
    } else {
        return 0;
    }
}


void MCP_clearRXnOVRFlags(void)
{
	modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
}

uint8_t MCP_getInterrupts(void)
{
    return readRegister(MCP_CANINTF);
}

void MCP_clearInterrupts(void)
{
    setRegister(MCP_CANINTF, 0);
}

uint8_t MCP_getInterruptMask(void)
{
    return readRegister(MCP_CANINTE);
}

void MCP_clearTXInterrupts(void)
{
    modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0);
}

void MCP_clearRXnOVR(void)
{
	uint8_t eflg = MCP_getErrorFlags();
	if (eflg != 0) {
		MCP_clearRXnOVRFlags();
		MCP_clearInterrupts();
		//modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
	}

}

void MCP_clearMERR()
{
	//modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
	//clearInterrupts();
	modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0);
}

void MCP_clearERRIF()
{
    //modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0);
    //clearInterrupts();
    modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0);
}

