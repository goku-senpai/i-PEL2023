/** @file spi.h
 *
 * Copyright (c) 2019 IACE
 */
#ifndef STM_SPI_H
#define STM_SPI_H

#include "stm/hal.h"
#include "stm/gpio.h"
#include "utils/RequestQueue.h"
/* *********************************************
 * When using this, use the following line in your interrupt header file (e.g. stm32f7xx_it.h)
 *
extern SPI_HandleTypeDef hHWSPI;
 *
 * *********************************************
 * and put the following interrupt handler in your interrupt implementation file (e.g. stm32f7xx_it.c)
 * (attention: configure the correct SPI peripheral according to your defined HW_SPI)
 *
void SPI3_IRQHandler(void) {
    HAL_SPI_IRQHandler(&hHWSPI);
}
 *
 */

extern "C" {
SPI_HandleTypeDef hHWSPI;
};

/**
 * Class abstracting an SPI device.
 * Implement your SPI device with this as a base class, then you can use the \ref HardwareSPI
 * to send/get data over the wire.
 */
class ChipSelect {
    DIO pin;
public:
    void selectChip(bool sel) {
        this->pin.set(!sel);
    }

    ChipSelect(uint32_t iCSPIN, GPIO_TypeDef *gpioCSPort) :
            pin(iCSPIN, gpioCSPort) {
        this->selectChip(false);
    }

    /**
     * callback. is called as soon as requested data arrived over wire.
     */
    virtual void callback(void *cbData) { }

};

/**
 * struct defining an SPI request.
 * use this when requesting data from the \ref HardwareSPI
 */
class SPIRequest {
public:
    ChipSelect *cs;     ///< pointer the \ref ChipSelect instance requesting data
    /// direction the data should travel
    enum eDir {
        MOSI,
        MISO,
        BOTH
    } dir;
    uint8_t *tData;     ///< pointer to data to be transmitted
    uint8_t *rData;     ///< pointer to receive buffer
    uint32_t dataLen;   ///< number of bytes to transfer
    void *cbData;     ///< data to use in callback

    /**
     * Standard constructor
     */
    SPIRequest() {};

    /**
     *
     * @param cs
     * @param dir
     * @param tData
     * @param rData
     * @param dataLen
     * @param cbData
     */
    SPIRequest(ChipSelect *cs, enum eDir dir, uint8_t *tData, uint8_t *rData,
            uint32_t dataLen, void *cbData) :
            cs(cs), dir(dir), tData(tData), rData(rData),
            dataLen(dataLen), cbData(cbData) {
        deepCopyData(tData);
    }

    ~SPIRequest() {
        if (tData) {
            delete[] tData;
        }
        cs = nullptr;
        dir = MOSI;
        tData = nullptr;
        rData = nullptr;
        dataLen = 0;
        cbData = nullptr;
    }

    SPIRequest &operator=(const SPIRequest &other) {
        cs = other.cs;
        dir = other.dir;
        tData = other.tData;
        rData = other.rData;
        dataLen = other.dataLen;
        cbData = other.cbData;
        deepCopyData(other.tData);
        return *this;
    }

private:
    void deepCopyData(uint8_t *other) {
        if (other) {
            tData = new uint8_t[dataLen]();
            for (int i = 0; i < dataLen; ++i) {
                this->tData[i] = other[i];
            }
        }
    }
};

/**
 * @brief Template class for hardware based SPI derivations
 * @todo make it possible to use more than one HardwareSPI at the same time
 */
class HardwareSPI : public RequestQueue<SPIRequest> {
public:
    /**
     * override virtual RequestQueue function
     * @return current time in ms
     */
    unsigned long getTime() override {
        return HAL_GetTick();
    }

    /**
     * override virtual RequestQueue function
     *
     * processes a request in the queue.
     * @param rq
     */
    void processRequest(SPIRequest &rq) override {
        // activate the chip
        rq.cs->selectChip(true);

        // transfer the data
        switch (rq.dir) {
        case SPIRequest::MOSI:
            HAL_SPI_Transmit_IT(&this->hSPI, rq.tData, rq.dataLen);
            break;
        case SPIRequest::MISO:
            HAL_SPI_Receive_IT(&this->hSPI, rq.rData, rq.dataLen);
            break;
        case SPIRequest::BOTH:
            HAL_SPI_TransmitReceive_IT(&this->hSPI, rq.tData, rq.rData, rq.dataLen);
            break;
        }
    }

private:
    /**
     * complete the data transfer, signal request completion
     * @param hspi
     */
    static void transferComplete(SPI_HandleTypeDef *hspi) {
        auto &r = pThis->lastRequest();
        // deactivate chip
        r.cs->selectChip(false);
        // signal data arrival
        if (r.cbData) {
            r.cs->callback(r.cbData);
        }
        // signal request complete
        pThis->endProcess();
    }

public:
    static HardwareSPI *master() {
        if (!pThis) {
            pThis = new HardwareSPI();
        }
        return pThis;
    }

private:
    /**
     * Initialize the MISO, MOSI, and CLK pins and configure SPI instance
     */
    HardwareSPI() : RequestQueue(50, HW_SPI_TIMEOUT) {
        this->initPins(HW_SPI_MISO_PIN, HW_SPI_MISO_PORT, HW_SPI_MISO_ALTERNATE,
                HW_SPI_MOSI_PIN, HW_SPI_MOSI_PORT, HW_SPI_MOSI_ALTERNATE,
                HW_SPI_SCK_PIN, HW_SPI_SCK_PORT, HW_SPI_SCK_ALTERNATE);
#if !defined(HW_SPI) || !defined(HW_SPI_BAUD_PRESCALER) ||\
    !defined(HW_SPI_CLK_POLARITY) || !defined(HW_SPI_CLK_PHASE)
#error "you have not set all necessary configuration defines!"
#endif
        this->config(HW_SPI, HW_SPI_BAUD_PRESCALER,
                HW_SPI_CLK_POLARITY, HW_SPI_CLK_PHASE);
    }

    //\cond false
    inline static HardwareSPI *pThis = nullptr;
    SPI_HandleTypeDef &hSPI = hHWSPI;

    void initPins(uint32_t iMISOPin, GPIO_TypeDef *gpioMISOPort, uint8_t iMISOAlternate,
                  uint32_t iMOSIPin, GPIO_TypeDef *gpioMOSIPort, uint8_t iMOSIAlternate,
                  uint32_t iSCKPin, GPIO_TypeDef *gpioSCKPort, uint8_t iSCKAlternate) {
        GPIO_InitTypeDef GPIO_InitStruct = {};
        GPIO_InitStruct.Pin = iMISOPin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = iMISOAlternate;
        HAL_GPIO_Init(gpioMISOPort, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = iMOSIPin;
        GPIO_InitStruct.Alternate = iMOSIAlternate;
        HAL_GPIO_Init(gpioMOSIPort, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = iSCKPin;
        GPIO_InitStruct.Alternate = iSCKAlternate;
        HAL_GPIO_Init(gpioSCKPort, &GPIO_InitStruct);
    }

    void config(SPI_TypeDef *dSPI, uint32_t iBaudPresc,
            uint32_t iClkPol, uint32_t iClkPhase) {
        hSPI.Instance = dSPI;
        hSPI.Init.Mode = SPI_MODE_MASTER;
        hSPI.Init.Direction = SPI_DIRECTION_2LINES;
        hSPI.Init.DataSize = SPI_DATASIZE_8BIT;
        hSPI.Init.CLKPolarity = iClkPol;
        hSPI.Init.CLKPhase = iClkPhase;
        hSPI.Init.NSS = SPI_NSS_SOFT;
        hSPI.Init.BaudRatePrescaler = iBaudPresc;
        hSPI.Init.FirstBit = SPI_FIRSTBIT_MSB;
        hSPI.Init.TIMode = SPI_TIMODE_DISABLE;
        hSPI.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        hSPI.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
        while(HAL_SPI_Init(&hSPI) != HAL_OK);

        HAL_SPI_RegisterCallback(&hSPI, HAL_SPI_TX_COMPLETE_CB_ID, transferComplete);
        HAL_SPI_RegisterCallback(&hSPI, HAL_SPI_RX_COMPLETE_CB_ID, transferComplete);
        HAL_SPI_RegisterCallback(&hSPI, HAL_SPI_TX_RX_COMPLETE_CB_ID, transferComplete);

        HAL_NVIC_SetPriority(SPI_IT_IRQn, SPI_IT_PRIO);
        HAL_NVIC_EnableIRQ(SPI_IT_IRQn);
    }
    //\endcond
};

/**
 * helper function for copying data bit by bit into a struct
 * @param dest  struct address
 * @param numbits lenght of struct in bits
 * @param szof  sizeof struct
 * @param src   source buffer
 * @param len   number of structs to copy
 */
static void bitwisecopy(uint8_t *dest, size_t numbits, size_t szof, uint8_t *src, size_t len) {
    uint32_t srcI = 0;
    for (size_t n = 0; n < len; ++n) {
        uint32_t destI = numbits;
        dest += n?szof:0;
        do {
            --destI;

            *(dest + destI / 8) &= ~(1U << destI % 8);
            *(dest + destI / 8) |= ((*(src + srcI / 8) >> 7 - srcI % 8) & 0x1U) << destI % 8;

            ++srcI;
        } while (destI > 0);
    }
}

#endif //STM_SPI_H

