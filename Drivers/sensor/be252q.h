//
// Created by lsc on 25-3-17.
//

#ifndef MFLY_BE252Q_H
#define MFLY_BE252Q_H
#include "stm32g4xx_hal.h"

#define RX_BUFFER_SIZE 1024

typedef struct {
    UART_HandleTypeDef* huart_rx;  // UART3
    UART_HandleTypeDef* huart_tx;  // UART2
    uint8_t rx_buffer[RX_BUFFER_SIZE];
    uint16_t rx_index;
    uint8_t data_ready;
} BE252Q_HandleTypeDef;

void BE252Q_Init(BE252Q_HandleTypeDef* hbe252q);
void BE252Q_UART_IRQHandler(BE252Q_HandleTypeDef* hbe252q);

#endif //MFLY_BE252Q_H
