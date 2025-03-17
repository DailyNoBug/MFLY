//
// Created by lsc on 25-3-17.
//

#include "be252q.h"

void BE252Q_Init(BE252Q_HandleTypeDef* hbe252q) {
    // 初始化接收缓冲区
    hbe252q->rx_index = 0;
    hbe252q->data_ready = 0;

    // 使能UART3接收中断
    HAL_UART_Receive_IT(hbe252q->huart_rx,
                        &hbe252q->rx_buffer[hbe252q->rx_index],
                        1);

    // 使能空闲中断
    __HAL_UART_ENABLE_IT(hbe252q->huart_rx, UART_IT_IDLE);
}

void BE252Q_UART_IRQHandler(BE252Q_HandleTypeDef* hbe252q) {
    // 处理空闲中断
    if(__HAL_UART_GET_FLAG(hbe252q->huart_rx, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(hbe252q->huart_rx);

        // 获取接收数据长度
        uint16_t data_length = hbe252q->rx_index;

        // 标记数据准备就绪
        hbe252q->data_ready = 1;

        // 重置接收索引
        hbe252q->rx_index = 0;

        // 通过UART2转发数据
        if(data_length > 0) {
            HAL_UART_Transmit(hbe252q->huart_tx,
                              hbe252q->rx_buffer,
                              data_length,
                              HAL_MAX_DELAY);
        }
    }

    // 处理接收完成中断
    HAL_UART_IRQHandler(hbe252q->huart_rx);
}