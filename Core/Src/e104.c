#include "e104.h"
#include <string.h>
#include <stdio.h>

static UART_HandleTypeDef *e104_uart = NULL;
static char rx_buf[32];

void E104_Init(UART_HandleTypeDef *huart) {
    e104_uart = huart;
}

static uint8_t check_at_response(uint32_t timeout_ms) {
    memset(rx_buf, 0, sizeof(rx_buf));
    
    __HAL_UART_CLEAR_FLAG(e104_uart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    
    HAL_UART_Receive(e104_uart, (uint8_t *)rx_buf, sizeof(rx_buf) - 1, timeout_ms);
    
    if (strstr(rx_buf, "+OK") != NULL || strstr(rx_buf, "+ok") != NULL) {
        return 1; 
    }
    return 0; 
}

static uint8_t send_at_cmd(const char *cmd, uint32_t timeout_ms) {
    HAL_UART_Transmit(e104_uart, (uint8_t *)cmd, strlen(cmd), 100);
    return check_at_response(timeout_ms);
}

void E104_Wake(void) {
    HAL_GPIO_WritePin(WAKE_UP_GPIO_Port, WAKE_UP_Pin, GPIO_PIN_RESET); 
    HAL_Delay(250); 

    HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, GPIO_PIN_RESET);
    HAL_Delay(800);   

    uint8_t dummy;
    while (HAL_UART_Receive(e104_uart, &dummy, 1, 10) == HAL_OK);
}

void E104_Sleep(void) {
    send_at_cmd("AT+ADV=0", 150);
    send_at_cmd("AT+SLEEP", 200);
    HAL_GPIO_WritePin(WAKE_UP_GPIO_Port, WAKE_UP_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
}

uint8_t E104_SetAdvertising(const BleAdvFrame_t *frame) {
    send_at_cmd("AT+ADV=0", 150);
    send_at_cmd("AT+ADVINTV=160", 150);
    send_at_cmd("AT+IPWR=125", 150);

    /* Wyślij AT+ADVDAT= a po nim surowe bajty struktury */
    const uint8_t cmd_prefix[] = "AT+ADVDAT=";
    HAL_UART_Transmit(e104_uart, (uint8_t *)cmd_prefix, 10, 100);
    HAL_UART_Transmit(e104_uart, (uint8_t *)frame, sizeof(BleAdvFrame_t), 100);
    if (!check_at_response(500)) return 0;

    if (!send_at_cmd("AT+ADV=1", 300)) return 0;

    return 1;
}