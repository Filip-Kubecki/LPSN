#ifndef E104_H_
#define E104_H_

#include "main.h"

typedef struct __attribute__((packed)) {
    uint16_t manufacturer_id; 
    int16_t temperature;      
    uint16_t humidity;         
    uint32_t pressure;        
    uint32_t magic_number;     
    uint8_t battery_percent;
    uint8_t device_id;   
} BleAdvFrame_t;

void    E104_Init(UART_HandleTypeDef *huart);
void    E104_Wake(void);
void    E104_Sleep(void);
uint8_t E104_SetAdvertising(const BleAdvFrame_t *frame);

#endif /* E104_H_ */
