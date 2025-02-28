#include "keypad.h"
#include "main.h"
#include "ring_buffer.h"

void keypad_init(void)
{
  HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_RESET);
}

uint8_t keypad_scan(uint16_t GPIO_Pin)
{
    uint8_t key = 0;
    switch (GPIO_Pin)
    {
        case COLUMN_1_Pin:
        if (HAL_GPIO_ReadPin(COLUMN_1_GPIO_Port, COLUMN_1_Pin) == GPIO_PIN_SET) {
            key = 'E';
            break;
        }
        HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_1_GPIO_Port, COLUMN_1_Pin) == GPIO_PIN_SET) {
            key = '1';
            break;
        }
        HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_1_GPIO_Port, COLUMN_1_Pin) == GPIO_PIN_SET) {
            key = '4';
            break;
        }
        HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_1_GPIO_Port, COLUMN_1_Pin) == GPIO_PIN_SET) {
            key = '7';
            break;
        }
        HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_1_GPIO_Port, COLUMN_1_Pin) == GPIO_PIN_SET) {
            key = '*';
            break;
        }
        break;
    
    case COLUMN_2_Pin:
        if (HAL_GPIO_ReadPin(COLUMN_2_GPIO_Port, COLUMN_2_Pin) == GPIO_PIN_SET) {
            key = 'E';
            break;
        }
        HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_2_GPIO_Port, COLUMN_2_Pin) == GPIO_PIN_SET) {
            key = '2';
            break;
        }
        HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_2_GPIO_Port, COLUMN_2_Pin) == GPIO_PIN_SET) {
            key = '5';
            break;
        }
        HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_2_GPIO_Port, COLUMN_2_Pin) == GPIO_PIN_SET) {
            key = '8';
            break;
        }
        HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_2_GPIO_Port, COLUMN_2_Pin) == GPIO_PIN_SET) {
            key = '0';
            break;
        }
        break;
    
    case COLUMN_3_Pin:
        if (HAL_GPIO_ReadPin(COLUMN_3_GPIO_Port, COLUMN_3_Pin) == GPIO_PIN_SET) {
            key = 'E';
            break;
        }
        HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_3_GPIO_Port, COLUMN_3_Pin) == GPIO_PIN_SET) {
            key = '3';
            break;
        }
        HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_3_GPIO_Port, COLUMN_3_Pin) == GPIO_PIN_SET) {
            key = '6';
            break;
        }
        HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_3_GPIO_Port, COLUMN_3_Pin) == GPIO_PIN_SET) {
            key = '9';
            break;
        }
        HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_3_GPIO_Port, COLUMN_3_Pin) == GPIO_PIN_SET) {
            key = '#';
            break;
        }
        break;
    
    case COLUMN_4_Pin:
        if (HAL_GPIO_ReadPin(COLUMN_4_GPIO_Port, COLUMN_4_Pin) == GPIO_PIN_SET) {
            key = 'E';
            break;
        }
        HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_4_GPIO_Port, COLUMN_4_Pin) == GPIO_PIN_SET) {
            key = 'A';
            break;
        }
        HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_4_GPIO_Port, COLUMN_4_Pin) == GPIO_PIN_SET) {
            key = 'B';
            break;
        }
        HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_4_GPIO_Port, COLUMN_4_Pin) == GPIO_PIN_SET) {
            key = 'C';
            break;
        }
        HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_SET);
        for (int i = 0; i < 10; i++);
        if (HAL_GPIO_ReadPin(COLUMN_4_GPIO_Port, COLUMN_4_Pin) == GPIO_PIN_SET) {
            key = 'D';
            break;
        }
        break;    
    }
    keypad_init();
  return key;
}