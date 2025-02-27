

#include "keypad.h"

#include "main.h"


static uint8_t keypad_debounce(uint16_t GPIO_Pin)
{
	static uint16_t last_pressed = 0xFFFF;
	static uint32_t last_tick = 0;

	if (last_pressed == GPIO_Pin) {
		if (HAL_GetTick() < (last_tick + 200)) {
			return 0;
		}
	}
	last_pressed = GPIO_Pin;
	last_tick = HAL_GetTick();

	return 1;
}

uint8_t keypad_scan_row(GPIO_TypeDef *COLUMNx_GPIO_Port, uint16_t COLUMNx_Pin)
{
	uint8_t row_pressed = 0xFF;
	HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(COLUMNx_GPIO_Port, COLUMNx_Pin) == 0) {
		row_pressed =  1;
		goto row_scan_end;
	}
	HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(COLUMNx_GPIO_Port, COLUMNx_Pin) == 0) {
		row_pressed =  2;
		goto row_scan_end;
	}
	HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(COLUMNx_GPIO_Port, COLUMNx_Pin) == 0) {
		row_pressed =  3;
		goto row_scan_end;
	}
	HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_RESET);
	if (HAL_GPIO_ReadPin(COLUMNx_GPIO_Port, COLUMNx_Pin) == 0) {
		row_pressed =  4;
		goto row_scan_end;
	}
row_scan_end:
	HAL_GPIO_WritePin(ROW_1_GPIO_Port, ROW_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROW_2_GPIO_Port, ROW_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROW_3_GPIO_Port, ROW_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROW_4_GPIO_Port, ROW_4_Pin, GPIO_PIN_SET);
	return row_pressed; // not detected
}

uint8_t keypad_chars[4][4] = {
		{'1', '2', '3', 'A'},
		{'4', '5', '6', 'B'},
		{'7', '8', '9', 'C'},
		{'*', '0', '#', 'D'},
};

uint8_t keypad_scan(uint16_t GPIO_Pin)
{
	uint8_t key_pressed = 0xFF;

	if (keypad_debounce(GPIO_Pin) == 0) {
		return key_pressed;
	}
	uint8_t row = 0xFF;
	switch (GPIO_Pin) {
	case COLUMN_1_Pin:
		row = keypad_scan_row(COLUMN_1_GPIO_Port, COLUMN_1_Pin);
		key_pressed = keypad_chars[row - 1][1 - 1];
		break;

	case COLUMN_2_Pin:
		row = keypad_scan_row(COLUMN_2_GPIO_Port, COLUMN_2_Pin);
		key_pressed = keypad_chars[row - 1][2 - 1];
		break;

	case COLUMN_3_Pin:
			row = keypad_scan_row(COLUMN_3_GPIO_Port, COLUMN_3_Pin);
			key_pressed = keypad_chars[row - 1][3 - 1];
			break;

	case COLUMN_4_Pin:
			row = keypad_scan_row(COLUMN_4_GPIO_Port, COLUMN_4_Pin);
			key_pressed = keypad_chars[row - 1][4 - 1];
			break;
	default:
		break;
	}
	return key_pressed;
}
