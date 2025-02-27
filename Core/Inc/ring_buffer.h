/*
 * ring_buffer.h
 *
 *  Created on: Feb 11, 2025
 *      Author: saaci
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <stdint.h>

typedef struct ring_buffer_ {
	uint8_t *buffer;
	uint8_t head;
	uint8_t tail;
	uint8_t is_full;

	uint8_t capacity;

} ring_buffer_t;

void ring_buffer_init(ring_buffer_t *rb, uint8_t *mem_add, uint8_t capacity);
void ring_buffer_reset(ring_buffer_t *rb);
uint8_t ring_buffer_size(ring_buffer_t *rb);
uint8_t ring_buffer_is_full(ring_buffer_t *rb);
uint8_t ring_buffer_is_empty(ring_buffer_t *rb);

void ring_buffer_write(ring_buffer_t *rb, uint8_t data);
uint8_t ring_buffer_read(ring_buffer_t *rb, uint8_t *byte);

#endif /* INC_RING_BUFFER_H_ */
