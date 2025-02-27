
#include "ring_buffer.h"


void ring_buffer_init(ring_buffer_t *rb, uint8_t *mem_add, uint8_t cap)
{
	rb->buffer = mem_add;
	rb->capacity = cap;

	ring_buffer_reset(rb);
}

/*
 * @brief Esta funcion reinicia los datos disponibles en el buffer
 *
 * @retval size: cantidad de datos disponibles
 */
void ring_buffer_reset(ring_buffer_t *rb)
{
	rb->head = 0;
	rb->tail = 0;
	rb->is_full = 0;
}

/*
 * @brief Esta funcion calcula los datos disponibles en el buffer
 *
 * @retval size: cantidad de datos disponibles
 */
uint8_t ring_buffer_size(ring_buffer_t *rb)
{
	uint8_t size = 0;
	if (rb->head >= rb->tail) {
		size = rb->head - rb->tail;
	} else if (rb->is_full == 0) {
		size = (rb->capacity - rb->tail) + rb->head;
	} else {
		size = rb->capacity;
	}
	return size;
}

/*
 * @brief Esta funcion revisa si el buffer esta lleno
 *
 * @retval is_full: 0 si no esta lleno, 1 si esta lleno
 */
uint8_t ring_buffer_is_full(ring_buffer_t *rb)
{
	return rb->is_full;
}

/*
 * @brief Esta funcion revisa si el buffer esta vacio
 *
 * @retval 0 si esta vacio, 1 si no esta vacio
 */
uint8_t ring_buffer_is_empty(ring_buffer_t *rb)
{
	return ((rb->head == rb->tail) && (rb->is_full == 0)) ? 1 : 0;
}

/**
 * @brief Esta funcion escribe un dato en el buffer circular
 *
 * @param data: el dato que se va a escribir
 *
 * @retval Ninguno
 */
void ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
	rb->buffer[rb->head] = data;
	rb->head = rb->head + 1;

	if (rb->head >= rb->capacity) { // si la cabeza llega al final de la memoria
		rb->head = 0;
	}

	if (rb->is_full != 0) { // si se pierden datos viejos
		rb->tail = rb->tail + 1;
	}

	if (rb->tail >= rb->capacity) { // si la cola llega al final de la memoria
		rb->tail = 0;
	}

	if (rb->head == rb->tail) { // si la cabeza alcanza la cola
		rb->is_full = 1;
	}
}

/**
 * @brief Esta funcion lee un dato del buffer circular
 *
 * @param data: la direccion de donde se va a escribir el dato
 *
 * @retval 1: hay datos disponibles, 0: no hay datos
 */
uint8_t ring_buffer_read(ring_buffer_t *rb, uint8_t *data) // 0x20
{
	if ((rb->is_full != 0) || (rb->head != rb->tail)) { // data available
		*data = rb->buffer[rb->tail]; // add: 0x20, val: buffer
		rb->tail = rb->tail + 1;
		if (rb->tail >= rb->capacity) {
			rb->tail = 0;
		}
		rb->is_full = 0;

		return 1; // buffer con datos
	}
	return 0; // buffer vacio
}

