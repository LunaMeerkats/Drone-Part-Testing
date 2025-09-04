/*
 * sensors_if.h
 *
 *  Created on: Sep 3, 2025
 *      Author: Daniel Smith
 */

#ifndef INC_SENSORS_SENSOR_IF_H_
#define INC_SENSORS_SENSOR_IF_H_

#include "main.h"
#include <stdint.h>

typedef struct sensor_if_vtbl_s sensor_if_vtbl_t;

//Sensor Interface table
typedef struct {
	// virtual table
    const sensor_if_vtbl_t *vTable;
    // driver state (cast inside driver)
    void *state;
} sensor_if_t;

//Virtual Sensor Interface of Function Pointers
struct sensor_if_vtbl_s {
    HAL_StatusTypeDef (*probe)(sensor_if_t *self);
    HAL_StatusTypeDef (*init)(sensor_if_t *self);
    HAL_StatusTypeDef (*read)(sensor_if_t *self);
    HAL_StatusTypeDef (*whoami)(sensor_if_t *self, uint8_t *out);
};

#endif /* INC_SENSORS_SENSOR_IF_H_ */
