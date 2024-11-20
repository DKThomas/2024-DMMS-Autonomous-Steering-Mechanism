/*
 * CubeMars_Functions.h
 *
 *  Created on: Nov 11, 2024
 *      Author: thomaskjeldsen
 */

#ifndef SRC_CUBEMARS_FUNCTIONS_H_
#define SRC_CUBEMARS_FUNCTIONS_H_

#include "main.h"

// Special CAN Commands
extern const uint8_t CAN_CMD_ENTER_MOTOR_CONTROL_MODE[8];
extern const uint8_t CAN_CMD_EXIT_MOTOR_CONTROL_MODE[8];
extern const uint8_t CAN_CMD_SET_ORIGIN[8];
extern const uint8_t CAN_CMD_READ_STATE[8];

extern const float P_MIN;
extern const float P_MAX;
extern const float V_MIN;
extern const float V_MAX;
extern const float T_MIN;
extern const float T_MAX;
extern const float KP_MIN;
extern const float KP_MAX;
extern const float KD_MIN;
extern const float KD_MAX;
#define I_MAX T_MAX // Not explicitly defined in manual

// Error Code Typedef
typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,						// Overvoltage
	FAULT_CODE_UNDER_VOLTAGE,						// Undervoltage
	FAULT_CODE_DRV,									// Driver fault
	FAULT_CODE_ABS_OVER_CURRENT,					// Motor overcurrent
	FAULT_CODE_OVER_TEMP_FET,						// MOS overtemperature
	FAULT_CODE_OVER_TEMP_MOTOR,						// Motor overtemperature
	FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,			// Driver overvoltage
	FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,			// Driver undervoltage
	FAULT_CODE_MCU_UNDER_VOLTAGE,					// MCU undervoltage
	FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,			// Undervoltage
	FAULT_CODE_ENCODER_SPI,							// SPI encoder fault
	FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,	// Encoder below minimum amplitude
	FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,	// Encoder above maximum amplitude
	FAULT_CODE_FLASH_CORRUPTION,					// Flash fault
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,		// Current sampling channel 1 fault
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,		// Current sampling channel 2 fault
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,		// Current sampling channel 3 fault
	FAULT_CODE_UNBALANCED_CURRENTS,					// Unbalanced currents
} mc_fault_code;

// Function Definitions
int   float_to_uint(float x,   float x_min, float x_max, unsigned int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void pack_cmd(uint8_t *data, float p_des, float v_des, float kp, float kd, float t_ff);
void cubemars_send_can_cmd(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, uint32_t *TxMailbox, float p_des, float v_des, float kp, float kd, float t_ff);
void cubemars_get_can_msg(uint8_t *RxData, int target_id, float *position, float *speed, float *torque, float *temperature, mc_fault_code *error);
void unpack_reply(uint8_t data[8], int target_id, float *position, float *speed, float *torque, float *temperature, mc_fault_code *error);
void cubemars_get_can_cmd4debug(uint8_t *RxData);
void unpack_cmd4debug(uint8_t data[8], float *p_ref, float *v_ref, float *kp_ref, float *kd_ref, float *t_ref);
void print_raw_data(uint8_t data[8]);
void print_cmd4debug(float p_ref, float v_ref, float kp_ref, float kd_ref, float t_ref);
void print_motor_data(float position, float speed, float torque, float temperature);
void print_motor_error(mc_fault_code error_code);

#endif /* SRC_CUBEMARS_FUNCTIONS_H_ */
