/*
 * CubeMars_Functions.c
 *
 *  Created on: Nov 11, 2024
 *      Author: thomaskjeldsen
 */

#include "cubemars_control.h"

/// 5.3 MIT Mode Communication Protocol ///

// Special CAN Commands
const uint8_t CAN_CMD_ENTER_MOTOR_CONTROL_MODE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t CAN_CMD_EXIT_MOTOR_CONTROL_MODE[8]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t CAN_CMD_SET_ORIGIN[8]               = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
const uint8_t CAN_CMD_READ_STATE[8]               = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

const float P_MIN =-12.5f;
const float P_MAX = 12.5f;
const float V_MIN =-50.0f;
const float V_MAX = 50.0f;
const float T_MIN =-18.0f;
const float T_MAX = 18.0f;
const float KP_MIN = 0;
const float KP_MAX = 500.0f;
const float KD_MIN = 0;
const float KD_MAX = 5.0f;

// (p. 44)
// When sending packets, all numbers need to go through the following function to be converted into integer values before being sent to the motor:
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
	/// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max - x_min;
	if(x < x_min) x = x_min;
	else if(x > x_max) x = x_max;
	return (int) ((x - x_min) * ((float)((1 << bits) / span)));
}

// (p. 44)
// When receiving, convert all values to floating-point numbers using the following function:
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

// MIT Mode Sending&Receiving Code Example (p. 43)
// Sending Example Code:
void pack_cmd(uint8_t *data, float p_des, float v_des, float kp, float kd, float t_ff) {
	/// limit data to be within bounds ///
	p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
	v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
	kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
	kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
	t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);

	/// convert floats to unsigned ints ///
	int p_int =  float_to_uint(p_des,  P_MIN,  P_MAX, 16);
	int v_int =  float_to_uint(v_des,  V_MIN,  V_MAX, 12);
	int kp_int = float_to_uint(   kp, KP_MIN, KP_MAX, 12);
	int kd_int = float_to_uint(   kd, KD_MIN, KD_MAX, 12);
	int t_int =  float_to_uint( t_ff,  T_MIN,  T_MAX, 12);

	/// pack ints into the can buffer ///
	data[0] = p_int >> 8; 							// Position High 8
	data[1] = p_int & 0xFF; 						// Position Low 8
	data[2] = v_int >> 4; 							// Speed High 8 bits
	data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);	// Speed Low 4 bits KP High 4 bits
	data[4] = kp_int & 0xFF; 						// KP Low 8 bits
	data[5] = kd_int >> 4; 							// Kd High 8 bits
	data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP Low 4 bits Torque High 4 bits
	data[7] = t_int & 0xFF; 						// Torque Low 8 bits
}

// Function to send command
void cubemars_send_can_cmd(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *TxHeader, uint32_t *TxMailbox, float p_des, float v_des, float kp, float kd, float t_ff) {
	// Convert and pack data
	uint8_t data[8];
	pack_cmd(data, p_des, v_des, kp, kd, t_ff);

	// Enter Motor Control Mode
	if (HAL_CAN_AddTxMessage(hcan, TxHeader, CAN_CMD_ENTER_MOTOR_CONTROL_MODE, TxMailbox) != HAL_OK) {
		// Handle error (optional: print error or use debugging tools)
		printf("Error: Failed to send Enter Motor Control Mode command.\n");
	}

	// Send position, speed, and control parameters
	if (HAL_CAN_AddTxMessage(hcan, TxHeader, data, TxMailbox) != HAL_OK) {
		// Handle error (optional: print error or use debugging tools)
		printf("Error: Failed to send motor control command.\n");
	}
}

void cubemars_get_can_msg(uint8_t *RxData, int target_id, float *position, float *speed, float *torque, float *temperature, mc_fault_code *error) {
	// Define local variables
	//float position, speed, torque, temperature;
	//mc_fault_code error;

	// Unpack received message
	unpack_reply(RxData, target_id, position, speed, torque, temperature, error);

	// Print data and fault description
	print_motor_data(*position, *speed, *torque, *temperature);
	print_motor_error(*error);
}

// (p. 44)
// Receiving Example Code
void unpack_reply(uint8_t data[8], int target_id, float *position, float *speed, float *torque, float *temperature, mc_fault_code *error) {
	/// unpack ints from can buffer ///
	int id = data[0]; 								// Driver ID
	int p_int = (data[1] << 8) |  data[2]; 			// Motor Position Data
	int v_int = (data[3] << 4) | (data[4]>>4); 		// Motor Speed Data
	int i_int = ((data[4] & 0xF) << 8) | data[5];	// Motor Torque Data
	int T_int = data[6];							// Motor Temperature Data
	int e_int = data[7];							// Motor Error Code

	/// convert ints to floats ///
	float p = uint_to_float(p_int,  P_MIN, P_MAX, 16);
	float v = uint_to_float(v_int,  V_MIN, V_MAX, 12);
	float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
	float T = T_int - 40.0f;						// Temperature range: -40~215

	// Read corresponding data based on ID
	if(id == target_id) {
		*position = p;
		*speed = v;
		*torque = i;
		*temperature = T;
		*error = (mc_fault_code)e_int;
	}
}

void print_motor_data(float position, float speed, float torque, float temperature) {
    printf("Position = %f, Speed = %f, Torque = %f, Temp = %f\n", position, speed, torque, temperature);
}

void print_motor_error(mc_fault_code error_code) {
    switch (error_code) {
        case FAULT_CODE_NONE:
            printf("No fault detected.\n"); // This line can be removed when functionality has been confirmed
            break;
        case FAULT_CODE_OVER_VOLTAGE:
            printf("Fault: Over Voltage.\n");
            break;
        case FAULT_CODE_UNDER_VOLTAGE:
            printf("Fault: Under Voltage.\n");
            break;
        case FAULT_CODE_DRV:
            printf("Fault: Driver Fault.\n");
            break;
        case FAULT_CODE_ABS_OVER_CURRENT:
            printf("Fault: Motor Overcurrent.\n");
            break;
        case FAULT_CODE_OVER_TEMP_FET:
            printf("Fault: MOS Overtemperature.\n");
            break;
        case FAULT_CODE_OVER_TEMP_MOTOR:
            printf("Fault: Motor Overtemperature.\n");
            break;
        case FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE:
            printf("Fault: Gate Driver Overvoltage.\n");
            break;
        case FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE:
            printf("Fault: Gate Driver Undervoltage.\n");
            break;
        case FAULT_CODE_MCU_UNDER_VOLTAGE:
            printf("Fault: MCU Undervoltage.\n");
            break;
        case FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET:
            printf("Fault: Booting from Watchdog Reset.\n");
            break;
        case FAULT_CODE_ENCODER_SPI:
            printf("Fault: SPI Encoder Error.\n");
            break;
        case FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE:
            printf("Fault: Encoder Below Minimum Amplitude.\n");
            break;
        case FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE:
            printf("Fault: Encoder Above Maximum Amplitude.\n");
            break;
        case FAULT_CODE_FLASH_CORRUPTION:
            printf("Fault: Flash Corruption.\n");
            break;
        case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1:
            printf("Fault: High Offset on Current Sensor 1.\n");
            break;
        case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2:
            printf("Fault: High Offset on Current Sensor 2.\n");
            break;
        case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3:
            printf("Fault: High Offset on Current Sensor 3.\n");
            break;
        case FAULT_CODE_UNBALANCED_CURRENTS:
            printf("Fault: Unbalanced Currents.\n");
            break;
        default:
            printf("Unknown fault code: %d\n", error_code);
            break;
    }
}
