/**
 * @file cubemars_control.c
 * @brief Implementation of functions for controlling CubeMars motors via CAN communication.
 *
 * This source file implements the functions declared in cubemars_control.h.
 * It provides functionality for:
 * - Packing and sending control commands to CubeMars motors in MIT Control Mode.
 * - Receiving and unpacking feedback from the motors.
 * - Printing motor states and error information for debugging.
 *
 * Dependencies:
 * - STM32 HAL library (for CAN communication).
 * - The CubeMars motor manual for parameter ranges and command details.
 *
 * Many of the functions in this document are based on example code in the
 * AK Series Module Driver User Manual V1.0.15.X, as referenced in the function comments below.
 *
 * Setup:
 * - Initialize the CAN peripheral before calling these functions.
 * - Ensure proper configuration of the CAN bus (baud rate, filters, etc.).
 *
 * Note:
 * Floating-point support for `printf` is required to print motor data and debugging information.
 *
 * Developed for UTS Motorsports Autonomous
 * Project 29 by Team 21
 * 43019 Design in Mechanical and Mechatronic Systems
 * University of Technology Sydney
 * November 2024
 */

#include "cubemars_control.h"

// Special CAN Commands
const uint8_t CAN_CMD_ENTER_MOTOR_CONTROL_MODE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t CAN_CMD_EXIT_MOTOR_CONTROL_MODE[8]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t CAN_CMD_SET_ORIGIN[8]               = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
const uint8_t CAN_CMD_READ_STATE[8]               = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

// Parameter Ranges
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

/**
 * @brief Converts a float value into an unsigned integer within a specified range and resolution. (AK Series User Manual, p. 44)
 *
 * @param x The input float value to convert.
 * @param x_min The minimum value of the range.
 * @param x_max The maximum value of the range.
 * @param bits The number of bits used to represent the unsigned integer.
 * @return The converted unsigned integer value.
 */
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
	/// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max - x_min;
	if(x < x_min) x = x_min;
	else if(x > x_max) x = x_max;
	return (int) ((x - x_min) * ((float)((1 << bits) / span)));
}

/**
 * @brief Converts an unsigned integer into a float within a specified range and resolution. (AK Series User Manual, p. 44)
 *
 * @param x_int The input unsigned integer to convert.
 * @param x_min The minimum value of the range.
 * @param x_max The maximum value of the range.
 * @param bits The number of bits used to represent the unsigned integer.
 * @return The converted float value.
 */
float uint_to_float(int x_int, float x_min, float x_max, unsigned int bits) {
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief Packs control parameters into an 8-byte CAN data buffer for transmission. (AK Series User Manual, p. 43)
 *
 * @param data The 8-byte CAN data buffer.
 * @param p_des Desired position.
 * @param v_des Desired speed.
 * @param kp Proportional gain.
 * @param kd Derivative gain.
 * @param t_ff Feedforward torque.
 */
void pack_cmd(uint8_t data[8], float p_des, float v_des, float kp, float kd, float t_ff) {
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

/**
 * @brief Sends a command to the CubeMars motor via CAN.
 *
 * @param hcan Pointer to the CAN handle.
 * @param TxHeader Pointer to the CAN TxHeader structure.
 * @param TxMailbox Pointer to the mailbox identifier for transmission.
 * @param p_des Desired position.
 * @param v_des Desired speed.
 * @param kp Proportional gain.
 * @param kd Derivative gain.
 * @param t_ff Feedforward torque.
 */
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

/**
 * @brief Unpacks a received CAN message into motor data. (AK Series User Manual, p. 44)
 *
 * @param data The received 8-byte CAN data buffer.
 * @param target_id The ID of the target motor.
 * @param position Pointer to store the unpacked position.
 * @param speed Pointer to store the unpacked speed.
 * @param torque Pointer to store the unpacked torque.
 * @param temperature Pointer to store the unpacked temperature.
 * @param error Pointer to store the unpacked error code.
 */
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

/**
 * @brief Processes a received CAN message from the CubeMars motor.
 *
 * @param data The received 8-byte CAN data buffer.
 * @param target_id The ID of the target motor.
 * @param position Pointer to store the received position.
 * @param speed Pointer to store the received speed.
 * @param torque Pointer to store the received torque.
 * @param temperature Pointer to store the received temperature.
 * @param error Pointer to store the received error code.
 */
void cubemars_get_can_msg(uint8_t data[8], int target_id, float *position, float *speed, float *torque, float *temperature, mc_fault_code *error) {
	// Define local variables
	//float position, speed, torque, temperature;
	//mc_fault_code error;

	// Unpack received message
	unpack_reply(data, target_id, position, speed, torque, temperature, error);

	// Print data and fault description
	print_motor_data(*position, *speed, *torque, *temperature);
	print_motor_error(*error);
}

/**
 * @brief Unpacks a received CAN message into command data.
 *
 * @param data The received 8-byte CAN data buffer.
 * @param p_ref Pointer to store the unpacked desired position.
 * @param v_ref Pointer to store the unpacked desired speed.
 * @param kp_ref Pointer to store the unpacked proportional gain.
 * @param kd_ref Pointer to store the unpacked derivative gain.
 * @param t_ref Pointer to store the unpacked feedforward torque.
 */
void unpack_cmd4debug(uint8_t data[8], float *p_ref, float *v_ref, float *kp_ref, float *kd_ref, float *t_ref) {
	/// unpack ints from can buffer ///
	int p_int = (data[0] << 8) | data[1];           // Motor Position Data
	int v_int = (data[2] << 4) | (data[3] >> 4);    // Motor Speed Data
	int kp_int = ((data[3] & 0xF) << 8) | data[4];	// Proportional Gain Data
	int kd_int = (data[5] << 4) | (data[6] >> 4);   // Derivative Gain Data
	int t_int = ((data[6] & 0xF) << 8) | data[7];   		// Feedforward Torque Data

	/// convert ints to floats ///
	float p_des = uint_to_float( p_int,   P_MIN,  P_MAX, 16);
	float v_des = uint_to_float( v_int,   V_MIN,  V_MAX, 12);
	float kp =    uint_to_float(kp_int, -KP_MIN, KP_MAX, 12);
	float kd =    uint_to_float(kd_int, -KD_MIN, KD_MAX, 12);
	float t_ff =  uint_to_float( t_int,  -T_MIN,  T_MAX, 12);

	// Read corresponding data
	*p_ref = p_des;
	*v_ref = v_des;
	*kp_ref = kp;
	*kd_ref = kd;
	*t_ref = t_ff;
}

/**
 * @brief Processes a received CAN message for debugging purposes,
 * 		  emulating a motor receiving the command (though without sending feedback),
 * 		  useful for confirming that data is transmitted correctly.
 *
 * @param data The received 8-byte CAN data buffer.
 */
void cubemars_get_can_cmd4debug(uint8_t data[8]) {
	// Define local variables
	float p_ref, v_ref, kp_ref, kd_ref, t_ref;

	// Unpack received message
	unpack_cmd4debug(data, &p_ref, &v_ref, &kp_ref, &kd_ref, &t_ref);

	// Print data and fault description
	print_cmd4debug(p_ref, v_ref, kp_ref, kd_ref, t_ref);
}

/**
 * @brief Prints raw CAN data for debugging purposes.
 *
 * @param data The received 8-byte CAN data buffer.
 */
void print_raw_data(uint8_t data[8]) {
    printf("RAW Data: %d %d %d %d %d %d %d %d\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

/**
 * @brief Prints motor data (position, speed, torque, and temperature).
 *
 * @param position Motor position.
 * @param speed Motor speed.
 * @param torque Motor torque.
 * @param temperature Motor temperature.
 */
void print_motor_data(float position, float speed, float torque, float temperature) {
    printf("Motor Data Received: Position = %f, Speed = %f, Torque = %f, Temp = %f\n", position, speed, torque, temperature);
}

/**
 * @brief Prints unpacked command data for debugging purposes.
 *
 * @param p_ref Desired position.
 * @param v_ref Desired speed.
 * @param kp_ref Proportional gain.
 * @param kd_ref Derivative gain.
 * @param t_ref Desired feedforward torque.
 */
void print_cmd4debug(float p_ref, float v_ref, float kp_ref, float kd_ref, float t_ref) {
    printf("CMD Received: p_des = %f, v_des = %f, kp = %f, kd = %f, t_ff = %f\n", p_ref, v_ref, kp_ref, kd_ref, t_ref);
}

/**
 * @brief Prints a description of a motor fault based on the error code.
 *
 * @param error_code The motor error code.
 */
void print_motor_error(mc_fault_code error_code) {
    switch (error_code) {
        case FAULT_CODE_NONE:
            printf("No fault detected.\n"); // This line can be removed/commented out when functionality has been confirmed
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
