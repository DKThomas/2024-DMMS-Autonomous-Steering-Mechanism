/*
 * CubeMars_FunctionsAndExamples.c
 *
 * Some functions and examples from "AK Series Module Driver User Manual.pdf" by CubeMars.
 * Functions are copied from manual. Some effort has been made to correct syntax errors.
 *
 * Note:
 * Exclude this file from build to avoid errors.
 * Modified functions are implemented in cubemars_control.c.
 *
 * Developed for UTS Motorsports Autonomous
 * Project 29 by Team 21
 * 43019 Design in Mechanical and Mechatronic Systems
 * University of Technology Sydney
 * November 2024
 */

/// 5.1 Servo Mode Control Modes and Explanation ///

// (p. 27)
// The following are examples of invoking library functions and macro definitions:
typedef enum {
	CAN_PACKET_SET_DUTY = 0, // Duty Cycle Mode
	CAN_PACKET_SET_CURRENT, // Current Loop Mode
	CAN_PACKET_SET_CURRENT_BRAKE, // Current Brake Mode
	CAN_PACKET_SET_RPM, // Speed Mode
	CAN_PACKET_SET_POS, // Position Mode
	CAN_PACKET_SET_ORIGIN_HERE, // Set Origin Mode
	CAN_PACKET_SET_POS_SPD, // Position-Speed Loop Mode
} CAN_PACKET_ID;

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	uint8_t i=0;
	if (len > 8) {
		len = 8;
	}
	CanTxMsg TxMessage; TxMessage.StdId = 0;
	TxMessage.IDE = CAN_ID_EXT; TxMessage.ExtId = id;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = len;
	for(i=0;i<len;i++)
		TxMessage.Data[i]=data[i];
	CAN_Transmit(CHASSIS_CAN, &TxMessage); //CAN port sends TxMessage data
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

// 5.1.7 Position-Speed Loop Mode (p. 31)
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA ) {
	int32_t send_index = 0;
	int16_t send_index1 = 4;
	uint8_t buffer[8];
	buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
	buffer_append_int16(buffer,spd/ 10.0, & send_index1);
	buffer_append_int16(buffer,RPA/ 10.0, & send_index1);
	comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index1);
}

/// 5.2 Servo Mode Message Formats ///

// 5.2.1 Servo Mode CAN Upload Message Protocol (p. 32)
// Example of Receiving Message
void motor_receive(float* motor_pos, float* motor_spd, float* cur, int_8* temp, int_8* error, rx_message) {
	int16_t pos_int = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
	int16_t spd_int = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];
	int16_t cur_int = ((rx_message)->Data[4] << 8) | (rx_message)->Data[5];
	&motor_pos= (float)( pos_int * 0.1f); // Motor Position
	&motor_spd= (float)( spd_int * 10.0f);// Motor Speed
	&motor_cur= (float) ( cur_int * 0.01f);// Motor Current
	&motor_temp= (rx_message)->Data[6] ;// Motor Temperature
	&motor_error= (rx_message)->Data[7] ;// Motor Error Code
}

/// 5.3 MIT Mode Communication Protocol ///

// Special CAN Codes
// Enter Motor Control Mode
EnterMotorControlMode = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFC};
// Exit Motor Control Mode
ExitMotorControlMode  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFD};
// Set Current Motor Position as zero position
SetOrigin = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFE};
// PS: (If you want to read the current state in a stateless manner, the command to send is
ReadState = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0XFC};

// MIT Mode Sending&Receiving Code Example (p. 43)
// Sending Example Code:
void pack_cmd(CANMessage * msg, float p_des, float v_des, float kp, float kd, float t_ff){
	/// limit data to be within bounds ///
	float P_MIN =-12.5f;
	float P_MAX = 12.5f;
	float V_MIN =-30.0f;
	float V_MAX = 30.0f;
	float T_MIN =-18.0f;
	float T_MAX = 18.0f;
	float Kp_MIN = 0;
	float Kp_MAX = 500.0f;
	float Kd_MIN = 0;
	float Kd_MAX = 5.0f;
	float Test_Pos=0.0f;

	p_des = fminf(fmaxf(P_MIN, p_des), P_MAX);
	v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
	kp = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
	kd = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
	t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);

	/// convert floats to unsigned ints ///
	int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
	int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
	int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
	int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
	int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

	/// pack ints into the can buffer ///
	msg->data[0] = p_int >> 8; 								// Position High 8
	msg->data[1] = p_int & 0xFF; 							// Position Low 8
	msg->data[2] = v_int >> 4; 								// Speed High 8 bits
	msg->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);	// Speed Low 4 bits KP High 4 bits
	msg->data[4] = kp_int & 0xFF; 							// KP Low 8 bits
	msg->data[5] = kd_int >> 4; 							// Kd High 8 bits
	msg->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8); 	// KP Low 4 bits Torque High 4 bits
	msg->data[7] = t_int & 0xFF; 							// Torque Low 8 bits
}

// (p. 44)
// When sending packets, all numbers need to go through the following function to be converted into integer values before being sent to the motor:
int float_to_uint(float x, float x_min, float x_max, unsigned int bits){
	/// Converts a float to an unsigned int, given range and number of bits ///
	float span = x_max - x_min;
	if(x < x_min) x = x_min;
	else if(x > x_max) x = x_max;
	return (int) ((x - x_min) * ((float)((1 << bits) / span)));
}

// (p. 44)
// Receiving Example Code
void unpack_reply(CANMessage msg){
	/// unpack ints from can buffer ///
	int id = msg.data[0]; 									// Driver ID
	int p_int = (msg.data[1] << 8) |  msg.data[2]; 			// Motor Position Data
	int v_int = (msg.data[3] << 4) | (msg.data[4]>>4); 		// Motor Speed Data
	int i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5];	// Motor Torque Data
	int T_int = msg.data[6];

	/// convert ints to floats ///
	float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
	float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
	float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
	float T = T_int;

	if(id == 1){
		postion = p; 		// Read corresponding data based on ID
		speed = v;
		torque = i;
		Temperature = T-40;	// Temperature range: -40~215
	}
}

// (p. 44)
// When receiving, convert all values to floating-point numbers using the following function:
float uint_to_float(int x_int, float x_min, float x_max, int bits){
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

