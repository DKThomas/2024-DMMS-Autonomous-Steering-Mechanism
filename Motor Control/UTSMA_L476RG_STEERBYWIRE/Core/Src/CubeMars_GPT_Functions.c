/*
 * CubeMars_GPT_Functions.c
 *
 *  Created on: Nov 11, 2024
 *      Author: thomaskjeldsen
 */

#include <stdint.h>
#include <stdio.h>

// Function to convert a float to an unsigned integer based on a specified bit width and range
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (int)((x - x_min) * ((1 << bits) / span));
}

// Function to send CAN message
void send_can_message(uint16_t can_id, uint16_t position, uint16_t speed, uint16_t kp, uint16_t kd, uint16_t current) {
    uint8_t data[8];

    // Position: Split into high and low bytes
    data[0] = (position >> 8) & 0xFF;  						// High 8 bits of position
    data[1] = position & 0xFF;         						// Low  8 bits of position

    // Speed: Split into high and low bytes
    data[2] = (speed >> 8) & 0xFF;     						// High 8 bits of speed
    data[3] = ((speed & 0x0F) << 4) | ((kp >> 8) & 0x0F); 	// Low  4 bits of speed + High 4 bits of KP

    // KP: Split into high and low bytes
    data[4] = kp & 0xFF;               						// Low 8 bits of KP

    // KD: Split into high and low bytes
    data[5] = (kd >> 8) & 0xFF;        						// High 8 bits of KD
    data[6] = ((kd & 0x0F) << 4) | ((current >> 8) & 0x0F);	// Low  4 bits of KD + High 4 bits of current

    // Current: Split into high and low bytes
	data[7] = current & 0xFF;								// Low 8 bits of current

    // Print the message (for demonstration; replace this with actual CAN transmission in your environment)
    printf("cansend can0 %03X#", can_id);
    for (int i = 0; i < 8; i++) {
        printf("%02X", data[i]);
    }
    printf("\n");
}

int main() {
    // Input variables for position, speed, KP, and KD
    float position_deg = 270.0;   // Desired position in degrees
    float speed_mps = 0.2;        // Desired speed in m/s
    float kp_value = 0.5;         // Proportional gain
    float kd_value = 0.2;         // Derivative gain

    // Convert inputs to integer values based on ranges and bit widths
    uint16_t position = float_to_uint(position_deg, -36000, 36000, 16);
    uint16_t speed = float_to_uint(speed_mps, -30.0, 30.0, 12);  // Example range for speed
    uint16_t kp = float_to_uint(kp_value, 0, 500.0, 12);         // KP range based on 0-500
    uint16_t kd = float_to_uint(kd_value, 0, 5.0, 12);           // KD range based on 0-5

    // Send the CAN message
    send_can_message(0x004, position, speed, kp, kd);

    return 0;
}

void send_special_can_command(CAN_HandleTypeDef *hcan, uint16_t can_id, const uint8_t command[8]) {
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = can_id;          // Standard CAN ID for the message
    tx_header.ExtId = 0;               // Not using extended ID
    tx_header.RTR = CAN_RTR_DATA;      // Data frame
    tx_header.IDE = CAN_ID_STD;        // Standard identifier
    tx_header.DLC = 8;                 // Data length code: 8 bytes
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox;
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, command, &tx_mailbox) != HAL_OK) {
        // Transmission error handling (e.g., print error or log)
        printf("Error: Special CAN command transmission failed.\n");
    }
}
