#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <dynamixel_sdk.h>

// Control table address
#define ADDR_TORQUE_ENABLE          (64)
#define ADDR_GOAL_POSITION          (116)
#define ADDR_PRESENT_POSITION       (132)
#define MINIMUM_POSITION_LIMIT      (0)
#define MAXIMUM_POSITION_LIMIT      (4095)
#define BAUDRATE                    (4000000)
#define PROTOCOL_VERSION            (2.0)
#define DXL_ID                      (1)
#define DEVICENAME                  ("/dev/ttyUSB0")

#define TORQUE_ENABLE               (1)
#define TORQUE_DISABLE              (0)
#define DXL_MOVING_STATUS_THRESHOLD (20)
#define ESC_ASCII_VALUE             (0x1b)

int main() {
    dynamixel::PortHandler* portHandler     = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler* packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    int index                               = 0;
    int dxl_comm_result                     = COMM_TX_FAIL; // Communication result
    int dxl_goal_position[2]                = {1000, 1100}; // Goal position

    uint8_t dxl_error                       = 0;            // DYNAMIXEL error
    int32_t dxl_present_position            = 0;            // Read 4 byte Position data

    // Open port
    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
    } else {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getchar();
        return 0;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    } else {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getchar();
        return 0;
    }

    // Enable DYNAMIXEL Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
        printf("Succeeded enabling DYNAMIXEL Torque.\n");
    }

    while (1) {
        printf("Press any key to continue. (Press [ESC] to exit)\n");
        if (getchar() == ESC_ASCII_VALUE)
            break;

// Write goal position
#if defined(XL320) // XL-320 uses 2 byte Position data
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION,
                                                        dxl_goal_position[index], &dxl_error);
#else
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION,
                                                        dxl_goal_position[index], &dxl_error);
#endif
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        } else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        do {
            // Read the Present Position
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION,
                                                           (uint32_t*)&dxl_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            } else if (dxl_error != 0) {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            printf("[ID:%03d] Goal Position:%03d  Present Position:%03d\n", DXL_ID, dxl_goal_position[index],
                   dxl_present_position);

        } while ((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

        // Switch the Goal Position
        if (index == 0) {
            index = 1;
        } else {
            index = 0;
        }
    }

    // Disable DYNAMIXEL Torque
    dxl_comm_result =
        packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
        printf("Succeeded disabling DYNAMIXEL Torque.\n");
    }

    // Close port
    portHandler->closePort();
    return 0;
}
