#ifndef MOTOR_A1B1_MSG
#define MOTOR_A1B1_MSG

#include <stdint.h>
typedef int16_t q15_t;

#pragma pack( 1 )

// Data structure for single sending data
typedef union {
  int32_t L;
  uint8_t u8[4];
  uint16_t u16[2];
  uint32_t u32;
  float F;
} COMData32;

typedef struct {
  // Define packet header
  unsigned char start[2]; // Header
  unsigned char motorID; // Motor ID 0,1,2,3 ... 0xBB means broadcast to all motors (no response in this case)
  unsigned char reserved;
} COMHead;

#pragma pack()

#pragma pack( 1 )

typedef struct {

  uint8_t fan_d;  // Cooling fan speed on the joint
  uint8_t Fmusic; // Motor sound frequency /64*1000 15.625f frequency increment
  uint8_t Hmusic; // Motor sound intensity Recommended value 4, sound intensity 0.1 increment
  uint8_t reserved4;

  uint8_t FRGB[4]; // Foot end LED

} LowHzMotorCmd;

typedef struct {     // Arranged in groups of 4 bytes, otherwise the compiler will pad
                     // Define data
  uint8_t mode;      // Joint mode selection
  uint8_t ModifyBit; // Motor control parameter modification bit
  uint8_t ReadBit;   // Motor control parameter send bit
  uint8_t reserved;

  COMData32 Modify; // Data for motor parameter modification
  // Actual torque command to FOC:
  // K_P*delta_Pos + K_W*delta_W + T
  q15_t T;     // Expected joint output torque (motor's own torque) x256, 7 + 8 description
  q15_t W;     // Expected joint speed (motor's own speed) x128, 8 + 7 description
  int32_t Pos; // Expected joint position x 16384/6.2832, 14-bit encoder (main controller 0 point correction, motor joint still uses encoder 0 point as reference)

  q15_t K_P; // Joint stiffness coefficient x2048  4+11 description
  q15_t K_W; // Joint velocity coefficient x1024  5+10 description

  uint8_t LowHzMotorCmdIndex; // Index of low frequency motor control command, 0-7, representing 8 bytes in LowHzMotorCmd
  uint8_t LowHzMotorCmdByte; // Byte of low frequency motor control command

  COMData32 Res[1]; // Communication reserved bytes used to implement other communication contents

} MasterComdV3; // With packet header and CRC, 34 bytes

typedef struct {
  // Define motor control command packet
  COMHead head;
  MasterComdV3 Mdata;
  COMData32 CRCdata;
} MasterComdDataV3; // Return data

// typedef struct {
// 	// Define total 485 data packet

//   MasterComdData M1;
// 	MasterComdData M2;
// 	MasterComdData M3;

// }DMA485TxDataV3;

#pragma pack()

#pragma pack( 1 )

typedef struct { // Arranged in groups of 4 bytes, otherwise the compiler will pad
  // Define data
  uint8_t mode;    // Current joint mode
  uint8_t ReadBit; // Motor control parameter modification success bit
  int8_t Temp;     // Current average motor temperature
  uint8_t MError;  // Motor error flag

  COMData32 Read; // Read current motor control data
  int16_t T;      // Current actual motor output torque 7 + 8 description

  int16_t W; // Current actual motor speed (high speed) 8 + 7 description
  float LW;  // Current actual motor speed (low speed)

  int16_t W2; // Current actual joint speed (high speed) 8 + 7 description
  float LW2;  // Current actual joint speed (low speed)

  int16_t Acc;    // Motor rotor acceleration 15+0 description (small inertia)
  int16_t OutAcc; // Output shaft acceleration 12+3 description (large inertia)

  int32_t Pos; // Current motor position (main controller 0 point correction, motor joint still uses encoder 0 point as reference)
  int32_t Pos2; // Joint encoder position (output encoder)

  int16_t gyro[3]; // Motor driver board 6-axis sensor data
  int16_t acc[3];

  // Force sensor data
  int16_t Fgyro[3]; //
  int16_t Facc[3];
  int16_t Fmag[3];
  uint8_t Ftemp; // 8-bit temperature 7 bits (-28~100 degrees) 1 bit 0.5 degree resolution

  int16_t Force16; // Force sensor high 16-bit data
  int8_t Force8;   // Force sensor low 8-bit data

  uint8_t FError; // Foot sensor error flag

  int8_t Res[1]; // Communication reserved byte

} ServoComdV3; // With packet header and CRC, 78 bytes (4+70+4)

typedef struct {
  // Define motor control command packet
  COMHead head;
  ServoComdV3 Mdata;

  COMData32 CRCdata;

} ServoComdDataV3; // Send data

// typedef struct {
// 	// Define total 485 receive data packet

//   ServoComdDataV3 M[3];
//  // uint8_t  nullbyte1;

// }DMA485RxDataV3;

#pragma pack()

//  00 00 00 00 00
//  00 00 00 00 00
//  00 00 00 00 00
//  00 00 00
// Default initialization of data packet
// Data packet sent by master
// clang-format off
/*
    Tx485Data[_FR][i].head.start[0] = 0xFE ;     Tx485Data[_FR][i].head.start[1] = 0xEE; // Packet header					 
    Tx485Data[_FR][i].Mdata.ModifyBit = 0xFF;    Tx485Data[_FR][i].Mdata.mode = 0;   // Default no data modification and motor's default working mode				
    Tx485Data[_FR][i].head.motorID = i;    0                                          // Target motor number
    Tx485Data[_FR][i].Mdata.T = 0.0f;                           // Default target joint output torque                      motor1.Extra_Torque = motorRxData[1].Mdata.T*0.390625f;     // N.M  converted to N.CM   IQ8 description
    Tx485Data[_FR][i].Mdata.Pos = 0x7FE95C80;                   // Default target joint position  position loop not enabled          14-bit resolution 
    Tx485Data[_FR][i].Mdata.W = 16000.0f;                       // Default target joint speed  speed loop not enabled          1+8+7 description     motor1.Target_Speed =  motorRxData[1].Mdata.W*0.0078125f;   // unit rad/s	       IQ7 description
    Tx485Data[_FR][i].Mdata.K_P = (q15_t)(0.6f*(1<<11));        // Default joint stiffness coefficient   4+11 description                     motor1.K_Pos = ((float)motorRxData[1].Mdata.K_P)/(1<<11);      // Communication data format describing stiffness  4+11
    Tx485Data[_FR][i].Mdata.K_W = (q15_t)(1.0f*(1<<10));        // Default joint speed coefficient   5+10 description                    motor1.K_Speed = ((float)motorRxData[1].Mdata.K_W)/(1<<10);    // Communication data format describing damping  5+10
*/
// clang-format on

#endif