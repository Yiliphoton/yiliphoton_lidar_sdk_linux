#pragma once

#include "core/driver/decoder/P60/P60_struct.hpp"

namespace lidar_sdk
{
namespace lidar
{

#pragma pack(push, 1)

typedef struct
{
    uint8_t  Header;
    uint8_t  AccelX_H;
    uint8_t  AccelX_L;
    uint8_t  AccelY_H;
    uint8_t  AccelY_L;
    uint8_t  AccelZ_H;
    uint8_t  AccelZ_L;
    uint8_t  GyroX_H;
    uint8_t  GyroX_L;
    uint8_t  GyroY_H;
    uint8_t  GyroY_L;
    uint8_t  GyroZ_H;
    uint8_t  GyroZ_L;
    int8_t   Temp;
    uint8_t  TimestampDelta_H;
    uint8_t  TimestampDelta_L;
    uint8_t  Reserved[16];
} P60ImuRawData;

typedef struct
{
    P60MsopHeader header;
    P60MsopBlock  blocks[P60_BLOCKS_PER_PACKET];
    P60ImuRawData imu;
    P60MsopTail   tail;
} P60IMUMsopPkt;

#pragma pack(pop)

}  // namespace lidar
}  // namespace lidar_sdk
