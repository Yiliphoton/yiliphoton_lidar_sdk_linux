#pragma once

#include "ylp_driver/driver/decoder/basic_attr.hpp"

namespace asensing
{
namespace lidar
{

#define V192_CHANNELS_PER_BLOCK        (192)
#define V192_POINTS_PER_ROW_MAX        (800)
#define V192_POINTS_PER_ROW            V192_POINTS_PER_ROW_MAX
#define V192_SIDES_OF_MIRROR           (4)
#define V192_BLOCKS_PER_PACKET         (1)
#define V192_AZIMUTH_UNIT              (0.01f)

#pragma pack(push, 1) // 不字节填充

typedef struct
{
    uint16_t Header;
    uint8_t  ProtocolVersion;
    uint16_t FrameID;
    uint16_t PacketSeq;
    uint8_t  ReturnMode;
    uint8_t  BlockNum;
    uint16_t AzimuthAngle;
    uint16_t ElevationAngleOffset;
    //TODO Timestamps 拆开
    uint8_t  UTCTime0;      /* Year   */
    uint8_t  UTCTime1;      /* Month  */
    uint8_t  UTCTime2;      /* Mday   */
    uint8_t  UTCTime3;      /* Hour   */
    uint8_t  UTCTime4;      /* Minute */
    uint8_t  UTCTime5;      /* Second */

    uint32_t Timestamp;     /* Microseconds */

    uint16_t SlotNum;

    //TODO Reserve 拆开
    uint8_t Reserve1;
    uint8_t Reserve2;
    uint8_t Reserve3;
    uint8_t Reserve4;
    uint8_t Reserve5;
    uint8_t Reserve6;
    uint8_t Reserve7;
} V192MsopHeader;

typedef struct
{
    uint16_t EchoDistance;        // 径向距离 分辨率 5mm
    uint8_t  EchoIntensity;       // 反射强度 0-255
    uint8_t  EchoConfidence;      // 回波置信度 0-255
} V192MsopChannel;

typedef struct
{
    V192MsopChannel channel[V192_CHANNELS_PER_BLOCK];
} V192MsopBlock;

typedef struct
{
    uint32_t CRC;
    uint16_t Footer;
} V192MsopTail;

typedef struct
{
    V192MsopHeader header;
    V192MsopBlock  blocks[V192_BLOCKS_PER_PACKET];
    V192MsopTail   tail;
} V192MsopPkt;

#pragma pack(pop) // 不字节填充

}// namespace lidar
}// namespace asensing
