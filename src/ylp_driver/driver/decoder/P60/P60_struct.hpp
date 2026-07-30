#pragma once

#include "ylp_driver/driver/decoder/basic_attr.hpp"

namespace asensing
{
namespace lidar
{

#define P60_CHANNELS_PER_BLOCK        (96)
#define P60_POINTS_PER_ROW_MAX        (1000)
#define P60_POINTS_PER_ROW            P60_POINTS_PER_ROW_MAX
#define P60_SIDES_OF_MIRROR           (4)
#define P60_BLOCKS_PER_PACKET         (1)
#define P60_AZIMUTH_UNIT              (0.01171875f)
#define P60_AZIMUTH_OFFSET            (-183.0f)
#define P60_ELEVATION_START           (50.0f)
#define P60_ELEVATION_STEP            (-0.625f)

#pragma pack(push, 1)

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
    uint8_t  UTCTime0;
    uint8_t  UTCTime1;
    uint8_t  UTCTime2;
    uint8_t  UTCTime3;
    uint8_t  UTCTime4;
    uint8_t  UTCTime5;
    uint32_t Timestamp;
    uint16_t SlotNum;
    uint8_t  productModel;
    uint8_t  Reserve2;
    uint8_t  Reserve3;
    uint8_t  Reserve4;
    uint8_t  Reserve5;
    uint8_t  Reserve6;
    uint8_t  Reserve7;
} P60MsopHeader;

typedef struct
{
    uint16_t Distance;
    uint8_t  Intensity;
    uint8_t  Confidence;
} P60EchoData;

typedef struct
{
    P60EchoData echo[2];   // echo[0] = first return, echo[1] = second return
} P60MsopChannel;          // 8 bytes per channel

typedef struct
{
    P60MsopChannel channel[P60_CHANNELS_PER_BLOCK];
} P60MsopBlock;

typedef struct
{
    uint32_t CRC;
    uint16_t Footer;
} P60MsopTail;

typedef struct
{
    P60MsopHeader header;
    P60MsopBlock  blocks[P60_BLOCKS_PER_PACKET];
    P60MsopTail   tail;
} P60MsopPkt;

#pragma pack(pop)

}// namespace lidar
}// namespace asensing
