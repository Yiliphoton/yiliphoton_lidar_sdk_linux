#pragma once

#include "core/driver/decoder/P60/decoder_P60.hpp"
#include "P60_IMU_struct.hpp"

#include <cstring>
#include <cmath>

namespace lidar_sdk
{
namespace lidar
{

template <typename T_PointCloud>
class DecoderP60IMU : public DecoderP60<T_PointCloud>
{
public:
    virtual ~DecoderP60IMU() = default;
    explicit DecoderP60IMU(const DecoderParam &param);
    virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);

private:
    static DecoderConstParam &getConstParam();

    double frame_start_ts_ = 0.0;
    uint8_t prev_imu_raw_[16] = {};
    bool imu_initialized_ = false;
};

template <typename T_PointCloud>
inline DecoderP60IMU<T_PointCloud>::DecoderP60IMU(const DecoderParam &param)
    : DecoderP60<T_PointCloud>(getConstParam(), param)
{
}

template <typename T_PointCloud>
inline DecoderConstParam &DecoderP60IMU<T_PointCloud>::getConstParam()
{
    static DecoderConstParam param = {
        838,   // MSOP_LEN: header(32) + 96ch*8B(768) + imu(32) + tail(6) = 838
        838,   // MSOP2_LEN
        108,   // DIFOP_LEN (unused)
        2, 4,  // MSOP_ID_LEN, DIFOP_ID_LEN
        {0x55, 0xC0}, {0xA5, 0xFF, 0x00, 0x5A},
        96, 1, P60_CHANNELS_PER_BLOCK,
        0.0f, 300.0f, 0.01f, 80.0f
    };
    return param;
}

template <typename T_PointCloud>
inline bool DecoderP60IMU<T_PointCloud>::decodeMsopPkt(const uint8_t *packet, size_t /*size*/)
{
    bool ret = false;

    const P60MsopHeader &header = *(const P60MsopHeader *)packet;
    const P60IMUMsopPkt &msoppkt = *(const P60IMUMsopPkt *)packet;

    // P60_IMU time layout (little-endian): Timestamp(uint32 LE, 4B) then sec,min,hour,day,month,year
    uint32_t timestamp_us;
    memcpy(&timestamp_us, packet + 13, 4);
    uint8_t sec   = packet[17];
    uint8_t min   = packet[18];
    uint8_t hour  = packet[19];
    uint8_t day   = packet[20];
    uint8_t month = packet[21];
    uint8_t year  = packet[22];

    struct tm t{};
    t.tm_year  = year + 100;
    t.tm_mon   = month - 1;
    t.tm_mday  = day;
    t.tm_hour  = hour;
    t.tm_min   = min;
    t.tm_sec   = sec;
    t.tm_isdst = 0;
    double pkt_ts = static_cast<double>(timegm(&t)) + timestamp_us / 1000000.0;

    uint16_t frame_id = header.FrameID;
    this->current_buffer_->frame_id = frame_id;

    if (this->split_strategy_.newPacket(frame_id))
    {
        this->points_num_ = 0;

        this->blockPut(this->current_buffer_, true);

        this->current_buffer_ = this->blockGet(P60_POINTS_PER_ROW, P60_CHANNELS_PER_BLOCK, true);
        this->current_buffer_->is_aligned = false;
        this->current_buffer_->points_offset = this->LD_azimuth_offset_num_;

        std::shared_ptr<T_PointCloud> cloud = this->point_cloud_;
        this->cb_split_frame_(192, (frame_start_ts_ != 0.0 ? frame_start_ts_ : pkt_ts));
        if (this->param_.process_by_thread)
            std::thread([this, cloud]{ this->processPointCloud(cloud); }).detach();
        else
            this->processPointCloud(cloud);

        frame_start_ts_ = pkt_ts;
        ret = true;
    }

    std::vector<RawPointT> &buffer  = this->current_buffer_->points_;
    std::vector<RawPointT> &buffer2 = this->current_buffer_->points2_;

    for (uint16_t blk = 0; blk < header.BlockNum; blk++)
    {
        for (uint16_t chan = 0; chan < P60_CHANNELS_PER_BLOCK; chan++)
        {
            this->points_num_++;

            const P60MsopChannel &unit = msoppkt.blocks[blk].channel[chan];
            uint32_t pos = header.SlotNum * P60_CHANNELS_PER_BLOCK + chan;

            if (pos >= buffer.size())
                continue;

            float azimuth_deg = header.AzimuthAngle * P60_AZIMUTH_UNIT
                              + this->azimuth_offset_[chan] + P60_AZIMUTH_OFFSET;
            while (azimuth_deg < 0.0f)    azimuth_deg += 360.0f;
            while (azimuth_deg >= 360.0f) azimuth_deg -= 360.0f;
            const float azimuth   = azimuth_deg * 100.0f;
            const float elevation = this->elevation_offset_[chan] * 100.0f;

            for (int echo = 0; echo < 2; echo++)
            {
                const float dist_cm = unit.echo[echo].Distance * 0.25f;
                auto &pt = (echo == 0) ? buffer[pos] : buffer2[pos];
                pt.distance     = dist_cm;
                pt.azimuth      = azimuth;
                pt.elevation    = elevation;
                pt.reflectivity = unit.echo[echo].Intensity;
                pt.confidence   = unit.echo[echo].Confidence;
                pt.timestamp    = pkt_ts;
                pt.ring         = chan;
            }
        }
    }

    // --- IMU parsing ---
    const P60ImuRawData &imu = msoppkt.imu;
    if (!imu_initialized_ || memcmp(&imu, prev_imu_raw_, 16) != 0)
    {
        imu_initialized_ = true;
        memcpy(prev_imu_raw_, &imu, 16);

        if (this->cb_imu_)
        {
            constexpr float ACCEL_SCALE = 4096.0f;
            constexpr float GYRO_SCALE  = 65.5f;
            constexpr float G_TO_MS2    = 9.80665f;
            constexpr float DEG_TO_RAD  = static_cast<float>(M_PI) / 180.0f;

            LidarImuData imu_data;
            imu_data.timestamp = pkt_ts;
            imu_data.accel_x = static_cast<int16_t>((imu.AccelX_H << 8) | imu.AccelX_L) / ACCEL_SCALE * G_TO_MS2;
            imu_data.accel_y = static_cast<int16_t>((imu.AccelY_H << 8) | imu.AccelY_L) / ACCEL_SCALE * G_TO_MS2;
            imu_data.accel_z = static_cast<int16_t>((imu.AccelZ_H << 8) | imu.AccelZ_L) / ACCEL_SCALE * G_TO_MS2;
            imu_data.gyro_x  = static_cast<int16_t>((imu.GyroX_H << 8) | imu.GyroX_L) / GYRO_SCALE * DEG_TO_RAD;
            imu_data.gyro_y  = static_cast<int16_t>((imu.GyroY_H << 8) | imu.GyroY_L) / GYRO_SCALE * DEG_TO_RAD;
            imu_data.gyro_z  = static_cast<int16_t>((imu.GyroZ_H << 8) | imu.GyroZ_L) / GYRO_SCALE * DEG_TO_RAD;
            imu_data.temperature = imu.Temp / 2.07f + 25.0f;

            this->cb_imu_(imu_data);
        }
    }

    return ret;
}

}  // namespace lidar
}  // namespace lidar_sdk
