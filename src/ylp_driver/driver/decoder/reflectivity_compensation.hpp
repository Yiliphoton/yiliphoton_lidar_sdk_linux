#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>
#include "ylp_driver/common/block_buffer.hpp"
#include "ylp_driver/common/ag_log.hpp"

namespace asensing::lidar {

struct ReflectivityCompensationProfile 
{
    size_t channel_group_size = 1;   // 每组通道数量，例如 24
    float distance_resolution = 1.f; // 距离索引步长（米）
    size_t rows = 0;                 // 帧宽
    size_t cols = 0;                 // 帧高
    size_t group_count = 0;          // 通道分组数
    size_t distance_bins = 0;        // 距离采样格点数
    const float* a_lut = nullptr;    // a 系数 LUT（group × distance 扁平数组）
    const float* b_lut = nullptr;    // b 系数 LUT
    const float* rho_sat_lut = nullptr; // 饱和值 LUT
    const int* intensity_bias = nullptr; //通道偏移
};

class ReflectivityCompensator {
public:
    virtual ~ReflectivityCompensator() = default;
    virtual void apply(BlockBuffer& frame) const = 0;
};

/**
 * 基于查表的补偿器工厂。
 */
std::shared_ptr<ReflectivityCompensator> makeTableCompensator(ReflectivityCompensationProfile profile);

} // namespace asensing::lidar