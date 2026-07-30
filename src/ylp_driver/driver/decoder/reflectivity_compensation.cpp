#include "ylp_driver/driver/decoder/reflectivity_compensation.hpp"
#include <algorithm>

namespace asensing {
namespace lidar {

namespace {

// c++14 版本的 clamp
template <typename T>
inline T clampValue(T value, T low, T high) 
{
    return std::max(low, std::min(value, high));
}

// 基于 LUT 的反射率补偿实现
class TableReflectivityCompensator final : public ReflectivityCompensator
{
public:

    explicit TableReflectivityCompensator(ReflectivityCompensationProfile profile)
        : profile_(profile) {}
    
    void apply(BlockBuffer& frame) const override 
    {

        if (!profileValid()) 
        {
            return;
        }
        

        const size_t expected_points = profile_.rows * profile_.cols;

        //匿名函数 参数是std::vector<RawPointT>& points 函数内部可以使用外面变量（用 & 捕获）
        auto applyToPoints = [&](std::vector<RawPointT>& points) {
            if (points.empty()) 
            {
                return;
            }

            const size_t count = expected_points > 0 ? std::min(points.size(), expected_points) : points.size();
            
            //开始反射率计算
            for (size_t i = 0; i < count; ++i)
            {
                RawPointT& pt = points[i];
                size_t group = profile_.channel_group_size > 0
                           ? static_cast<size_t>(pt.ring / profile_.channel_group_size)
                           : 0;

                if (group >= profile_.group_count) 
                {
                    group = profile_.group_count - 1;
                }

                size_t distance_idx = profile_.distance_resolution > 0.f
                                        ? static_cast<size_t>(pt.distance * profile_.distance_resolution)
                                        : 0;

                if (distance_idx >= profile_.distance_bins) 
                {
                    distance_idx = profile_.distance_bins - 1;
                }
                
                const size_t lut_index = group * profile_.distance_bins + distance_idx;
                const float a = profile_.a_lut[lut_index];
                const float b = profile_.b_lut[lut_index];
                const float rho_sat = profile_.rho_sat_lut[lut_index];
                

                if (b <= 1e-6f) 
                {
                    pt.reflectivity = 0;
                    continue;
                }

                const float peak_sat = a + b * rho_sat;
                
                float raw_value = static_cast<float>(pt.reflectivity);

                raw_value = raw_value - profile_.intensity_bias[group];

                float normalized = 0.f;
                if (raw_value > peak_sat) 
                {
                    normalized = 1.f;
                } 
                else 
                {
                    normalized = (raw_value - a) / b;
                    normalized = clampValue(normalized, 0.f, 1.f);
                }
                const float scaled = normalized * 255.f;
                pt.reflectivity = static_cast<uint8_t>(clampValue(scaled, 0.f, 255.f));
            }

        };

        applyToPoints(frame.points_);
        if (frame.is_dual)
        {
            applyToPoints(frame.points2_);
        }
    }
private:
    bool profileValid() const {
        return profile_.a_lut != nullptr && profile_.b_lut != nullptr &&
           profile_.rho_sat_lut != nullptr && profile_.group_count > 0 &&
           profile_.distance_bins > 0;
    }

    ReflectivityCompensationProfile profile_;
};

// 基于PW的反射率补偿实现
class PWReflectivityCompensator final : public ReflectivityCompensator
{
public:

    explicit PWReflectivityCompensator(ReflectivityCompensationProfile profile)
        : profile_(profile) {}
    
    void apply(BlockBuffer& frame) const override 
    {

        if (!profileValid()) 
        {
            return;
        }
        

        const size_t expected_points = profile_.rows * profile_.cols;

        //匿名函数 参数是std::vector<RawPointT>& points 函数内部可以使用外面变量（用 & 捕获）
        auto applyToPoints = [&](std::vector<RawPointT>& points) {
            if (points.empty()) 
            {
                return;
            }

            const size_t count = expected_points > 0 ? std::min(points.size(), expected_points) : points.size();//count等于点云数量
            
            //开始反射率计算
            for (size_t i = 0; i < count; ++i)
            {
                RawPointT& pt = points[i];
                //这里group算是第几个LD
                size_t group = profile_.channel_group_size > 0 //channel_group_size 24
                           ? static_cast<size_t>(pt.ring / profile_.channel_group_size)
                           : 0;

                if (group >= profile_.group_count) //防溢出
                {
                    group = profile_.group_count - 1;
                }
                //distance是unit.EchoDistance; distance * 0.005 / 0.64
                float distance_m = profile_.distance_resolution > 0.f
                                        ? static_cast<size_t>(pt.distance * profile_.distance_resolution)
                                        : 0;
                size_t distance_idx = distance_m / 0.64f;

                if (distance_idx >= profile_.distance_bins) 
                {
                    distance_idx = profile_.distance_bins - 1;
                }
                
                const size_t lut_index = group * profile_.distance_bins + distance_idx;//这里展开成一维的数组索引

                //a是stepLow, b是stephigh, rho_sat_lut是pHigh

                const float stepLow = profile_.a_lut[lut_index];
                const float stephigh = profile_.b_lut[lut_index];
                const float pHigh = profile_.rho_sat_lut[lut_index];
                
                float intensity_f;
                float peak = static_cast<float>(pt.reflectivity);
                float width = static_cast<float>(pt.confidence);
                float pw = peak * width;
                
                if(pw >= static_cast<int>(pHigh))
                {
                    intensity_f = 120 + (pw - pHigh) * (stephigh);
                }
                else
                {
                    intensity_f = 120 - (pHigh - pw) * stepLow;
                }

                pt.reflectivity = static_cast<uint8_t>(clampValue(intensity_f, 0.f, 255.f));
            }

        };

        applyToPoints(frame.points_);
        if (frame.is_dual)
        {
            applyToPoints(frame.points2_);
        }
    }
private:
    bool profileValid() const {
        return profile_.a_lut != nullptr && profile_.b_lut != nullptr &&
           profile_.rho_sat_lut != nullptr && profile_.group_count > 0 &&
           profile_.distance_bins > 0;
    }

    ReflectivityCompensationProfile profile_;
};



} // namespace

std::shared_ptr<ReflectivityCompensator> makeTableCompensator(ReflectivityCompensationProfile profile)
{
    if (profile.a_lut == nullptr || profile.b_lut == nullptr ||
        profile.rho_sat_lut == nullptr || profile.group_count == 0 ||
        profile.distance_bins == 0) {
        return nullptr;
    }
    return std::make_shared<PWReflectivityCompensator>(profile);
}

} // namespace lidar
} // namespace asensing