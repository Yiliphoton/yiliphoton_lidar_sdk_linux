#pragma once
#include <cstddef>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace asensing
{
namespace lidar
{

template <typename PointT>
class VoxelFilter
{
public:
    enum class Strategy 
    {
        kLastPoint,        // 默认：最后一个点覆盖
        kMaxReflectivity   // 每个体素保留强度最高的点
    };

    VoxelFilter(float leaf, Strategy strategy = Strategy::kMaxReflectivity) : leaf_size_(leaf), strategy_(strategy) {}
    void apply(std::vector<PointT> &points)
    {
        if (points.empty() || leaf_size_ <= 0.f)
            return;

        struct Key
        {
            int x, y, z;
            bool operator==(const Key &other) const
            {
                return x == other.x && y == other.y && z == other.z;
            }
        };
        struct KeyHasher
        {
            std::size_t operator()(const Key &k) const
            {
                std::size_t h1 = std::hash<int>()(k.x);
                std::size_t h2 = std::hash<int>()(k.y);
                std::size_t h3 = std::hash<int>()(k.z);
                return h1 ^ (h2 << 1) ^ (h3 << 2);
            }
        };

        struct VoxelEntry {
            PointT point;
            float max_reflectivity;
        };

        // std::unordered_map<Key, PointT, KeyHasher> voxels;
        std::unordered_map<Key, VoxelEntry, KeyHasher> voxels;

        voxels.reserve(points.size());

        for (const auto &p : points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;
            Key key
            {
                static_cast<int>(std::floor(p.x / leaf_size_)),
                static_cast<int>(std::floor(p.y / leaf_size_)),
                static_cast<int>(std::floor(p.z / leaf_size_))
            };
            auto it = voxels.find(key);
            if (it == voxels.end())
            {
                voxels.emplace(key, VoxelEntry{p, p.intensity});
            }
            else if (strategy_ == Strategy::kMaxReflectivity)
            {
                if (p.intensity > it->second.max_reflectivity) 
                {
                    it->second.point = p;
                    it->second.max_reflectivity = p.intensity;
                }
            }
            else
            {
                 it->second.point = p;
            }
        }

        points.clear();
        points.reserve(voxels.size());
        for (auto& kv : voxels) {
            points.emplace_back(kv.second.point);
        }
    }

private:
    float leaf_size_ = 0.f;
    Strategy strategy_ = Strategy::kMaxReflectivity;
};

}
} // namespace
