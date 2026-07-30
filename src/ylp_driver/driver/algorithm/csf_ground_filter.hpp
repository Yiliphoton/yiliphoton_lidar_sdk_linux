#pragma once
#include <cstddef>
#include <cmath>
#include <unordered_map>
#include <vector>
#include "ylp_driver/common/ag_log.hpp"

namespace asensing
{
namespace lidar
{

// CSF 参数，可以根据场景微调
struct CSFParams
{
    float grid_resolution      = 0.5f;   // 网格分辨率 [m]，越小越精细但越慢
    float cloth_height         = 2.0f;   // 布初始高度（相对于最高点，反转坐标系后）
    float time_step            = 0.1f;   // 时间步长 dt
    float gravity              = -9.8f;  // 重力加速度（负值，向下）
    int   iterations           = 200;    // 迭代次数
    float smoothness           = 0.2f;   // 平滑系数，0~1，越大布越“软”
    float height_threshold     = 0.3f;   // 地面判定阈值（点相对于布的高度差）
};

template <typename PointT>
class CsfGroundFilter
{
public:
    //VoxelFilter(float leaf, Strategy strategy = Strategy::kMaxReflectivity) : leaf_size_(leaf), strategy_(strategy) {}
    CsfGroundFilter(CSFParams& params) : params_(params) {}
    void apply(std::vector<PointT> &points);

private:
    float get_rgb(float r, float g, float b);
    CSFParams& params_;
    std::vector<int> ground_indices;
    std::vector<int> nonground_indices;
};

template <typename PointT>
inline void CsfGroundFilter<PointT>::apply(std::vector<PointT> &points)
{
    // YLP_DEBUG << "params_.height_threshold: " << params_.height_threshold << YLP_REND;

    auto idx2D = [&](int ix, int iy, int nx)
    {
        return iy * nx + ix;
    };

    ground_indices.clear();
    nonground_indices.clear();
    if (points.empty())
        return;

    // 1. 计算点云在 XY 平面上的包围盒，并反转 Z 轴（z' = -z）
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();

    float max_inv_z = -std::numeric_limits<float>::max(); // 反转后的最大 z'

    std::vector<float> inv_z(points.size()); // 存储反转后的 z'

    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto& p = points[i];

        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;

        float z_inv = -p.z;  // 反转 z
        inv_z[i] = z_inv;
        if (z_inv > max_inv_z) max_inv_z = z_inv;
    }

    // 2. 构建布网格（XY 平面规则网格）
    float res = params_.grid_resolution;
    if (res <= 0.0f)
        res = 0.5f;

    int grid_cols = static_cast<int>(std::floor((max_x - min_x) / res)) + 1;
    int grid_rows = static_cast<int>(std::floor((max_y - min_y) / res)) + 1;

    if (grid_cols <= 0 || grid_rows <= 0)
        return;

    const int N = grid_cols * grid_rows;

    // 布节点高度 & 速度（在反转坐标系中）
    std::vector<float> cloth_z(N, 0.0f);
    std::vector<float> vel_z(N, 0.0f);

    // 每个格子的 "最高" 反转高度（对应地面，树木/建筑在反转后更低）
    const float NEG_INF = -std::numeric_limits<float>::max();
    std::vector<float> cell_max_inv_z(N, NEG_INF);

    // 每个点所属的格子索引（加速后面分类）
    std::vector<int> point_cell_idx(points.size(), -1);

    // 3. 统计每个网格 cell 内点的最大反转高度（作为潜在地面高度）
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto& p = points[i];
        int ix = static_cast<int>(std::floor((p.x - min_x) / res));
        int iy = static_cast<int>(std::floor((p.y - min_y) / res));

        if (ix < 0 || ix >= grid_cols || iy < 0 || iy >= grid_rows)
            continue;

        int id = idx2D(ix, iy, grid_cols);
        float z_inv = inv_z[i];
        if (z_inv > cell_max_inv_z[id])
        {
            cell_max_inv_z[id] = z_inv;
        }
        point_cell_idx[i] = id;
    }

    // 4. 初始化布高度：放在所有点之上的某个高度
    float cloth_init_z = max_inv_z + params_.cloth_height;
    for (int i = 0; i < N; ++i)
    {
        cloth_z[i] = cloth_init_z;
        vel_z[i]   = 0.0f;
    }

    // 5. 迭代进行布模拟：
    //    - 重力积分
    //    - 邻域平滑（模拟弹性约束）
    //    - 与 cell_max_inv_z 发生碰撞检测并校正
    const int iterations = std::max(1, params_.iterations);
    const float dt       = params_.time_step;
    const float g        = params_.gravity;
    const float lambda   = params_.smoothness; // 平滑系数

    for (int it = 0; it < iterations; ++it)
    {
        // 5.1 重力 & 速度积分
        for (int i = 0; i < N; ++i)
        {
            vel_z[i] += g * dt;         // v = v + g*dt
            cloth_z[i] += vel_z[i] * dt; // z = z + v*dt
        }

        // 5.2 邻域平滑（简单 4 邻域平均）
        std::vector<float> new_cloth_z = cloth_z;

        for (int iy = 0; iy < grid_rows; ++iy)
        {
            for (int ix = 0; ix < grid_cols; ++ix)
            {
                int id = idx2D(ix, iy, grid_cols);

                float sum = cloth_z[id];
                int count = 1;

                if (ix > 0)
                {
                    sum += cloth_z[idx2D(ix - 1, iy, grid_cols)];
                    ++count;
                }
                if (ix < grid_cols - 1)
                {
                    sum += cloth_z[idx2D(ix + 1, iy, grid_cols)];
                    ++count;
                }
                if (iy > 0)
                {
                    sum += cloth_z[idx2D(ix, iy - 1, grid_cols)];
                    ++count;
                }
                if (iy < grid_rows - 1)
                {
                    sum += cloth_z[idx2D(ix, iy + 1, grid_cols)];
                    ++count;
                }

                float avg = sum / static_cast<float>(count);
                // 当前位置向邻域平均值移动 lambda 比例
                new_cloth_z[id] = cloth_z[id] + lambda * (avg - cloth_z[id]);
            }
        }

        cloth_z.swap(new_cloth_z);

        // 5.3 碰撞检测：布不能穿过 cell_max_inv_z（如果该 cell 有点）
        for (int i = 0; i < N; ++i)
        {
            float z_surface = cell_max_inv_z[i];
            if (z_surface == NEG_INF)
                continue; // 该格子没有点，跳过

            if (cloth_z[i] < z_surface)
            {
                cloth_z[i] = z_surface;
                vel_z[i]   = 0.0f; // 碰撞后速度清零
            }
        }
    }

    // 6. 迭代结束后，cloth_z 即为反转坐标系下的地面近似
    //    对每个点，根据其所在格子的布高度进行 ground / non-ground 分类
    const float height_th = params_.height_threshold;

    for (size_t i = 0; i < points.size(); ++i)
    {
        int cell_id = point_cell_idx[i];
        if (cell_id < 0 || cell_id >= N)
        {
            // 安全起见，超出网格的点归为非地面
            nonground_indices.push_back(static_cast<int>(i));
            continue;
        }

        float z_surface = cell_max_inv_z[cell_id];
        if (z_surface == NEG_INF)
        {
            // 该格子本来就没统计到点，保守起见当成非地面
            nonground_indices.push_back(static_cast<int>(i));
            continue;
        }

        float cloth_z_here = cloth_z[cell_id];  // 反转坐标系下的布高度
        float z_inv        = inv_z[i];         // 反转坐标系下的点高度

        // 注意：反转坐标系下，地面点 z_inv ≈ cloth_z_here
        //       高于地面的物体点 z_inv 更小（更负）
        float diff = cloth_z_here - z_inv;  // 地面 - 点

        if (diff < height_th) // 点距离地面较近，认为是地面
        {
            ground_indices.push_back(static_cast<int>(i));
        }
        else
        {
            nonground_indices.push_back(static_cast<int>(i));
        }
    }

    for (int idx : ground_indices)
    {
        // YLP_DEBUG << "idx: " << idx << YLP_REND;
        points[idx].rgb = get_rgb(128.0f, 0.f, 255.f);
    }

}

template <typename PointT>
inline float CsfGroundFilter<PointT>::get_rgb(float r, float g, float b)
{
    // 将 RGB 值强制转换为 0-255 范围的 uint8_t 并确保其不会溢出
    uint8_t R = static_cast<uint8_t>(std::round(r));
    uint8_t G = static_cast<uint8_t>(std::round(g));
    uint8_t B = static_cast<uint8_t>(std::round(b));

    // 将 RGB 值组合成一个 uint32_t
    uint32_t v = (uint32_t(R) << 16) | (uint32_t(G) << 8) | uint32_t(B);

    // 使用 memcpy 将 uint32_t 转换为 float 类型
    float out;
    std::memcpy(&out, &v, sizeof(float));
    
    return out;
}

}
} // namespace
