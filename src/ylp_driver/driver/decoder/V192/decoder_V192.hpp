#pragma once

#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <sstream>
#include <thread>
#include <vector>
#include <climits>
#include <unistd.h>
#include "V192_struct.hpp"

using json = nlohmann::json;

namespace asensing
{
namespace lidar
{

template <typename T_PointCloud>
class DecoderV192 : public Decoder<T_PointCloud>
{
public:
    virtual ~DecoderV192() = default;
    explicit DecoderV192(const YLPDecoderParam &param);
    virtual void decodeDifopPkt(const uint8_t *pkt, size_t size);
    virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
    virtual bool processPointCloud(std::shared_ptr<T_PointCloud> cloud);
    void loadCalibration(const std::string &filename);

private:
    const std::string kDefaultCalibFile = "V192-Correction.json";
    const float kElevationStart = -12.48f;
    const float kElevationStep  =   0.13f;

    static YLPDecoderConstParam &getConstParam();

    bool fillPointCloud(std::shared_ptr<T_PointCloud> cloud,
                        std::vector<RawPointT> &buffer,
                        size_t width, size_t height,
                        uint32_t frameId, double z_plane);
    bool loadWalkErrorCalibration(const std::string &filename);
    float getWalkError(uint16_t chan, uint8_t confidence) const;
    float getReflectivityFromLut(uint16_t chan, float raw_reflectivity, uint8_t confidence, float distance_cm) const;

    SplitStrategyBySeq split_strategy_;
    float elevation_offset_[V192_CHANNELS_PER_BLOCK] = {0};
    float azimuth_offset_[V192_CHANNELS_PER_BLOCK] = {0};
    float elevation_mirror_offset_[V192_SIDES_OF_MIRROR] = {0};
    bool elevation_mirror_offset_enable_ = false;
    uint8_t LD_azimuth_offset_num_ = 0;
    std::vector<std::vector<float>> intensity_phighs_;
    std::vector<std::vector<float>> intensity_steplowks_;
    std::vector<std::vector<float>> reflectivity_lut_;
    std::vector<std::vector<float>> walkerror_lut_;
    std::shared_ptr<BlockBuffer> current_buffer_;
    uint32_t points_num_ = 0;

    A2AlgorithmParam algorithmParam_;

    size_t channel_group_size_ = 24;
    float lut_distance_resolution_ = 0.01f;
    size_t lut_group_count_ = 0;
    size_t lut_distance_bins_ = 0;
    std::vector<float> a_lut_;
    std::vector<float> b_lut_;
    std::vector<float> rho_sat_lut_;
    std::vector<int> intensity_bias_;
};

template <typename T_PointCloud>
inline DecoderV192<T_PointCloud>::DecoderV192(const YLPDecoderParam &param)
    : Decoder<T_PointCloud>(getConstParam(), param)
{
    this->packet_duration_ = 0.1 / V192_POINTS_PER_ROW;

    if (this->param_.calib_file.empty())
        this->param_.calib_file = kDefaultCalibFile;

    loadCalibration(this->param_.calib_file);
    YLP_MSG << "[V192] calib_file=" << this->param_.calib_file
            << " IsCalibrated=" << this->IsCalibrated << YLP_REND;
    YLP_MSG << "[V192] elevation[0]=" << elevation_offset_[0]
            << " elevation[95]=" << elevation_offset_[95]
            << " elevation[191]=" << elevation_offset_[191] << YLP_REND;
    this->filter_ = std::make_shared<SoftwareFilter>(algorithmParam_);

    if (this->param_.enable_fov_aligned)
        YLP_ERROR << "V192: enable_fov_aligned must be false" << YLP_REND;

    current_buffer_ = this->blockGet(V192_POINTS_PER_ROW, V192_CHANNELS_PER_BLOCK, true);
    current_buffer_->is_aligned = false;
    current_buffer_->points_offset = LD_azimuth_offset_num_;
    current_buffer_->is_dual = true;
    this->echo_mode_ = ECHO_DUAL;

    ReflectivityCompensationProfile profile;
    profile.channel_group_size = channel_group_size_;
    profile.distance_resolution = lut_distance_resolution_;
    profile.rows = V192_POINTS_PER_ROW;
    profile.cols = V192_CHANNELS_PER_BLOCK;
    profile.group_count = lut_group_count_;
    profile.distance_bins = lut_distance_bins_;
    profile.a_lut = a_lut_.data();
    profile.b_lut = b_lut_.data();
    profile.rho_sat_lut = rho_sat_lut_.data();
    profile.intensity_bias = intensity_bias_.data();
    this->setReflectivityCompensator(makeTableCompensator(profile));
}

template <typename T_PointCloud>
inline YLPDecoderConstParam &DecoderV192<T_PointCloud>::getConstParam()
{
    static YLPDecoderConstParam param = {
        806,   // MSOP_LEN
        1216,  // MSOP2_LEN
        108,   // DIFOP_LEN (unused for V192)
        2, 4,  // MSOP_ID_LEN, DIFOP_ID_LEN
        {0x55, 0xC0}, {0xA5, 0xFF, 0x00, 0x5A},
        192, 1, V192_CHANNELS_PER_BLOCK,
        0.0f, 300.0f, 0.005f, 80.0f
    };
    return param;
}

template <typename T_PointCloud>
inline void DecoderV192<T_PointCloud>::decodeDifopPkt(const uint8_t* /*packet*/, size_t /*size*/)
{
    // V192 has no DIFOP packet
}

template <typename T_PointCloud>
inline bool DecoderV192<T_PointCloud>::decodeMsopPkt(const uint8_t *packet, size_t /*size*/)
{
    bool ret = false;

    const V192MsopHeader &header = *(const V192MsopHeader *)packet;
    const V192MsopPkt &msoppkt = *(const V192MsopPkt *)packet;

    // Use host clock (lidar clock not supported)
    double pkt_ts = getTimeHostWithAG();

    uint16_t frame_id = header.FrameID;
    current_buffer_->frame_id = frame_id;

    if (split_strategy_.newPacket(frame_id))
    {
        points_num_ = 0;

        this->point_cloud_->data = current_buffer_->data;
        this->blockPut(current_buffer_, true);

        current_buffer_ = this->blockGet(V192_POINTS_PER_ROW, V192_CHANNELS_PER_BLOCK, true);
        current_buffer_->is_aligned = false;
        current_buffer_->points_offset = LD_azimuth_offset_num_;

        std::shared_ptr<T_PointCloud> cloud = this->point_cloud_;
        if (this->param_.process_by_thread)
            std::thread([this, cloud]{ processPointCloud(cloud); }).detach();
        else
            processPointCloud(cloud);

        this->cb_split_frame_(192, this->cloudTs());
        this->first_point_ts_ = pkt_ts;
        ret = true;
    }

    std::vector<RawPointT> &buffer  = current_buffer_->points_;
    std::vector<RawPointT> &buffer2 = current_buffer_->points2_;

    for (uint16_t blk = 0; blk < header.BlockNum; blk++)
    {
        for (uint16_t chan = 0; chan < 192; chan++)
        {
            points_num_++;

            const V192MsopChannel &unit = msoppkt.blocks[blk].channel[chan];
            uint32_t pos = header.SlotNum * V192_CHANNELS_PER_BLOCK + chan;

            if (pos >= buffer.size())
                continue;

            const uint8_t intensity = unit.EchoIntensity;
            const float azimuth = (header.AzimuthAngle * V192_AZIMUTH_UNIT + azimuth_offset_[chan]) * 100.0f;
            const float elevation = elevation_offset_[chan] * 100.0f;
            const float walkerror = getWalkError(chan, unit.EchoConfidence);
            const float corrected_distance = unit.EchoDistance * 0.5f + walkerror;
            const float corrected_reflectivity = getReflectivityFromLut(chan, static_cast<float>(intensity), unit.EchoConfidence, corrected_distance);

            if (header.ReturnMode == 0)
            {
                buffer[pos].distance    = corrected_distance;
                buffer[pos].azimuth     = azimuth;
                buffer[pos].elevation   = elevation;
                buffer[pos].reflectivity = corrected_reflectivity;
                buffer[pos].confidence  = unit.EchoConfidence;
                buffer[pos].timestamp   = header.Timestamp;
                buffer[pos].ring        = chan;
            }
            else
            {
                buffer2[pos].distance    = corrected_distance;
                buffer2[pos].azimuth     = azimuth;
                buffer2[pos].elevation   = elevation;
                buffer2[pos].reflectivity = corrected_reflectivity;
                buffer2[pos].confidence  = unit.EchoConfidence;
                buffer2[pos].timestamp   = header.Timestamp;
                buffer2[pos].ring        = chan;
            }

        }
    }

    this->prev_pkt_ts_ = pkt_ts;
    return ret;
}

template <typename T_PointCloud>
inline bool DecoderV192<T_PointCloud>::processPointCloud(std::shared_ptr<T_PointCloud> cloud)
{
    std::shared_ptr<BlockBuffer> block = this->block_queue_.popWait();
    if (!block)
        return false;

    std::vector<RawPointT> &buffer  = block->points_;
    std::vector<RawPointT> &buffer2 = block->points2_;
    double z_plane = 0.0;

    this->filter_->filter(block, 0, buffer.size(), z_plane);
    this->filter_->compensation(block, 0, buffer.size(), this->param_.enable_fov_aligned);

    fillPointCloud(cloud, buffer,  block->width, block->height, block->frame_id,
                   z_plane * this->const_param_.DISTANCE_RES);
    if (block->is_dual)
        fillPointCloud(cloud, buffer2, block->width, block->height, block->frame_id,
                       z_plane * this->const_param_.DISTANCE_RES);

    cloud->is_dual  = block->is_dual;
    cloud->frame_id = block->frame_id;

    this->blockPut(block, false);
    this->cb_finish_(cloud);
    return true;
}

template <typename T_PointCloud>
inline bool DecoderV192<T_PointCloud>::fillPointCloud(std::shared_ptr<T_PointCloud> cloud,
    std::vector<RawPointT> &buffer, size_t width, size_t height,
    uint32_t /*frameId*/, double z_plane)
{
    if (buffer.empty())
        return false;

    for (uint32_t pos = 0; pos < width * height; pos++)
    {
        RawPointT &rawPoint = buffer[pos];
        float distance  = rawPoint.distance * this->const_param_.DISTANCE_RES * 2.0f; // undo *0.5 scale
        float azimuth   = rawPoint.azimuth;
        float elevation = rawPoint.elevation;

        if (algorithmParam_.pointcloud_filter.enable && algorithmParam_.plane_ghost_filter.enable)
        {
            if (distance * SIN(elevation) < z_plane - algorithmParam_.plane_ghost_filter.z_plane_threshold)
                distance = 0;
        }

        typename T_PointCloud::PointT point;
        if (this->distance_section_.in(distance))
        {
            float x = distance * COS(elevation) * COS(azimuth);
            float y = distance * COS(elevation) * SIN(azimuth);
            float z = distance * SIN(elevation);
            // rotate coordinate frame
            std::swap(x, y); y = -y;
            this->transformPoint(x, y, z);

            setX(point, x); setY(point, y); setZ(point, z);
            setIntensity(point, rawPoint.reflectivity);
            setTimestamp(point, rawPoint.timestamp);
            setRing(point, rawPoint.ring);
            setRange(point, rawPoint.distance);
            setConfidence(point, static_cast<float>(rawPoint.confidence));
#ifdef POINT_TYPE_XYZIRTC
            setRGB(point, this->intensityToPackedRGB(rawPoint.reflectivity));
#endif
        }
        else
        {
            setX(point, NAN); setY(point, NAN); setZ(point, NAN);
            setIntensity(point, 0);
            setTimestamp(point, 0);
            setRing(point, rawPoint.ring);
            setRange(point, 0);
            setConfidence(point, 0.0f);
        }
        cloud->points.emplace_back(point);
    }
    return true;
}

template <typename T_PointCloud>
inline bool DecoderV192<T_PointCloud>::loadWalkErrorCalibration(const std::string &filename)
{
    walkerror_lut_.clear();
    if (filename.empty())
        return false;

    std::ifstream f(filename.c_str());
    if (!f.is_open())
    {
        YLP_WARNING << "Cannot open walkerror file: " << filename << YLP_REND;
        return false;
    }

    auto trim = [](std::string s) {
        const char *ws = " \t\r\n{}[]";
        const auto beg = s.find_first_not_of(ws);
        if (beg == std::string::npos) return std::string();
        const auto end = s.find_last_not_of(ws);
        return s.substr(beg, end - beg + 1);
    };

    std::string line;
    while (std::getline(f, line))
    {
        std::vector<float> row;
        std::stringstream ss(line);
        std::string token;
        while (std::getline(ss, token, ','))
        {
            token = trim(token);
            if (token.empty())
                continue;
            try
            {
                row.push_back(std::stof(token));
            }
            catch (...)
            {
            }
        }
        if (!row.empty())
            walkerror_lut_.push_back(row);
    }

    if (walkerror_lut_.empty())
    {
        YLP_WARNING << "Empty walkerror calibration file: " << filename << YLP_REND;
        return false;
    }

    const size_t cols = walkerror_lut_.front().size();
    for (const auto &row : walkerror_lut_)
    {
        if (row.size() != cols)
        {
            YLP_WARNING << "Invalid walkerror calibration shape: " << filename << YLP_REND;
            walkerror_lut_.clear();
            return false;
        }
    }

    YLP_MSG << "[V192] walkerror_file=" << filename
            << " rows=" << walkerror_lut_.size()
            << " cols=" << cols << YLP_REND;
    return true;
}

template <typename T_PointCloud>
inline float DecoderV192<T_PointCloud>::getWalkError(uint16_t chan, uint8_t confidence) const
{
    if (walkerror_lut_.empty())
        return 0.0f;

    const size_t row_idx = std::min<size_t>(chan, walkerror_lut_.size() - 1);
    const auto &row = walkerror_lut_[row_idx];
    if (row.empty())
        return 0.0f;

    const size_t bin = std::min<size_t>(confidence, row.size() - 1);
    return row[bin];
}

template <typename T_PointCloud>
inline float DecoderV192<T_PointCloud>::getReflectivityFromLut(uint16_t chan, float raw_reflectivity, uint8_t confidence, float distance_cm) const
{
    if (reflectivity_lut_.size() < 24)
        return raw_reflectivity;

    const size_t ld = std::min<size_t>(chan / 24, 7);
    const size_t dist_bin = std::min<size_t>(std::max<int>(0, static_cast<int>(distance_cm / 64.0f)), 127);

    const auto &pHighRows = reflectivity_lut_;
    constexpr size_t low_k_offset  = 8;
    constexpr size_t high_k_offset = 16;

    if (ld >= pHighRows.size() ||
        low_k_offset + ld >= reflectivity_lut_.size() ||
        high_k_offset + ld >= reflectivity_lut_.size())
        return raw_reflectivity;

    if (dist_bin >= reflectivity_lut_[ld].size() ||
        dist_bin >= reflectivity_lut_[low_k_offset + ld].size() ||
        dist_bin >= reflectivity_lut_[high_k_offset + ld].size())
        return raw_reflectivity;

    const float pHigh = reflectivity_lut_[ld][dist_bin];
    const float steplowK = reflectivity_lut_[low_k_offset + ld][dist_bin];
    const float stephighK = reflectivity_lut_[high_k_offset + ld][dist_bin];
    const float pw = raw_reflectivity * static_cast<float>(confidence);
    const float pLow = pHigh - 60.0f / std::max(steplowK, 1e-6f);

    float reflectivity = 0.0f;
    if (pw <= pHigh)
        reflectivity = 60.0f + (pw - pLow) * steplowK;
    else
        reflectivity = 120.0f + (pw - pHigh) * stephighK;

    return std::min(255.0f, std::max(0.0f, reflectivity));
}

template <typename T_PointCloud>
inline void DecoderV192<T_PointCloud>::loadCalibration(const std::string &filename)
{
    if (filename.empty() || filename.find("V192") == std::string::npos)
    {
        this->IsCalibrated = false;
        this->CalibEnabled = false;
        return;
    }

    // Build search path: caller can set calib_file to an absolute path or just a filename.
    // If just a filename, look next to the executable (or caller can set an absolute path).
    std::ifstream f(filename.c_str());
    if (!f.is_open())
    {
        // Try next to the running executable (Linux: /proc/self/exe)
        char buf[PATH_MAX];
        ssize_t len = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
        if (len > 0)
        {
            buf[len] = '\0';
            std::string exe_dir(buf, static_cast<std::size_t>(::strrchr(buf, '/') + 1 - buf));
            std::string basename = filename.substr(filename.rfind('/') + 1);
            f.open(exe_dir + basename);
        }
    }
    if (!f.is_open())
    {
        YLP_WARNING << "Cannot open calibration file: " << filename << YLP_REND;
        this->IsCalibrated = false;
        this->CalibEnabled = true;
        return;
    }

    json data = json::parse(f);
    f.close();

    auto reset_reflectivity_lut = [&]() {
        lut_group_count_ = 0; lut_distance_bins_ = 0;
        channel_group_size_ = 24; lut_distance_resolution_ = 0.01f;
        a_lut_.clear(); b_lut_.clear(); rho_sat_lut_.clear(); intensity_bias_.clear();
    };

    auto reset_walkerror_lut = [&]() {
        walkerror_lut_.clear();
    };

    auto loadTxtTable = [](const std::string &filename, std::vector<std::vector<float>> &dst) -> bool {
        dst.clear();
        std::ifstream f(filename.c_str());
        if (!f.is_open())
            return false;

        auto trim = [](std::string s) {
            const char *ws = " \t\r\n{}[]";
            const auto beg = s.find_first_not_of(ws);
            if (beg == std::string::npos) return std::string();
            const auto end = s.find_last_not_of(ws);
            return s.substr(beg, end - beg + 1);
        };

        std::string line;
        while (std::getline(f, line))
        {
            std::vector<float> row;
            std::stringstream ss(line);
            std::string token;
            while (std::getline(ss, token, ','))
            {
                token = trim(token);
                if (token.empty())
                    continue;
                try
                {
                    row.push_back(std::stof(token));
                }
                catch (...)
                {
                }
            }
            if (!row.empty())
                dst.push_back(row);
        }
        return !dst.empty();
    };

    reset_reflectivity_lut();
    reset_walkerror_lut();

    

    if (data.contains("reflectivity_lut") && data["reflectivity_lut"].is_object())
    {
        const json &lut = data["reflectivity_lut"];
        size_t groups = 0, bins = 0;

        auto flatten = [&](const json &node, std::vector<float> &dst) -> bool {
            if (!node.is_array() || node.empty()) return false;
            const size_t lg = node.size();
            const json &row0 = node.at(0);
            if (!row0.is_array() || row0.empty()) return false;
            const size_t lb = row0.size();
            dst.resize(lg * lb);
            for (size_t g = 0; g < lg; ++g) {
                if (!node[g].is_array() || node[g].size() != lb) return false;
                for (size_t d = 0; d < lb; ++d)
                    dst[g * lb + d] = node[g][d].get<float>();
            }
            if (groups == 0 && bins == 0) { groups = lg; bins = lb; }
            else if (groups != lg || bins != lb) return false;
            return true;
        };

        bool lut_ok = lut.contains("a") && lut.contains("b") && lut.contains("rho_sat_min") &&
                      flatten(lut["a"], a_lut_) &&
                      flatten(lut["b"], b_lut_) &&
                      flatten(lut["rho_sat_min"], rho_sat_lut_);

        if (lut_ok)
        {
            const size_t eg = lut.value("group_count",    static_cast<size_t>(groups));
            const size_t eb = lut.value("distance_bins",  static_cast<size_t>(bins));
            if ((eg != 0 && eg != groups) || (eb != 0 && eb != bins))
                lut_ok = false;
        }

        if (lut_ok)
        {
            lut_group_count_ = groups; lut_distance_bins_ = bins;
            channel_group_size_ = lut.value("channel_group_size", channel_group_size_);
            if (channel_group_size_ == 0) channel_group_size_ = 1;
            lut_distance_resolution_ = lut.value("distance_resolution", lut_distance_resolution_);
            if (lut_distance_resolution_ <= 0.0f) lut_distance_resolution_ = 0.01f;

            intensity_bias_.assign(lut_group_count_, 0);
            if (lut.contains("intensity_bias") && lut["intensity_bias"].is_array())
            {
                const json &bias = lut["intensity_bias"];
                for (size_t g = 0; g < std::min(bias.size(), intensity_bias_.size()); ++g)
                    intensity_bias_[g] = bias[g].get<int>();
            }
        }
        else
        {
            YLP_WARNING << "Invalid reflectivity_lut in calibration file" << YLP_REND;
            reset_reflectivity_lut();
        }
    }

    if (data["elevation"].is_array() && data["elevation"].size() == V192_CHANNELS_PER_BLOCK)
    {
        for (int i = 0; i < V192_CHANNELS_PER_BLOCK; i++)
            elevation_offset_[i] = data["elevation"][i];
        this->IsCalibrated = true;
    }
    else
    {
        for (int i = 0; i < V192_CHANNELS_PER_BLOCK; i++)
            elevation_offset_[i] = kElevationStart + kElevationStep * i;
        this->IsCalibrated = false;
    }

    if (data["elevation_mirror_offset"].is_array() &&
        data["elevation_mirror_offset"].size() == V192_SIDES_OF_MIRROR)
    {
        for (int i = 0; i < V192_SIDES_OF_MIRROR; i++)
            elevation_mirror_offset_[i] = data["elevation_mirror_offset"][i];
        this->IsCalibrated = true;
    }

    if (data["elevation_mirror_offset_enable"].is_boolean())
        elevation_mirror_offset_enable_ = data["elevation_mirror_offset_enable"];

    if (data["LD_azimuth_offset_num"].is_number_unsigned())
        LD_azimuth_offset_num_ = data["LD_azimuth_offset_num"];

    if (data["LD_azimuth_offset"].is_array() &&
        data["LD_azimuth_offset"].size() == V192_CHANNELS_PER_BLOCK)
    {
        for (int i = 0; i < V192_CHANNELS_PER_BLOCK; i++)
            azimuth_offset_[i] = data["LD_azimuth_offset"][i];
        this->IsCalibrated = true;
    }

    if (data.contains("walkerror_file") && data["walkerror_file"].is_string())
        loadWalkErrorCalibration(data["walkerror_file"].get<std::string>());

    auto loadNestedFloatArray = [&](const char *key, std::vector<std::vector<float>> &dst) {
        if (!data.contains(key) || !data[key].is_array()) return;
        const auto &arr = data[key];
        dst.resize(arr.size());
        for (size_t i = 0; i < arr.size(); i++) {
            if (!arr[i].is_array()) continue;
            dst[i].resize(arr[i].size());
            for (size_t j = 0; j < arr[i].size(); j++)
                dst[i][j] = arr[i][j].get<float>();
        }
        this->IsCalibrated = true;
    };
    loadNestedFloatArray("intensity_phighs",    intensity_phighs_);
    loadNestedFloatArray("intensity_steplowks", intensity_steplowks_);

    if (data["algorithm"].is_object())
    {
        const json &alg = data["algorithm"];
        const json &cr  = alg["crosstalk_filter"];
        const json &nf  = alg["point_cloud_filter"];

        algorithmParam_.crosstalk_filter.enable                      = cr["enable"];
        algorithmParam_.crosstalk_filter.emission_num                = cr["emission_num"];
        algorithmParam_.crosstalk_filter.algo_version_cr             = cr["algo_version_cr"];
        algorithmParam_.crosstalk_filter.href_pos_th                 = cr["href_pos_th"];
        algorithmParam_.crosstalk_filter.crosstalk_dis_th            = cr["crosstalk_dis_th"];
        algorithmParam_.crosstalk_filter.abnorm_width_th             = cr["abnorm_width_th"];
        algorithmParam_.crosstalk_filter.echo_select                 = cr["echo_select"].get<std::vector<unsigned int>>();
        algorithmParam_.crosstalk_filter.peak_select_flag            = cr["peak_select_flag"];
        algorithmParam_.crosstalk_filter.col_helper_flag             = cr["col_helper_flag"];
        algorithmParam_.crosstalk_filter.dy_pw_refer_flag            = cr["dy_pw_refer_flag"];
        algorithmParam_.crosstalk_filter.high_out_flag               = cr["high_out_flag"];
        algorithmParam_.crosstalk_filter.key_dis                     = cr["key_dis"].get<std::vector<double>>();
        algorithmParam_.crosstalk_filter.thres_func_param            = cr["thres_func_param"];
        algorithmParam_.crosstalk_filter.ratio_high                  = cr["ratio_high"];
        algorithmParam_.crosstalk_filter.ratio_low                   = cr["ratio_low"];
        algorithmParam_.crosstalk_filter.neibor_key_dis              = cr["neibor_key_dis"].get<std::vector<double>>();
        algorithmParam_.crosstalk_filter.neibor_value_param          = cr["neibor_value_param"];
        algorithmParam_.crosstalk_filter.echo_option                 = cr["echo_option"];
        algorithmParam_.crosstalk_filter.refer_option                = cr["refer_option"];
        algorithmParam_.crosstalk_filter.pw_define_option            = cr["pw_define_option"];
        algorithmParam_.crosstalk_filter.use_path_protect_flag       = cr["use_path_protect_flag"];
        algorithmParam_.crosstalk_filter.use_path_filter_flag        = cr["use_path_filter_flag"];
        algorithmParam_.crosstalk_filter.use_path_filter_dis         = cr["use_path_filter_dis"];
        algorithmParam_.crosstalk_filter.use_path_protect_dis        = cr["use_path_protect_dis"];
        algorithmParam_.crosstalk_filter.fake_high_filter_flag       = cr["fake_high_filter_flag"];
        algorithmParam_.crosstalk_filter.fake_high_filter_hight      = cr["fake_high_filter_hight"];
        algorithmParam_.crosstalk_filter.fake_high_filter_harder_flag  = cr["fake_high_filter_harder_flag"];
        algorithmParam_.crosstalk_filter.fake_high_filter_harder_hight = cr["fake_high_filter_harder_hight"];
        algorithmParam_.crosstalk_filter.fake_path_filter_flag       = cr["fake_path_filter_flag"];
        algorithmParam_.crosstalk_filter.fake_down_pixel_num         = cr["fake_down_pixel_num"];
        algorithmParam_.crosstalk_filter.fake_use_ld_all_flag        = cr["fake_use_ld_all_flag"];
        algorithmParam_.crosstalk_filter.low_ref_set_value           = cr["low_ref_set_value"];
        algorithmParam_.crosstalk_filter.low_ref_set_flag            = cr["low_ref_set_flag"];
        algorithmParam_.crosstalk_filter.gf_scale_max_height         = cr["gf_scale_max_height"];
        algorithmParam_.crosstalk_filter.need_valid_neighbor_num     = cr["need_valid_neighbor_num"];
        algorithmParam_.crosstalk_filter.gf_scale_assist_flag        = cr["gf_scale_assist_flag"];
        algorithmParam_.crosstalk_filter.peak_filter_flag            = cr["peak_filter_flag"];
        algorithmParam_.crosstalk_filter.peak_filter_float_assist_flag = cr["peak_filter_float_assist_flag"];
        algorithmParam_.crosstalk_filter.peak_thres_offset           = cr["peak_thres_offset"];

        algorithmParam_.pointcloud_filter.enable                     = nf["enable"];
        algorithmParam_.pointcloud_filter.filterFlag                 = nf["filter_flag"];
        algorithmParam_.pointcloud_filter.groundProtectFlag          = nf["ground_protect_flag"];
        algorithmParam_.pointcloud_filter.ground_angle_threshold     = nf["ground_angle_threshold"];
        algorithmParam_.pointcloud_filter.gHIdx                      = nf["gHIdx"];
        algorithmParam_.pointcloud_filter.gLIdx                      = nf["gLIdx"];
        algorithmParam_.pointcloud_filter.diffIdx                    = nf["diffIdx"];
        algorithmParam_.pointcloud_filter.dis_fixed                  = nf["dis_fixed"];
        algorithmParam_.pointcloud_filter.dis_thread_fixed           = nf["dis_thread_fixed"];
        algorithmParam_.pointcloud_filter.dis_delta                  = nf["dis_delta"];
        algorithmParam_.pointcloud_filter.dis_k                      = nf["dis_k"];
        algorithmParam_.pointcloud_filter.dis_thread_dy              = nf["dis_thread_dy"];
        algorithmParam_.pointcloud_filter.num_thread_dy              = nf["num_thread_dy"];
        algorithmParam_.pointcloud_filter.ground_protect_dis         = nf["ground_protect_dis"];
        algorithmParam_.pointcloud_filter.sensorMountAngle           = nf["sensorMountAngle"];
        algorithmParam_.pointcloud_filter.dis_seg                    = nf["dis_seg"].get<std::vector<double>>();
        algorithmParam_.pointcloud_filter.thres_func_param           = nf["thres_func_param"];
        algorithmParam_.pointcloud_filter.smooth_flag                = nf["smooth_flag"];
        algorithmParam_.pointcloud_filter.fix_ground_walkerror_flag  = nf["fix_ground_walkerror_flag"];
        algorithmParam_.pointcloud_filter.check_neibor_num           = nf["check_neibor_num"];
        algorithmParam_.pointcloud_filter.neibor_interval            = nf["neibor_interval"];
        algorithmParam_.pointcloud_filter.valid_num_thres            = nf["valid_num_thres"];
        algorithmParam_.pointcloud_filter.close_dis_thres            = nf["close_dis_thres"];
        algorithmParam_.pointcloud_filter.algo_version_nf            = nf["algo_version_nf"];
        algorithmParam_.pointcloud_filter.groundCompensationFlag     = nf["groundCompensationFlag"];
        algorithmParam_.pointcloud_filter.groundDisThr               = nf["groundDisThr"];
        algorithmParam_.pointcloud_filter.groundCompensationThr      = nf["groundCompensationThr"];
        algorithmParam_.pointcloud_filter.ground_unprotect_dis       = nf["ground_unprotect_dis"];
        algorithmParam_.pointcloud_filter.inter_process_flag         = nf["inter_process_flag"];
        algorithmParam_.pointcloud_filter.inter_dis_thres            = nf["inter_dis_thres"];
        algorithmParam_.pointcloud_filter.planar_assist_filter_flag  = nf["planar_assist_filter_flag"];
        algorithmParam_.pointcloud_filter.planar_assist_thres        = nf["planar_assist_thres"];
        algorithmParam_.pointcloud_filter.color_debug                = nf["color_debug"];
        algorithmParam_.pointcloud_filter.ground_filter_flag         = nf["ground_filter_flag"];
        algorithmParam_.pointcloud_filter.ground_filter_params       = nf["ground_filter_params"];
        algorithmParam_.pointcloud_filter.ref_calib_flag             = nf["ref_calib_flag"];
        algorithmParam_.pointcloud_filter.ref_set_params             = nf["ref_set_params"].get<std::vector<int>>();

        const json &gc = alg["ghost_cleaning"];
        algorithmParam_.plane_ghost_filter.enable          = gc["enable"];
        algorithmParam_.plane_ghost_filter.z_plane_threshold = gc["z_plane_threshold"];

        const json &bl = alg["blind_line_filter"];
        algorithmParam_.blind_line_filter.enable        = bl["enable"];
        algorithmParam_.blind_line_filter.blind_line_dis = bl["blind_line_dis"];

        const json &dp = alg["dynamic_power_compensation"];
        algorithmParam_.dynamic_power.enable                   = dp["enable"];
        algorithmParam_.dynamic_power.distance_threshold_start = dp["distance_threshold_start"];
        algorithmParam_.dynamic_power.distance_threshold_end   = dp["distance_threshold_end"];
        algorithmParam_.dynamic_power.k = dp["k"];
        algorithmParam_.dynamic_power.b = dp["b"];
        algorithmParam_.dynamic_power.d = dp["d"];

        const json &ba = alg["blind_area_filter"];
        algorithmParam_.blind_area_filter.enable       = ba["enable"];
        algorithmParam_.blind_area_filter.azimuth0     = ba["azimuth0"];
        algorithmParam_.blind_area_filter.azimuth1     = ba["azimuth1"];
        algorithmParam_.blind_area_filter.azimuth2     = ba["azimuth2"];
        algorithmParam_.blind_area_filter.azimuth3     = ba["azimuth3"];
        algorithmParam_.blind_area_filter.azimuth4     = ba["azimuth4"];
        algorithmParam_.blind_area_filter.minDisInvalid = ba["minDisInvalid"];
        algorithmParam_.blind_area_filter.maxDisInvalid = ba["maxDisInvalid"];
        algorithmParam_.blind_area_filter.windowStep   = ba["windowStep"];
        algorithmParam_.blind_area_filter.windowWidth  = ba["windowWidth"];
        algorithmParam_.blind_area_filter.windowHeight = ba["windowHeight"];

        const json &ef = alg["expansion_filter"];
        algorithmParam_.expansion_filter.enable     = ef["enable"];
        algorithmParam_.expansion_filter.highRefThr = ef["highRefThr"];
        algorithmParam_.expansion_filter.step       = ef["step"];
        algorithmParam_.expansion_filter.nearThr    = ef["nearThr"];
        algorithmParam_.expansion_filter.weakRefThr = ef["weakRefThr"];

        const json &tf = alg["trailing_filter"];
        algorithmParam_.trailing_filter.enable = tf["enable"];
        algorithmParam_.trailing_filter.dthr0  = tf["dthr0"];
        algorithmParam_.trailing_filter.dthr1  = tf["dthr1"];
        algorithmParam_.trailing_filter.dthr2  = tf["dthr2"];
        algorithmParam_.trailing_filter.dthr4  = tf["dthr4"];
        algorithmParam_.trailing_filter.phithr = tf["phithr"];
        algorithmParam_.trailing_filter.kslop  = tf["kslop"];

        algorithmParam_.print();
    }
}

} // namespace lidar
} // namespace asensing
