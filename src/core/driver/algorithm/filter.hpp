#pragma once

#include <cstdint>
#include <memory>
#include "core/common/log.hpp"
#include "core/common/error_code.hpp"
#include "core/common/block_buffer.hpp"
#include "core/driver/decoder/trigon.hpp"

namespace lidar_sdk
{
namespace lidar
{

// BlockBuffer, RawPointT, Trigon etc. are in lidar_sdk::lidar; bring them in.




struct A0AlgorithmParam
{
  int delt_threshold = 15;
  int num_threshold = 10;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             A0 Algorithm Parameters " << LOG_REND;
    LOG_INFOL << "delt_threshold: " << delt_threshold << LOG_REND;
    LOG_INFOL << "num_threshold: " << num_threshold << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct A2PointCloudFilterParam
{
  bool enable = false;
  bool filterFlag = true;
  bool groundProtectFlag = true;
  uint8_t ground_angle_threshold = 5;
  uint8_t gHIdx = 60; // 43
  uint8_t gLIdx = 0;  // 20
  uint8_t diffIdx = 12;
  uint16_t dis_fixed = 3000;
  uint16_t dis_thread_fixed = 30;
  uint16_t dis_delta = 3000;
  uint16_t dis_k = 10;
  uint16_t dis_thread_dy = 30;
  uint8_t num_thread_dy = 4;
  uint16_t ground_protect_dis = 6000;
  uint16_t sensorMountAngle = 0;
  std::vector<double> dis_seg = {3000, 50000};
  std::vector<std::vector<double>> thres_func_param = {{30, 0, 3000, 3}, {30, 10, 3000, 3}};
  float groundDisThr = 7.0 / 30;
  uint16_t groundCompensationThr = 600;
  uint8_t groundCompensationFlag = 2; //0:不补偿  1:补偿一个通道  2:补偿两个通道

  unsigned int algo_version_nf = 1;
  bool smooth_flag = true;
  bool fix_ground_walkerror_flag = true;
  int check_neibor_num = 3;
  int neibor_interval = 1;
  unsigned int valid_num_thres = 5;
  unsigned int close_dis_thres = 50;

  bool inter_process_flag = true;
  unsigned int inter_dis_thres;
  bool planar_assist_filter_flag = true;
  float planar_assist_thres = 0.05;
  bool color_debug = false;
  bool ground_filter_flag = false;
  std::vector<std::vector<double>> ground_filter_params;
  uint16_t ground_unprotect_dis = 1000;

  bool ref_calib_flag = false;
  std::vector<std::vector<double>> ref_calib_params;
  std::vector<int> ref_set_params = {10, 210, 240};

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             PointCloud Filter Parameters " << LOG_REND;
    LOG_INFOL << "filterFlag: " << filterFlag << LOG_REND;
    LOG_INFOL << "groundProtectFlag: " << groundProtectFlag << LOG_REND;
    LOG_INFOL << "ground_angle_threshold: "   << (int)ground_angle_threshold << LOG_REND;
    LOG_INFOL << "gHIdx: "   << (int)gHIdx << LOG_REND;
    LOG_INFOL << "gLIdx: "   << (int)gLIdx << LOG_REND;
    LOG_INFOL << "diffIdx: "   << (int)diffIdx << LOG_REND;
    LOG_INFOL << "dis_fixed: "   << dis_fixed << LOG_REND;
    LOG_INFOL << "dis_thread_fixed: "   << dis_thread_fixed << LOG_REND;
    LOG_INFOL << "dis_delta: "   << dis_delta << LOG_REND;
    LOG_INFOL << "dis_k: "   << dis_k << LOG_REND;
    LOG_INFOL << "dis_thread_dy: "   << dis_thread_dy << LOG_REND;
    LOG_INFOL << "num_thread_dy: "   << (int)num_thread_dy << LOG_REND;
    LOG_INFOL << "ground_protect_dis: "   << (int)ground_protect_dis << LOG_REND;
    LOG_INFOL << "sensorMountAngle: "   << (int)sensorMountAngle << LOG_REND;
    LOG_INFOL << "dis_seg: ";
    for (size_t i = 0; i < dis_seg.size(); i++){
      LOG_INFOL << dis_seg[i] << ",";
    }
    LOG_INFO << LOG_REND;
    LOG_INFOL << "thres_func_param: " << LOG_REND;
    for (size_t i = 0; i < thres_func_param.size(); i++){
      for (size_t j = 0; j < thres_func_param.front().size(); j++){
        LOG_INFOL << thres_func_param[i][j] << ",";
      }
      LOG_INFO << LOG_REND;
    }
    LOG_INFOL << "smooth_flag: "   << smooth_flag << LOG_REND;
    LOG_INFOL << "fix_ground_walkerror_flag: "   << fix_ground_walkerror_flag << LOG_REND;
    LOG_INFOL << "check_neibor_num: "   << check_neibor_num << LOG_REND;
    LOG_INFOL << "neibor_interval: "   << neibor_interval << LOG_REND;
    LOG_INFOL << "valid_num_thres: "   << valid_num_thres << LOG_REND;
    LOG_INFOL << "close_dis_thres: "   << close_dis_thres << LOG_REND;
    LOG_INFOL << "algo_version_nf: "   << algo_version_nf << LOG_REND;
    LOG_INFOL << "groundCompensationFlag: "   << (int)groundCompensationFlag << LOG_REND;
    LOG_INFOL << "groundDisThr: "   << groundDisThr << LOG_REND;
    LOG_INFOL << "groundCompensationThr: "   << groundCompensationThr << LOG_REND;

    LOG_INFOL << "inter_process_flag: "   << inter_process_flag << LOG_REND;
    LOG_INFOL << "inter_dis_thres: "   << inter_dis_thres << LOG_REND;
    LOG_INFOL << "planar_assist_filter_flag: "   << planar_assist_filter_flag << LOG_REND;
    LOG_INFOL << "planar_assist_thres: "   << planar_assist_thres << LOG_REND;
    LOG_INFOL << "color_debug: "   << color_debug << LOG_REND;
    LOG_INFOL << "ground_filter_flag: "   << ground_filter_flag << LOG_REND;
    LOG_INFOL << "ground_filter_params: " << LOG_REND;
    for (size_t i = 0; i < ground_filter_params.size(); i++){
      for (size_t j = 0; j < ground_filter_params.front().size(); j++){
        LOG_INFOL << ground_filter_params[i][j] << ",";
      }
      LOG_INFO << LOG_REND;
    }
    LOG_INFOL << "ground_unprotect_dis: "   << (int)ground_unprotect_dis << LOG_REND;

    LOG_INFOL << "ref_calib_flag: "   << ref_calib_flag << LOG_REND;
    LOG_INFOL << "ref_set_params: ";
    for (size_t i = 0; i < ref_set_params.size(); i++){
      LOG_INFOL << (int)ref_set_params[i] << ",";
    }
    LOG_INFO << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct A2CrosstalkFilterParam
{
  bool enable = false;
  int href_pos_th = 100;                //高反位置阈值, unit: cm
  double crosstalk_dis_th = 100.0;      //串扰判定距离阈值, unit: cm
  double abnorm_width_th = 60;          //异常脉宽阈值, unit: ns
  unsigned int algo_version_cr;
  std::vector<unsigned int> echo_select;
  bool peak_select_flag = true;
  bool col_helper_flag = true; 
  bool dy_pw_refer_flag = false;
  bool high_out_flag = true;
  bool valid_echo_assist_flag = false;
  std::vector<double> key_dis;
  std::vector<std::vector<double>> thres_func_param;
  double ratio_high;
  double ratio_low;
  std::vector<double> neibor_key_dis;
  std::vector<std::vector<unsigned int>> neibor_value_param;
  unsigned int echo_option;
  unsigned int refer_option;
  unsigned int pw_define_option;
  unsigned int emission_num;

  bool use_path_protect_flag;
  bool use_path_filter_flag;
  uint32_t use_path_filter_dis = 20;
  uint8_t use_path_protect_dis = 30;
  bool fake_high_filter_flag = false;
  uint32_t fake_high_filter_hight = 400;
  bool fake_high_filter_harder_flag = false;
  uint32_t fake_high_filter_harder_hight = 800;

  bool fake_path_filter_flag = false;
  bool fake_use_ld_all_flag = false;
  int fake_down_pixel_num = 12;
  uint8_t low_ref_set_value = 10;
  bool low_ref_set_flag = false;

  bool weak_repair_flag = true;
  uint8_t weak_repair_dis_thres = 150;

  bool peak_filter_flag = false;
  bool peak_filter_float_assist_flag = true;
  std::vector<std::vector<double>> peak_filter_params;
  uint8_t peak_thres_offset;

  bool gf_scale_assist_flag;
  uint16_t gf_scale_assist_work_range = 30000;
  uint8_t gf_scale_max_height = 40;
  uint8_t need_valid_neighbor_num = 1;

  std::vector<uint8_t> min_pixel_num_list = {};
  uint16_t dis_step = 300;
  bool run_init_once_flag = false;
  uint8_t start_count = 0;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             Crosstalk Filter Parameters " << LOG_REND;
    LOG_INFOL << "emission_num: " << emission_num << LOG_REND;
    LOG_INFOL << "href_pos_th: "   << href_pos_th << LOG_REND;
    LOG_INFOL << "crosstalk_dis_th: "   << crosstalk_dis_th << LOG_REND;
    LOG_INFOL << "abnorm_width_th: "   << abnorm_width_th << LOG_REND;
    LOG_INFOL << "peak_select_flag: "   << peak_select_flag << LOG_REND;
    LOG_INFOL << "col_helper_flag: "   << col_helper_flag << LOG_REND;
    LOG_INFOL << "dy_pw_refer_flag: "   << dy_pw_refer_flag << LOG_REND;
    LOG_INFOL << "high_out_flag: "   << high_out_flag << LOG_REND;
    LOG_INFOL << "valid_echo_assist_flag: "   << valid_echo_assist_flag << LOG_REND;
    LOG_INFOL << "ratio_high: "   << ratio_high << LOG_REND;
    LOG_INFOL << "ratio_low: "   << ratio_low << LOG_REND;
    LOG_INFOL << "echo_select: ";
    for (size_t i = 0; i < echo_select.size(); i++){
      LOG_INFOL << echo_select[i] << ",";
    }LOG_INFO << LOG_REND;
    LOG_INFOL << "key_dis: ";
    for (size_t i = 0; i < key_dis.size(); i++){
      LOG_INFOL << key_dis[i] << ",";
    }LOG_INFO << LOG_REND;
    LOG_INFOL << "thres_func_param: " << LOG_REND;
    for (size_t i = 0; i < thres_func_param.size(); i++){
      for (size_t j = 0; j < thres_func_param.front().size(); j++){
        LOG_INFOL << thres_func_param[i][j] << ",";
      }
      LOG_INFO << LOG_REND;
    }
    LOG_INFOL << "neibor_key_dis: ";
    for (size_t i = 0; i < neibor_key_dis.size(); i++){
      LOG_INFOL << neibor_key_dis[i] << ",";
    }LOG_INFO << LOG_REND;
    LOG_INFOL << "neibor_value_param: " << LOG_REND;
    for (size_t i = 0; i < neibor_value_param.size(); i++){
      for (size_t j = 0; j < neibor_value_param.front().size(); j++){
        LOG_INFOL << neibor_value_param[i][j] << ",";
      }
      LOG_INFO << LOG_REND;
    }
    LOG_INFOL << "echo_option: "   << echo_option << LOG_REND;
    LOG_INFOL << "refer_option: "   << refer_option << LOG_REND;
    LOG_INFOL << "pw_define_option: "   << pw_define_option << LOG_REND;

    LOG_INFOL << "use_path_protect_flag: "   << use_path_protect_flag << LOG_REND;
    LOG_INFOL << "use_path_filter_flag: "   << use_path_filter_flag << LOG_REND;
    LOG_INFOL << "use_path_filter_dis: "   << (int)use_path_filter_dis << LOG_REND;
    LOG_INFOL << "use_path_protect_dis: "   << (int)use_path_protect_dis << LOG_REND;
    LOG_INFOL << "fake_high_filter_flag: "   << fake_high_filter_flag << LOG_REND;
    LOG_INFOL << "fake_high_filter_hight: "   << (int)fake_high_filter_hight << LOG_REND;
    LOG_INFOL << "fake_high_filter_harder_flag: "   << fake_high_filter_harder_flag << LOG_REND;
    LOG_INFOL << "fake_high_filter_harder_hight: "   << (int)fake_high_filter_harder_hight << LOG_REND;

    LOG_INFOL << "fake_path_filter_flag: "   << fake_path_filter_flag << LOG_REND;
    LOG_INFOL << "fake_use_ld_all_flag: "   << fake_use_ld_all_flag << LOG_REND;
    LOG_INFOL << "fake_down_pixel_num: "   << fake_down_pixel_num << LOG_REND;
    LOG_INFOL << "low_ref_set_value: "   << (int)low_ref_set_value << LOG_REND;
    LOG_INFOL << "low_ref_set_flag: "   << low_ref_set_flag << LOG_REND;

    LOG_INFOL << "weak_repair_flag: "   << weak_repair_flag << LOG_REND;
    LOG_INFOL << "weak_repair_dis_thres: "   << (int)weak_repair_dis_thres << LOG_REND;

    LOG_INFOL << "peak_filter_flag: "   << peak_filter_flag << LOG_REND;
    LOG_INFOL << "peak_filter_float_assist_flag: "   << peak_filter_float_assist_flag << LOG_REND;
    LOG_INFOL << "peak_thres_offset: "   << (int)peak_thres_offset << LOG_REND;

    LOG_INFOL << "run_init_once_flag: "   << run_init_once_flag << LOG_REND;
    LOG_INFOL << "gf_scale_max_height: "   << (int)gf_scale_max_height << LOG_REND;
    LOG_INFOL << "need_valid_neighbor_num: "   << (int)need_valid_neighbor_num << LOG_REND;
    LOG_INFOL << "gf_scale_assist_flag: "   << gf_scale_assist_flag << LOG_REND;

    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct A2PlaneGhostFilterParam
{
  bool enable = false;
  float z_plane_threshold = 0.1;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             Plane Ghost Filter Parameters " << LOG_REND;
    LOG_INFOL << "z_plane_threshold: " << z_plane_threshold << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct A2BlindLineFilterParam
{
  bool enable = false;
  float blind_line_dis = 50.0;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             Blind Line Repair Parameters " << LOG_REND;
    LOG_INFOL << "blind_line_dis: " << blind_line_dis << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct A2DynamicPowerFilterParam
{
  bool enable = false;
  float distance_threshold_start = 5.0;
  float distance_threshold_end = 100.0;
  float k = 1.0;
  float b = 0.0;
  float d = 20.0;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             Dynamic Power Compensation Parameters " << LOG_REND;
    LOG_INFOL << "distance_threshold_start: " << distance_threshold_start << LOG_REND;
    LOG_INFOL << "distance_threshold_end: " << distance_threshold_end << LOG_REND;
    LOG_INFOL << "k: " << k << LOG_REND;
    LOG_INFOL << "b: " << b << LOG_REND;
    LOG_INFOL << "d: " << d << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct A2BlindAreaFilterParam
{
  bool enable = false;
  float azimuth0 = 0.0;
  float azimuth1 = 0.0;
  float azimuth2 = 0.0;
  float azimuth3 = 0.0;
  float azimuth4 = 0.0;
  float minDisInvalid = 0.5;
  float maxDisInvalid = 2.0;
  int windowStep = 20;
  int windowWidth = 40;
  int windowHeight = 48;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             Blind Area Filter Parameters " << LOG_REND;
    LOG_INFOL << "azimuth0: " << azimuth0 << LOG_REND;
    LOG_INFOL << "azimuth1: " << azimuth1 << LOG_REND;
    LOG_INFOL << "azimuth2: " << azimuth2 << LOG_REND;
    LOG_INFOL << "azimuth3: " << azimuth3 << LOG_REND;
    LOG_INFOL << "azimuth4: " << azimuth4 << LOG_REND;
    LOG_INFOL << "minDisInvalid: " << minDisInvalid << LOG_REND;
    LOG_INFOL << "maxDisInvalid: " << maxDisInvalid << LOG_REND;
    LOG_INFOL << "windowStep: " << windowStep << LOG_REND;
    LOG_INFOL << "windowWidth: " << windowWidth << LOG_REND;
    LOG_INFOL << "windowHeight: " << windowHeight << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct ExpansionFilter
{
  bool enable = false;
  unsigned int highRefThr = 150;
  unsigned int step = 10;
  unsigned int nearThr = 30;
  unsigned int weakRefThr = 30;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             ExpansionFilter Parameters " << LOG_REND;
    LOG_INFOL << "enable: " << enable << LOG_REND;
    LOG_INFOL << "highRefThr: " << highRefThr << LOG_REND;
    LOG_INFOL << "step: " << step << LOG_REND;
    LOG_INFOL << "nearThr: " << nearThr << LOG_REND;
    LOG_INFOL << "weakRefThr: " << weakRefThr << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct trailingFilter
{
  bool enable = false;
  double dthr0 = 100;
  double dthr1 = 310;
  double dthr2 = 30;
  double dthr4 = 20000;
  double phithr= 20;
  double kslop = 0.3;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             trailingFilter Parameters " << LOG_REND;
    LOG_INFOL << "enable: " << enable << LOG_REND;
    LOG_INFOL << "dthr0: " << dthr0 << LOG_REND;
    LOG_INFOL << "dthr1: " << dthr1 << LOG_REND;
    LOG_INFOL << "dthr2: " << dthr2 << LOG_REND;
    LOG_INFOL << "dthr4: " << dthr4 << LOG_REND;
    LOG_INFOL << "phithr: " << phithr << LOG_REND;
    LOG_INFOL << "kslop: " << kslop << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct A2BfsSegmentParam
{
  bool enable = false;
  float alpha_h_deg = 0.28f;
  float alpha_v_deg = 0.16f;
  float segment_theta_deg = 60.0f;
  int   min_size_near = 30;
  int   min_size_mid  = 10;
  int   min_size_far  = 3;
  float dis_near = 3000.0f;
  float dis_far  = 8000.0f;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             BfsSegment Parameters " << LOG_REND;
    LOG_INFOL << "alpha_h_deg: "       << alpha_h_deg       << LOG_REND;
    LOG_INFOL << "alpha_v_deg: "       << alpha_v_deg       << LOG_REND;
    LOG_INFOL << "segment_theta_deg: " << segment_theta_deg << LOG_REND;
    LOG_INFOL << "min_size_near: "     << min_size_near     << LOG_REND;
    LOG_INFOL << "min_size_mid: "      << min_size_mid      << LOG_REND;
    LOG_INFOL << "min_size_far: "      << min_size_far      << LOG_REND;
    LOG_INFOL << "dis_near: "          << dis_near          << LOG_REND;
    LOG_INFOL << "dis_far: "           << dis_far           << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct A2RangeImageSmoothParam
{
  bool enable = false;
  int half_azimuth = 2;
  int half_ring = 1;
  float ratio_thr = 0.05f;
  int min_valid = 3;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             RangeImageSmooth Parameters " << LOG_REND;
    LOG_INFOL << "half_azimuth: " << half_azimuth << LOG_REND;
    LOG_INFOL << "half_ring: "    << half_ring     << LOG_REND;
    LOG_INFOL << "ratio_thr: "    << ratio_thr     << LOG_REND;
    LOG_INFOL << "min_valid: "    << min_valid      << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;
  }
};

struct A2AlgorithmParam
{
  A2PointCloudFilterParam   pointcloud_filter;
  A2CrosstalkFilterParam    crosstalk_filter;
  A2PlaneGhostFilterParam   plane_ghost_filter;
  A2BlindLineFilterParam    blind_line_filter;
  A2DynamicPowerFilterParam dynamic_power;
  A2BlindAreaFilterParam    blind_area_filter;
  ExpansionFilter           expansion_filter;
  trailingFilter            trailing_filter;
  A2RangeImageSmoothParam   range_image;
  A2BfsSegmentParam         bfs_segment;

  void print() const
  {
    LOG_INFO << "-------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             Algorithm Parameters  " << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;

    if (pointcloud_filter.enable)   pointcloud_filter.print();
    if (crosstalk_filter.enable)    crosstalk_filter.print();
    if (plane_ghost_filter.enable)  plane_ghost_filter.print();
    if (blind_line_filter.enable)   blind_line_filter.print();
    if (dynamic_power.enable)       dynamic_power.print();
    if (blind_area_filter.enable)   blind_area_filter.print();
    if (expansion_filter.enable)    expansion_filter.print();
    if (trailing_filter.enable)     trailing_filter.print();
    if (range_image.enable)  range_image.print();
    if (bfs_segment.enable)         bfs_segment.print();
  }
};

class SoftwareFilter
{
public:
  SoftwareFilter (struct A2AlgorithmParam params)
    : algorithmParam(params)
  {
    std::string gf_calib_param_path, ref_calib_param_path, peak_filter_param_path;
    #ifdef RUN_IN_ROS1_OR_ROS2
      int param_len;
      if (algorithmParam.crosstalk_filter.algo_version_cr == 1){
        gf_calib_param_path = (std::string)PROJECT_PATH + "/src/core/driver/algorithm/test/1.txt";
        readMNText(gf_calib_param_path, param_len, algorithmParam.crosstalk_filter.thres_func_param);
      }
      if (algorithmParam.pointcloud_filter.algo_version_nf == 2){
        ref_calib_param_path = (std::string)PROJECT_PATH + "/src/core/driver/algorithm/test/2.txt";
        peak_filter_param_path = (std::string)PROJECT_PATH + "/src/core/driver/algorithm/test/3.txt";
        readMNText(ref_calib_param_path, param_len, algorithmParam.pointcloud_filter.ref_calib_params);
        readMNText(peak_filter_param_path, param_len, algorithmParam.crosstalk_filter.peak_filter_params);
      }
    #endif
  }

  bool filter(std::shared_ptr<BlockBuffer>& block, size_t offset, size_t length, double& z_plane);
  bool compensation(std::shared_ptr<BlockBuffer>& block, size_t offset, size_t length, bool aligned);

  bool rangeImageSmooth(std::shared_ptr<BlockBuffer>& block);
  bool bfsSegmentFilter(std::shared_ptr<BlockBuffer>& block);
  bool pointCloudFilter(std::shared_ptr<BlockBuffer>& block, size_t offset, size_t length, uint16_t mount_angle, double& z_plane);
  bool crosstalkFilter(std::shared_ptr<BlockBuffer>& block, size_t offset, size_t length);
  
  bool pointCloudDouble(std::shared_ptr<BlockBuffer>& block, size_t offset, size_t length);
  bool repairBlindLine(std::shared_ptr<BlockBuffer>& block, bool aligned);
  bool dynamicPowerCompensation(std::shared_ptr<BlockBuffer>& block, bool aligned);
  bool blindAreaFilter(std::shared_ptr<BlockBuffer>& block, bool aligned);

#ifndef UNIT_TEST
protected:
#endif

  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

#ifndef UNIT_TEST
private:
#endif

  bool neiborProtect(std::vector<std::vector<RawPointT>*>& buffer_ptr_, RawPointT* ptr_in_, const unsigned int dis_similar_thres,
    const int idx, const uint8_t pIdx, const uint8_t pixelNumPerSlot, const uint8_t echo_num);
  bool neiborPointProtectModule(std::vector<std::vector<int>>& point_label, RawPointT* ptr_in_,
      const std::vector<double>& neibor_key_dis, const std::vector<std::vector<unsigned int>>& neibor_value_param,
      const std::vector<std::vector<int>>& temp_pixel_info_last, const std::vector<std::vector<int>>& pixel_info_last, const unsigned int pIdx, 
      const unsigned int crs_pIdx, const unsigned int e_idx, const unsigned int dis_similar_thres);
  bool crosstalkFilter_LastVersion(std::shared_ptr<BlockBuffer> &block, size_t offset, size_t length);
  bool crosstalkFilter_CurVerison(std::shared_ptr<BlockBuffer> &block, size_t offset, size_t length);
  bool crosstalkFilter_Test(std::shared_ptr<BlockBuffer> &block, size_t offset, size_t length);

  bool echoPeakFilter(RawPointT* point_, const uint8_t ld_pixel_num, const uint8_t ring_s, const uint8_t ring_e);
  bool echoGfJudge(RawPointT& point);
  bool echoRefCalib(RawPointT& point);

  bool pointCloudFilter_LastVersion(std::shared_ptr<BlockBuffer> &block, size_t offset, size_t length, uint16_t mount_angle, double &z_plane);
  bool pointCloudFilter_CurVerison(std::shared_ptr<BlockBuffer> &block, size_t offset, size_t length, uint16_t mount_angle, double &z_plane);
  bool pointCloudFilter_Test(std::shared_ptr<BlockBuffer> &block, size_t offset, size_t length, uint16_t mount_angle, double &z_plane);
  bool trailingFilter(std::shared_ptr<BlockBuffer> &block, size_t offset, size_t length);
  bool expansionFilter(std::shared_ptr<BlockBuffer> &block, size_t offset, size_t length);
  bool newNeighborAssist(std::vector<std::vector<RawPointT>*>& buffer_ptr_, RawPointT* ptr_in_, const unsigned int dis_similar_thres,
    const int idx, const uint8_t pIdx, const uint8_t pixelNumPerSlot, const uint8_t echo_num, int direction = 1);
  bool readMNText(const std::string data_path, const int N,
            std::vector<std::vector<double>>& dataout);


  struct A2AlgorithmParam algorithmParam;
};
  double rad2deg(double radians);
  double deg2rad(double degrees);
}  // namespace lidar
}  // namespace lidar_sdk
