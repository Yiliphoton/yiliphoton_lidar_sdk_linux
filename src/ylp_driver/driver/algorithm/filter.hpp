#pragma once

#include <cstdint>
#include <memory>
#include "ylp_driver/common/ag_log.hpp"
#include "ylp_driver/common/error_code.hpp"
#include "ylp_driver/common/block_buffer.hpp"
#include "ylp_driver/driver/decoder/trigon.hpp"

namespace asensing
{
namespace lidar
{

// BlockBuffer, RawPointT, Trigon etc. are in yiliphoton::lidar; bring them in.




struct A0AlgorithmParam
{
  int delt_threshold = 15;
  int num_threshold = 10;

  void print() const
  {
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             A0 Algorithm Parameters " << YLP_REND;
    YLP_INFOL << "delt_threshold: " << delt_threshold << YLP_REND;
    YLP_INFOL << "num_threshold: " << num_threshold << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
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
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             PointCloud Filter Parameters " << YLP_REND;
    YLP_INFOL << "filterFlag: " << filterFlag << YLP_REND;
    YLP_INFOL << "groundProtectFlag: " << groundProtectFlag << YLP_REND;
    YLP_INFOL << "ground_angle_threshold: "   << (int)ground_angle_threshold << YLP_REND;
    YLP_INFOL << "gHIdx: "   << (int)gHIdx << YLP_REND;
    YLP_INFOL << "gLIdx: "   << (int)gLIdx << YLP_REND;
    YLP_INFOL << "diffIdx: "   << (int)diffIdx << YLP_REND;
    YLP_INFOL << "dis_fixed: "   << dis_fixed << YLP_REND;
    YLP_INFOL << "dis_thread_fixed: "   << dis_thread_fixed << YLP_REND;
    YLP_INFOL << "dis_delta: "   << dis_delta << YLP_REND;
    YLP_INFOL << "dis_k: "   << dis_k << YLP_REND;
    YLP_INFOL << "dis_thread_dy: "   << dis_thread_dy << YLP_REND;
    YLP_INFOL << "num_thread_dy: "   << (int)num_thread_dy << YLP_REND;
    YLP_INFOL << "ground_protect_dis: "   << (int)ground_protect_dis << YLP_REND;
    YLP_INFOL << "sensorMountAngle: "   << (int)sensorMountAngle << YLP_REND;
    YLP_INFOL << "dis_seg: ";
    for (size_t i = 0; i < dis_seg.size(); i++){
      YLP_INFOL << dis_seg[i] << ",";
    }
    YLP_INFO << YLP_REND;
    YLP_INFOL << "thres_func_param: " << YLP_REND;
    for (size_t i = 0; i < thres_func_param.size(); i++){
      for (size_t j = 0; j < thres_func_param.front().size(); j++){
        YLP_INFOL << thres_func_param[i][j] << ",";
      }
      YLP_INFO << YLP_REND;
    }
    YLP_INFOL << "smooth_flag: "   << smooth_flag << YLP_REND;
    YLP_INFOL << "fix_ground_walkerror_flag: "   << fix_ground_walkerror_flag << YLP_REND;
    YLP_INFOL << "check_neibor_num: "   << check_neibor_num << YLP_REND;
    YLP_INFOL << "neibor_interval: "   << neibor_interval << YLP_REND;
    YLP_INFOL << "valid_num_thres: "   << valid_num_thres << YLP_REND;
    YLP_INFOL << "close_dis_thres: "   << close_dis_thres << YLP_REND;
    YLP_INFOL << "algo_version_nf: "   << algo_version_nf << YLP_REND;
    YLP_INFOL << "groundCompensationFlag: "   << (int)groundCompensationFlag << YLP_REND;
    YLP_INFOL << "groundDisThr: "   << groundDisThr << YLP_REND;
    YLP_INFOL << "groundCompensationThr: "   << groundCompensationThr << YLP_REND;

    YLP_INFOL << "inter_process_flag: "   << inter_process_flag << YLP_REND;
    YLP_INFOL << "inter_dis_thres: "   << inter_dis_thres << YLP_REND;
    YLP_INFOL << "planar_assist_filter_flag: "   << planar_assist_filter_flag << YLP_REND;
    YLP_INFOL << "planar_assist_thres: "   << planar_assist_thres << YLP_REND;
    YLP_INFOL << "color_debug: "   << color_debug << YLP_REND;
    YLP_INFOL << "ground_filter_flag: "   << ground_filter_flag << YLP_REND;
    YLP_INFOL << "ground_filter_params: " << YLP_REND;
    for (size_t i = 0; i < ground_filter_params.size(); i++){
      for (size_t j = 0; j < ground_filter_params.front().size(); j++){
        YLP_INFOL << ground_filter_params[i][j] << ",";
      }
      YLP_INFO << YLP_REND;
    }
    YLP_INFOL << "ground_unprotect_dis: "   << (int)ground_unprotect_dis << YLP_REND;

    YLP_INFOL << "ref_calib_flag: "   << ref_calib_flag << YLP_REND;
    YLP_INFOL << "ref_set_params: ";
    for (size_t i = 0; i < ref_set_params.size(); i++){
      YLP_INFOL << (int)ref_set_params[i] << ",";
    }
    YLP_INFO << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
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
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             Crosstalk Filter Parameters " << YLP_REND;
    YLP_INFOL << "emission_num: " << emission_num << YLP_REND;
    YLP_INFOL << "href_pos_th: "   << href_pos_th << YLP_REND;
    YLP_INFOL << "crosstalk_dis_th: "   << crosstalk_dis_th << YLP_REND;
    YLP_INFOL << "abnorm_width_th: "   << abnorm_width_th << YLP_REND;
    YLP_INFOL << "peak_select_flag: "   << peak_select_flag << YLP_REND;
    YLP_INFOL << "col_helper_flag: "   << col_helper_flag << YLP_REND;
    YLP_INFOL << "dy_pw_refer_flag: "   << dy_pw_refer_flag << YLP_REND;
    YLP_INFOL << "high_out_flag: "   << high_out_flag << YLP_REND;
    YLP_INFOL << "valid_echo_assist_flag: "   << valid_echo_assist_flag << YLP_REND;
    YLP_INFOL << "ratio_high: "   << ratio_high << YLP_REND;
    YLP_INFOL << "ratio_low: "   << ratio_low << YLP_REND;
    YLP_INFOL << "echo_select: ";
    for (size_t i = 0; i < echo_select.size(); i++){
      YLP_INFOL << echo_select[i] << ",";
    }YLP_INFO << YLP_REND;
    YLP_INFOL << "key_dis: ";
    for (size_t i = 0; i < key_dis.size(); i++){
      YLP_INFOL << key_dis[i] << ",";
    }YLP_INFO << YLP_REND;
    YLP_INFOL << "thres_func_param: " << YLP_REND;
    for (size_t i = 0; i < thres_func_param.size(); i++){
      for (size_t j = 0; j < thres_func_param.front().size(); j++){
        YLP_INFOL << thres_func_param[i][j] << ",";
      }
      YLP_INFO << YLP_REND;
    }
    YLP_INFOL << "neibor_key_dis: ";
    for (size_t i = 0; i < neibor_key_dis.size(); i++){
      YLP_INFOL << neibor_key_dis[i] << ",";
    }YLP_INFO << YLP_REND;
    YLP_INFOL << "neibor_value_param: " << YLP_REND;
    for (size_t i = 0; i < neibor_value_param.size(); i++){
      for (size_t j = 0; j < neibor_value_param.front().size(); j++){
        YLP_INFOL << neibor_value_param[i][j] << ",";
      }
      YLP_INFO << YLP_REND;
    }
    YLP_INFOL << "echo_option: "   << echo_option << YLP_REND;
    YLP_INFOL << "refer_option: "   << refer_option << YLP_REND;
    YLP_INFOL << "pw_define_option: "   << pw_define_option << YLP_REND;

    YLP_INFOL << "use_path_protect_flag: "   << use_path_protect_flag << YLP_REND;
    YLP_INFOL << "use_path_filter_flag: "   << use_path_filter_flag << YLP_REND;
    YLP_INFOL << "use_path_filter_dis: "   << (int)use_path_filter_dis << YLP_REND;
    YLP_INFOL << "use_path_protect_dis: "   << (int)use_path_protect_dis << YLP_REND;
    YLP_INFOL << "fake_high_filter_flag: "   << fake_high_filter_flag << YLP_REND;
    YLP_INFOL << "fake_high_filter_hight: "   << (int)fake_high_filter_hight << YLP_REND;
    YLP_INFOL << "fake_high_filter_harder_flag: "   << fake_high_filter_harder_flag << YLP_REND;
    YLP_INFOL << "fake_high_filter_harder_hight: "   << (int)fake_high_filter_harder_hight << YLP_REND;

    YLP_INFOL << "fake_path_filter_flag: "   << fake_path_filter_flag << YLP_REND;
    YLP_INFOL << "fake_use_ld_all_flag: "   << fake_use_ld_all_flag << YLP_REND;
    YLP_INFOL << "fake_down_pixel_num: "   << fake_down_pixel_num << YLP_REND;
    YLP_INFOL << "low_ref_set_value: "   << (int)low_ref_set_value << YLP_REND;
    YLP_INFOL << "low_ref_set_flag: "   << low_ref_set_flag << YLP_REND;

    YLP_INFOL << "weak_repair_flag: "   << weak_repair_flag << YLP_REND;
    YLP_INFOL << "weak_repair_dis_thres: "   << (int)weak_repair_dis_thres << YLP_REND;

    YLP_INFOL << "peak_filter_flag: "   << peak_filter_flag << YLP_REND;
    YLP_INFOL << "peak_filter_float_assist_flag: "   << peak_filter_float_assist_flag << YLP_REND;
    YLP_INFOL << "peak_thres_offset: "   << (int)peak_thres_offset << YLP_REND;

    YLP_INFOL << "run_init_once_flag: "   << run_init_once_flag << YLP_REND;
    YLP_INFOL << "gf_scale_max_height: "   << (int)gf_scale_max_height << YLP_REND;
    YLP_INFOL << "need_valid_neighbor_num: "   << (int)need_valid_neighbor_num << YLP_REND;
    YLP_INFOL << "gf_scale_assist_flag: "   << gf_scale_assist_flag << YLP_REND;

    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
  }
};

struct A2PlaneGhostFilterParam
{
  bool enable = false;
  float z_plane_threshold = 0.1;

  void print() const
  {
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             Plane Ghost Filter Parameters " << YLP_REND;
    YLP_INFOL << "z_plane_threshold: " << z_plane_threshold << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
  }
};

struct A2BlindLineFilterParam
{
  bool enable = false;
  float blind_line_dis = 50.0;

  void print() const
  {
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             Blind Line Repair Parameters " << YLP_REND;
    YLP_INFOL << "blind_line_dis: " << blind_line_dis << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
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
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             Dynamic Power Compensation Parameters " << YLP_REND;
    YLP_INFOL << "distance_threshold_start: " << distance_threshold_start << YLP_REND;
    YLP_INFOL << "distance_threshold_end: " << distance_threshold_end << YLP_REND;
    YLP_INFOL << "k: " << k << YLP_REND;
    YLP_INFOL << "b: " << b << YLP_REND;
    YLP_INFOL << "d: " << d << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
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
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             Blind Area Filter Parameters " << YLP_REND;
    YLP_INFOL << "azimuth0: " << azimuth0 << YLP_REND;
    YLP_INFOL << "azimuth1: " << azimuth1 << YLP_REND;
    YLP_INFOL << "azimuth2: " << azimuth2 << YLP_REND;
    YLP_INFOL << "azimuth3: " << azimuth3 << YLP_REND;
    YLP_INFOL << "azimuth4: " << azimuth4 << YLP_REND;
    YLP_INFOL << "minDisInvalid: " << minDisInvalid << YLP_REND;
    YLP_INFOL << "maxDisInvalid: " << maxDisInvalid << YLP_REND;
    YLP_INFOL << "windowStep: " << windowStep << YLP_REND;
    YLP_INFOL << "windowWidth: " << windowWidth << YLP_REND;
    YLP_INFOL << "windowHeight: " << windowHeight << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
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
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             ExpansionFilter Parameters " << YLP_REND;
    YLP_INFOL << "enable: " << enable << YLP_REND;
    YLP_INFOL << "highRefThr: " << highRefThr << YLP_REND;
    YLP_INFOL << "step: " << step << YLP_REND;
    YLP_INFOL << "nearThr: " << nearThr << YLP_REND;
    YLP_INFOL << "weakRefThr: " << weakRefThr << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
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
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             trailingFilter Parameters " << YLP_REND;
    YLP_INFOL << "enable: " << enable << YLP_REND;
    YLP_INFOL << "dthr0: " << dthr0 << YLP_REND;
    YLP_INFOL << "dthr1: " << dthr1 << YLP_REND;
    YLP_INFOL << "dthr2: " << dthr2 << YLP_REND;
    YLP_INFOL << "dthr4: " << dthr4 << YLP_REND;
    YLP_INFOL << "phithr: " << phithr << YLP_REND;
    YLP_INFOL << "kslop: " << kslop << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
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
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             BfsSegment Parameters " << YLP_REND;
    YLP_INFOL << "alpha_h_deg: "       << alpha_h_deg       << YLP_REND;
    YLP_INFOL << "alpha_v_deg: "       << alpha_v_deg       << YLP_REND;
    YLP_INFOL << "segment_theta_deg: " << segment_theta_deg << YLP_REND;
    YLP_INFOL << "min_size_near: "     << min_size_near     << YLP_REND;
    YLP_INFOL << "min_size_mid: "      << min_size_mid      << YLP_REND;
    YLP_INFOL << "min_size_far: "      << min_size_far      << YLP_REND;
    YLP_INFOL << "dis_near: "          << dis_near          << YLP_REND;
    YLP_INFOL << "dis_far: "           << dis_far           << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
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
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             RangeImageSmooth Parameters " << YLP_REND;
    YLP_INFOL << "half_azimuth: " << half_azimuth << YLP_REND;
    YLP_INFOL << "half_ring: "    << half_ring     << YLP_REND;
    YLP_INFOL << "ratio_thr: "    << ratio_thr     << YLP_REND;
    YLP_INFOL << "min_valid: "    << min_valid      << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;
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
  A2RangeImageSmoothParam   range_image_smooth;
  A2BfsSegmentParam         bfs_segment;

  void print() const
  {
    YLP_INFO << "-------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             A2 Algorithm Parameters 0402 " << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;

    if (pointcloud_filter.enable)   pointcloud_filter.print();
    if (crosstalk_filter.enable)    crosstalk_filter.print();
    if (plane_ghost_filter.enable)  plane_ghost_filter.print();
    if (blind_line_filter.enable)   blind_line_filter.print();
    if (dynamic_power.enable)       dynamic_power.print();
    if (blind_area_filter.enable)   blind_area_filter.print();
    if (expansion_filter.enable)    expansion_filter.print();
    if (trailing_filter.enable)     trailing_filter.print();
    if (range_image_smooth.enable)  range_image_smooth.print();
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
        gf_calib_param_path = (std::string)PROJECT_PATH + "/src/driver_sdk/src/ylp_driver/driver/algorithm/test/1.txt";
        readMNText(gf_calib_param_path, param_len, algorithmParam.crosstalk_filter.thres_func_param);
      }
      if (algorithmParam.pointcloud_filter.algo_version_nf == 2){
        ref_calib_param_path = (std::string)PROJECT_PATH + "/src/driver_sdk/src/ylp_driver/driver/algorithm/test/2.txt";
        peak_filter_param_path = (std::string)PROJECT_PATH + "/src/driver_sdk/src/ylp_driver/driver/algorithm/test/3.txt";
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
    const int idx, const uint8_t pIdx, const uint8_t pixelNumPerSlot, const uint8_t echo_num);
  bool readMNText(const std::string data_path, const int N, 
            std::vector<std::vector<double>>& dataout);


  struct A2AlgorithmParam algorithmParam;
};
  double rad2deg(double radians);
  double deg2rad(double degrees);
}  // namespace lidar
}  // namespace asensing