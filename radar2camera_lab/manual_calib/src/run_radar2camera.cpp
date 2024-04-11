/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Core>
#include <iostream>
#include <string>

#include "extrinsic_param.hpp"
#include "homography_param.hpp"
#include "intrinsic_param.hpp"
#include "projector_radar.hpp"

using namespace std;

#define GL_GPU_MEM_INFO_CURRENT_AVAILABLE_MEM_NVX 0x9049
#define MAX_RADAR_TIME_GAP 15 * 1e6
#define APPLY_COLOR_TO_LIDAR_INTENSITY // to set intensity colored or not

double cali_scale_degree_ = 0.3;
double cali_scale_trans_ = 0.06;
double cali_scale_fxfy_ = 1.005;
static Eigen::Matrix4d calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix4d orign_calibration_matrix_ = Eigen::Matrix4d::Identity();
static Eigen::Matrix3d intrinsic_matrix_ = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d orign_intrinsic_matrix_ = Eigen::Matrix3d::Identity();
std::vector<float> distortions_;
std::vector<Eigen::Matrix4d> modification_list_;

bool kbhit() {
  termios term;
  tcgetattr(0, &term);
  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);
  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

void CalibrationInit(Eigen::Matrix4d json_param) {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  calibration_matrix_ = json_param;
  orign_calibration_matrix_ = json_param;
  modification_list_.reserve(12);  // default: 12

  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;

    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());

    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;


    // modification_list_[i] = tmp;
    modification_list_.push_back(tmp);

  }
  std::cout << "=>Calibration scale Init!\n";
}

void CalibrationScaleChange() {
  Eigen::Matrix4d init_cali;
  init_cali << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  modification_list_.reserve(12);
  for (int32_t i = 0; i < 12; i++) {
    std::vector<int> transform_flag(6, 0);
    transform_flag[i / 2] = (i % 2) ? (-1) : 1;
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d rot_tmp;
    rot_tmp =
        Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitZ());
    tmp.block(0, 0, 3, 3) = rot_tmp;
    tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
    tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
    tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
    modification_list_[i] = tmp;
  }
  std::cout << "=>Calibration scale update done!\n";
}

void saveResult(const cv::Mat &calib_img, const cv::Mat &bv_img,
                const int &frame_id) {
  std::string file_name = "calibration_" + std::to_string(frame_id) + ".txt";
  std::ofstream fCalib(file_name);
  if (!fCalib.is_open()) {
    std::cerr << "open file " << file_name << " failed." << std::endl;
    return;
  }
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "R:\n"
         << calibration_matrix_(0, 0) << " " << calibration_matrix_(0, 1) << " "
         << calibration_matrix_(0, 2) << "\n"
         << calibration_matrix_(1, 0) << " " << calibration_matrix_(1, 1) << " "
         << calibration_matrix_(1, 2) << "\n"
         << calibration_matrix_(2, 0) << " " << calibration_matrix_(2, 1) << " "
         << calibration_matrix_(2, 2) << std::endl;
  fCalib << "t: " << calibration_matrix_(0, 3) << " "
         << calibration_matrix_(1, 3) << " " << calibration_matrix_(2, 3)
         << std::endl;
  fCalib << "\nIntrinsic:" << std::endl;
  fCalib << intrinsic_matrix_(0, 0) << " " << intrinsic_matrix_(0, 1) << " "
         << intrinsic_matrix_(0, 2) << "\n"
         << intrinsic_matrix_(1, 0) << " " << intrinsic_matrix_(1, 1) << " "
         << intrinsic_matrix_(1, 2) << "\n"
         << intrinsic_matrix_(2, 0) << " " << intrinsic_matrix_(2, 1) << " "
         << intrinsic_matrix_(2, 2) << std::endl;

  fCalib << "************* json format *************" << std::endl;
  fCalib << "Extrinsic:" << std::endl;
  fCalib << "[" << calibration_matrix_(0, 0) << "," << calibration_matrix_(0, 1)
         << "," << calibration_matrix_(0, 2) << "," << calibration_matrix_(0, 3)
         << "],"
         << "[" << calibration_matrix_(1, 0) << "," << calibration_matrix_(1, 1)
         << "," << calibration_matrix_(1, 2) << "," << calibration_matrix_(1, 3)
         << "],"
         << "[" << calibration_matrix_(2, 0) << "," << calibration_matrix_(2, 1)
         << "," << calibration_matrix_(2, 2) << "," << calibration_matrix_(2, 3)
         << "],"
         << "[" << calibration_matrix_(3, 0) << "," << calibration_matrix_(3, 1)
         << "," << calibration_matrix_(3, 2) << "," << calibration_matrix_(3, 3)
         << "]" << std::endl;
  fCalib << "\nIntrinsic:" << std::endl;
  fCalib << "[" << intrinsic_matrix_(0, 0) << "," << intrinsic_matrix_(0, 1)
         << "," << intrinsic_matrix_(0, 2) << "],"
         << "[" << intrinsic_matrix_(1, 0) << "," << intrinsic_matrix_(1, 1)
         << "," << intrinsic_matrix_(1, 2) << "],"
         << "[" << intrinsic_matrix_(2, 0) << "," << intrinsic_matrix_(2, 1)
         << "," << intrinsic_matrix_(2, 2) << "]" << std::endl;

  fCalib << "\nDistortion:" << std::endl;
  fCalib << "[";
  for (size_t i = 0; i < distortions_.size(); i++) {
    fCalib << distortions_[i];
    if (i == distortions_.size() - 1)
      continue;
    fCalib << ",";
  }
  fCalib << "]";
  fCalib.close();

  std::string img_name = "calibimg_" + std::to_string(frame_id) + ".jpg";
  cv::imwrite(img_name, calib_img);

  std::string bv_img_name = "calibimg_bv_" + std::to_string(frame_id) + ".jpg";
  cv::imwrite(bv_img_name, bv_img);
}

bool ManualCalibration(int key_input) {
  char table[] = {'q', 'a', 'w', 's', 'e', 'd', 'r', 'f', 't', 'g', 'y', 'h'};
  bool real_hit = false;
  for (int32_t i = 0; i < 12; i++) {
    if (key_input == table[i]) {
      calibration_matrix_ = calibration_matrix_ * modification_list_[i];
      real_hit = true;
    }
  }

  return real_hit;
}

bool ReadRadarPoint(const std::string &radar_csv_path,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr radar_points,
                    const int &radar_type) {
  radar_points->points.clear();
  std::ifstream file(radar_csv_path);
  if (!file.is_open()) {
    std::cout << "ERROR--->>> cannot open: " << radar_csv_path << std::endl;
    return false;
  }
  std::string line;
  getline(file, line);
  
  bool whether_first = true;
  std::string first_time_str;
  while (getline(file, line)) {
    std::stringstream ss(line);
    std::string str;
    // std::string time_str;
    std::string position_x_str;
    std::string position_y_str;
    std::string position_z_str;
    std::string intensity_str;
    int index = 0;
    while (getline(ss, str, ' ')) {  // 原为逗号分隔符“，”
      if (index == 0) {
        position_x_str = str;
      } else if (index == 1) {
        position_y_str = str;
      } else if (index == 2) {
        position_z_str = str;
      } else if (index == 7) {
        intensity_str = str;
      }
      index++;
    }

    // pcl::PointXYZ radar_point;
    pcl::PointXYZI radar_point;
    radar_point.x = std::atof(position_x_str.c_str());
    radar_point.y = std::atof(position_y_str.c_str());
    radar_point.z = std::atof(position_z_str.c_str());
    radar_point.intensity = std::atof(intensity_str.c_str());
    std::cout << "radar_point.intensity: " << radar_point.intensity << std::endl;
    if (std::abs(radar_point.intensity) < 1e5) {
      continue;
    }
    // std::cout<< "radar_point.x: " << radar_point.x << std::endl;
    // std::cout<< "radar_point.y: " << radar_point.y << std::endl;
    // std::cout<< "radar_point.z: " << radar_point.z << std::endl;
    std::cout<< "radar_point.intensity: " << radar_point.intensity << std::endl;

    radar_points->points.push_back(radar_point);
  }
  std::cout << "radar point size: " << radar_points->points.size()<< std::endl;
  return true;
}


int main(int argc, char **argv) {
  if (argc != 5) {
    cout << "Usage: ./run_radar2camera <image_path> <radar_file_path> "
            "<intrinsic_json> <homo_json> <extrinsic_json>"
            "\nexample:\n\t"
            "./bin/run_radar2camera data/0.jpg data/front_radar.csv "
            "data/center_camera-intrinsic.json "
            // "data/center_camera-homography.json "
            "data/radar-to-center_camera-extrinsic.json"
         << endl;
    return 0;
  }

  string camera_path = argv[1];
  string radar_csv_path = argv[2];
  string intrinsic_json = argv[3];
  // string homography_json = argv[4];
  string radar2camera_extrinsic_json = argv[4];  //原本是argv[5]
  int radar_type = 1;  // 可以删掉

  cv::Mat img = cv::imread(camera_path);

  std::cout << intrinsic_json << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (!ReadRadarPoint(radar_csv_path, cloud, radar_type)) {
    std::cerr << "[ERROR]Couldn't read radar file.\n";
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZI> pcd = *cloud;
  Eigen::Matrix3d K;
  std::vector<double> dist;
  LoadIntrinsic(intrinsic_json, K, dist);
  for (size_t i = 0; i < dist.size(); i++) {
    distortions_.push_back(dist[i]);
  }

  intrinsic_matrix_ = K;
  orign_intrinsic_matrix_ = intrinsic_matrix_;
  // Intrinsic parameters
  std::cout << "intrinsic:\n"
            << K(0, 0) << " " << K(0, 1) << " " << K(0, 2) << "\n"
            << K(1, 0) << " " << K(1, 1) << " " << K(1, 2) << "\n"
            << K(2, 0) << " " << K(2, 1) << " " << K(2, 2) << "\n";
  
  // Camera distortion
  std::cout << "dist:\n" << dist[0] << " " << dist[1] << "\n";

  // Eigen::Matrix3d homograph;
  // LoadHomography(homography_json, homograph);
  // std::cout << "Homograph:\n" << homograph << std::endl;

  Eigen::Matrix4d json_param;
  LoadExtrinsic(radar2camera_extrinsic_json, json_param);
  std::cout << "Extrinsic:\n" << json_param << std::endl;

  cout << "Loading data completed!" << endl;
  // view
  int width = img.cols;
  int height = img.rows;
  int bv_width = width / 2;
  int bv_height = height / 2;

  cout << "Begin initialization ..." << endl;

  CalibrationInit(json_param);

  cout << "Initialization completed" << endl;

  Projector projector;
  projector.init(img, bv_width, bv_height); // , homograph 原本放在第二个位置

  cout << "Load Point Cloud ..." << endl;

  projector.loadPointCloud(pcd);

  cout << "Load Point Cloud Completed!" << endl;

  std::cout << "width:" << width << " , height:" << height << std::endl;
  std::cout << "bird view width:" << bv_width
            << " , bird view height:" << bv_height << std::endl;
  // const int width = 1920, height = 1200;
  // pangolin::CreateWindowAndBind("radar2camera player",
  //                               (width + bv_width) / 0.9 * 0.5, height * 0.5);
  pangolin::CreateWindowAndBind("radar2camera player",
                                width * 0.5, (height + bv_height) * 0.45);
  glEnable(GL_DEPTH_TEST);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1920, 1080, 500, 500, 512, 389, 0.1, 1000), // width and height, 
      pangolin::ModelViewLookAt(0, 0, 100, 0, 0, 0, 0.0, 1.0, 0.0));

  // pangolin::View &project_image =
  //     pangolin::Display("project")
  //         .SetBounds(0.0, 1.0, 0.1, 0.7, -1.0 * width / height)
  //         .SetLock(pangolin::LockLeft, pangolin::LockTop);

  pangolin::View &project_image =
      pangolin::Display("project")
          .SetBounds(0.33, 1.0, 0.1, 1.0, -1.0 * width / height)
          .SetLock(pangolin::LockLeft, pangolin::LockTop);

  // The left one: bird view
  pangolin::View &birdview_image =
      pangolin::Display("birdview")
          .SetBounds(0, 0.33, 0.1, 0.55, -1.0 * bv_width / bv_height)
          .SetLock(pangolin::LockLeft, pangolin::LockTop);
  
  // The right one: front view
  pangolin::View &frontview_image =
      pangolin::Display("frontview")
          .SetBounds(0, 0.33, 0.55, 1.0, -1.0 * bv_width / bv_height)
          .SetLock(pangolin::LockLeft, pangolin::LockTop);

  unsigned char *imageArray = new unsigned char[3 * width * height];
  pangolin::GlTexture imageTexture(width, height, GL_RGB, false, 0, GL_RGB,
                                   GL_UNSIGNED_BYTE);
  unsigned char *imageArray_bv = new unsigned char[3 * bv_width * bv_height];
  pangolin::GlTexture imageTexture_bv(bv_width, bv_height, GL_RGB, false, 0,
                                      GL_RGB, GL_UNSIGNED_BYTE);
  unsigned char *imageArray_fv = new unsigned char[3 * bv_width * bv_height];
  pangolin::GlTexture imageTexture_fv(bv_width, bv_height, GL_RGB, false, 0,
                                      GL_RGB, GL_UNSIGNED_BYTE);

  // control panel
  pangolin::CreatePanel("cp").SetBounds(pangolin::Attach::Pix(30), 1.0, 0.0,
                                        //   pangolin::Attach::Pix(150));
                                        0.1);
  pangolin::Var<double> degreeStep("cp.deg step", 0.3, 0, 1); // logscale
  pangolin::Var<double> tStep("cp.t step(cm)", 6, 0, 15);
  // pangolin::Var<double> fxfyScale("cp.fxfy scale", 1.005, 1, 1.1);
  pangolin::Var<int> pointSize("cp.point size", 6, 1, 10);

  pangolin::Var<bool> addXdegree("cp.+ x degree", false, false);
  pangolin::Var<bool> minusXdegree("cp.- x degree", false, false);
  pangolin::Var<bool> addYdegree("cp.+ y degree", false, false);
  pangolin::Var<bool> minusYdegree("cp.- y degree", false, false);
  pangolin::Var<bool> addZdegree("cp.+ z degree", false, false);
  pangolin::Var<bool> minusZdegree("cp.- z degree", false, false);
  pangolin::Var<bool> addXtrans("cp.+ x trans", false, false);
  pangolin::Var<bool> minusXtrans("cp.- x trans", false, false);
  pangolin::Var<bool> addYtrans("cp.+ y trans", false, false);
  pangolin::Var<bool> minusYtrans("cp.- y trans", false, false);
  pangolin::Var<bool> addZtrans("cp.+ z trans", false, false);
  pangolin::Var<bool> minusZtrans("cp.- z trans", false, false);

  pangolin::Var<bool> resetButton("cp.Reset", false, false);
  pangolin::Var<bool> saveImg("cp.Save Result", false, false);

  std::vector<pangolin::Var<bool>> mat_calib_box;
  mat_calib_box.push_back(addXdegree);
  mat_calib_box.push_back(minusXdegree);
  mat_calib_box.push_back(addYdegree);
  mat_calib_box.push_back(minusYdegree);
  mat_calib_box.push_back(addZdegree);
  mat_calib_box.push_back(minusZdegree);
  mat_calib_box.push_back(addXtrans);
  mat_calib_box.push_back(minusXtrans);
  mat_calib_box.push_back(addYtrans);
  mat_calib_box.push_back(minusYtrans);
  mat_calib_box.push_back(addZtrans);
  mat_calib_box.push_back(minusZtrans);

  cv::Mat current_frame;
  cv::Mat bv_frame;
  cv::Mat fv_frame;

  projector.ProjectToRawImage(img, intrinsic_matrix_, dist, calibration_matrix_,
                              current_frame, bv_frame, fv_frame);

  int frame_num = 0;

  std::cout << "\n=>START\n";
  while (!pangolin::ShouldQuit()) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (degreeStep.GuiChanged()) {
      cali_scale_degree_ = degreeStep.Get();
      CalibrationScaleChange();
      std::cout << "Degree calib scale changed to " << cali_scale_degree_
                << " degree\n";
    }
    if (tStep.GuiChanged()) {
      cali_scale_trans_ = tStep.Get() / 100.0;
      CalibrationScaleChange();
      std::cout << "Trans calib scale changed to " << cali_scale_trans_ * 100
                << " cm\n";
    }
    if (pointSize.GuiChanged()) {
      int ptsize = pointSize.Get();
      projector.setPointSize(ptsize);
      projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                  calibration_matrix_, current_frame, bv_frame, fv_frame);
      std::cout << "point size changed to " << ptsize << std::endl;
    }
    for (int i = 0; i < 12; i++) {
      if (pangolin::Pushed(mat_calib_box[i])) {
        calibration_matrix_ = calibration_matrix_ * modification_list_[i];
        std::cout << "Changed!\n";
        projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                    calibration_matrix_, current_frame,
                                    bv_frame, fv_frame);
      }
    }

    if (pangolin::Pushed(resetButton)) {
      calibration_matrix_ = orign_calibration_matrix_;
      intrinsic_matrix_ = orign_intrinsic_matrix_;
      projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                  calibration_matrix_, current_frame, bv_frame, fv_frame);
      std::cout << "Reset!\n";
    }
    if (pangolin::Pushed(saveImg)) {
      saveResult(current_frame, bv_frame, frame_num);
      std::cout << "\n==>Save Result " << frame_num << std::endl;
      Eigen::Matrix4d transform = calibration_matrix_;
      cout << "Transfromation Matrix:\n" << transform << std::endl;
      frame_num++;
    }

    if (kbhit()) {
      int c = getchar();
      if (ManualCalibration(c)) {
        Eigen::Matrix4d transform = calibration_matrix_;
        cout << "\nTransfromation Matrix:\n" << transform << std::endl;
      }
      projector.ProjectToRawImage(img, intrinsic_matrix_, dist,
                                  calibration_matrix_, current_frame, bv_frame, fv_frame);
    }

    imageArray = current_frame.data;
    imageArray_bv = bv_frame.data;
    imageArray_fv = fv_frame.data;
    imageTexture.Upload(imageArray, GL_BGR, GL_UNSIGNED_BYTE);
    imageTexture_bv.Upload(imageArray_bv, GL_BGR, GL_UNSIGNED_BYTE);
    imageTexture_fv.Upload(imageArray_fv, GL_BGR, GL_UNSIGNED_BYTE);

    project_image.Activate();
    glColor3f(1.0, 1.0, 1.0);
    imageTexture.RenderToViewportFlipY();
    birdview_image.Activate();
    glColor3f(1.0, 1.0, 1.0);
    imageTexture_bv.RenderToViewportFlipY();
    frontview_image.Activate();
    glColor3f(1.0, 1.0, 1.0);
    imageTexture_fv.RenderToViewportFlipY();

    pangolin::FinishFrame();
    glFinish();
  }

  Eigen::Matrix4d transform = calibration_matrix_;
  cout << "\nFinal Transfromation Matrix:\n" << transform << std::endl;

  return 0;
}
