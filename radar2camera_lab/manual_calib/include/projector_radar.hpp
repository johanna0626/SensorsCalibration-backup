/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once

#include <algorithm>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include <vector>

#include "birdview.hpp"
#include "pcl/io/pcd_io.h"

struct Pt {
  cv::Point point;
  float dist;
  float x;
  float intensity;
};

class Projector {
public:
  cv::Mat oriCloud;
  std::vector<float> intensitys;
  int point_size_ = 6;
  // Eigen::Matrix3d homograph_;
  BirdView bv_handler;
  BirdView fv_handler;

  void init(const cv::Mat &img, // const Eigen::Matrix3d &homograph,
            const int &bv_w, const int &bv_h) {
    // homograph_ = homograph;
    bv_handler.init_bv(img, bv_w, bv_h); //, homograph
    fv_handler.init_fv(img, bv_w, bv_h); //, homograph
  }

  void setPointSize(int size) { point_size_ = size; }

  bool loadPointCloud(pcl::PointCloud<pcl::PointXYZI> pcl) {
    oriCloud = cv::Mat(cv::Size(pcl.points.size(), 3), CV_32FC1);
    for (size_t i = 0; i < pcl.points.size(); ++i) {
      oriCloud.at<float>(0, i) = pcl.points[i].x;
      oriCloud.at<float>(1, i) = pcl.points[i].y;
      oriCloud.at<float>(2, i) = pcl.points[i].z;
      intensitys.push_back(pcl.points[i].intensity);
    }
    return true;
  }

  cv::Scalar fakeColor(float value) {
    float posSlope = 255 / 60.0;
    float negSlope = -255 / 60.0;
    value *= 255;
    cv::Vec3f color;
    if (value < 60) {
      color[0] = 255;
      color[1] = posSlope * value + 0;
      color[2] = 0;
    } else if (value < 120) {
      color[0] = negSlope * value + 2 * 255;
      color[1] = 255;
      color[2] = 0;
    } else if (value < 180) {
      color[0] = 0;
      color[1] = 255;
      color[2] = posSlope * value - 2 * 255;
    } else if (value < 240) {
      color[0] = 0;
      color[1] = negSlope * value + 4 * 255;
      color[2] = 255;
    } else if (value < 300) {
      color[0] = posSlope * value - 4 * 255;
      color[1] = 0;
      color[2] = 255;
    } else {
      color[0] = 255;
      color[1] = 0;
      color[2] = negSlope * value + 6 * 255;
    }
    return cv::Scalar(color[0], color[1], color[2]);
  }

  void ProjectToRawMat(cv::Mat img, cv::Mat K, cv::Mat D, cv::Mat R, cv::Mat T,
                       cv::Mat &current_frame, cv::Mat &bv_frame, cv::Mat &fv_frame) {
    cv::Mat I = cv::Mat::eye(3, 3, CV_32FC1);
    cv::Mat mapX, mapY;
    cv::Mat outImg = cv::Mat(img.size(), CV_32FC3);
    cv::initUndistortRectifyMap(K, D, I, K, img.size(), CV_32FC1, mapX, mapY);
    cv::remap(img, outImg, mapX, mapY, cv::INTER_LINEAR);

    // Compute Euclidean distance
    cv::Mat dist = oriCloud.rowRange(0, 1).mul(oriCloud.rowRange(0, 1)) +
                   oriCloud.rowRange(1, 2).mul(oriCloud.rowRange(1, 2)) +
                   oriCloud.rowRange(2, 3).mul(oriCloud.rowRange(2, 3));
    cv::Mat R_ = R;
    cv::Mat T_ = T;

    // R T 是相机和雷达之间的转换矩阵

    
    cv::Mat transCloud2d = R_ * oriCloud + repeat(T_, 1, oriCloud.cols);  //把雷达（世界）坐标系下的点云转换到相机坐标系
    cv::Mat projCloud2d = K * transCloud2d; //把相机坐标系下的点云转换到图像坐标系
    // float maxDist = 0;
    // float maxx = 0;
    float maxIntensity = 10000;
    std::vector<Pt> points;
    std::vector<cv::Point2f> bv_radar_pts;
    std::vector<cv::Point2f> fv_radar_pts;

    // project bird view points
    for (int32_t i = 0; i < transCloud2d.cols; ++i) {
      float x = transCloud2d.at<float>(0, i);
      float y = transCloud2d.at<float>(1, i);
      float z = transCloud2d.at<float>(2, i);

      // Projection from bird view? why not x, y, 1
      // Eigen::Vector3d world_pt(x, z, 1);
      Eigen::Vector3d world_pt(x, y, z);

      // Try to omit homograph
      // Eigen::Vector3d bv_pt = homograph_.inverse().eval() * world_pt;
      Eigen::Vector3d bv_pt = world_pt;
      Eigen::Vector3d fv_pt = world_pt;
      // bv_pt /= bv_pt(2);
      // cv::Point result(cvRound(bv_pt(0)), cvRound(bv_pt(1)));
      // int x2d = cvRound(bv_pt(0));
      // int y2d = cvRound(bv_pt(1));

      float d = sqrt(dist.at<float>(0, i));
      // if (x2d >= 0 && y2d >= 0 && x2d < img.cols && y2d < img.rows && z > 0) {
      // maxx = std::max(maxx, std::fabs(x));
      // maxDist = std::max(maxDist, d);
      // float intensity = points[i].intensity;
      // maxIntensity = std::max(maxIntensity, intensity);
      // points.push_back(Pt{cv::Point(x2d, y2d), d, std::fabs(z)});
      // bv_radar_pts.push_back(cv::Point2f(x2d, y2d));
      bv_radar_pts.push_back(cv::Point2f(bv_pt(0), bv_pt(1)));
      fv_radar_pts.push_back(cv::Point2f(fv_pt(1), fv_pt(2)));

      // 把图像坐标系下的点转移到像素坐标系
      float u = projCloud2d.at<float>(0, i)/projCloud2d.at<float>(2, i);
      float v = projCloud2d.at<float>(1, i)/projCloud2d.at<float>(2, i);
      float intensity = intensitys[i];

      points.push_back(Pt{cv::Point(u, v), d, std::fabs(z), intensity}); //
      // }
    }

    bv_frame = bv_handler.drawProjectBirdView(point_size_, bv_radar_pts);
    fv_frame = fv_handler.drawProjectFrontView(point_size_, fv_radar_pts);

    sort(points.begin(), points.end(),
         [](const Pt &a, const Pt &b) { return a.dist > b.dist; });
    for (size_t i = 0; i < points.size(); ++i) {
      cv::Scalar color;
      // distance
      float d = points[i].dist;
      float x = points[i].x;
      float intensity = points[i].intensity;
      // std::cout << "points[i].point: " << points[i].point << std::endl;

      // color = fakeColor(d / maxDist);
      // color = fakeColor(x / maxx);
      color = fakeColor(intensity / maxIntensity);
      circle(outImg, points[i].point, point_size_, color, -1);
    }
    current_frame = outImg;
  }

  // cv::Mat ProjectToRawImage(cv::Mat img,
  //                           Eigen::Matrix3d K,
  //                           std::vector<double> D,
  //                           Eigen::Matrix4d json_param) {
  void ProjectToRawImage(const cv::Mat &img, const Eigen::Matrix3d &K,
                         const std::vector<double> &D,
                         const Eigen::Matrix4d &json_param,
                         cv::Mat &current_frame, cv::Mat &bv_frame, cv::Mat &fv_frame) {
    cv::Mat K1, D1, R1, T1;
    float k[9], d[8], r[9], t[3];

    k[0] = K(0, 0);
    k[1] = K(0, 1);
    k[2] = K(0, 2);
    k[3] = K(1, 0);
    k[4] = K(1, 1);
    k[5] = K(1, 2);
    k[6] = K(2, 0);
    k[7] = K(2, 1);
    k[8] = K(2, 2);

    // d[0] = D(0);
    // d[1] = D(1);
    // d[2] = D(2);
    // d[3] = D(3);
    for (size_t i = 0; i < D.size(); i++) {
      d[i] = D[i];
    }

    r[0] = json_param(0, 0);
    r[1] = json_param(0, 1);
    r[2] = json_param(0, 2);
    r[3] = json_param(1, 0);
    r[4] = json_param(1, 1);
    r[5] = json_param(1, 2);
    r[6] = json_param(2, 0);
    r[7] = json_param(2, 1);
    r[8] = json_param(2, 2);

    t[0] = json_param(0, 3);
    t[1] = json_param(1, 3);
    t[2] = json_param(2, 3);

    K1 = cv::Mat(3, 3, CV_32FC1, k);
    D1 = cv::Mat(D.size(), 1, CV_32FC1, d);
    R1 = cv::Mat(3, 3, CV_32FC1, r);
    T1 = cv::Mat(3, 1, CV_32FC1, t);
    // cv::Mat img = cv::imread(imgName);
    ProjectToRawMat(img, K1, D1, R1, T1, current_frame, bv_frame, fv_frame);
  }
};
