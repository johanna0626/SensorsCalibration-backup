/*
 * Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
 * Limited. All rights reserved.
 * Yan Guohang <yanguohang@pjlab.org.cn>
 * Ouyang Jinhua <ouyangjinhua@pjlab.org.cn>
 */
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <string>

#include <vector>

class BirdView {
public:
  Eigen::Matrix3d ground2img_hmat_;
  Eigen::Matrix3d img2ground_hmat_;
  cv::Mat img2bv_hmat_;
  // cv::Mat ground2bv_hmat_;
  int width_;
  int height_;

  cv::Mat image_;
  cv::Mat const_image_;
  std::vector<cv::Point2f> picked_points_;

  cv::Size origin_img_size_;
  cv::Mat grid_image_;
  cv::Mat grid_image_fv_;

  static void mouse_event(int event, int x, int y, int flags, void *ustc) {
    BirdView *temp = reinterpret_cast<BirdView *>(ustc);
    temp->mouse_event(event, x, y);
  }


  void mouse_event(int event, int x, int y) {
    if (event == cv::EVENT_LBUTTONDOWN) {
      picked_points_.push_back(cv::Point2f(x, y));
      std::cout << "picked " << x << ", " << y << std::endl;
      cv::circle(image_, cv::Point(x, y), 6, cv::Scalar(0, 0, 255), -1);
      cv::imshow("image", image_);
    }
  }

  // void PickPoint() {
  //   cv::namedWindow("image", 0);
  //   cv::setMouseCallback("image", mouse_event, this);
  //   cv::imshow("image", image_);
  //   while (true) {
  //     int key = cv::waitKey(10);
  //     key = key % 255;

  //     // evaluate
  //     if (key == 13) // enter
  //     {
  //       bool flag = computeHomo();
  //       if (flag)
  //         break;
  //     }
  //   }
  //   cv::destroyWindow("image");
  // }

  // void PickPoint_fv() {
  //   cv::namedWindow("image", 0);
  //   cv::setMouseCallback("image", mouse_event, this);
  //   cv::imshow("image", image_);
  //   while (true) {
  //     int key = cv::waitKey(10);
  //     key = key % 255;

  //     // evaluate
  //     if (key == 13) // enter
  //     {
  //       bool flag = computeHomo_fv();
  //       if (flag)
  //         break;
  //     }
  //   }
  //   cv::destroyWindow("image");
  // }

  bool computeHomo() {
    // 修改，去掉计算homo的部分，只保留bv的画布
    // int picked_num = picked_points_.size();
    // if (picked_num != 4) {
    //   std::cerr << "[ERROR]Invalid Labeled Point Num, Must select 4 "
    //                "points(leftdown, leftup, rightdown, rightup).\n";
    //   picked_points_.clear();
    //   const_image_.copyTo(image_);
    //   cv::imshow("image", image_);
    //   std::cerr << "Picked points is cleared. Please label again.\n";
    //   return false;
    // }

    std::vector<cv::Point2f> ground_pts;
    std::vector<cv::Point2f> img_pts;
    std::vector<cv::Point2f> bv_pts;
    // for (int i = 0; i < 4; i++) {
    //   Eigen::Vector3d ground_pt =
    //       img2ground_hmat_ *
    //       Eigen::Vector3d(picked_points_[i].x, picked_points_[i].y, 1);
    //   ground_pt /= ground_pt(2);
    //   ground_pts.push_back(cv::Point2f(ground_pt(0), ground_pt(1)));
    // }


    float left = width_ / 3;
    float right = width_ - width_ / 3;
    float up = height_ / 3;
    float down = height_ - height_ / 3;

    // bv_pts.push_back(cv::Point2f(left, down));
    // bv_pts.push_back(cv::Point2f(left, up));
    // bv_pts.push_back(cv::Point2f(right, down));
    // bv_pts.push_back(cv::Point2f(right, up));

    // ground2bv_hmat_ = cv::findHomography(ground_pts, bv_pts);
    grid_image_ =
        cv::Mat(cv::Size(width_, height_), CV_32FC3, cv::Scalar(235, 235, 235));
    // draw lines
    int gap = (width_ + 10.0) / 10.0;
    for (int i = 0; i < 10; i++) {
      cv::Point start = cv::Point(gap * i, 0);
      cv::Point end = cv::Point(gap * i, height_);
      cv::line(grid_image_, start, end, cv::Scalar(140, 140, 140), 2);
    }
    gap = (height_ + 10.0) / 15.0;
    for (int i = 0; i < 15; i++) {
      cv::Point start = cv::Point(0, gap * i);
      cv::Point end = cv::Point(width_, gap * i);
      cv::line(grid_image_, start, end, cv::Scalar(140, 140, 140), 2);
    }
    
    // draw vertical lines
    cv::line(grid_image_, cv::Point(0, int(up)),
             cv::Point(width_, int(up)), cv::Scalar(225, 80, 0), 4);
    cv::line(grid_image_, cv::Point(int(right), 0),
             cv::Point(int(right), height_), cv::Scalar(225, 80, 0), 4);
    cv::imwrite("grid_image.jpg", grid_image_);
    return true;
  }
  
  bool computeHomo_fv() {
    // int picked_num = picked_points_.size();
    // if (picked_num != 4) {
    //   std::cerr << "[ERROR]Invalid Labeled Point Num, Must select 4 "
    //                "points(leftdown, leftup, rightdown, rightup).\n";
    //   picked_points_.clear();
    //   const_image_.copyTo(image_);
    //   cv::imshow("image", image_);
    //   std::cerr << "Picked points is cleared. Please label again.\n";
    //   return false;
    // }

    std::vector<cv::Point2f> ground_pts;
    std::vector<cv::Point2f> img_pts;
    std::vector<cv::Point2f> fv_pts;
    // for (int i = 0; i < 4; i++) {
    //   Eigen::Vector3d ground_pt =
    //       img2ground_hmat_ *
    //       Eigen::Vector3d(picked_points_[i].x, picked_points_[i].y, 1);
    //   ground_pt /= ground_pt(2);
    //   ground_pts.push_back(cv::Point2f(ground_pt(0), ground_pt(1)));
    // }
    float left = width_ / 3;
    float right = width_ - width_ / 3;
    float up = height_ / 3;
    float down = height_ - height_ / 3;

    // fv_pts.push_back(cv::Point2f(left, down));
    // fv_pts.push_back(cv::Point2f(left, up));
    // fv_pts.push_back(cv::Point2f(right, down));
    // fv_pts.push_back(cv::Point2f(right, up));

    // ground2bv_hmat_ = cv::findHomography(ground_pts, fv_pts);
    grid_image_fv_ =
        cv::Mat(cv::Size(width_, height_), CV_32FC3, cv::Scalar(235, 235, 235));
    // draw lines
    int gap = (width_ + 10.0) / 10.0;
    for (int i = 0; i < 10; i++) {
      cv::Point start = cv::Point(gap * i, 0);
      cv::Point end = cv::Point(gap * i, height_);
      cv::line(grid_image_fv_, start, end, cv::Scalar(140, 140, 140), 2);
    }
    gap = (height_ + 10.0) / 15.0;
    for (int i = 0; i < 15; i++) {
      cv::Point start = cv::Point(0, gap * i);
      cv::Point end = cv::Point(width_, gap * i);
      cv::line(grid_image_fv_, start, end, cv::Scalar(140, 140, 140), 2);
    }
    
    // draw vertical lines
    cv::line(grid_image_fv_, cv::Point(0, int(down)),
             cv::Point(width_, int(down)), cv::Scalar(225, 80, 0), 4);
    cv::line(grid_image_fv_, cv::Point(int(right), 0),
             cv::Point(int(right), height_), cv::Scalar(225, 80, 0), 4);
    cv::imwrite("grid_image_fv.jpg", grid_image_fv_);
    return true;
  }

  // void init_bv(const cv::Mat &img, const int &width, const int &height,
  //           const Eigen::Matrix3d &img2ground_hmat) {
  void init_bv(const cv::Mat &img, const int &width, const int &height) {
    origin_img_size_ = img.size();
    img.copyTo(image_);
    cv::putText(image_,
                "Pick 2 pairs of vertical points (x and y direction). Press Enter to compute homography.",
                cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.3,
                cv::Scalar(30, 30, 30), 4);
    image_.copyTo(const_image_);

    width_ = width;
    height_ = height;
    // img2ground_hmat_ = img2ground_hmat;
    // Eigen::Matrix3d ground2img_hmat_ = img2ground_hmat.inverse().eval();
    // PickPoint();
  }

  // add initialization for fv
  // void init_fv(const cv::Mat &img, const int &width, const int &height,
  //           const Eigen::Matrix3d &img2ground_hmat) {
  void init_fv(const cv::Mat &img, const int &width, const int &height) {
    origin_img_size_ = img.size();
    img.copyTo(image_);
    cv::putText(image_,
                "Pick 2 pairs of vertical points (y and z direction). Press Enter to compute homography.",
                cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1.3,
                cv::Scalar(30, 30, 30), 4);
    image_.copyTo(const_image_);

    width_ = width;
    height_ = height;
    // img2ground_hmat_ = img2ground_hmat;
    // Eigen::Matrix3d ground2img_hmat_ = img2ground_hmat.inverse().eval();
    // PickPoint_fv();
  }

  cv::Mat drawProjectBirdView(const int &point_size,
                              const std::vector<cv::Point2f> &grount_pts) {
    cv::Mat output;
    output = cv::imread("grid_image.jpg");

    // Eigen::Matrix3d hmat;
    // cv::cv2eigen(ground2bv_hmat_, hmat);

    // Initialization
    double min_xbv = std::numeric_limits<double>::max();
    double min_ybv = std::numeric_limits<double>::max();
    double max_xbv = std::numeric_limits<double>::lowest();
    double max_ybv = std::numeric_limits<double>::lowest();

    for (size_t i = 0; i < grount_pts.size(); i++) {
        Eigen::Vector2d bv_pt(grount_pts[i].x, grount_pts[i].y);
        // Eigen::Vector3d bv_pt = hmat * pt;
        // bv_pt /= bv_pt(2);

        // int xbv = cvRound(bv_pt(0));
        // int ybv = cvRound(bv_pt(1));

        double xbv = cvRound(bv_pt(0));
        double ybv = cvRound(bv_pt(1));

        // Update
        min_xbv = std::min(min_xbv, static_cast<double>(xbv));
        min_ybv = std::min(min_ybv, static_cast<double>(ybv));
        max_xbv = std::max(max_xbv, static_cast<double>(xbv));
        max_ybv = std::max(max_ybv, static_cast<double>(ybv));
    }

    for (size_t i = 0; i < grount_pts.size(); i++) {
      Eigen::Vector2d bv_pt(grount_pts[i].x, grount_pts[i].y);
      // Eigen::Vector3d bv_pt = hmat * pt;
      // bv_pt /= bv_pt(2);
      // int xbv = cvRound(bv_pt(0));
      // int ybv = cvRound(bv_pt(1));

      // int xbv = cvRound((bv_pt(0) - min_xbv) / (max_xbv - min_xbv) * output.cols);
      // int ybv = cvRound((bv_pt(1) - min_ybv) / (max_ybv - min_ybv) * output.rows);

      double xbv = (bv_pt(0) - min_xbv) / (max_xbv - min_xbv) * output.cols;
      double ybv = (bv_pt(1) - min_ybv) / (max_ybv - min_ybv) * output.rows;

      // std::cout << "x: " << xbv << " "
      //           << "y: " << ybv << std::endl;

      cv::Scalar color = cv::Scalar(0, 0, 255);
      cv::circle(output, cv::Point(xbv, ybv), point_size, color, -1);
    }
    return output;
  }

  // add for front view
  // Point2f中储存的本来就是y和z坐标
  cv::Mat drawProjectFrontView(const int &point_size,
                              const std::vector<cv::Point2f> &grount_pts) {
    cv::Mat output;
    output = cv::imread("grid_image_fv.jpg");

    // Eigen::Matrix3d hmat;
    // cv::cv2eigen(ground2bv_hmat_, hmat);

    // Initialization
    double min_yfv = std::numeric_limits<double>::max();
    double min_zfv = std::numeric_limits<double>::max();
    double max_yfv = std::numeric_limits<double>::lowest();
    double max_zfv = std::numeric_limits<double>::lowest();

    for (size_t i = 0; i < grount_pts.size(); i++) {
      Eigen::Vector2d fv_pt(grount_pts[i].x, grount_pts[i].y);
      // Eigen::Vector3d fv_pt = pt(1.0, grount_pts[i].y, grount_pts[i].z);
      // Eigen::Vector3d fv_pt = hmat * pt;
      // bv_pt /= bv_pt(2);

      // int yfv = cvRound(fv_pt(0));
      // int zfv = cvRound(fv_pt(1));

      double yfv = fv_pt(0);
      double zfv = fv_pt(1);

      // Update
      min_yfv = std::min(min_yfv, static_cast<double>(yfv));
      min_zfv = std::min(min_zfv, static_cast<double>(zfv));
      max_yfv = std::max(max_yfv, static_cast<double>(yfv));
      max_zfv = std::max(max_zfv, static_cast<double>(zfv));
    }

    for (size_t i = 0; i < grount_pts.size(); i++) {
      Eigen::Vector2d fv_pt(grount_pts[i].x, grount_pts[i].y);
      // Eigen::Vector3d pt(grount_pts[i].x * 0.2, grount_pts[i].y * 0.2, 1.0);
      // Eigen::Vector3d fv_pt = hmat * pt;
      // fv_pt /= fv_pt(0);
      // int yfv = cvRound(fv_pt(1));
      // int zfv = cvRound(fv_pt(2));

      // int yfv = cvRound((fv_pt(0) - min_yfv) / (max_yfv - min_yfv) * output.cols);
      // int zfv = cvRound((fv_pt(1) - min_zfv) / (max_zfv - min_zfv) * output.rows);

      double yfv = (fv_pt(0) - min_yfv) / (max_yfv - min_yfv) * output.cols;
      double zfv = (fv_pt(1) - min_zfv) / (max_zfv - min_zfv) * output.rows;

      // std::cout << "y: " << ybv << " "
      //           << "z: " << zbv << std::endl;
      cv::Scalar color = cv::Scalar(0, 0, 255);
      cv::circle(output, cv::Point(yfv, zfv), point_size, color, -1);
    }
    return output;
  }

};
