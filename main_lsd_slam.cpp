/*
Example command:

./build.sh

./build/main_lsd_slam --camera_config ~/slam/cameraConfig.cfg \
 --input_dir ~/slam/imgs --out_dir ~/slam/out --display_depth_map=true

*/

/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University
*of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by daeyun on 9/8/15.
//

#include <algorithm>
#include <boost/thread.hpp>
#include <dirent.h>
#include <fstream>
#include <vector>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <sstream>
#include <DataStructures/Frame.h>

#include "SlamSystem.h"
#include "opencv2/opencv.hpp"
#include "IOWrapper/Output3DWrapper.h"
#include "util/Undistorter.h"
#include "util/globalFuncs.h"
#include "util/settings.h"

DEFINE_string(input_dir, "./", "Input image directory path.");
DEFINE_string(camera_config, "./cameraConfig.cfg", "Camera config file path.");
DEFINE_string(out_dir, "./", "Output directory path.");

DEFINE_bool(display_depth_map, true, "Show GUI visualization window.");

namespace fs = ::boost::filesystem;

using lsd_slam::Undistorter;
using lsd_slam::Output3DWrapper;
using lsd_slam::SlamSystem;
using lsd_slam::KeyFrameGraph;
using lsd_slam::Frame;

// Get image filenames in the given input directory path.
bool GetImageFiles(const fs::path& root, std::vector<fs::path>* ret) {
  if (!fs::exists(root) || !fs::is_directory(root)) {
    return false;
  }

  static const std::vector<std::string> exts{".jpg", ".png"};
  fs::directory_iterator endit;
  for (fs::directory_iterator it(root); it != endit; ++it) {
    if (fs::is_regular_file(*it) &&
        std::find(exts.begin(), exts.end(), it->path().extension()) !=
            exts.end()) {
      ret->push_back(it->path());
    }
  }

  std::sort(ret->begin(), ret->end());

  return !ret->empty();
}

// SlamSystem will write data to this object.
class FileOutput3DWrapper : public Output3DWrapper {
 public:
  FileOutput3DWrapper(int width, int height) {}
  ~FileOutput3DWrapper() {}

  void publishKeyframeGraph(KeyFrameGraph* graph) override {}

  // publishes a keyframe. if that frame already existis, it is overwritten,
  // otherwise it is added.
  void publishKeyframe(Frame* kf) override {
    // TODO(daeyun) Process output. Available: pose, depth map
  }

  // published a tracked frame that did not become a keyframe (yet; i.e. has no
  // depth data)
  void publishTrackedFrame(Frame* kf) override {
    // TODO(daeyun) Process output. Available: pose
  }

  // publishes graph and all constraints, as well as updated KF poses.
  void publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory,
                         std::string identifier) override {}

  void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt,
                                  std::string identifier) override {}

  void publishDebugInfo(Eigen::Matrix<float, 20, 1> data) override {}
};

using namespace lsd_slam;
int main(int argc, char* argv[]) {
  FLAGS_logtostderr = true;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  std::vector<fs::path> files;
  CHECK(GetImageFiles(FLAGS_input_dir, &files))
      << "Invalid --input_dir: " << FLAGS_input_dir;

  LOG(INFO) << files.size() << " images found in " << FLAGS_input_dir;

  auto* undistorter =
      Undistorter::getUndistorterForFile(FLAGS_camera_config.c_str());

  CHECK(undistorter) << "Invalid camera config file: " << FLAGS_camera_config;

  // Create output directory.
  fs::path dir(FLAGS_out_dir);
  if (fs::create_directory(dir)) {
    LOG(INFO) << "Directory was created successfully";
  }

  lsd_slam::displayDepthMap = FLAGS_display_depth_map;

  int w = undistorter->getOutputWidth();
  int h = undistorter->getOutputHeight();

  int w_inp = undistorter->getInputWidth();
  int h_inp = undistorter->getInputHeight();

  float fx = undistorter->getK().at<double>(0, 0);
  float fy = undistorter->getK().at<double>(1, 1);
  float cx = undistorter->getK().at<double>(2, 0);
  float cy = undistorter->getK().at<double>(2, 1);
  Sophus::Matrix3f K;
  K << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;

  FileOutput3DWrapper outputWrapper(w, h);

  SlamSystem system(w, h, K, true);
  system.setVisualization(&outputWrapper);

  cv::Mat img_undistorted = cv::Mat(h, w, CV_8U);
  unsigned int index = 0;
  double fake_timestamp = 0;

  for (const auto& file : files) {
    cv::Mat img = cv::imread(file.string(), CV_LOAD_IMAGE_GRAYSCALE);

    if (img.rows != h_inp || img.cols != w_inp) {
      LOG(ERROR) << "Invalid image " << file << ". Skipping.";
      continue;
    }
    CHECK_EQ(img.type(), CV_8U);

    undistorter->undistort(img, img_undistorted);

    if (index == 0) {
      system.randomInit(img_undistorted.data, fake_timestamp, index);
    } else {
      system.trackFrame(img_undistorted.data, index, true, fake_timestamp);
    }
    LOG(INFO) << file.string();

    fake_timestamp += 0.03;
    index++;
  }

  system.finalize();
  return 0;
}
