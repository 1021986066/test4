#if !defined(RM_ARMOR_H_)
#define RM_ARMOR_H_

#include "bayer_hack.h"
#include "kcftracker.hpp"
#include "least_square.h"
#include "uart.h"

#include <stdlib.h>
#include <sys/time.h>
#include <algorithm>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define BAYER_HACK HACKING_ON

#define SHOW_NO 0
#define SHOW_ALL 1

#ifdef _DEBUG
#define DRAW SHOW_ALL
#else  //_DEBUG
#define DRAW NO_SHOW
#endif  //_DEBUG

#define NOT_FOUND_LEAVE -1
#define NOT_FOUND 0
#define SMALL_ARMOR 1
#define LARGE_ARMOR 2

// the possible side light of armor
struct Light {
  cv::RotatedRect rect;
  std::vector<cv::Point> contour;
  float angle;
  float area;
  Light(cv::RotatedRect& r, std::vector<cv::Point>& c, float& ag, float& a)
      : rect(r), contour(c), angle(ag), area(a){};
  bool operator<(Light& l2) { return this->rect.center.x < l2.rect.center.x; }
  bool operator<=(Light& l2) { return this->rect.center.x <= l2.rect.center.x; }
  bool operator>(Light& l2) { return this->rect.center.x > l2.rect.center.x; }
  bool operator>=(Light& l2) { return this->rect.center.x >= l2.rect.center.x; }
};

// the overall class to detect armor
class Armor {
 private:
  enum FSMState {
    FAST_EXPLORE,
    FAST_TRACK,
    SLOW_EXPLORE,
    SLOW_TRACK
  } fsm_state_;

  cv::Rect2d armor_box_;
  cv::Rect2d armor_last_box_;
  KCFTracker kcf_tracker_;
  Uart uart_;
  double running_time_;
  long found_ctr_;
  long unfound_ctr_;

  int src_width_, src_height_;

  const int BORDER_IGNORE;
  const int BOX_EXTRA;

  int GRAY_THRESH;

  long CONTOUR_AREA_MAX;
  long CONTOUR_AREA_MIN;
  long CONTOUR_LENGTH_MIN;
  float CONTOUR_HW_RATIO_MAX;
  float SLOW_CONTOUR_HW_RATIO_MAX;
  float CONTOUR_HW_RATIO_MIN;
  float SLOW_CONTOUR_HW_RATIO_MIN;
  float CONTOUR_ANGLE_MAX;

  float TWIN_ANGEL_MAX;
  float TWIN_LENGTH_RATIO_MAX;
  float SLOW_TWIN_LENGTH_RATIO_MAX;
  float TWIN_DISTANCE_N_MIN;
  float SLOW_TWIN_DISTANCE_N_MIN;
  float TWIN_DISTANCE_N_MAX;
  float SLOW_TWIN_DISTANCE_N_MAX;
  float TWIN_DISTANCE_T_MAX;
  float TWIN_AREA_MAX;

  int FAST_EXPLORE_TRACK_THRES;
  int FAST_EXPLORE_SEND_STOP_THRES;
  int FAST_TRACK_SLOW_THRES;
  // float FAST_TRACK_CHECK_RATIO;
  int FAST_TRACK_EXPLORE_THRES;

  int SLOW_EXPLORE_TRACK_THRES;
  int SLOW_EXPLORE_SEND_STOP_THRES;
  int SLOW_TRACK_CHECK_THRES;
  float SLOW_TRACK_CHECK_RATIO;
  int SLOW_TRACK_EXPLORE_THRES;

  long total_contour_area;
  int armor_type_;

 public:
  Armor();
  void init();
  int run(cv::Mat& frame);

 private:
  void transferState(FSMState s);
  bool explore(cv::Mat& frame);
  bool fastExplore(cv::Mat& frame);
  bool fastSelectContours(cv::Mat& frame, std::vector<Light>& lights);
  bool fastPairContours(std::vector<Light>& lights);
  bool slowExplore(cv::Mat& frame);
  bool slowSelectContours(cv::Mat& frame, std::vector<Light>& lights);
  bool slowPairContours(std::vector<Light>& lights);
  void trackInit(cv::Mat& frame);
  bool track(cv::Mat& frame);
  void splitBayerBG(cv::Mat& frame, cv::Mat& blue, cv::Mat& red);
};

#endif  // RM_ARMOR_H_
