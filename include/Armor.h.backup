#include "Serial.h"
#include "kcftracker.hpp"
#include "precom.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <sys/time.h>

#define NOT_FOUND 0
#define FOUND_BORDER 1
#define FOUND_CENTER 2

using namespace cv;
using namespace std;

class Armor {
private:
    enum State {
        EXPLORE,
        TRACK_INIT,
        TRACK
    } state;
    Rect2d bbox;
    Rect2d bbox_last;
    KCFTracker tracker;
    Serial serial;
    double timer;
    long found_ctr;
    long unfound_ctr;

    int srcW, srcH;

    int BORDER_IGNORE;
    int BOX_EXTRA;

    int GRAY_THRESH;

    long CONTOUR_AREA_MAX;
    long CONTOUR_AREA_MIN;
    int CONTOUR_LENGTH_MIN;
    float CONTOUR_HW_RATIO_MAX;
    float CONTOUR_HW_RATIO_MIN;

    float TWIN_ANGEL_MAX;
    float TWIN_LENGTH_RATIO_MAX;
    float TWIN_DISTANCE_N_MIN;
    float TWIN_DISTANCE_N_MAX;
    float TWIN_DISTANCE_T_MAX;

    int EXPLORE_TRACK_THRES;
    int EXPLORE_SEND_STOP_THRES;
    int TRACK_CHECK_THRES;
    float TRACK_CHECK_RATIO;
    int TRACK_EXPLORE_THRES;

    long total_contour_area;

public:
    Armor();
    void init();
    int run(Mat& frame);

private:
    void transferState(State s);
    bool explore(Mat& frame);
    void trackInit(Mat& frame);
    bool track(Mat& frame);
};
