#include "Serial.h"
#include "kcftracker.hpp"
#include "precom.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <sys/time.h>
#include <algorithm>

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
    float CONTOUR_ANGLE_MAX;

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

struct Light {
    RotatedRect rect;
    vector<Point> contour;
    float angle;
    Light(RotatedRect & r, vector<Point>& c, float & ag):
        rect(r), contour(c), angle(ag){};
};
class LeastSquare		
{		
public:		
  bool is_kxb;		
  float tx, ty;		
  float k,b,kh,bh; //斜率k,截距b		
  LeastSquare(vector<Point>& point)  //构造函数，输入点，得到斜率		
  {		
    vector<int> x;		
    vector<int> y;		
    for(unsigned int i=0;i<point.size();++i){		
      x.push_back(point[i].x);		
      y.push_back(point[i].y);		
    }		
    float t1= 0.0, t2= 0.0, t3= 0.0, t4= 0.0, t5= 0.0;		
    for(unsigned int i= 0; i < x.size(); ++i)		
    {		
      t1+= x[i] * x[i];		
      t2+= x[i];		
      t3+= x[i] * y[i];		
      t4+= y[i];		
      t5+= y[i] * y[i];		
    }		
    k= (t3 * x.size() - t2 * t4) / (t1 * x.size() - t2 * t2);		
    //b= (t1 * t4 - t2 * t3) / (t1 * x.size() - t2 * t2);		
    kh= (t3 * x.size() - t2 * t4) / (t5 * x.size() - t4 * t4);		
    //bh= (t5 * t2 - t4 * t3) / (t5 * x.size() - t4 * t4);		
  }		
  float getAngle(){		
    return atan(k)*180/PI;		
  }		
  float getAngleh(){		
    return 90-atan(kh)*180/PI;		
  }		
  float getFinalAngle(){		
    if(k<1)		
        return getAngleh();		
    else		
        return getAngle();		
  }		
};
