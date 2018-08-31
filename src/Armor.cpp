#include "Armor.h"
double tic()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((float)t.tv_sec + ((float)t.tv_usec) / 1000000.);
}

// sort lights by x
bool less_x(const Light & l1, const Light & l2) {
    return l1.rect.center.x < l2.rect.center.x;
}

Armor::Armor()
    : tracker(false, true, false, false){};

void Armor::init()
{
    // init serial
    serial.init();

    // fps
    timer = tic();

    // state machine
    state = FAST_EXPLORE;

    found_ctr   = 0;
    unfound_ctr = 0;
#if BAYER_HACK == HACKING_OFF
    srcW = 640;
    srcH = 480;
#elif BAYER_HACK == HACKING_ON
    srcW = 320;
    srcH = 240;
#endif

    // track
    BORDER_IGNORE = 10;
    BOX_EXTRA     = 10;

#if BAYER_HACK == HACKING_OFF
    GRAY_THRESH   = 240;
#elif BAYER_HACK == HACKING_ON
    GRAY_THRESH   = 10;
#endif

    // select contours
#if BAYER_HACK == HACKING_OFF
    CONTOUR_AREA_MIN     = 30;//20
    CONTOUR_AREA_MAX     = 3000;//2000
    CONTOUR_LENGTH_MIN   = 10;//20
#elif BAYER_HACK == HACKING_ON
    CONTOUR_AREA_MIN     = 5;//20
    CONTOUR_AREA_MAX     = 1000;//2000
    CONTOUR_LENGTH_MIN   = 5;//20
#endif
    CONTOUR_HW_RATIO_MIN = 1.0;//2.5
    SLOW_CONTOUR_HW_RATIO_MIN = 3.0;//2.5
    CONTOUR_HW_RATIO_MAX = 15;
    SLOW_CONTOUR_HW_RATIO_MAX = 7;
    CONTOUR_ANGLE_MAX    = 20.0;

    // pair lights
    TWIN_ANGEL_MAX        = 5.001;
    TWIN_LENGTH_RATIO_MAX = 2.0;
    SLOW_TWIN_LENGTH_RATIO_MAX = 1.2;
    TWIN_DISTANCE_N_MIN   = 1.3;//1.7
    SLOW_TWIN_DISTANCE_N_MIN   = 2.0;//1.7
    TWIN_DISTANCE_N_MAX   = 3.8;//3.8
    SLOW_TWIN_DISTANCE_N_MAX   = 2.6;//1.7
    TWIN_DISTANCE_T_MAX   = 1.4;
    TWIN_AREA_MAX         = 1.2;

    // state machine
    FAST_EXPLORE_TRACK_THRES     = 1;
    FAST_EXPLORE_SEND_STOP_THRES = 5;
    FAST_TRACK_SLOW_THRES       = 1; //3
    //FAST_TRACK_CHECK_RATIO       = 0.4;
    FAST_TRACK_EXPLORE_THRES     = 1; //2

    SLOW_EXPLORE_TRACK_THRES     = 1;
    SLOW_EXPLORE_SEND_STOP_THRES = 5;
    SLOW_TRACK_CHECK_THRES       = 3;
    SLOW_TRACK_CHECK_RATIO       = 0.4;
    SLOW_TRACK_EXPLORE_THRES     = 3;

    ARMOR_CLASS = NOT_FOUND;
}

int Armor::run(Mat& frame)
{
    if (frame.empty())
        return -1;
#if VIDEO == VIDEO_FILE
    cvtColor(frame, frame, CV_BGR2GRAY);
#endif
#if BAYER_HACK == HACKING_ON
    static Mat blue(frame.rows/2, frame.cols/2, CV_8UC1);
    static Mat red(frame.rows/2, frame.cols/2, CV_8UC1);
    splitBayerBG(frame, blue, red);
    //frame = blue - red;
    frame = red - blue;
#endif
#if DRAW == SHOW_ALL
    imshow("frame", frame);
#endif

    if (state == FAST_EXPLORE) {
        if (fastExplore(frame)) {
            ++found_ctr;
            unfound_ctr = 0;
        } else {
            ++unfound_ctr;
            found_ctr = 0;
        }

        if (found_ctr >= FAST_EXPLORE_TRACK_THRES) {
            serial.sendTarget((bbox.x + bbox.width / 2),
                    (bbox.y + bbox.height / 2), ARMOR_CLASS);
            // init track with this frame
            // otherwise, if use next frame, the area may change
            trackInit(frame);
            bbox_last   = bbox;
            transferState(FAST_TRACK);
        }
        if (unfound_ctr >= FAST_EXPLORE_SEND_STOP_THRES) {
            serial.sendTarget(srcW / 2, srcH / 2, NOT_FOUND);
            found_ctr   = 0;
            unfound_ctr = 0;
        }
    } else if (state == FAST_TRACK) {
        if (track(frame)) {
            int x        = bbox.x + bbox.width / 2;
            int y        = bbox.y + bbox.height / 2;
            int x_last   = bbox_last.x + bbox_last.width / 2;
            int center_x = 2 * x - srcW / 2;
            int center_y = 2 * y - srcH / 2;
            // Assume the box run at const velocity
            // Predict if the center is still in box at next frame
            if (bbox_last.x < center_x
                    && center_x < bbox_last.x + bbox_last.width
                    && bbox_last.y < center_y
                    && center_y < bbox_last.y + bbox_last.height) {
                // if center is in box, predict it run at const velocity
                serial.sendTarget(2 * x - x_last, y, ARMOR_CLASS);
            } else {
                serial.sendTarget(x, y, ARMOR_CLASS);
            }
            ++found_ctr;
            unfound_ctr = 0;
            bbox_last   = bbox;
        } else {
            ++unfound_ctr;
            found_ctr = 0;
        }

        if (found_ctr >= FAST_TRACK_SLOW_THRES) {
            // check whether the robot slows down
            if (srcW/2 - 30 < bbox.x + bbox.width/2 
                    && bbox.x + bbox.width/2 < srcW/2 + 30) {
                transferState(SLOW_EXPLORE);
            }

            // check whether tracking the wrong area
            Mat roi = frame.clone()(bbox);
            threshold(roi, roi, GRAY_THRESH, 255, THRESH_BINARY);
            if (countNonZero(roi) 
                    < SLOW_TRACK_CHECK_RATIO * total_contour_area) {
                serial.sendTarget(srcW / 2, srcH / 2, NOT_FOUND);
                transferState(FAST_EXPLORE);
            }

            // add for secure
            // if it stay FAST TRACK too long, 
            // it means the target beyond the shooting range
            // send NOT FOUND LEAVE IMMEDIATELY during frame 500~800, about 3 seconds
            if (found_ctr >= 500) {
                serial.sendTarget(srcW / 2, srcH / 2, NOT_FOUND_LEAVE);
                if (found_ctr >= 800) {
                    transferState(FAST_EXPLORE);
                }
            }
        }

        // sometimes, tracker only miss 1 frame
        if (unfound_ctr >= FAST_TRACK_EXPLORE_THRES) {
            transferState(FAST_EXPLORE);
        }
#if DRAW == SHOW_ALL
        // Draw the tracked object
        rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
        // Display frame.
        imshow("Tracking", frame);
#endif
    } else if (state == SLOW_EXPLORE) {
        if (slowExplore(frame)) {
            ++found_ctr;
            unfound_ctr = 0;
        } else {
            ++unfound_ctr;
            found_ctr = 0;
        }

        if (found_ctr >= SLOW_EXPLORE_TRACK_THRES) {
            cout << "Find: " << ARMOR_CLASS << endl;
            serial.sendTarget((bbox.x + bbox.width / 2),
                    (bbox.y + bbox.height / 2), ARMOR_CLASS);
            trackInit(frame);
            bbox_last   = bbox;
            transferState(SLOW_TRACK);
        }
        if (unfound_ctr >= SLOW_EXPLORE_SEND_STOP_THRES) {
            serial.sendTarget(srcW / 2, srcH / 2, NOT_FOUND_LEAVE);
            transferState(FAST_EXPLORE);
        }
    } else if (state == SLOW_TRACK) {
        if (track(frame)) {
            int x        = bbox.x + bbox.width / 2;
            int y        = bbox.y + bbox.height / 2;
            int x_last   = bbox_last.x + bbox_last.width / 2;
            int center_x = 2 * x - srcW / 2;
            int center_y = 2 * y - srcH / 2;
            // Assume the box run at const velocity
            // Predict if the center is still in box at next frame
            if (bbox_last.x < center_x
                    && center_x < bbox_last.x + bbox_last.width
                    && bbox_last.y < center_y
                    && center_y < bbox_last.y + bbox_last.height) {
                // if center is in box, predict it run at const velocity
                serial.sendTarget(2 * x - x_last, y, ARMOR_CLASS);
            } else {
                serial.sendTarget(x, y, ARMOR_CLASS);
            }
            ++found_ctr;
            unfound_ctr = 0;
            bbox_last   = bbox;
        } else {
            ++unfound_ctr;
            found_ctr = 0;
        }

        // check if the box is still tracking armor
        if (found_ctr >= SLOW_TRACK_CHECK_THRES) {
            Mat roi = frame.clone()(bbox);
            threshold(roi, roi, GRAY_THRESH, 255, THRESH_BINARY);
            if (countNonZero(roi) < SLOW_TRACK_CHECK_RATIO * total_contour_area) {
                serial.sendTarget(srcW / 2, srcH / 2, NOT_FOUND);
                transferState(FAST_EXPLORE);
            }
        }

        // sometimes, tracker only miss 1 frame
        if (unfound_ctr >= SLOW_TRACK_EXPLORE_THRES) {
            transferState(FAST_EXPLORE);
        }
#if DRAW == SHOW_ALL
        // Draw the tracked object
        rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
        // Display frame.
        imshow("Tracking", frame);
#endif
    }
    float fps = 1 / (tic() - timer);
    cout << "fps: " << fps << endl;
    timer = tic();
    return 0;
}

void Armor::transferState(State s)
{
    found_ctr   = 0;
    unfound_ctr = 0;
    state = s;
    cout << "Transfer to State: " << s << endl;
}

void Armor::trackInit(Mat& frame)
{
#if DRAW == SHOW_ALL
    // Display bounding box.
    rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
    imshow("TrackInit", frame);
#endif
    tracker.init(bbox, frame);
}

bool Armor::track(Mat& frame)
{
    // Update the tracking result
    bool ok = true;
    bbox = tracker.update(frame);
    if (bbox.x < BORDER_IGNORE || bbox.y < BORDER_IGNORE
        || bbox.x + bbox.width > srcW - BORDER_IGNORE
        || bbox.y + bbox.height > srcH - BORDER_IGNORE) {
        ok = false;
    }
    return ok;
}

//bool Armor::explore(Mat& frame)
//{
    //static Mat bin;
    //threshold(frame, bin, GRAY_THRESH, 255, THRESH_BINARY);
//#if DRAW == SHOW_ALL
    //imshow("gray", bin);
//#endif
    //vector<vector<Point> > contours;
    //vector<long> areas;
    //vector<Light> lights;
    //findContours(bin, contours,
        //CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    ////select contours by area, length, width/height
    //for (unsigned int i = 0; i < contours.size(); ++i) {
        //long area = contourArea(contours.at(i));
        ////cout << "area:" << area << endl;
        //if (area > CONTOUR_AREA_MAX
                //|| area < CONTOUR_AREA_MIN) {
//#  if DRAW == SHOW_ALL
            //drawContours(bin, contours, i, Scalar(50), CV_FILLED);
//#  endif
            //continue;
        //}
        //RotatedRect rec = minAreaRect(contours.at(i));
        //Size2f size     = rec.size;
        //// get a (longer) as length
        //float a = size.height > size.width
            //? size.height
            //: size.width;
        //float b = size.height < size.width
            //? size.height
            //: size.width;
        ////cout << "length: " << a << endl;
        //if (a < CONTOUR_LENGTH_MIN) {
            //continue;
        //}
        ////check if it is thin
        ////cout << "a / b: " << a / b << endl;
        //if (a / b > CONTOUR_HW_RATIO_MAX
                //|| a / b < CONTOUR_HW_RATIO_MIN) {
//#  if DRAW == SHOW_ALL
            //drawContours(bin, contours, i, Scalar(100), CV_FILLED);
//#  endif
            //continue;
        //}

        ////cout << "Area Ratio: " << area / size.area() << endl;
        //if (area/size.area() < 0.7)
            //continue;

        ////float angle = rec.angle;
        ////angle = - angle;
        ////if (size.width < size.height)		
            ////angle += 90.0;		
        ////cout << "RotatedRect: " << angle << endl;
        //LeastSquare leasq(contours[i]);
        ////cout << "LeastSquare: " << leasq.getAngle() << " | " << leasq.getAngleh() << endl;
        //float angle = leasq.getAngleh();
        //if (angle > 90.0 + CONTOUR_ANGLE_MAX  
                //|| angle < 90.0 - CONTOUR_ANGLE_MAX)
            //continue;
        ////cout << "push back" << endl;
        //lights.push_back(Light(rec, contours[i], angle, area));
        //areas.push_back(area);
    //}
    //if (lights.size() < 2)
        //return false;
    //int light1 = -1, light2 = -1;
    //float min_angle = TWIN_ANGEL_MAX;
    //float min_similarity = 3.0;
    //sort(lights.begin(), lights.end(), less_x);
    //// cout << "lights: " << lights.size() << endl;
    //// pair lights by length, distance, angel
    //for (unsigned int i = 0; i < lights.size()-1; ++i) {
        //int j = i + 1;
        //Point2f pi   = lights.at(i).rect.center;
        //Point2f pj   = lights.at(j).rect.center;
        //Size2f sizei = lights.at(i).rect.size;
        //Size2f sizej = lights.at(j).rect.size;
        //float ai = sizei.height > sizei.width
            //? sizei.height
            //: sizei.width;
        //float aj = sizej.height > sizej.width
            //? sizej.height
            //: sizej.width;
        //// length similar
        ////cout << "Twin length: " << ai/aj << endl;
        //if (ai / aj > TWIN_LENGTH_RATIO_MAX
                //|| aj / ai > TWIN_LENGTH_RATIO_MAX)
            //continue;

        //float anglei = lights[i].angle;
        //float anglej = lights[j].angle;
        //cout << "light(i) angle:" << anglei
            //<<" light(j) angle" << anglej <<endl;		
        //float similarity = matchShapes(lights[i].contour, lights[j].contour, CV_CONTOURS_MATCH_I2, 0);
        //cout << "Similar: " << similarity << endl;
        //if (similarity > min_similarity) {
            //continue;
        //}
        //min_similarity = similarity * 2;
        //min_similarity = min_similarity < 3.0 ? min_similarity : 3.0;
        //if (abs(anglei - anglej) > min_angle) {
            //continue;
        //}
        //min_angle = abs(anglei - anglej);
        //min_angle = min_angle < TWIN_ANGEL_MAX ? min_angle : TWIN_ANGEL_MAX;

        //float distance_n = abs((pi.x - pj.x) * cos((anglei + 90) * PI / 180)
            //+ (pi.y - pj.y) * sin((anglei + 90) * PI / 180));
        //// normal distance range in about 1 ~ 2 times of length
        //// cout << "Distance n: " << distance_n / ai << endl;
        //// add the large armor on hero, which should be 3 ~ 4 times of length. Maybe negative influence on small armor detection.
        //if (distance_n < TWIN_DISTANCE_N_MIN * ai || distance_n > 2 * TWIN_DISTANCE_N_MAX * ai
            //|| distance_n < TWIN_DISTANCE_N_MIN * aj || distance_n > 2 * TWIN_DISTANCE_N_MAX * aj) {
//#if DRAW == SHOW_ALL
            //drawContours(bin, contours, i, Scalar(150), CV_FILLED);
            //drawContours(bin, contours, j, Scalar(150), CV_FILLED);
//#endif
            //continue;
        //}
        //if (distance_n > 1.0 * TWIN_DISTANCE_N_MAX * ai
                //&& distance_n > 1.0 * TWIN_DISTANCE_N_MAX * aj) {
            //ARMOR_CLASS = LARGE_ARMOR;
            //cout << "Hero!" << endl;
            //cout << "Distance n: " << distance_n / ai << endl;
        //} else {
            //ARMOR_CLASS = SMALL_ARMOR;
        //}
        //// direction distance should be small
        //float distance_t = abs((pi.x - pj.x) * cos((anglei)*PI / 180)
            //+ (pi.y - pj.y) * sin((anglei)*PI / 180));
        ////cout << "Distance t: " << distance_t / ai << endl;
        //if (distance_t > TWIN_DISTANCE_T_MAX * ai || distance_t > TWIN_DISTANCE_T_MAX * aj) {
//#if DRAW == SHOW_ALL
            //drawContours(bin, contours, i, Scalar(150), CV_FILLED);
            //drawContours(bin, contours, j, Scalar(150), CV_FILLED);
//#endif
            //continue;
        //}
        ////min_similarity = similarity;
        //light1 = i;
        //light2 = j;
        ////min_angle = 1.0;

        ////float anglei = lights[i].angle;
        ////float anglej = lights[j].angle;
        //////cout << "light(i) angle:" << anglei
            //////<<" light(j) angle" << anglej <<endl;		
        //////cout << "light i x: " << lights[i].rect.center.x
            //////<< " light j x: " << lights[j].rect.center.x << endl;

        ////if (abs(anglei - anglej) < min_angle) {
            ////float distance_n = abs((pi.x - pj.x) * cos((anglei + 90) * PI / 180)
                ////+ (pi.y - pj.y) * sin((anglei + 90) * PI / 180));
            ////// normal distance range in about 1 ~ 2 times of length
            ////// cout << "Distance n: " << distance_n / ai << endl;
            ////// add the large armor on hero, which should be 3 ~ 4 times of length. Maybe negative influence on small armor detection.
            ////if (distance_n < TWIN_DISTANCE_N_MIN * ai || distance_n > 2 * TWIN_DISTANCE_N_MAX * ai
                ////|| distance_n < TWIN_DISTANCE_N_MIN * aj || distance_n > 2 * TWIN_DISTANCE_N_MAX * aj) {
////#if DRAW == SHOW_ALL
                ////drawContours(bin, contours, i, Scalar(150), CV_FILLED);
                ////drawContours(bin, contours, j, Scalar(150), CV_FILLED);
////#endif
                ////continue;
            ////}
            ////if (distance_n > 1.0 * TWIN_DISTANCE_N_MAX * ai
                    ////&& distance_n > 1.0 * TWIN_DISTANCE_N_MAX * aj) {
                ////ARMOR_CLASS = LARGE_ARMOR;
                ////cout << "Hero!" << endl;
                ////cout << "Distance n: " << distance_n / ai << endl;
            ////} else {
                ////ARMOR_CLASS = SMALL_ARMOR;
            ////}
            ////// direction distance should be small
            ////float distance_t = abs((pi.x - pj.x) * cos((anglei)*PI / 180)
                ////+ (pi.y - pj.y) * sin((anglei)*PI / 180));
            //////cout << "Distance t: " << distance_t / ai << endl;
            ////if (distance_t > TWIN_DISTANCE_T_MAX * ai || distance_t > TWIN_DISTANCE_T_MAX * aj) {
////#if DRAW == SHOW_ALL
                ////drawContours(bin, contours, i, Scalar(150), CV_FILLED);
                ////drawContours(bin, contours, j, Scalar(150), CV_FILLED);
////#endif
                ////continue;
            ////}
            ////light1    = i;
            ////light2    = j;
            ////min_angle = abs(anglei - anglej);
        ////}

    //}
//#if DRAW == SHOW_ALL
    //imshow("gray", bin);
//#endif
    //if (light1 == -1 || light2 == -1 || min_angle == TWIN_ANGEL_MAX)
        //return false;
    ////cout << "min i:" << light1 << " j:" << light2 << " angel:" << min_angle << endl;
    //// get and extend box for track init
    //Rect2d reci = lights.at(light1).rect.boundingRect();
    //Rect2d recj = lights.at(light2).rect.boundingRect();
    ////cout << "L1 i, j Similar: " << matchShapes(lights[light1].contour, lights[light2].contour, CV_CONTOURS_MATCH_I1, 0) << endl;
    ////cout << "L1 1, 2 Similar: " << matchShapes(lights[0].contour, lights[1].contour, CV_CONTOURS_MATCH_I1, 0) << endl;
    ////cout << "L2 i, j Similar: " << matchShapes(lights[light1].contour, lights[light2].contour, CV_CONTOURS_MATCH_I2, 0) << endl;
    ////cout << "L2 1, 2 Similar: " << matchShapes(lights[0].contour, lights[1].contour, CV_CONTOURS_MATCH_I2, 0) << endl;
    ////cout << "L3 i, j Similar: " << matchShapes(lights[light1].contour, lights[light2].contour, CV_CONTOURS_MATCH_I3, 0) << endl;
    ////cout << "L3 1, 2 Similar: " << matchShapes(lights[0].contour, lights[1].contour, CV_CONTOURS_MATCH_I3, 0) << endl;
    //float min_x, min_y, max_x, max_y;
    //if (reci.x < recj.x) {
        //min_x = reci.x;
        //max_x = recj.x + recj.width;
    //} else {
        //min_x = recj.x;
        //max_x = reci.x + reci.width;
    //}
    //if (reci.y < recj.y) {
        //min_y = reci.y;
        //max_y = recj.y + recj.height;
    //} else {
        //min_y = recj.y;
        //max_y = reci.y + reci.height;
    //}
    //min_x -= BOX_EXTRA;
    //max_x += BOX_EXTRA;
    //min_y -= BOX_EXTRA;
    //max_y += BOX_EXTRA;
    //if (min_x < 0 || max_x > srcW || min_y < 0 || max_y > srcH) {
        //return false;
    //}
    //bbox = Rect2d(min_x, min_y,
        //max_x - min_x, max_y - min_y);
    //total_contour_area = areas.at(light1) + areas.at(light2);
//#if DRAW == SHOW_ALL
    //rectangle(bin, bbox, Scalar(255), 3);
    //imshow("gray", bin);
//#endif
    //return true;
//}

bool Armor::fastExplore(Mat& frame)
{
    //lights.clear();
    //areas.clear();
    vector<Light> lights;
    if(fastSelectContours(frame, lights) == false)
        return false;
    return fastPairContours(lights);
}

bool Armor::fastSelectContours(Mat& frame, vector<Light>& lights)
{
    static Mat bin;
    threshold(frame, bin, GRAY_THRESH, 255, THRESH_BINARY);
#if DRAW == SHOW_ALL
    imshow("gray", bin);
#endif
    vector<vector<Point> > contours;
    //areas.clear();
    //lights.clear();
    findContours(bin, contours,
        CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    //select contours by area, length, width/height
    for (unsigned int i = 0; i < contours.size(); ++i) {
        float area = contourArea(contours.at(i));
        //cout << "area:" << area << endl;
        if (area > CONTOUR_AREA_MAX
                || area < CONTOUR_AREA_MIN) {
#  if DRAW == SHOW_ALL
            drawContours(bin, contours, i, Scalar(50), CV_FILLED);
#  endif
            continue;
        }
        RotatedRect rec = minAreaRect(contours.at(i));
        Size2f size     = rec.size;
        // get a (longer) as length
        float a = size.height > size.width
            ? size.height
            : size.width;
        float b = size.height < size.width
            ? size.height
            : size.width;
        //cout << "length: " << a << endl;
        if (a < CONTOUR_LENGTH_MIN) {
            continue;
        }
        //check if it is thin
        //cout << "a / b: " << a / b << endl;
        if (a / b > CONTOUR_HW_RATIO_MAX
                || a / b < CONTOUR_HW_RATIO_MIN) {
#  if DRAW == SHOW_ALL
            drawContours(bin, contours, i, Scalar(100), CV_FILLED);
#  endif
            continue;
        }

        //cout << "Area Ratio: " << area / size.area() << endl;
        if (area/size.area() < 0.6)
            continue;

        //cout << "RotatedRect: " << angle << endl;
        LeastSquare leasq(contours[i]);
        //cout << "LeastSquare: " << leasq.getAngle() << " | " << leasq.getAngleh() << endl;
        float angle = leasq.getAngleh();
        if (angle > 120.0
                || angle < 60.0)
            continue;
        //cout << "push back" << endl;
        lights.push_back(Light(rec, contours[i], angle, area));
        //areas.push_back(area);
    }
    if (lights.size() < 2)
        return false;
    return true;
}

bool Armor::fastPairContours(vector<Light>& lights)
{

    int light1 = -1, light2 = -1;
    float min_similarity = 3.0;
    sort(lights.begin(), lights.end(), less_x);
    // cout << "lights: " << lights.size() << endl;
    // pair lights by length, distance, angel
    for (unsigned int i = 0; i < lights.size()-1; ++i) {
        int j = i + 1;
        Point2f pi   = lights.at(i).rect.center;
        Point2f pj   = lights.at(j).rect.center;
        Size2f sizei = lights.at(i).rect.size;
        Size2f sizej = lights.at(j).rect.size;
        float ai = sizei.height > sizei.width
            ? sizei.height
            : sizei.width;
        float aj = sizej.height > sizej.width
            ? sizej.height
            : sizej.width;
        // length similar
        //cout << "Twin length: " << ai/aj << endl;
        if (ai / aj > TWIN_LENGTH_RATIO_MAX
                || aj / ai > TWIN_LENGTH_RATIO_MAX)
            continue;

        float anglei = lights[i].angle;
        float anglej = lights[j].angle;
        //cout << "light(i) angle:" << anglei
            //<<" light(j) angle" << anglej <<endl;		
        float similarity = matchShapes(lights[i].contour, lights[j].contour, CV_CONTOURS_MATCH_I2, 0);
        //cout << "Similar: " << similarity << endl;
        if (similarity > min_similarity) {
            continue;
        }
        if (abs(anglei - anglej) > 10.0) {
            continue;
        }

        float distance_n = abs((pi.x - pj.x) * cos((anglei + 90) * PI / 180)
            + (pi.y - pj.y) * sin((anglei + 90) * PI / 180));
        // normal distance range in about 1 ~ 2 times of length
         //cout << "Distance n: " << distance_n / ai << endl;
        // add the large armor on hero, which should be 3 ~ 4 times of length. Maybe negative influence on small armor detection.
        if (distance_n < TWIN_DISTANCE_N_MIN * ai || distance_n > 2 * TWIN_DISTANCE_N_MAX * ai
            || distance_n < TWIN_DISTANCE_N_MIN * aj || distance_n > 2 * TWIN_DISTANCE_N_MAX * aj) {
            continue;
        }
        min_similarity = similarity ;
        light1 = i;
        light2 = j;
        ARMOR_CLASS = SMALL_ARMOR;
    }

    if (light1 == -1 || light2 == -1 )
        return false;
    //cout << "min i:" << light1 << " j:" << light2 << " angel:" << min_angle << endl;
    // get and extend box for track init
    Rect2d reci = lights.at(light1).rect.boundingRect();
    Rect2d recj = lights.at(light2).rect.boundingRect();
    float min_x, min_y, max_x, max_y;
    if (reci.x < recj.x) {
        min_x = reci.x;
        max_x = recj.x + recj.width;
    } else {
        min_x = recj.x;
        max_x = reci.x + reci.width;
    }
    if (reci.y < recj.y) {
        min_y = reci.y;
        max_y = recj.y + recj.height;
    } else {
        min_y = recj.y;
        max_y = reci.y + reci.height;
    }
    min_x -= BOX_EXTRA;
    max_x += BOX_EXTRA;
    min_y -= BOX_EXTRA;
    max_y += BOX_EXTRA;
    if (min_x < 0 || max_x > srcW || min_y < 0 || max_y > srcH - 20) {
        return false;
    }
    bbox = Rect2d(min_x, min_y,
        max_x - min_x, max_y - min_y);
    total_contour_area = lights.at(light1).area + lights.at(light2).area;
    return true;
}

bool Armor::slowExplore(Mat& frame)
{
    vector<Light> lights;
    if (slowSelectContours(frame, lights) == false)
        return false;
    return slowPairContours(lights);
}

bool Armor::slowSelectContours(Mat& frame, vector<Light>& lights)
{
    static Mat bin;
    threshold(frame, bin, GRAY_THRESH, 255, THRESH_BINARY);
#if DRAW == SHOW_ALL
    imshow("gray", bin);
#endif
    vector<vector<Point> > contours;
    //areas.clear();
    //lights.clear();
    findContours(bin, contours,
        CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    //select contours by area, length, width/height
    for (unsigned int i = 0; i < contours.size(); ++i) {
        float area = contourArea(contours.at(i));
        //cout << "area:" << area << endl;
        if (area > CONTOUR_AREA_MAX
                || area < CONTOUR_AREA_MIN) {
#  if DRAW == SHOW_ALL
            drawContours(bin, contours, i, Scalar(50), CV_FILLED);
#  endif
            continue;
        }
        RotatedRect rec = minAreaRect(contours.at(i));
        Size2f size     = rec.size;
        // get a (longer) as length
        float a = size.height > size.width
            ? size.height
            : size.width;
        float b = size.height < size.width
            ? size.height
            : size.width;
        //cout << "length: " << a << endl;
        if (a < CONTOUR_LENGTH_MIN) {
            continue;
        }
        //check if it is thin
        if (a / b > SLOW_CONTOUR_HW_RATIO_MAX
                || a / b < SLOW_CONTOUR_HW_RATIO_MIN) {
#  if DRAW == SHOW_ALL
            drawContours(bin, contours, i, Scalar(100), CV_FILLED);
#  endif
            continue;
        }
        //cout << "a / b: " << a / b << endl;

        //cout << "Area Ratio: " << area / size.area() << endl;
        if (area/size.area() < 0.7)
            continue;

        float angle = -rec.angle;
        //cout << "RotatedRect: " << angle << endl;
        if (size.width < size.height)		
            angle += 90.0;		
        if (angle > 90.0 + CONTOUR_ANGLE_MAX  
                || angle < 90.0 - CONTOUR_ANGLE_MAX)
            continue;
        //cout << "push back" << endl;
        lights.push_back(Light(rec, contours[i], angle, area));
        //areas.push_back(area);
    }
    if (lights.size() < 2)
        return false;
    return true;
}

bool Armor::slowPairContours(vector<Light>& lights)
{

    int light1 = -1, light2 = -1;
    float min_angle = TWIN_ANGEL_MAX;
    float min_distance_n = 2 * SLOW_TWIN_DISTANCE_N_MAX;
    sort(lights.begin(), lights.end(), less_x);
    // cout << "lights: " << lights.size() << endl;
    // pair lights by length, distance, angel
    for (unsigned int i = 0; i < lights.size()-1; ++i) {
        int j = i + 1;
        Point2f pi   = lights.at(i).rect.center;
        Point2f pj   = lights.at(j).rect.center;
        Size2f sizei = lights.at(i).rect.size;
        Size2f sizej = lights.at(j).rect.size;
        float ai = sizei.height > sizei.width
            ? sizei.height
            : sizei.width;
        float aj = sizej.height > sizej.width
            ? sizej.height
            : sizej.width;
        // length similar
        if (ai / aj > SLOW_TWIN_LENGTH_RATIO_MAX
                || aj / ai > SLOW_TWIN_LENGTH_RATIO_MAX)
            continue;
        //cout << "Twin length: " << ai/aj << endl;

        float anglei = lights[i].angle;
        float anglej = lights[j].angle;
        cout << "light(i) angle:" << anglei
            <<" light(j) angle" << anglej <<endl;		
        if (anglei > 90 && 90 > anglej) {
            continue;
        }
        if (anglei < 90 && 90 < anglej) {
            continue;
        }
        if (abs(anglei - anglej) > min_angle) {
            continue;
        }

        float distance_n = abs((pi.x - pj.x) * cos((anglei + 90) * PI / 180)
            + (pi.y - pj.y) * sin((anglei + 90) * PI / 180));
        // normal distance range in about 1 ~ 2 times of length
        // add the large armor on hero, which should be 3 ~ 4 times of length. Maybe negative influence on small armor detection.
        ai = (ai + aj) / 2;
        //cout << "Distance n: " << distance_n / ai << endl;
        if (distance_n > min_distance_n * ai) {
            continue;
        }
        if (distance_n < SLOW_TWIN_DISTANCE_N_MIN * ai) {
            continue;
        }
        if (distance_n > SLOW_TWIN_DISTANCE_N_MAX * ai
                && distance_n < 2 * SLOW_TWIN_DISTANCE_N_MIN * ai ) {
            continue;
        }
        if (distance_n > 2 * SLOW_TWIN_DISTANCE_N_MAX * ai) {
            continue;
        }

        // direction distance should be small
        float distance_t = abs((pi.x - pj.x) * cos((anglei)*PI / 180)
            + (pi.y - pj.y) * sin((anglei)*PI / 180));
        //cout << "Distance t: " << distance_t / ai << endl;
        if (distance_t > TWIN_DISTANCE_T_MAX * ai || distance_t > TWIN_DISTANCE_T_MAX * aj) {
            continue;
        }

        light1 = i;
        light2 = j;
        min_angle = abs(anglei - anglej);
        min_distance_n = distance_n / ai;
        if (distance_n > 1.0 * TWIN_DISTANCE_N_MAX * ai) {
            ARMOR_CLASS = LARGE_ARMOR;
            cout << "Hero!" << endl;
            cout << "Distance n: " << distance_n / ai << endl;
        } else {
            ARMOR_CLASS = SMALL_ARMOR;
            cout << "Infanity" << endl;
        }
    }

    if (light1 == -1 || light2 == -1 )
        return false;
    //cout << "min i:" << light1 << " j:" << light2 << " angel:" << min_angle << endl;
    // get and extend box for track init
    Rect2d reci = lights.at(light1).rect.boundingRect();
    Rect2d recj = lights.at(light2).rect.boundingRect();
    float min_x, min_y, max_x, max_y;
    if (reci.x < recj.x) {
        min_x = reci.x;
        max_x = recj.x + recj.width;
    } else {
        min_x = recj.x;
        max_x = reci.x + reci.width;
    }
    if (reci.y < recj.y) {
        min_y = reci.y;
        max_y = recj.y + recj.height;
    } else {
        min_y = recj.y;
        max_y = reci.y + reci.height;
    }
    min_x -= BOX_EXTRA;
    max_x += BOX_EXTRA;
    min_y -= BOX_EXTRA;
    max_y += BOX_EXTRA;
    if (min_x < 0 || max_x > srcW || min_y < 0 || max_y > srcH) {
        return false;
    }
    bbox = Rect2d(min_x, min_y,
        max_x - min_x, max_y - min_y);
    total_contour_area = lights.at(light1).area + lights.at(light2).area;
    return true;
}

void Armor::splitBayerBG(Mat& frame, Mat& blue, Mat& red)
{
    uchar* data;
    uchar* bayer_data[2];
    for (int i=0; i<frame.rows; ++i) {
        data = frame.ptr<uchar>(i);
        bayer_data[0] = blue.ptr<uchar>(i/2);
        for (int j=0; j<blue.cols; ++j, data+=2) {
            bayer_data[0][j] = *data;
        }
        data = frame.ptr<uchar>(++i)+1;
        bayer_data[1] = red.ptr<uchar>(i/2);
        for (int j=0; j<red.cols; ++j, data+=2) {
            bayer_data[1][j] = *data;
        }
    }

}
