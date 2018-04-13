#include "Armor.h"
double tic()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((float)t.tv_sec + ((float)t.tv_usec) / 1000000.);
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
    state = EXPLORE;

    found_ctr = 0;
    unfound_ctr = 0;

    srcW = 640;
    srcH = 480;

    // track
    BORDER_IGNORE = 10;
    BOX_EXTRA = 10;

    GRAY_THRESH = 235;

    // select contours
    CONTOUR_AREA_MIN = 2;//20
    CONTOUR_AREA_MAX = 2000;//2000
    CONTOUR_LENGTH_MIN = 10;//20
    CONTOUR_HW_RATIO_MIN = 1.0;//2.5
    CONTOUR_HW_RATIO_MAX = 15;

    // pair lights
    TWIN_ANGEL_MAX = 5.001;
    TWIN_LENGTH_RATIO_MAX = 1.2;
    TWIN_DISTANCE_N_MIN = 0.5;//1.7
    TWIN_DISTANCE_N_MAX = 3.8;//3.8
    TWIN_DISTANCE_T_MAX = 1.4;

    // state machine
    EXPLORE_TRACK_THRES = 2;
    EXPLORE_SEND_STOP_THRES = 5;
    TRACK_CHECK_THRES = 3;
    TRACK_CHECK_RATIO = 0.4;
    TRACK_EXPLORE_THRES = 2;
}

int Armor::run(Mat& frame)
{
    if (frame.empty())
        return -1;
#if VIDEO == VIDEO_FILE
    cvtColor(frame, frame, CV_BGR2GRAY);
#endif
#ifdef DRAW
/*----------------------------------------------------------------------------
    namedWindow("Adjustion",CV_WINDOW_AUTOSIZE);
    cvCreateTrackbar("CONTOUR_AREA_MAX","Adjustion",&CONTOUR_AREA_MAX,3000);
    cvCreateTrackbar("CONTOUR_AREA_MIN","Adjustion",&CONTOUR_AREA_MIN,30);
    cvCreateTrackbar("CONTOUR_LENGTH_MIN","Adjustion",&CONTOUR_LENGTH_MIN,30);
    cvCreateTrackbar("CONTOUR_AREA_MIN","Adjustion",&CONTOUR_HW_RATIO_MIN,3.0);
    cvCreateTrackbar("CONTOUR_AREA_MIN","Adjustion",&CONTOUR_HW_RATIO_MIN,3.0);
    cvCreateTrackbar("CONTOUR_AREA_MIN","Adjustion",&CONTOUR_HW_RATIO_MAX,20);
    cvCreateTrackbar("CONTOUR_AREA_MIN","Adjustion",&TWIN_DISTANCE_N_MIN,1.5);
    cvCreateTrackbar("CONTOUR_AREA_MIN","Adjustion",&TWIN_DISTANCE_N_MAX,4.0);
----------------------------------------------------------------------------*/
    //line(frame,Point(0,100),Point(100,0),Scalar(255,255,255),1,8);
    imshow("frame", frame);
#endif

    if (state == EXPLORE) {
        if (explore(frame)) {
            ++found_ctr;
            unfound_ctr = 0;
        } else {
            ++unfound_ctr;
            found_ctr = 0;
        }

        if (found_ctr >= EXPLORE_TRACK_THRES) {
            serial.sendTarget((bbox.x + bbox.width / 2),
                    (bbox.y + bbox.height / 2), FOUND_BORDER);
            transferState(TRACK_INIT);
            found_ctr = 0;
            unfound_ctr = 0;
            bbox_last = bbox;
        }
        if (unfound_ctr >= EXPLORE_SEND_STOP_THRES) {
            serial.sendTarget(srcW / 2, srcH / 2, NOT_FOUND);
            found_ctr = 0;
            unfound_ctr = 0;
        }
    } else if (state == TRACK_INIT) {
        trackInit(frame);
        transferState(TRACK);
    } else if (state == TRACK) {
        if (track(frame)) {
/*            float fps = 1 / (tic() - timer);
            cout << "fps: " << fps << endl;
            timer = tic();
*/
            int x = bbox.x + bbox.width / 2;
            int y = bbox.y + bbox.height / 2;
            int x_last = bbox_last.x + bbox_last.width / 2;
            //int y_last = bbox_last.y + bbox_last.height / 2;
            int center_x = 2 * x - srcW / 2;
            int center_y = 2 * y - srcH / 2;
            // Assume the box run at const velocity
            // Predict if the center is still in box at next frame
            if (bbox_last.x < center_x
                    && center_x < bbox_last.x + bbox_last.width
                    && bbox_last.y < center_y
                    && center_y < bbox_last.y + bbox_last.height) {
                // if center is in box, predict it run at const velocity
                serial.sendTarget(2 * x - x_last, y, FOUND_CENTER);
            } else {
                serial.sendTarget(x, y, FOUND_BORDER);
            }
            ++found_ctr;
            unfound_ctr = 0;
            bbox_last = bbox;
        } else {
            ++unfound_ctr;
            found_ctr = 0;
        }

        // check if the box is still tracking armor
        if (found_ctr >= TRACK_CHECK_THRES) {
            Mat roi = frame(bbox);
            threshold(roi, roi, GRAY_THRESH, 255, THRESH_BINARY);
            if (countNonZero(roi) < TRACK_CHECK_RATIO * total_contour_area) {
                transferState(EXPLORE);
                found_ctr = 0;
                unfound_ctr = 0;
            }
        }
        // sometimes, tracker only miss 1 frame
        if (unfound_ctr >= TRACK_EXPLORE_THRES) {
            transferState(EXPLORE);
            unfound_ctr = 0;
            found_ctr = 0;
        }
#ifdef DRAW
        // Draw the tracked object
        rectangle(frame, bbox, Scalar(255, 0, 0), 2, 1);
        // Display frame.
        imshow("Tracking", frame);
        //waitKey(0);
#endif
    }
    float fps = 1 / (tic() - timer);
    cout << "fps: " << fps << endl;
    timer = tic();
    return 0;
}

void Armor::transferState(State s)
{
    state = s;
}

void Armor::trackInit(Mat& frame)
{
#ifdef DRAW
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

bool Armor::explore(Mat& frame)
{
    static Mat bin;
    threshold(frame, bin, GRAY_THRESH, 255, THRESH_BINARY);
#ifdef DRAW
    imshow("gray", bin);
#endif
    vector<int> lightx;
    vector<int> lighty;
    vector<vector<Point> > contours;
    vector<RotatedRect> lights;
    vector<long> areas;
    vector<vector<Point> > vertices;
    findContours(bin, contours,
        CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    //select contours by area, length, width/height
    for (unsigned int i = 0; i < contours.size(); ++i) {
        long area = contourArea(contours.at(i));
        //cout << "area:" << area << endl;
        if (area > CONTOUR_AREA_MAX
                || area < CONTOUR_AREA_MIN) {
#  ifdef DRAW
            drawContours(bin, contours, i, Scalar(50), CV_FILLED);
#  endif
            continue;
        }
        RotatedRect rec = minAreaRect(contours.at(i));
        Size2f size = rec.size;
        // get a (longer) as length
        float a = size.height > size.width
            ? size.height
            : size.width;
        float b = size.height < size.width
            ? size.height
            : size.width;
        if (a < CONTOUR_LENGTH_MIN) {
            continue;
        }
        //check if it is thin
        //cout << "a / b: " << a / b << endl;
        if (a / b > CONTOUR_HW_RATIO_MAX
                || a / b < CONTOUR_HW_RATIO_MIN) {
#  ifdef DRAW
            drawContours(bin, contours, i, Scalar(100), CV_FILLED);
#  endif
            continue;
        }
        vertices.push_back(contours.at(i));
        lights.push_back(rec);
        areas.push_back(area);
    }
    if (lights.size() < 2)
        return false;
    int light1 = -1, light2 = -1;
    float min_angel = TWIN_ANGEL_MAX;
    // pair lights by length, distance, angel
    for (unsigned int i = 0; i < lights.size(); ++i) {
        for (unsigned int j = i + 1; j < lights.size(); ++j) {
            Point2f pi = lights.at(i).center;
            Point2f pj = lights.at(j).center;
            Size2f sizei = lights.at(i).size;
            Size2f sizej = lights.at(j).size;
            float ai = sizei.height > sizei.width
                ? sizei.height
                : sizei.width;
            float aj = sizej.height > sizej.width
                ? sizej.height
                : sizej.width;
            // length similar
            if (ai / aj > TWIN_LENGTH_RATIO_MAX
                    || aj / ai > TWIN_LENGTH_RATIO_MAX)
                continue;
//Using function fitLine to fitting
/*
            Vec4f line_parai, line_paraj;
            fitLine(vertices.at(i),line_parai,DIST_L2, 0, 1e-2, 1e-2);
            fitLine(vertices.at(j),line_paraj,DIST_L2, 0, 1e-2, 1e-2);
            if(abs(line_parai[0]/line_parai[1])>=0.00001 &&abs(line_paraj[0]/line_paraj[1])>=0.00001){
                double ki=line_parai[1]/line_parai[0];
                double kj=line_paraj[1]/line_paraj[0];
                angel_i=atan(ki)*180/PI;
                angel_j=atan(kj)*180/PI;
                cout << "line_parai = " << line_parai <<" ki "<<ki <<" "<<angel_i<<endl;
                cout << "line_paraj = " << line_paraj <<" kj "<<kj <<" "<<angel_j<<endl;
            }
            else{
                angel_i=90;
                angel_j=90;
                cout <<"ki -> infty 90"<<endl;
                cout <<"kj -> infty 90"<<endl;
            }
*/
//Using function LeastSquare to fitting
            LeastSquare leastsqi(vertices.at(i));
            angel_i=leastsqi.getfinalangle();
            //leastsqi.getline();
            LeastSquare leastsqj(vertices.at(j));
            angel_j=leastsqj.getfinalangle();
            //leastsqj.getline();
            cout << "angel_i"<<angel_i<<endl;
            cout << "angel_j"<<angel_j<<endl;
            // angel parallel
            float angeli = lights.at(i).angle;
            float angelj = lights.at(j).angle;
            //cout<<"light(i) angle:"<<angeli<<" light(j) angle"<<angelj<<endl;
            if (sizei.width < sizei.height)
                angeli += 90.0;
            if (sizej.width < sizej.height)
                angelj += 90.0;
            if (abs(angel_i - angel_j) < min_angel) {
                float distance_n = abs((pi.x - pj.x) * cos((angeli + 90) * PI / 180)
                    + (pi.y - pj.y) * sin((angeli + 90) * PI / 180));
                // normal distance range in about 1 ~ 2 times of length
                if (distance_n < TWIN_DISTANCE_N_MIN * ai || distance_n > TWIN_DISTANCE_N_MAX * ai
                    || distance_n < TWIN_DISTANCE_N_MIN * aj || distance_n > TWIN_DISTANCE_N_MAX * aj) {
#ifdef DRAW
                    drawContours(bin, contours, i, Scalar(150), CV_FILLED);
                    drawContours(bin, contours, j, Scalar(150), CV_FILLED);
#endif
                    continue;
                }
                // direction distance should be small
                float distance_t = abs((pi.x - pj.x) * cos((angeli)*PI / 180)
                    + (pi.y - pj.y) * sin((angeli)*PI / 180));
                if (distance_t > TWIN_DISTANCE_T_MAX * ai || distance_t > TWIN_DISTANCE_T_MAX * aj) {
#ifdef DRAW
                    drawContours(bin, contours, i, Scalar(150), CV_FILLED);
                    drawContours(bin, contours, j, Scalar(150), CV_FILLED);
#endif
                    continue;
                }
                light1 = i;
                light2 = j;
                min_angel = abs(angeli - angelj);
            }
        }
    }
#ifdef DRAW
    for(unsigned int i=0;i<vertices.size();++i){
      LeastSquare leastsq(vertices.at(i));
      line(bin,Point(leastsq.bh,0),Point(0,-leastsq.bh/leastsq.kh),Scalar(255,255,255),1,8);
    }
#endif
    if (light1 == -1 || light2 == -1 || min_angel == TWIN_ANGEL_MAX)
        return false;
    //cout << "min i:" << light1 << " j:" << light2 << " angel:" << min_angel << endl;
    // get and extend box for track init
    Rect2d reci = lights.at(light1).boundingRect();
    Rect2d recj = lights.at(light2).boundingRect();
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
    total_contour_area = areas.at(light1) + areas.at(light2);
#ifdef DRAW
    rectangle(bin, bbox, Scalar(255), 3);
    imshow("gray", bin);
#endif
    return true;
}
