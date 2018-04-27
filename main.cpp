/*
Detect Armor in RoboMaster
Copyright 2018 JachinShen(jachinshen@foxmail.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
//Precompile paramaters
#include "precom.h"
#include "Armor.h"
//wrapper for Global Shutter Camera
#include "GlobalCamera.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int main(void)
{
#if VIDEO == VIDEO_CAMERA
    GlobalCamera video;
#endif
#if VIDEO == VIDEO_FILE
    VideoCapture video;
#endif

// Read video
#if VIDEO == VIDEO_CAMERA
    if (video.init() == 0) {
        cout << "Global Shutter Camera Init successfully!" << endl;
    } else {
        cout << "Global Shutter Camera Init Failed!" << endl;
        return -1;
    }
#endif
#if VIDEO == VIDEO_FILE
    video.open("/home/jachinshen/Videos/Robo/station-infanity/rotate.avi");
    if (video.isOpened())
        cout << "Open Video Successfully!" << endl;
    else {
        cout << "Open Video failed!" << endl;
        return -1;
    }
#endif

    Armor armor;
    armor.init();

// use 2 frames for parallel process
// when loading picture from camera to frame1, process frame2
#if OPENMP_SWITCH == OPENMP_RUN
    Mat frame1, frame2;
    bool ok = true;

    video.read(frame2);
    while (ok) {
#   pragma omp parallel sections num_threads(2)
        {
#       pragma omp section
            {
                video.read(frame1);
            }
#       pragma omp section
            {
                if (armor.run(frame2) < 0) {
                    cout << "Error!" << endl;
                    ok = false;
                }
            }
        }

// wait for both section completed
#   pragma omp barrier

#   pragma omp parallel sections num_threads(2)
        {
#       pragma omp section
            {
                video.read(frame2);
            }
#       pragma omp section
            {
                if (armor.run(frame1) < 0) {
                    cout << "Error!" << endl;
                    ok = false;
                }
            }
        }
#   pragma omp barrier
    }
#else
    Mat frame;
    while (video.read(frame)) {
        armor.run(frame);
        cv::waitKey(1);
    }
    cout << "End!" << endl;
#endif
}
