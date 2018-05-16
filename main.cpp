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
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int main(void)
{
    while(1) {
#if VIDEO == VIDEO_CAMERA
        GlobalCamera video;
#endif
#if VIDEO == VIDEO_FILE
        VideoCapture video;
#endif

#if RECORD == RECORD_ON
        fstream video_file;
        video_file.open("../hello.txt", ios::in );
        string video_id_str;
        video_file >> video_id_str;
        video_file.close();

        video_file.open("../hello.txt", ios::out );
        int video_id = atoi(video_id_str.c_str()) + 1;
        video_file << video_id ;
        video_file.close();

#if PLATFORM == PC
        string file_name = string(getpwuid(getuid())->pw_dir) 
            + "/Videos/Record" + video_id_str + ".avi";
# else
        string file_name = "/home/ubuntu/Videos/Record" + video_id_str + ".avi";
#endif

        cout << "File name: " << file_name << endl;
        VideoWriter g_writer;
        g_writer.open(file_name, CV_FOURCC('P', 'I', 'M', '1'), 120,
                cv::Size(640, 480), 0);
#endif
        // Read video
#if VIDEO == VIDEO_CAMERA
        if (video.init() == 0) {
            cout << "Global Shutter Camera Init successfully!" << endl;
        } else {
            cout << "Global Shutter Camera Init Failed!" << endl;
            //return -1;
            continue;
        }
#endif
#if VIDEO == VIDEO_FILE
        video.open("/home/jachinshen/Videos/Record32.avi");
        if (video.isOpened())
            cout << "Open Video Successfully!" << endl;
        else {
            cout << "Open Video failed!" << endl;
            //return -1;
            continue;
        }
#endif

        Armor armor;
        armor.init();

        // use 2 frames for parallel process
        // when loading picture from camera to frame1, process frame2
#if OPENMP_SWITCH == OPENMP_RUN
        Mat frame1, frame2;
        bool ok = true;

        for (int i=0; i<10; ++i)
            video.read(frame2);

        while (ok) {
#       pragma omp parallel sections 
            {
#           pragma omp section
                {
                    video.read(frame1);
                }
#           pragma omp section
                {
                    if (armor.run(frame2) < 0) {
                        cout << "Error!" << endl;
                        ok = false;
                    }
                }
#           if RECORD == RECORD_ON
#           pragma omp section
                {
                    g_writer.write(frame2);
                }
#           endif
            }
            // wait for both section completed
#       pragma omp barrier
            /*****************************************/
#       pragma omp parallel sections
            {
#           pragma omp section
                {
                    video.read(frame2);
                }
#           pragma omp section
                {
                    if (armor.run(frame1) < 0) {
                        cout << "Error!" << endl;
                        ok = false;
                    }
                }
#           if RECORD == RECORD_ON
#           pragma omp section
                {
                    g_writer.write(frame1);
                }
#           endif
            }
#       pragma omp barrier
        }
#   else
        Mat frame;
        for (int i=0; i<10; ++i)
            video.read(frame);
        while (video.read(frame)) {
#       if RECORD == RECORD_ON
            g_writer.write(frame);
#       endif
            armor.run(frame);
            cv::waitKey(0);
        }
        cout << "End!" << endl;
#endif
    }
}
