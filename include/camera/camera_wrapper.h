#include <iostream>
#include "camera/CameraApi.h"  //相机SDK的API头文件
#include "ctrl_param.h"

#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

class GlobalShutterCamera {
 private:
  unsigned char* g_pRgbBuffer;  //处理后数据缓存区
  int iCameraCounts;
  int iStatus;
  tSdkCameraDevInfo tCameraEnumList;
  int hCamera;
  tSdkCameraCapbility tCapability;  //设备描述信息
  tSdkFrameHead sFrameInfo;
  BYTE* pbyBuffer;
  IplImage* iplImage;
  int channel;

 public:
  GlobalShutterCamera(){};
  ~GlobalShutterCamera();
  int init();
  bool read(cv::Mat& src);
  bool read_raw(cv::Mat& src);
};