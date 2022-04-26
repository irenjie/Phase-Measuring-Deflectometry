#ifndef CAMERACONTROL_H
#define CAMERACONTROL_H

#include <iostream>
#include <GalaxyIncludes.h>

class CameraControl {
  public:
    CameraControl();
    void initCamera();
    void uninitCamera();
    bool openCamera(uint32_t exposureTime = 0);
    bool closeCamera();
    CImageDataPointer getImage();

  private:
    CGXFeatureControlPointer ObjFeatureControlPtr;
    CGXStreamPointer ObjStreamPtr;
    CGXDevicePointer ObjDevicePtr;
};

#endif // CAMERACONTROL_H
