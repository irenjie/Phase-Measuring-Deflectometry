#include "cameracontrol.h"


CameraControl::CameraControl() {

}

void CameraControl::initCamera() {
    IGXFactory::GetInstance().Init();
}

void CameraControl::uninitCamera() {
    IGXFactory::GetInstance().Uninit();
}

bool CameraControl::openCamera(uint32_t exposureTime) {
    gxdeviceinfo_vector vectorDeviceInfo;
    IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);
    ObjDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);

    ObjFeatureControlPtr = ObjDevicePtr->GetRemoteFeatureControl();
    if (exposureTime != 0)
        ObjFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(exposureTime);  // 设置曝光时间
    ObjFeatureControlPtr->GetEnumFeature("PixelFormat")->SetValue("Mono8");

    ObjStreamPtr = ObjDevicePtr->OpenStream(0);
    ObjStreamPtr->SetAcqusitionBufferNumber(2); // 设置 buffer大小
    CGXFeatureControlPointer objStreamFeatureControlPtr = ObjStreamPtr->GetFeatureControl();
    objStreamFeatureControlPtr->GetEnumFeature("StreamBufferHandlingMode")->SetValue("NewestOnly");

    ObjStreamPtr->StartGrab();
    ObjFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();
    return true;
}

bool CameraControl::closeCamera() {
    ObjFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();
    ObjStreamPtr->StopGrab();
    ObjStreamPtr->Close();
    ObjDevicePtr->Close();
    return true;
}

CImageDataPointer CameraControl::getImage() {
    CImageDataPointer objImageDataPtr;
    try {
        objImageDataPtr = ObjStreamPtr->GetImage(500);
    } catch (CGalaxyException& e) {
        std::cout << "Exception code: " << e.GetErrorCode() << std::endl;
        std::cout << "Exception info: " << e.what() << std::endl;
    }
    return objImageDataPtr;
}
