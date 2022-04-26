#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <json.hpp>
#include <opencv2/core/eigen.hpp>
#include <QFileDialog>
#include <QThread>

using json = nlohmann::json;

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow) {
    ui->setupUi(this);

    thread_fullScreen = new QThread;
    fs = new FullScreen;
    fs->moveToThread(thread_fullScreen);

    connect(this, &MainWindow::projPat, fs, &FullScreen::fs_one);
    connect(fs, &FullScreen::sendOK, this, &MainWindow::projAndCapImg);

    thread_fullScreen->start();
}

MainWindow::~MainWindow() {
    delete ui;
}

// 读取 json 配置文件, 相机内参、条纹参数等, 按钮单击
void MainWindow::on_load_cfg_Button_clicked() {
    // 选择，读取 json 格式配置文件
    QString cfgPath = QFileDialog::getOpenFileName(this, tr("打开配置文件"), "D:/Program/3DReconstruct/Phase Measuring Deflectometry/PMD",
                      tr("json files(*.json);;All files(*.*)"));
    json cfg;
    {
        std::ifstream is(cfgPath.toStdString());
        is >> cfg;
    }
    ui->output_Edit->setPlainText("读取配置文件");

    ui->output_Edit->appendPlainText("读取图片尺寸，相机内参，计算相机光线");
    // 图片尺寸，相机内参，相机光线
    {
        std::vector<float> t1 = cfg.at("cameraMatrix");
        MatrixXfR t2(1, t1.size());
        t2.row(0) = VectorXf::Map(&t1[0], t1.size());
        t2.resize(3, 3);
        PMDcfg.cameraMatrix = t2;

        std::vector<float> t3 = cfg.at("distCoeffs");
        Mat t5 = Mat(t3);
        PMDcfg.distcoeffs = t5.clone();

        std::vector<uint16_t> t4 = cfg.at("image_size");
        PMDcfg.img_size.width = t4[0];
        PMDcfg.img_size.height = t4[1];
        //        configCameraRay(PMDcfg.cameraMatrix, PMDcfg.img_size, 1.0, PMDcfg.camera_rays);
    }
    // intersection of camera rays and reference plane
    ui->output_Edit->appendPlainText("读取CWT，计算参考平面与相机光线交点");
    {
        std::vector<float> t1 = cfg.at("CWT");
        MatrixXfR t2(1, t1.size());
        t2.row(0) = VectorXf::Map(&t1[0], t1.size());
        t2.resize(4, 4);
        PMDcfg.CWT = t2;
        //        configRefPlane(PMDcfg.CWT.block(0, 3, 3, 1), PMDcfg.CWT.block(0, 2, 3, 1), PMDcfg.camera_rays, PMDcfg.refPlane);
    }

    // 读取工作目录，屏幕信息，条纹信息，debug, 投影条纹,投影等待时间
    ui->output_Edit->appendPlainText("读取屏幕信息，条纹信息，debug,投影条纹");
    {
        cfg.at("workDir").get_to(PMDcfg.workDir);
        Screen screen = {cfg.at("screenWidth"), cfg.at("screenHeight"), cfg.at("screenCols"), cfg.at("screenRows")};
        PMDcfg.screen = screen;
        PMDcfg.period_width = cfg.at("period_width");
        PMDcfg.period_height = cfg.at("period_height");
        PMDcfg.debug = cfg.at("debug");
        PMDcfg.screen_delay = cfg.at("screen_delay");

        std::string patterns_path = PMDcfg.workDir + "/patterns";
        QDir dir(QString::fromStdString(patterns_path));
        auto imgPathList = dir.entryList();
        for (int i = 0; i < imgPathList.size() - 2; i++)
            PMDcfg.patterns.push_back(cv::imread(patterns_path + "/" + imgPathList[i + 2].toStdString(), cv::IMREAD_GRAYSCALE));
    }


    test();
}

void MainWindow::projAndCapImg() {
    // 采集图像，camera to Eigen matrix
    auto img_cameraPtr = cameraControl.getImage();
    MatrixXi m;
    cv::Mat cvImage;
    void* pRaw8Buffer = NULL;
    pRaw8Buffer = img_cameraPtr->ConvertToRaw8(GX_BIT_0_7);
    cvImage.create(img_cameraPtr->GetHeight(), img_cameraPtr->GetWidth(), CV_8U);
    memcpy(cvImage.data, pRaw8Buffer, img_cameraPtr->GetPayloadSize());
    cv::cv2eigen(cvImage, m);
    PMDcfg.img_pats.push_back(m);

    // 若拍摄完成，调用重建方法
    if (PMDcfg.img_pats.size() == PMDcfg.patterns.size()) {
        cameraControl.closeCamera();
        cameraControl.uninitCamera();
        fs->closeWindow();
        test();
    } else
        projPat(PMDcfg.patterns[PMDcfg.img_pats.size()], PMDcfg.screen_delay);
}

void MainWindow::on_sys_calib_Button_clicked() {
    QString cfgPath = QFileDialog::getOpenFileName(this, tr("打开配置文件"), "D:/Program/3DReconstruct/Phase Measuring Deflectometry/PMD",
                      tr("json files(*.json);;All files(*.*)"));
    json cfg;
    std::ifstream is(cfgPath.toStdString());
    is >> cfg;
    ArucoBoard board = {cv::Size(cfg.at("marker_width"),  cfg.at("marker_height")), cfg.at("marker_length"), cfg.at("marker_separation"),
                        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250), cfg.at("screenOffset_X"), cfg.at("screenOffset_Y")
                       };

    // 加载系统标定图片
    QString imgs_path = QString::fromStdString(PMDcfg.workDir + "/sys_calib_imgs");
    QDir dir(imgs_path);
    std::vector<Mat> imgs_board(3);
    auto imgPathList = dir.entryList();
    for (int i = 0; i < imgPathList.size() - 2; i++)
        imgs_board[i] = cv::imread(imgs_path.toStdString() + "/" + imgPathList[i + 2].toStdString(), cv::IMREAD_GRAYSCALE);

    // aruco 检测
    Mat cameraMatrix;
    cv::eigen2cv(PMDcfg.cameraMatrix, cameraMatrix);
    vector<Mat> Rmats(3);
    vector<cv::Vec3f> Tvecs(3);
    vector<vector<int>> Ids(3);
    vector<vector<vector<cv::Point2f>>> corners(3), rejectedCandidates(3);
    aruco_analyze(imgs_board, board, cameraMatrix, PMDcfg.distcoeffs, Rmats, Tvecs, Ids, corners, rejectedCandidates, PMDcfg.workDir);

    // 系统标定参数设置
    Matrix3f BVR = Matrix3f::Identity(3, 3);
    BVR(2, 2) = -1;
    Vector3f BVTL(-board.screenOffset_X, -board.screenOffset_Y, 0);
    vector<Matrix3f> CBR(3);
    vector<Vector3f> CBTL(3);
    vector<Matrix3f> CVR(3);
    vector<Vector3f> CVTL(3);
    Matrix3f CSR;
    VectorXf CSTL_D;            // translation .. and distance between mirror and camera
    Matrix4f BVT(4, 4);
    BVT << BVR, BVTL,
        0, 0, 0, 1;
    vector<Matrix4f> CBT(3);
    vector<Matrix4f> CVT(3);
    vector<Vector3f> n(3);      // mirror noraml in camera coordinate

    for (uint16_t i = 0; i < Rmats.size(); i++) {
        CBR[i] = Matrix3f(3, 3);
        cv::cv2eigen(Rmats[i], CBR[i]);
        CBTL[i] = Vector3f(3);
        cv::cv2eigen(Tvecs[i], CBTL[i]);
    }

    for (uint16_t i = 0; i < CBR.size(); i++) {
        CBT[i] = Matrix4f(4, 4);
        CVT[i] = Matrix4f(4, 4);
        CBT[i] << CBR[i], CBTL[i],
            0, 0, 0, 1;
        CVT[i] = CBT[i] * BVT;

        CVR[i] = Matrix3f(3, 3);
        CVTL[i] = Vector3f(3);
        CVR[i] << CVT[i].block<3, 3>(0, 0);
        CVTL[i] << CVT[i].block<3, 1>(0, 3);
    }

    system_calib(CVR, CVTL, CSR, CSTL_D, n);

    // save 2 file
    std::ofstream fout;
    fout.open(PMDcfg.workDir + "/sys_calib_output/sys_cali_result.txt");
    fout << "rotation of screen relative to camera:" << endl << CSR << endl << endl;
    fout << "translation of screen relative to camera:" << endl << CSTL_D << endl << endl;

    // 计算重投影误差(像素)
    if (true) {
        // 反算CVT
        vector<Mat> rvecs_m(3);
        vector<Mat> rvecs(3);
        vector<Mat> tvecs(3);
        Matrix3f I3 = Matrix3f::Identity(3, 3);
        std::vector<Matrix3f> CVR(3);
        std::vector<Vector3f> CVTL(3);
        for (int i = 0; i < 3; i++) {
            CVR[i] = (I3 - 2 * n[i] * n[i].transpose()) * CSR;
            CVTL[i] = (I3 - 2 * n[i] * n[i].transpose()) * CSTL_D.head(3) - 2 * CSTL_D[3 + i] * n[i];
            CVR[i].block(0, 2, 3, 1) *= -1;
            cv::eigen2cv(CVR[i], rvecs_m[i]);
            cv::eigen2cv(CVTL[i], tvecs[i]);
            cv::Rodrigues(rvecs_m[i], rvecs[i]);
        }

        // 计算特征点极其三维坐标
        vector<vector<cv::Point3f>> objectPoints(3);
        vector<vector<cv::Point2f>> imagePoints(3);

        cv::Mat img = cv::imread(PMDcfg.workDir + "/sys_calib_output/aruco_8x5.bmp", cv::IMREAD_GRAYSCALE);
        std::vector<int> Ids_origin;
        std::vector<std::vector<cv::Point2f>> corners_origin, rejectedCandidates_origin;
        auto parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(img, board.dictionary, corners_origin, Ids_origin, parameters, rejectedCandidates_origin, cameraMatrix, PMDcfg.distcoeffs);
        for (int i = 0; i < 3; i++) {
            for (std::vector<int>::size_type j = 0; j < Ids[i].size(); j++) {
                for (int k = 0; k < 4; k++) {
                    imagePoints[i].push_back(corners[i][j][k]);
                    int marker_num = board.markers_size.width * board.markers_size.height;
                    cv::Point2f op(corners_origin[marker_num - 1 - Ids[i][j]][k]);
                    float pix_size = PMDcfg.screen.width / PMDcfg.screen.cols;
                    objectPoints[i].push_back(cv::Point3f(op.x * pix_size, (1080 - op.y)*pix_size, 0));
                }
            }
        }
        std::vector<float> perViewErrors;
        double total_error = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, PMDcfg.distcoeffs, perViewErrors);
        fout << "perViewErrors: ";
        for (std::vector<float>::size_type i = 0; i < perViewErrors.size(); ++i)
            fout << perViewErrors[i] << ",";
        fout << endl << "total error: " << total_error << endl;
    }
    fout.close();

    ui->output_Edit->setPlainText("system calibration complete");
}

void MainWindow::test() {
    // 算数测试
    if (false) {
        ArrayXXf a(2, 2);
        MatrixXf b(2, 2);
        ArrayXf c(2);
        a << 1, 2,
        3, 4;
        b << 5, 6,
        7, 8;
        c << 10, 20;
        Eigen::Vector3f d(1, 2, 3);
        Vector3f t(4, 5, 6);
        Matrix3f f(3, 3);
        f.row(0) = d(0) * t.transpose();

        cout << f << endl;
    }

    // camera rays test, 使用camera_rays 的数据，给定像素坐标，求世界坐标与相机标定时的是否一致.(测试通过)
    if (false) {
        Mat distCoeffs = (cv::Mat_<float>(1, 5) << -0.185079, 0.451621, 0.000411, -0.000481, -3.101673);
        Matrix4f extrinsic;
        Eigen::Vector2i point_img(1517, 748);   // col，row
        Vector3f point_W(0, 0.025, 0);
        extrinsic << -0.01655164577240431, 0.9990216629204951, 0.04100926770613657, -0.05811072408646133,
                  0.9831808940391766, 0.008800869557167967, 0.1824222417671788, -0.05914208463991447,
                  0.1818828541082149, 0.04333891681394951, -0.9823646805900768, 0.6099327406613485,
                  0, 0, 0, 1;
        cv::Size s = cv::Size(4024, 3096);

        Vector3f M(0, 0, 0);
        Vector3f V = PMDcfg.camera_rays.col(point_img(1) * s.width + point_img(0));
        Vector3f N(0.041009, 0.182422, -0.98236);
        Vector3f P(-0.0581107, -0.059142, 0.6099327);
        Vector3f O(3);

        float t = ((P(0) - M(0)) * N(0) + (P(1) - M(1)) * N(1) + (P(2) - M(2)) * N(2)) /
                  (V(0) * N(0) + V(1) * N(1) + V(2) * N(2));
        O(0) = M(0) + V(0) * t;
        O(1) = M(1) + V(1) * t;
        O(2) = M(2) + V(2) * t;
        Vector4f p_o;
        p_o << O, 1;
        p_o = extrinsic.inverse() * p_o;
        cout << "real intersection:" << endl << point_W << endl <<
             "compute intersection: " << endl << p_o << endl;

        Vector4f p_w;
        p_w << point_W, 1;
        p_w = extrinsic * p_w;
        cout << "real intersection:" << endl << p_w << endl <<
             "compute intersection: " << endl << O << endl;

    }

    // 参考平面测试，使用refPlane的数据，给定像素坐标，求世界坐标与相机标定时的是否一致.(测试通过)
    if (false) {
        Matrix4f extrinsic;
        Eigen::Vector2i point_img(1517, 748);   // col，row
        Vector4f point_W(0, 0.025, 0, 1);
        cv::Size s = cv::Size(4024, 3096);
        Vector3f V = PMDcfg.refPlane.col(point_img(1) * s.width + point_img(0));
        cout << "标定点: " << endl << PMDcfg.CWT* point_W << endl
             << "refPlane: " << endl << V << endl;
    }

    // 测试相移法.(测试通过)
    if (false) {
        QString pics_path("D:/Program/3DReconstruct/Phase Measuring Deflectometry/patterns");
        QDir dir(pics_path);
        std::vector<MatrixXf> pics(16);
        auto imgPathList = dir.entryList();
        std::vector<MatrixXf> wraped_ps_maps(4);
        std::vector<MatrixXf> ps_maps(2);

        for (int i = 0; i < imgPathList.size() - 2; i++)
            cv::cv2eigen(cv::imread(pics_path.toStdString() + "/" + imgPathList[i + 2].toStdString(), cv::IMREAD_GRAYSCALE), pics[i]);

        phase_shifting(pics, wraped_ps_maps);
        phase_unwrapping(wraped_ps_maps, PMDcfg.period_width, PMDcfg.period_height, ps_maps);
        std::string path = "D:/Program/3DReconstruct/Phase Measuring Deflectometry/ps_maps";
        save_matrix_as_img(wraped_ps_maps[0], path + "/wraped_hf_x.bmp");
        save_matrix_as_img(wraped_ps_maps[1], path + "/wraped_hf_y.bmp");
        save_matrix_as_img(wraped_ps_maps[2], path + "/wraped_lf_x.bmp");
        save_matrix_as_img(wraped_ps_maps[3], path + "/wraped_lf_y.bmp");

        save_matrix_as_img(ps_maps[0], path + "/hf_x.bmp");
        save_matrix_as_img(ps_maps[1], path + "/hf_y.bmp");

        // 测试数据
        std::ofstream fout("D:/Program/3DReconstruct/Phase Measuring Deflectometry/test.txt");
        MatrixXf m = ps_maps[1].block(0, 0, 200, 10);
        VectorXf t(200);
        for (uint16_t i = 0; i < 200; i++)
            t(i) = i;
        m.col(0) = t;
        fout << m;
        fout.close();
        ui->output_Edit->appendPlainText("test finish");
    }

    // 相机.(测试通过)
    if (false) {
        IGXFactory::GetInstance().Init();
        gxdeviceinfo_vector vectorDeviceInfo;
        IGXFactory::GetInstance().UpdateDeviceList(1000, vectorDeviceInfo);

        CGXDevicePointer ObjDevicePtr = IGXFactory::GetInstance().OpenDeviceBySN(vectorDeviceInfo[0].GetSN(), GX_ACCESS_EXCLUSIVE);

        CGXFeatureControlPointer ObjFeatureControlPtr = ObjDevicePtr->GetRemoteFeatureControl();
        ObjFeatureControlPtr->GetFloatFeature("ExposureTime")->SetValue(30 * 1000);
        // 设置像素格式
        ObjFeatureControlPtr->GetEnumFeature("PixelFormat")->SetValue("Mono8");
        CGXStreamPointer ObjStreamPtr = ObjDevicePtr->OpenStream(0);
        ObjStreamPtr->SetAcqusitionBufferNumber(1);
        ObjStreamPtr->StartGrab();
        ObjFeatureControlPtr->GetCommandFeature("AcquisitionStart")->Execute();
        // 采单帧
        for (int i = 0; i < 10; i++) {
            CImageDataPointer objImageDataPtr = ObjStreamPtr->GetImage(500); // 超时时间500ms
            if (objImageDataPtr->GetStatus() == GX_FRAME_STATUS_SUCCESS) {
                // 采集成功
                cv::Mat cvImage;
                // 图像格式转换
                void* pRaw8Buffer = NULL;
                // 假设原始数据是Mono8图像
                pRaw8Buffer = objImageDataPtr->ConvertToRaw8(GX_BIT_0_7);
                cvImage.create(objImageDataPtr->GetHeight(), objImageDataPtr->GetWidth(), CV_8U);
                memcpy(cvImage.data, pRaw8Buffer, objImageDataPtr->GetPayloadSize());
                cv::namedWindow("测试", 0);
                cout << cvImage.size << endl;
                cv::imshow("测试", cvImage);
                cv::waitKey(500);
            } else {
                cout << "拍摄失败" << endl;
            }
        }

        ObjFeatureControlPtr->GetCommandFeature("AcquisitionStop")->Execute();
        ObjStreamPtr->StopGrab();
        ObjStreamPtr->Close();
        ObjDevicePtr->Close();
        IGXFactory::GetInstance().Uninit();
    }

    // 条纹投影+相机拍摄
    if (false) {
        if (PMDcfg.img_pats.size() != PMDcfg.patterns.size()) {
            fs->initWindow();
            cameraControl.initCamera();
            cameraControl.openCamera(18 * 1000);
            //            QThread::msleep(500);   // 等待全屏和相机初始化完成
            projPat(PMDcfg.patterns[0], PMDcfg.screen_delay);
            cameraControl.getImage();
            return;
        }
        for (int i = 0; i < PMDcfg.img_pats.size(); ++i) {
            cv::Mat img;
            cv::eigen2cv(PMDcfg.img_pats[i], img);
            cv::imwrite(PMDcfg.workDir + "/img_pats/" + std::string(2 - std::to_string(i).length(), '0') + std::to_string(i) + ".bmp", img);
        }
        cout << "拍摄完成: " << PMDcfg.img_pats.size() << endl;

    }

    // 系统标定测试
    if (false) {

    }
}

















