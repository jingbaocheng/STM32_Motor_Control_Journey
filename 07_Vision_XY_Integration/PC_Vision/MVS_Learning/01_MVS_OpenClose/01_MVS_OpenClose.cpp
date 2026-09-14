#include <stdio.h>
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include "CoordinateMapper.h"


// =========================================================
// 目标位姿
// =========================================================
struct ObjectPose
{
    double u;
    double v;
    double angle;
    bool valid;
};


// =========================================================
// 视觉检测函数
// 输入：BGR图像 image
// 输出：pose + rotatedBox
// =========================================================
bool DetectObject(
    const cv::Mat& image,
    ObjectPose& pose,
    cv::RotatedRect& rotatedBox
)
{
    // 每次调用先认为检测失败
    pose = { 0.0, 0.0, 0.0, false };

    if (image.empty())
    {
        return false;
    }

    // -----------------------------------------------------
    // 1. BGR -> Gray
    // -----------------------------------------------------
    cv::Mat grayImage;

    cv::cvtColor(
        image,
        grayImage,
        cv::COLOR_BGR2GRAY
    );


    // -----------------------------------------------------
    // 2. Gray -> Binary
    // -----------------------------------------------------
    cv::Mat binaryImage;

    cv::threshold(
        grayImage,
        binaryImage,
        100,
        255,
        cv::THRESH_BINARY
    );


    // -----------------------------------------------------
    // 3. Binary -> Contours
    // -----------------------------------------------------
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(
        binaryImage,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );


    // -----------------------------------------------------
    // 4. 面积筛选
    // -----------------------------------------------------
    std::vector<std::vector<cv::Point>> filteredContours;

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);

        if (area > 500)
        {
            filteredContours.push_back(contours[i]);
        }
    }


    // -----------------------------------------------------
    // 5. 汇总筛选后的所有轮廓点
    // -----------------------------------------------------
    std::vector<cv::Point> allPoints;

    for (size_t i = 0; i < filteredContours.size(); i++)
    {
        for (size_t j = 0;
            j < filteredContours[i].size();
            j++)
        {
            allPoints.push_back(
                filteredContours[i][j]
            );
        }
    }


    // 没有有效目标
    if (allPoints.empty())
    {
        return false;
    }


    // -----------------------------------------------------
    // 6. 最小外接旋转矩形
    // -----------------------------------------------------
    rotatedBox = cv::minAreaRect(allPoints);


    // -----------------------------------------------------
    // 7. 保存目标中心
    // -----------------------------------------------------
    pose.u = rotatedBox.center.x;
    pose.v = rotatedBox.center.y;


    // -----------------------------------------------------
    // 8. 统一成长边角度
    // -----------------------------------------------------
    pose.angle = rotatedBox.angle;

    if (rotatedBox.size.width <
        rotatedBox.size.height)
    {
        pose.angle += 90.0;
    }


    // 检测有效
    pose.valid = true;

    return true;
}


// =========================================================
// main
// =========================================================
int main()
{
    CoordinateMapper mapper;
    
    int nRet = MV_OK;
    void* handle = NULL;


    // =====================================================
    // 1. 枚举设备
    // =====================================================
    MV_CC_DEVICE_INFO_LIST stDeviceList = { 0 };

    nRet = MV_CC_EnumDevices(
        MV_GIGE_DEVICE | MV_USB_DEVICE,
        &stDeviceList
    );

    if (nRet != MV_OK)
    {
        printf(
            "EnumDevices failed! nRet = 0x%x\n",
            nRet
        );

        return -1;
    }

    printf(
        "Find %d device(s).\n",
        stDeviceList.nDeviceNum
    );

    if (stDeviceList.nDeviceNum == 0)
    {
        printf("No camera found.\n");
        return -1;
    }


    // =====================================================
    // 2. 创建 Handle
    // =====================================================
    nRet = MV_CC_CreateHandle(
        &handle,
        stDeviceList.pDeviceInfo[0]
    );

    if (nRet != MV_OK)
    {
        printf(
            "CreateHandle failed! nRet = 0x%x\n",
            nRet
        );

        return -1;
    }

    printf("CreateHandle success.\n");


    // =====================================================
    // 3. 打开相机
    // =====================================================
    nRet = MV_CC_OpenDevice(handle);

    if (nRet != MV_OK)
    {
        printf(
            "OpenDevice failed! nRet = 0x%x\n",
            nRet
        );

        MV_CC_DestroyHandle(handle);

        return -1;
    }

    printf("OpenDevice success.\n");


    // =====================================================
    // 查看自动曝光状态
    // =====================================================
    MVCC_ENUMVALUE stExposureAuto = { 0 };

    nRet = MV_CC_GetEnumValue(
        handle,
        "ExposureAuto",
        &stExposureAuto
    );

    if (nRet == MV_OK)
    {
        printf(
            "ExposureAuto = %u\n",
            stExposureAuto.nCurValue
        );
    }
    else
    {
        printf(
            "Get ExposureAuto failed! nRet = 0x%x\n",
            nRet
        );
    }


    // =====================================================
    // 查看当前曝光时间
    // =====================================================
    MVCC_FLOATVALUE stExposureTime = { 0 };

    nRet = MV_CC_GetFloatValue(
        handle,
        "ExposureTime",
        &stExposureTime
    );

    if (nRet == MV_OK)
    {
        printf(
            "ExposureTime = %.2f us\n",
            stExposureTime.fCurValue
        );

        printf(
            "ExposureTime Min = %.2f us\n",
            stExposureTime.fMin
        );

        printf(
            "ExposureTime Max = %.2f us\n",
            stExposureTime.fMax
        );
    }
    else
    {
        printf(
            "Get ExposureTime failed! nRet = 0x%x\n",
            nRet
        );
    }


    // =====================================================
    // 设置曝光时间 20ms
    // =====================================================
    nRet = MV_CC_SetFloatValue(
        handle,
        "ExposureTime",
        20000.0f
    );

    if (nRet == MV_OK)
    {
        printf(
            "Set ExposureTime = 20000 us success.\n"
        );
    }
    else
    {
        printf(
            "Set ExposureTime failed! nRet = 0x%x\n",
            nRet
        );
    }


    // =====================================================
    // 4. 连续采集模式：关闭 Trigger
    // =====================================================
    nRet = MV_CC_SetEnumValue(
        handle,
        "TriggerMode",
        MV_TRIGGER_MODE_OFF
    );

    if (nRet != MV_OK)
    {
        printf(
            "Set TriggerMode OFF failed! nRet = 0x%x\n",
            nRet
        );

        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);

        return -1;
    }

    printf("TriggerMode OFF success.\n");


    // =====================================================
    // 5. 开始采集
    // =====================================================
    nRet = MV_CC_StartGrabbing(handle);

    if (nRet != MV_OK)
    {
        printf(
            "StartGrabbing failed! nRet = 0x%x\n",
            nRet
        );

        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);

        return -1;
    }

    printf("StartGrabbing success.\n");
    printf(
        "Press ESC or Q in image window to exit.\n"
    );
    // 进入循环前，只执行一次
    cv::namedWindow("Contours", cv::WINDOW_NORMAL);
    cv::resizeWindow("Contours", 960, 720);
    // 第一个真实标定点：连续采10个有效结果
    double sumU = 0.0;
    double sumV = 0.0;
    int sampleCount = 0;
    bool pointCaptured = false;
    // =====================================================
    // 6. 循环取图
    // =====================================================
    while (true)
    {
        MV_FRAME_OUT stOutFrame = { 0 };

        nRet = MV_CC_GetImageBuffer(
            handle,
            &stOutFrame,
            1000
        );

        if (nRet != MV_OK)
        {
            printf(
                "GetImageBuffer failed! nRet = 0x%x\n",
                nRet
            );

            continue;
        }


        printf(
            "FrameNum = %d, Width = %d, "
            "Height = %d, PixelType = 0x%x\n",

            stOutFrame.stFrameInfo.nFrameNum,
            stOutFrame.stFrameInfo.nWidth,
            stOutFrame.stFrameInfo.nHeight,
            stOutFrame.stFrameInfo.enPixelType
        );


        int width =
            stOutFrame.stFrameInfo.nWidth;

        int height =
            stOutFrame.stFrameInfo.nHeight;


        cv::Mat image;


        // -------------------------------------------------
        // Mono8
        // -------------------------------------------------
        if (
            stOutFrame.stFrameInfo.enPixelType ==
            PixelType_Gvsp_Mono8
            )
        {
            cv::Mat raw(
                height,
                width,
                CV_8UC1,
                stOutFrame.pBufAddr
            );

            // raw 指向 MVS SDK Buffer
            // 必须复制出来
            image = raw.clone();
        }


        // -------------------------------------------------
        // Bayer RG8
        // -------------------------------------------------
        else if (
            stOutFrame.stFrameInfo.enPixelType ==
            PixelType_Gvsp_BayerRG8
            )
        {
            cv::Mat raw(
                height,
                width,
                CV_8UC1,
                stOutFrame.pBufAddr
            );

            cv::cvtColor(
                raw,
                image,
                cv::COLOR_BayerRG2BGR
            );
        }


        // -------------------------------------------------
        // Bayer BG8
        // -------------------------------------------------
        else if (
            stOutFrame.stFrameInfo.enPixelType ==
            PixelType_Gvsp_BayerBG8
            )
        {
            cv::Mat raw(
                height,
                width,
                CV_8UC1,
                stOutFrame.pBufAddr
            );

            cv::cvtColor(
                raw,
                image,
                cv::COLOR_BayerBG2BGR
            );
        }


        // -------------------------------------------------
        // Bayer GR8
        // -------------------------------------------------
        else if (
            stOutFrame.stFrameInfo.enPixelType ==
            PixelType_Gvsp_BayerGR8
            )
        {
            cv::Mat raw(
                height,
                width,
                CV_8UC1,
                stOutFrame.pBufAddr
            );


            // 每30帧查看一次原始亮度
            if (
                stOutFrame.stFrameInfo.nFrameNum %
                30 == 0
                )
            {
                double minVal = 0.0;
                double maxVal = 0.0;

                cv::minMaxLoc(
                    raw,
                    &minVal,
                    &maxVal
                );

                cv::Scalar meanVal =
                    cv::mean(raw);

                printf(
                    "RAW: min = %.0f, "
                    "max = %.0f, "
                    "mean = %.2f\n",

                    minVal,
                    maxVal,
                    meanVal[0]
                );
            }


            cv::cvtColor(
                raw,
                image,
                cv::COLOR_BayerGR2BGR
            );
        }


        // -------------------------------------------------
        // Bayer GB8
        // -------------------------------------------------
        else if (
            stOutFrame.stFrameInfo.enPixelType ==
            PixelType_Gvsp_BayerGB8
            )
        {
            cv::Mat raw(
                height,
                width,
                CV_8UC1,
                stOutFrame.pBufAddr
            );

            cv::cvtColor(
                raw,
                image,
                cv::COLOR_BayerGB2BGR
            );
        }


        // -------------------------------------------------
        // 不支持的格式
        // -------------------------------------------------
        else
        {
            printf(
                "Unsupported PixelType: 0x%x\n",
                stOutFrame.stFrameInfo.enPixelType
            );
        }


        // =================================================
        // 7. 归还 MVS Buffer
        // =================================================
        nRet = MV_CC_FreeImageBuffer(
            handle,
            &stOutFrame
        );

        if (nRet != MV_OK)
        {
            printf(
                "FreeImageBuffer failed! "
                "nRet = 0x%x\n",
                nRet
            );
        }


        // =================================================
        // 8. 调用视觉检测模块
        // =================================================
        if (!image.empty())
        {
            ObjectPose pose =
            {
                0.0,
                0.0,
                0.0,
                false
            };

            cv::RotatedRect rotatedBox;


            bool detected = DetectObject(
                image,
                pose,
                rotatedBox
            );


            // 原图复制一份
            // 专门用于画调试信息
            cv::Mat contourImage =
                image.clone();


            // ---------------------------------------------
            // 检测成功
            // ---------------------------------------------
            if (detected)
            {
                printf(
                    "Pose: U = %.2f, "
                    "V = %.2f, "
                    "Angle = %.2f\n",

                    pose.u,
                    pose.v,
                    pose.angle
                );
                // -----------------------------------------
   // 第一个真实标定点：采10帧求平均
   // -----------------------------------------
                if (!pointCaptured)
                {
                    sumU += pose.u;
                    sumV += pose.v;
                    sampleCount++;

                    printf(
                        "Calibration sampling: %d / 10\n",
                        sampleCount
                    );

                    if (sampleCount == 10)
                    {
                        double avgU = sumU / 10.0;
                        double avgV = sumV / 10.0;

                        printf("\n");
                        printf("=============================\n");
                        printf("Calibration Point 1\n");
                        printf("U_avg = %.3f\n", avgU);
                        printf("V_avg = %.3f\n", avgV);
                        printf("X = 0.000 mm\n");
                        printf("Y = 0.000 mm\n");
                        printf("=============================\n\n");

                        pointCaptured = true;
                    }
                }

                // -----------------------------------------
                // 取旋转矩形四个顶点
                // -----------------------------------------
                cv::Point2f vertices[4];

                rotatedBox.points(vertices);


                // -----------------------------------------
                // 画绿色旋转矩形
                // -----------------------------------------
                for (int i = 0; i < 4; i++)
                {
                    cv::line(
                        contourImage,

                        vertices[i],

                        vertices[
                            (i + 1) % 4
                        ],

                        cv::Scalar(
                            0,
                            255,
                            0
                        ),

                        3
                    );
                }


                // -----------------------------------------
                // 画蓝色中心点
                // -----------------------------------------
                cv::circle(
                    contourImage,

                    cv::Point(
                        cvRound(pose.u),
                        cvRound(pose.v)
                    ),

                    8,

                    cv::Scalar(
                        255,
                        0,
                        0
                    ),

                    -1
                );
            }
            else
            {
                printf(
                    "Object not detected.\n"
                );
            }


            // ---------------------------------------------
            // 显示
            // ---------------------------------------------
         

            cv::imshow(
                "Contours",
                contourImage
            );
        }


        // =================================================
        // 9. 键盘退出
        // =================================================
        int key = cv::waitKey(1);

        if (
            key == 27 ||
            key == 'q' ||
            key == 'Q' ||

            cv::getWindowProperty(
                "Contours",
                cv::WND_PROP_VISIBLE
            ) < 1
            )
        {
            break;
        }
    }


    // =====================================================
    // 10. 停止采集
    // =====================================================
    nRet = MV_CC_StopGrabbing(handle);

    if (nRet != MV_OK)
    {
        printf(
            "StopGrabbing failed! nRet = 0x%x\n",
            nRet
        );
    }
    else
    {
        printf(
            "StopGrabbing success.\n"
        );
    }


    // =====================================================
    // 11. 关闭设备
    // =====================================================
    nRet = MV_CC_CloseDevice(handle);

    if (nRet != MV_OK)
    {
        printf(
            "CloseDevice failed! nRet = 0x%x\n",
            nRet
        );
    }
    else
    {
        printf(
            "CloseDevice success.\n"
        );
    }


    // =====================================================
    // 12. 销毁 Handle
    // =====================================================
    nRet = MV_CC_DestroyHandle(handle);

    if (nRet != MV_OK)
    {
        printf(
            "DestroyHandle failed! nRet = 0x%x\n",
            nRet
        );
    }
    else
    {
        printf(
            "DestroyHandle success.\n"
        );
    }


    cv::destroyAllWindows();

    return 0;
}
