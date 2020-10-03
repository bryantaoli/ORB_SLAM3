/**
 * @file Tracking.cc
 * @brief 追踪线程
 * 
 */
/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Initializer.h"
#include"G2oTypes.h"
#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>
#include<chrono>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>
#include <include/MLPnPsolver.h>


using namespace std;

// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyFrame数据类型   

namespace ORB_SLAM3
{

///构造函数
Tracking::Tracking(
    System *pSys,               //系统实例
    ORBVocabulary* pVoc,        //BOW字典
    FrameDrawer *pFrameDrawer,  //帧绘制器
    MapDrawer *pMapDrawer,      //地图点绘制器
    Atlas *pAtlas,              //地图句柄
    KeyFrameDatabase* pKFDB,    //关键帧产生的词袋数据库
    const string &strSettingPath, //配置文件路径
    const int sensor,           //传感器类型
    const string &_nameSeq):
        mState(NO_IMAGES_YET),  //当前系统还没有准备好
        mSensor(sensor), 
        mTrackedFr(0), 
        mbStep(false),
        mbOnlyTracking(false),   //处于SLAM模式
        mbMapUpdated(false), 
        mbVO(false),            //当处于纯跟踪模式的时候，这个变量表示了当前跟踪状态的好坏
        mpORBVocabulary(pVoc), 
        mpKeyFrameDB(pKFDB),
        mpInitializer(static_cast<Initializer*>(NULL)), //暂时给地图初始化器设置为空指针
        mpSystem(pSys), 
        mpViewer(NULL),     //注意可视化的查看器是可选的，因为ORB-SLAM2最后是被编译成为一个库，所以对方人拿过来用的时候也应该有权力说我不要可视化界面（何况可视化界面也要占用不少的CPU资源）
        mpFrameDrawer(pFrameDrawer), 
        mpMapDrawer(pMapDrawer), 
        mpAtlas(pAtlas), 
        mnLastRelocFrameId(0), 
        time_recently_lost(5.0),
        mnInitialFrameId(0), 
        mbCreatedMap(false), 
        mnFirstFrameId(0),      //恢复为0,没有进行这个过程的时候的默认值
        mpCamera2(nullptr)
{
    // Load camera parameters from settings file
    // Step 1 从配置文件中加载相机参数
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool b_parse_cam = ParseCamParamFile(fSettings);
    if(!b_parse_cam)
    {
        std::cout << "*Error with the camera parameters in the config file*" << std::endl;
    }

    // Load ORB parameters
    bool b_parse_orb = ParseORBParamFile(fSettings);
    if(!b_parse_orb)
    {
        std::cout << "*Error with the ORB parameters in the config file*" << std::endl;
    }

    initID = 0; lastID = 0;

    // Load IMU parameters
    bool b_parse_imu = true;
    if(sensor==System::IMU_MONOCULAR || sensor==System::IMU_STEREO)
    {
        b_parse_imu = ParseIMUParamFile(fSettings);
        if(!b_parse_imu)
        {
            std::cout << "*Error with the IMU parameters in the config file*" << std::endl;
        }

        mnFramesToResetIMU = mMaxFrames;
    }

    mbInitWith3KFs = false;


    //Rectification parameters
    /*mbNeedRectify = false;
    if((mSensor == System::STEREO || mSensor == System::IMU_STEREO) && sCameraName == "PinHole")
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fSettings["LEFT.K"] >> K_l;
        fSettings["RIGHT.K"] >> K_r;

        fSettings["LEFT.P"] >> P_l;
        fSettings["RIGHT.P"] >> P_r;

        fSettings["LEFT.R"] >> R_l;
        fSettings["RIGHT.R"] >> R_r;

        fSettings["LEFT.D"] >> D_l;
        fSettings["RIGHT.D"] >> D_r;

        int rows_l = fSettings["LEFT.height"];
        int cols_l = fSettings["LEFT.width"];
        int rows_r = fSettings["RIGHT.height"];
        int cols_r = fSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty()
                || rows_l==0 || cols_l==0 || rows_r==0 || cols_r==0)
        {
            mbNeedRectify = false;
        }
        else
        {
            mbNeedRectify = true;
            // M1r y M2r son los outputs (igual para l)
            // M1r y M2r son las matrices relativas al mapeo correspondiente a la rectificación de mapa en el eje X e Y respectivamente
            //cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
            //cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
        }


    }
    else
    {
        int cols = 752;
        int rows = 480;
        cv::Mat R_l = cv::Mat::eye(3, 3, CV_32F);
    }*/

    mnNumDataset = 0;

    if(!b_parse_cam || !b_parse_orb || !b_parse_imu)
    {
        std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
        try
        {
            throw -1;
        }
        catch(exception &e)
        {

        }
    }

    //f_track_stats.open("tracking_stats"+ _nameSeq + ".txt");
    /*f_track_stats.open("tracking_stats.txt");
    f_track_stats << "# timestamp, Num KF local, Num MP local, time" << endl;
    f_track_stats << fixed ;*/

#ifdef SAVE_TIMES
    f_track_times.open("tracking_times.txt");
    f_track_times << "# ORB_Ext(ms), Stereo matching(ms), Preintegrate_IMU(ms), Pose pred(ms), LocalMap_track(ms), NewKF_dec(ms)" << endl;
    f_track_times << fixed ;
#endif
}

Tracking::~Tracking()
{
    //f_track_stats.close();
#ifdef SAVE_TIMES
    f_track_times.close();
#endif
}

bool Tracking::ParseCamParamFile(cv::FileStorage &fSettings)
{
    mDistCoef = cv::Mat::zeros(4,1,CV_32F);
    cout << endl << "Camera Parameters: " << endl;
    bool b_miss_params = false;

    string sCameraName = fSettings["Camera.type"];
    if(sCameraName == "PinHole")
    {
        float fx, fy, cx, cy;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(0) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(1) = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p1"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(2) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.p2"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.at<float>(3) = node.real();
        }
        else
        {
            std::cerr << "*Camera.p2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            mDistCoef.resize(5);
            mDistCoef.at<float>(4) = node.real();
        }

        if(b_miss_params)
        {
            return false;
        }

        vector<float> vCamCalib{fx,fy,cx,cy};

        mpCamera = new Pinhole(vCamCalib);

        mpAtlas->AddCamera(mpCamera);


        std::cout << "- Camera: Pinhole" << std::endl;
        std::cout << "- fx: " << fx << std::endl;
        std::cout << "- fy: " << fy << std::endl;
        std::cout << "- cx: " << cx << std::endl;
        std::cout << "- cy: " << cy << std::endl;
        std::cout << "- k1: " << mDistCoef.at<float>(0) << std::endl;
        std::cout << "- k2: " << mDistCoef.at<float>(1) << std::endl;


        std::cout << "- p1: " << mDistCoef.at<float>(2) << std::endl;
        std::cout << "- p2: " << mDistCoef.at<float>(3) << std::endl;

        if(mDistCoef.rows==5)
            std::cout << "- k3: " << mDistCoef.at<float>(4) << std::endl;

        mK = cv::Mat::eye(3,3,CV_32F);
        mK.at<float>(0,0) = fx;
        mK.at<float>(1,1) = fy;
        mK.at<float>(0,2) = cx;
        mK.at<float>(1,2) = cy;

    }
    else if(sCameraName == "KannalaBrandt8")
    {
        float fx, fy, cx, cy;
        float k1, k2, k3, k4;

        // Camera calibration parameters
        cv::FileNode node = fSettings["Camera.fx"];
        if(!node.empty() && node.isReal())
        {
            fx = node.real();
        }
        else
        {
            std::cerr << "*Camera.fx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.fy"];
        if(!node.empty() && node.isReal())
        {
            fy = node.real();
        }
        else
        {
            std::cerr << "*Camera.fy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cx"];
        if(!node.empty() && node.isReal())
        {
            cx = node.real();
        }
        else
        {
            std::cerr << "*Camera.cx parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.cy"];
        if(!node.empty() && node.isReal())
        {
            cy = node.real();
        }
        else
        {
            std::cerr << "*Camera.cy parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        // Distortion parameters
        node = fSettings["Camera.k1"];
        if(!node.empty() && node.isReal())
        {
            k1 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k1 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }
        node = fSettings["Camera.k2"];
        if(!node.empty() && node.isReal())
        {
            k2 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k2 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k3"];
        if(!node.empty() && node.isReal())
        {
            k3 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k3 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        node = fSettings["Camera.k4"];
        if(!node.empty() && node.isReal())
        {
            k4 = node.real();
        }
        else
        {
            std::cerr << "*Camera.k4 parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

        if(!b_miss_params)
        {
            vector<float> vCamCalib{fx,fy,cx,cy,k1,k2,k3,k4};
            mpCamera = new KannalaBrandt8(vCamCalib);

            std::cout << "- Camera: Fisheye" << std::endl;
            std::cout << "- fx: " << fx << std::endl;
            std::cout << "- fy: " << fy << std::endl;
            std::cout << "- cx: " << cx << std::endl;
            std::cout << "- cy: " << cy << std::endl;
            std::cout << "- k1: " << k1 << std::endl;
            std::cout << "- k2: " << k2 << std::endl;
            std::cout << "- k3: " << k3 << std::endl;
            std::cout << "- k4: " << k4 << std::endl;

            mK = cv::Mat::eye(3,3,CV_32F);
            mK.at<float>(0,0) = fx;
            mK.at<float>(1,1) = fy;
            mK.at<float>(0,2) = cx;
            mK.at<float>(1,2) = cy;
        }

        if(mSensor==System::STEREO || mSensor==System::IMU_STEREO){
            // Right camera
            // Camera calibration parameters
            cv::FileNode node = fSettings["Camera2.fx"];
            if(!node.empty() && node.isReal())
            {
                fx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.fy"];
            if(!node.empty() && node.isReal())
            {
                fy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.fy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cx"];
            if(!node.empty() && node.isReal())
            {
                cx = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cx parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.cy"];
            if(!node.empty() && node.isReal())
            {
                cy = node.real();
            }
            else
            {
                std::cerr << "*Camera2.cy parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            // Distortion parameters
            node = fSettings["Camera2.k1"];
            if(!node.empty() && node.isReal())
            {
                k1 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k1 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }
            node = fSettings["Camera2.k2"];
            if(!node.empty() && node.isReal())
            {
                k2 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k2 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k3"];
            if(!node.empty() && node.isReal())
            {
                k3 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k3 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }

            node = fSettings["Camera2.k4"];
            if(!node.empty() && node.isReal())
            {
                k4 = node.real();
            }
            else
            {
                std::cerr << "*Camera2.k4 parameter doesn't exist or is not a real number*" << std::endl;
                b_miss_params = true;
            }


            int leftLappingBegin = -1;
            int leftLappingEnd = -1;

            int rightLappingBegin = -1;
            int rightLappingEnd = -1;

            node = fSettings["Camera.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                leftLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                leftLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera.lappingEnd not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingBegin"];
            if(!node.empty() && node.isInt())
            {
                rightLappingBegin = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingBegin not correctly defined" << std::endl;
            }
            node = fSettings["Camera2.lappingEnd"];
            if(!node.empty() && node.isInt())
            {
                rightLappingEnd = node.operator int();
            }
            else
            {
                std::cout << "WARNING: Camera2.lappingEnd not correctly defined" << std::endl;
            }

            node = fSettings["Tlr"];
            if(!node.empty())
            {
                mTlr = node.mat();
                if(mTlr.rows != 3 || mTlr.cols != 4)
                {
                    std::cerr << "*Tlr matrix have to be a 3x4 transformation matrix*" << std::endl;
                    b_miss_params = true;
                }
            }
            else
            {
                std::cerr << "*Tlr matrix doesn't exist*" << std::endl;
                b_miss_params = true;
            }

            if(!b_miss_params)
            {
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[0] = leftLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera)->mvLappingArea[1] = leftLappingEnd;

                mpFrameDrawer->both = true;

                vector<float> vCamCalib2{fx,fy,cx,cy,k1,k2,k3,k4};
                mpCamera2 = new KannalaBrandt8(vCamCalib2);

                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[0] = rightLappingBegin;
                static_cast<KannalaBrandt8*>(mpCamera2)->mvLappingArea[1] = rightLappingEnd;

                std::cout << "- Camera1 Lapping: " << leftLappingBegin << ", " << leftLappingEnd << std::endl;

                std::cout << std::endl << "Camera2 Parameters:" << std::endl;
                std::cout << "- Camera: Fisheye" << std::endl;
                std::cout << "- fx: " << fx << std::endl;
                std::cout << "- fy: " << fy << std::endl;
                std::cout << "- cx: " << cx << std::endl;
                std::cout << "- cy: " << cy << std::endl;
                std::cout << "- k1: " << k1 << std::endl;
                std::cout << "- k2: " << k2 << std::endl;
                std::cout << "- k3: " << k3 << std::endl;
                std::cout << "- k4: " << k4 << std::endl;

                std::cout << "- mTlr: \n" << mTlr << std::endl;

                std::cout << "- Camera2 Lapping: " << rightLappingBegin << ", " << rightLappingEnd << std::endl;
            }
        }

        if(b_miss_params)
        {
            return false;
        }

        mpAtlas->AddCamera(mpCamera);
        mpAtlas->AddCamera(mpCamera2);
    }
    else
    {
        std::cerr << "*Not Supported Camera Sensor*" << std::endl;
        std::cerr << "Check an example configuration file with the desired sensor" << std::endl;
    }

    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
    {
        cv::FileNode node = fSettings["Camera.bf"];
        if(!node.empty() && node.isReal())
        {
            mbf = node.real();
        }
        else
        {
            std::cerr << "*Camera.bf parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO)
    {
        float fx = mpCamera->getParameter(0);
        cv::FileNode node = fSettings["ThDepth"];
        if(!node.empty()  && node.isReal())
        {
            mThDepth = node.real();
            mThDepth = mbf*mThDepth/fx;
            cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }
        else
        {
            std::cerr << "*ThDepth parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }


    }

    if(mSensor==System::RGBD)
    {
        cv::FileNode node = fSettings["DepthMapFactor"];
        if(!node.empty() && node.isReal())
        {
            mDepthMapFactor = node.real();
            if(fabs(mDepthMapFactor)<1e-5)
                mDepthMapFactor=1;
            else
                mDepthMapFactor = 1.0f/mDepthMapFactor;
        }
        else
        {
            std::cerr << "*DepthMapFactor parameter doesn't exist or is not a real number*" << std::endl;
            b_miss_params = true;
        }

    }

    if(b_miss_params)
    {
        return false;
    }

    return true;
}

bool Tracking::ParseORBParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    int nFeatures, nLevels, fIniThFAST, fMinThFAST;
    float fScaleFactor;

    cv::FileNode node = fSettings["ORBextractor.nFeatures"];
    if(!node.empty() && node.isInt())
    {
        nFeatures = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nFeatures parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.scaleFactor"];
    if(!node.empty() && node.isReal())
    {
        fScaleFactor = node.real();
    }
    else
    {
        std::cerr << "*ORBextractor.scaleFactor parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.nLevels"];
    if(!node.empty() && node.isInt())
    {
        nLevels = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.nLevels parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.iniThFAST"];
    if(!node.empty() && node.isInt())
    {
        fIniThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.iniThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["ORBextractor.minThFAST"];
    if(!node.empty() && node.isInt())
    {
        fMinThFAST = node.operator int();
    }
    else
    {
        std::cerr << "*ORBextractor.minThFAST parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    // tracking过程都会用到mpORBextractorLeft作为特征点提取器
    mpORBextractorLeft = new ORBextractor(
                            nFeatures,      //参数的含义还是看上面的注释吧
                            fScaleFactor,
                            nLevels,
                            fIniThFAST,
                            fMinThFAST);

    // 如果是双目，tracking过程中还会用用到mpORBextractorRight作为右目特征点提取器
    if(mSensor==System::STEREO || mSensor==System::IMU_STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // 在单目初始化的时候，会用mpIniORBextractor来作为特征点提取器
    if(mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR)
        mpIniORBextractor = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    return true;
}

bool Tracking::ParseIMUParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::Mat Tbc;
    cv::FileNode node = fSettings["Tbc"];
    if(!node.empty())
    {
        Tbc = node.mat();
        if(Tbc.rows != 4 || Tbc.cols != 4)
        {
            std::cerr << "*Tbc matrix have to be a 4x4 transformation matrix*" << std::endl;
            b_miss_params = true;
        }
    }
    else
    {
        std::cerr << "*Tbc matrix doesn't exist*" << std::endl;
        b_miss_params = true;
    }

    cout << endl;

    cout << "Left camera to Imu Transform (Tbc): " << endl << Tbc << endl;

    float freq, Ng, Na, Ngw, Naw;

    node = fSettings["IMU.Frequency"];
    if(!node.empty() && node.isInt())
    {
        freq = node.operator int();
    }
    else
    {
        std::cerr << "*IMU.Frequency parameter doesn't exist or is not an integer*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseGyro"];
    if(!node.empty() && node.isReal())
    {
        Ng = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseGyro parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.NoiseAcc"];
    if(!node.empty() && node.isReal())
    {
        Na = node.real();
    }
    else
    {
        std::cerr << "*IMU.NoiseAcc parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.GyroWalk"];
    if(!node.empty() && node.isReal())
    {
        Ngw = node.real();
    }
    else
    {
        std::cerr << "*IMU.GyroWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["IMU.AccWalk"];
    if(!node.empty() && node.isReal())
    {
        Naw = node.real();
    }
    else
    {
        std::cerr << "*IMU.AccWalk parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    if(b_miss_params)
    {
        return false;
    }

    const float sf = sqrt(freq);
    cout << endl;
    cout << "IMU frequency: " << freq << " Hz" << endl;
    cout << "IMU gyro noise: " << Ng << " rad/s/sqrt(Hz)" << endl;
    cout << "IMU gyro walk: " << Ngw << " rad/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer noise: " << Na << " m/s^2/sqrt(Hz)" << endl;
    cout << "IMU accelerometer walk: " << Naw << " m/s^3/sqrt(Hz)" << endl;

    mpImuCalib = new IMU::Calib(Tbc,Ng*sf,Na*sf,Ngw/sf,Naw/sf);

    mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);


    return true;
}

//设置局部建图器
void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

//设置回环检测器
void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

//设置可视化查看器
void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

// 输入左右目图像，可以为RGB、BGR、RGBA、GRAY
// 1、将图像转为mImGray和imGrayRight并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
void Tracking::SetStepByStep(bool bSet)
{
    bStepByStep = bSet;
}



cv::Mat Tracking::GrabImageStereo(
        const cv::Mat &imRectLeft,      //左侧图像
        const cv::Mat &imRectRight,     //右侧图像
        const double &timestamp,        //时间戳
        string filename)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    // step 1 ：将RGB或RGBA图像转为灰度图像
    mImRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }

    // 这里考虑得十分周全,甚至连四通道的图像都考虑到了
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    if (mSensor == System::STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera);
    else if(mSensor == System::STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr);
    else if(mSensor == System::IMU_STEREO && !mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,&mLastFrame,*mpImuCalib);
    else if(mSensor == System::IMU_STEREO && mpCamera2)
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth,mpCamera,mpCamera2,mTlr,&mLastFrame,*mpImuCalib);

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    // Step 3 ：跟踪
    Track();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();

    /*cout << "trracking time: " << t_track << endl;
    f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
    f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
    f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
    f_track_times << mTime_PreIntIMU << ",";
    f_track_times << mTime_PosePred << ",";
    f_track_times << mTime_LocalMapTrack << ",";
    f_track_times << mTime_NewKF_Dec << ",";
    f_track_times << t_track << endl;
#endif

    //返回位姿
    return mCurrentFrame.mTcw.clone();
}

// 输入左目RGB或RGBA图像和深度图
// 1、将图像转为mImGray和imDepth并初始化mCurrentFrame
// 2、进行tracking过程
// 输出世界坐标系到该帧相机坐标系的变换矩阵
cv::Mat Tracking::GrabImageRGBD(
    const cv::Mat &imRGB,       //彩色图像
    const cv::Mat &imD,         //深度图像
    const double &timestamp,    //时间戳
    string filename)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    // step 1：将RGB或RGBA图像转为灰度图像
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // step 2 ：将深度相机的disparity转为Depth , 也就是转换成为真正尺度下的深度
    //这里的判断条件感觉有些尴尬
    //前者和后者满足一个就可以了
    //满足前者意味着,mDepthMapFactor 相对1来讲要足够大
    //满足后者意味着,如果深度图像不是浮点型? 才会执行
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)

        //将图像转换成为另外一种数据类型,具有可选的数据大小缩放系数
        imDepth.convertTo(imDepth,          //输出图像
                         CV_32F,            //输出图像的数据类型
                         mDepthMapFactor);  //缩放系数

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    
    // 步骤3：构造Frame
    mCurrentFrame = Frame(
                mImGray,    //灰度图像
                imDepth,        //深度图像
                timestamp,          //时间戳
                mpORBextractorLeft, //ORB特征提取器
                mpORBVocabulary,    //词典
                mK,         //相机内参矩阵
                mDistCoef,  //相机的去畸变参数
                mbf,        //相机基线*相机焦距
                mThDepth,   //内外点区分深度阈值
                mpCamera);  

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    // 步骤4：跟踪
    Track();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();

    /*f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
    f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
    f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
    f_track_times << mTime_PreIntIMU << ",";
    f_track_times << mTime_PosePred << ",";
    f_track_times << mTime_LocalMapTrack << ",";
    f_track_times << mTime_NewKF_Dec << ",";
    f_track_times << t_track << endl;
#endif
    //返回当前帧的位姿
    return mCurrentFrame.mTcw.clone();
}

/**
 * @brief 
 * 输入左目RGB或RGBA图像，输出世界坐标系到该帧相机坐标系的变换矩阵
 * 
 * @param[in] im 单目图像
 * @param[in] timestamp 时间戳
 * @return cv::Mat 
 * 
 * Step 1 ：将彩色图像转为灰度图像
 * Step 2 ：构造Frame
 * Step 3 ：跟踪
 */
cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename)
{
    mImGray = im;

    // Step 1 ：将彩色图像转为灰度图像
    //若图片是3、4通道的，还需要转化成灰度图
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // Step 2 ：构造Frame
    //判断该帧是不是初始化
    if (mSensor == System::MONOCULAR)
    {
        //mpIniORBextractor     初始化ORB特征点提取器会提取2倍的指定特征点数目
        //mpORBextractorLeft,     //正常运行的时的ORB特征点提取器，提取指定数目特征点
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)//没有成功初始化的前一个状态就是NO_IMAGES_YET
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth);
    }
    else if(mSensor == System::IMU_MONOCULAR)
    {
        if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        {
            mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
        }
        else
            mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mpCamera,mDistCoef,mbf,mThDepth,&mLastFrame,*mpImuCalib);
    }

    if (mState==NO_IMAGES_YET)
        t0=timestamp;

    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    mCurrentFrame.mNameFile = filename;
    mCurrentFrame.mnDataset = mnNumDataset;

    lastID = mCurrentFrame.mnId;

    // Step 3 ：跟踪
    Track();

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    double t_track = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();

    /*f_track_stats << setprecision(0) << mCurrentFrame.mTimeStamp*1e9 << ",";
    f_track_stats << mvpLocalKeyFrames.size() << ",";
    f_track_stats << mvpLocalMapPoints.size() << ",";
    f_track_stats << setprecision(6) << t_track << endl;*/

#ifdef SAVE_TIMES
    f_track_times << mCurrentFrame.mTimeORB_Ext << ",";
    f_track_times << mCurrentFrame.mTimeStereoMatch << ",";
    f_track_times << mTime_PreIntIMU << ",";
    f_track_times << mTime_PosePred << ",";
    f_track_times << mTime_LocalMapTrack << ",";
    f_track_times << mTime_NewKF_Dec << ",";
    f_track_times << t_track << endl;
#endif
    //返回当前帧的位姿
    return mCurrentFrame.mTcw.clone();
}


void Tracking::GrabImuData(const IMU::Point &imuMeasurement)
{
    unique_lock<mutex> lock(mMutexImuQueue);
    mlQueueImuData.push_back(imuMeasurement);
}

void Tracking::PreintegrateIMU()
{
    //cout << "start preintegration" << endl;

    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    // cout << "start loop. Total meas:" << mlQueueImuData.size() << endl;

    mvImuFromLastFrame.clear();
    mvImuFromLastFrame.reserve(mlQueueImuData.size());
    if(mlQueueImuData.size() == 0)
    {
        Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
        mCurrentFrame.setIntegrated();
        return;
    }

    while(true)
    {
        bool bSleep = false;
        {
            unique_lock<mutex> lock(mMutexImuQueue);
            if(!mlQueueImuData.empty())
            {
                IMU::Point* m = &mlQueueImuData.front();
                cout.precision(17);
                if(m->t<mCurrentFrame.mpPrevFrame->mTimeStamp-0.001l)
                {
                    mlQueueImuData.pop_front();
                }
                else if(m->t<mCurrentFrame.mTimeStamp-0.001l)
                {
                    mvImuFromLastFrame.push_back(*m);
                    mlQueueImuData.pop_front();
                }
                else
                {
                    mvImuFromLastFrame.push_back(*m);
                    break;
                }
            }
            else
            {
                break;
                bSleep = true;
            }
        }
        if(bSleep)
            usleep(500);
    }


    const int n = mvImuFromLastFrame.size()-1;
    IMU::Preintegrated* pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,mCurrentFrame.mImuCalib);

    for(int i=0; i<n; i++)
    {
        float tstep;
        cv::Point3f acc, angVel;
        if((i==0) && (i<(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tini = mvImuFromLastFrame[i].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tini/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tini/tab))*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }
        else if(i<(n-1))
        {
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a)*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w)*0.5f;
            tstep = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
        }
        else if((i>0) && (i==(n-1)))
        {
            float tab = mvImuFromLastFrame[i+1].t-mvImuFromLastFrame[i].t;
            float tend = mvImuFromLastFrame[i+1].t-mCurrentFrame.mTimeStamp;
            acc = (mvImuFromLastFrame[i].a+mvImuFromLastFrame[i+1].a-
                    (mvImuFromLastFrame[i+1].a-mvImuFromLastFrame[i].a)*(tend/tab))*0.5f;
            angVel = (mvImuFromLastFrame[i].w+mvImuFromLastFrame[i+1].w-
                    (mvImuFromLastFrame[i+1].w-mvImuFromLastFrame[i].w)*(tend/tab))*0.5f;
            tstep = mCurrentFrame.mTimeStamp-mvImuFromLastFrame[i].t;
        }
        else if((i==0) && (i==(n-1)))
        {
            acc = mvImuFromLastFrame[i].a;
            angVel = mvImuFromLastFrame[i].w;
            tstep = mCurrentFrame.mTimeStamp-mCurrentFrame.mpPrevFrame->mTimeStamp;
        }

        if (!mpImuPreintegratedFromLastKF)
            cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
        mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(acc,angVel,tstep);
        pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(acc,angVel,tstep);
    }

    mCurrentFrame.mpImuPreintegratedFrame = pImuPreintegratedFromLastFrame;
    mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
    mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

    mCurrentFrame.setIntegrated();

    Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
}


bool Tracking::PredictStateIMU()
{
    if(!mCurrentFrame.mpPrevFrame)
    {
        Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    if(mbMapUpdated && mpLastKeyFrame)
    {
        const cv::Mat twb1 = mpLastKeyFrame->GetImuPosition();
        const cv::Mat Rwb1 = mpLastKeyFrame->GetImuRotation();
        const cv::Mat Vwb1 = mpLastKeyFrame->GetVelocity();

        const cv::Mat Gz = (cv::Mat_<float>(3,1) << 0,0,-IMU::GRAVITY_VALUE);
        const float t12 = mpImuPreintegratedFromLastKF->dT;

        cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1*mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
        cv::Mat twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
        cv::Mat Vwb2 = Vwb1 + t12*Gz + Rwb1*mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);
        mCurrentFrame.mPredRwb = Rwb2.clone();
        mCurrentFrame.mPredtwb = twb2.clone();
        mCurrentFrame.mPredVwb = Vwb2.clone();
        mCurrentFrame.mImuBias = mpLastKeyFrame->GetImuBias();
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else if(!mbMapUpdated)
    {
        const cv::Mat twb1 = mLastFrame.GetImuPosition();
        const cv::Mat Rwb1 = mLastFrame.GetImuRotation();
        const cv::Mat Vwb1 = mLastFrame.mVw;
        const cv::Mat Gz = (cv::Mat_<float>(3,1) << 0,0,-IMU::GRAVITY_VALUE);
        const float t12 = mCurrentFrame.mpImuPreintegratedFrame->dT;

        cv::Mat Rwb2 = IMU::NormalizeRotation(Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaRotation(mLastFrame.mImuBias));
        cv::Mat twb2 = twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaPosition(mLastFrame.mImuBias);
        cv::Mat Vwb2 = Vwb1 + t12*Gz + Rwb1*mCurrentFrame.mpImuPreintegratedFrame->GetDeltaVelocity(mLastFrame.mImuBias);

        mCurrentFrame.SetImuPoseVelocity(Rwb2,twb2,Vwb2);
        mCurrentFrame.mPredRwb = Rwb2.clone();
        mCurrentFrame.mPredtwb = twb2.clone();
        mCurrentFrame.mPredVwb = Vwb2.clone();
        mCurrentFrame.mImuBias =mLastFrame.mImuBias;
        mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return true;
    }
    else
        cout << "not IMU prediction!!" << endl;

    return false;
}


void Tracking::ComputeGyroBias(const vector<Frame*> &vpFs, float &bwx,  float &bwy, float &bwz)
{
    const int N = vpFs.size();
    vector<float> vbx;
    vbx.reserve(N);
    vector<float> vby;
    vby.reserve(N);
    vector<float> vbz;
    vbz.reserve(N);

    cv::Mat H = cv::Mat::zeros(3,3,CV_32F);
    cv::Mat grad  = cv::Mat::zeros(3,1,CV_32F);
    for(int i=1;i<N;i++)
    {
        Frame* pF2 = vpFs[i];
        Frame* pF1 = vpFs[i-1];
        cv::Mat VisionR = pF1->GetImuRotation().t()*pF2->GetImuRotation();
        cv::Mat JRg = pF2->mpImuPreintegratedFrame->JRg;
        cv::Mat E = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaRotation().t()*VisionR;
        cv::Mat e = IMU::LogSO3(E);
        assert(fabs(pF2->mTimeStamp-pF1->mTimeStamp-pF2->mpImuPreintegratedFrame->dT)<0.01);

        cv::Mat J = -IMU::InverseRightJacobianSO3(e)*E.t()*JRg;
        grad += J.t()*e;
        H += J.t()*J;
    }

    cv::Mat bg = -H.inv(cv::DECOMP_SVD)*grad;
    bwx = bg.at<float>(0);
    bwy = bg.at<float>(1);
    bwz = bg.at<float>(2);

    for(int i=1;i<N;i++)
    {
        Frame* pF = vpFs[i];
        pF->mImuBias.bwx = bwx;
        pF->mImuBias.bwy = bwy;
        pF->mImuBias.bwz = bwz;
        pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias);
        pF->mpImuPreintegratedFrame->Reintegrate();
    }
}

void Tracking::ComputeVelocitiesAccBias(const vector<Frame*> &vpFs, float &bax,  float &bay, float &baz)
{
    const int N = vpFs.size();
    const int nVar = 3*N +3; // 3 velocities/frame + acc bias
    const int nEqs = 6*(N-1);

    cv::Mat J(nEqs,nVar,CV_32F,cv::Scalar(0));
    cv::Mat e(nEqs,1,CV_32F,cv::Scalar(0));
    cv::Mat g = (cv::Mat_<float>(3,1)<<0,0,-IMU::GRAVITY_VALUE);

    for(int i=0;i<N-1;i++)
    {
        Frame* pF2 = vpFs[i+1];
        Frame* pF1 = vpFs[i];
        cv::Mat twb1 = pF1->GetImuPosition();
        cv::Mat twb2 = pF2->GetImuPosition();
        cv::Mat Rwb1 = pF1->GetImuRotation();
        cv::Mat dP12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaPosition();
        cv::Mat dV12 = pF2->mpImuPreintegratedFrame->GetUpdatedDeltaVelocity();
        cv::Mat JP12 = pF2->mpImuPreintegratedFrame->JPa;
        cv::Mat JV12 = pF2->mpImuPreintegratedFrame->JVa;
        float t12 = pF2->mpImuPreintegratedFrame->dT;
        // Position p2=p1+v1*t+0.5*g*t^2+R1*dP12
        J.rowRange(6*i,6*i+3).colRange(3*i,3*i+3) += cv::Mat::eye(3,3,CV_32F)*t12;
        J.rowRange(6*i,6*i+3).colRange(3*N,3*N+3) += Rwb1*JP12;
        e.rowRange(6*i,6*i+3) = twb2-twb1-0.5f*g*t12*t12-Rwb1*dP12;
        // Velocity v2=v1+g*t+R1*dV12
        J.rowRange(6*i+3,6*i+6).colRange(3*i,3*i+3) += -cv::Mat::eye(3,3,CV_32F);
        J.rowRange(6*i+3,6*i+6).colRange(3*(i+1),3*(i+1)+3) += cv::Mat::eye(3,3,CV_32F);
        J.rowRange(6*i+3,6*i+6).colRange(3*N,3*N+3) -= Rwb1*JV12;
        e.rowRange(6*i+3,6*i+6) = g*t12+Rwb1*dV12;
    }

    cv::Mat H = J.t()*J;
    cv::Mat B = J.t()*e;
    cv::Mat x(nVar,1,CV_32F);
    cv::solve(H,B,x);

    bax = x.at<float>(3*N);
    bay = x.at<float>(3*N+1);
    baz = x.at<float>(3*N+2);

    for(int i=0;i<N;i++)
    {
        Frame* pF = vpFs[i];
        x.rowRange(3*i,3*i+3).copyTo(pF->mVw);
        if(i>0)
        {
            pF->mImuBias.bax = bax;
            pF->mImuBias.bay = bay;
            pF->mImuBias.baz = baz;
            pF->mpImuPreintegratedFrame->SetNewBias(pF->mImuBias);
        }
    }
}

void Tracking::ResetFrameIMU()
{
    // TODO To implement...
}

/*
 * @brief Main tracking function. It is independent of the input sensor.
 *
 * track包含两部分：估计运动、跟踪局部地图
 * 
 * Step 1：初始化
 * Step 2：跟踪
 * Step 3：记录位姿信息，用于轨迹复现
 */
void Tracking::Track()
{
#ifdef SAVE_TIMES
    mTime_PreIntIMU = 0;
    mTime_PosePred = 0;
    mTime_LocalMapTrack = 0;
    mTime_NewKF_Dec = 0;
#endif

    if (bStepByStep)
    {
        while(!mbStep)
            usleep(500);
        mbStep = false;
    }

    if(mpLocalMapper->mbBadImu)
    {
        cout << "TRACK: Reset map because local mapper set the bad imu flag " << endl;
        mpSystem->ResetActiveMap();
        return;
    }

    Map* pCurrentMap = mpAtlas->GetCurrentMap();

    // track包含两部分：估计运动、跟踪局部地图
    
    // mState为tracking的状态，包括 SYSTME_NOT_READY, NO_IMAGE_YET, NOT_INITIALIZED, OK, LOST
    // 如果图像复位过、或者第一次运行，则为NO_IMAGE_YET状态
    if(mState!=NO_IMAGES_YET)
    {
        if(mLastFrame.mTimeStamp>mCurrentFrame.mTimeStamp)
        {
            cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
            unique_lock<mutex> lock(mMutexImuQueue);
            mlQueueImuData.clear();
            CreateMapInAtlas();
            return;
        }
        else if(mCurrentFrame.mTimeStamp>mLastFrame.mTimeStamp+1.0)
        {
            cout << "id last: " << mLastFrame.mnId << "    id curr: " << mCurrentFrame.mnId << endl;
            if(mpAtlas->isInertial())
            {

                if(mpAtlas->isImuInitialized())
                {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    if(!pCurrentMap->GetIniertialBA2())
                    {
                        mpSystem->ResetActiveMap();
                    }
                    else
                    {
                        CreateMapInAtlas();
                    }
                }
                else
                {
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }
            }

            return;
        }
    }


    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && mpLastKeyFrame)
        mCurrentFrame.SetNewBias(mpLastKeyFrame->GetImuBias());

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }
   
    // mLastProcessedState 存储了Tracking最新的状态，用于FrameDrawer中的绘制
    mLastProcessedState=mState;

    if ((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) && !mbCreatedMap)
    {
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
#endif
        PreintegrateIMU();
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        mTime_PreIntIMU = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t1 - t0).count();
#endif


    }
    mbCreatedMap = false;

    // Get Map Mutex -> Map cannot be changed
    // 上锁。保证地图不会发生变化
    // 疑问:这样子会不会影响地图的更新?
    // 回答：主要耗时在构造帧中特征点的提取和匹配部分,在那个时候地图是没有被上锁的,有足够的时间更新地图
    unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

    mbMapUpdated = false;

    int nCurMapChangeIndex = pCurrentMap->GetMapChangeIndex();
    int nMapChangeIndex = pCurrentMap->GetLastMapChange();
    if(nCurMapChangeIndex>nMapChangeIndex)
    {
        // cout << "Map update detected" << endl;
        pCurrentMap->SetLastMapChange(nCurMapChangeIndex);
        mbMapUpdated = true;
    }

    // Step 1：初始化
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD || mSensor==System::IMU_STEREO)
            //双目RGBD相机的初始化共用一个函数
            StereoInitialization();
        else
        {
            //单目初始化
            MonocularInitialization();
        }

        //更新帧绘制器中存储的最新状态
        mpFrameDrawer->Update(this);

        //这个状态量在上面的初始化函数中被更新
        if(mState!=OK) // If rightly initialized, mState=OK
        {
            mLastFrame = Frame(mCurrentFrame);
            return;
        }

        if(mpAtlas->GetAllMaps().size() == 1)
        {
            mnFirstFrameId = mCurrentFrame.mnId;
        }
    }
    else
    {
        // System is initialized. Track Frame.
        // bOK为临时变量，用于表示每个函数是否执行成功
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        // mbOnlyTracking等于false表示正常SLAM模式（定位+地图更新），mbOnlyTracking等于true表示仅定位模式
        // tracking 类构造时默认为false。在viewer中有个开关ActivateLocalizationMode，可以控制是否开启mbOnlyTracking
        if(!mbOnlyTracking)
        {
#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point timeStartPosePredict = std::chrono::steady_clock::now();
#endif

            // State OK
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.
            // Step 2：跟踪进入正常SLAM模式，有地图更新
            // 正常初始化成功
            if(mState==OK)
            {

                // Local Mapping might have changed some MapPoints tracked in last frame
                // Step 2.1 检查并更新上一帧被替换的MapPoints
                // 局部建图线程则可能会对原有的地图点进行替换.在这里进行检查
                CheckReplacedInLastFrame();
                
                // Step 2.2 运动模型是空的或刚完成重定位，跟踪参考关键帧；否则恒速模型跟踪
                // 第一个条件,如果运动模型为空,说明是刚初始化开始，或者已经跟丢了
                // 第二个条件,如果当前帧紧紧地跟着在重定位的帧
                if((mVelocity.empty() && !pCurrentMap->isImuInitialized()) || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    //Verbose::PrintMess("TRACK: Track with respect to the reference KF ", Verbose::VERBOSITY_DEBUG);
                    // 用最近的关键帧来跟踪当前的普通帧
                    // 通过BoW的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点都对应3D点重投影误差即可得到位姿
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    //Verbose::PrintMess("TRACK: Track with motion model", Verbose::VERBOSITY_DEBUG);
                    // 用最近的普通帧来跟踪当前的普通帧
                    // 根据恒速模型设定当前帧的初始位姿
                    // 通过投影的方式在参考帧中找当前帧特征点的匹配点
                    // 优化每个特征点所对应3D点的投影误差即可得到位姿
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        //根据恒速模型失败了，只能根据参考关键帧来跟踪
                        bOK = TrackReferenceKeyFrame();
                }


                if (!bOK)
                {
                    if ( mCurrentFrame.mnId<=(mnLastRelocFrameId+mnFramesToResetIMU) &&
                         (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
                    {
                        mState = LOST;
                    }
                    else if(pCurrentMap->KeyFramesInMap()>10)
                    {
                        cout << "KF in map: " << pCurrentMap->KeyFramesInMap() << endl;
                        mState = RECENTLY_LOST;
                        mTimeStampLost = mCurrentFrame.mTimeStamp;
                        //mCurrentFrame.SetPose(mLastFrame.mTcw);
                    }
                    else
                    {
                        mState = LOST;
                    }
                }
            }
            else
            {

                if (mState == RECENTLY_LOST)
                {
                    Verbose::PrintMess("Lost for a short time", Verbose::VERBOSITY_NORMAL);

                    bOK = true;
                    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))
                    {
                        if(pCurrentMap->isImuInitialized())
                            PredictStateIMU();
                        else
                            bOK = false;

                        if (mCurrentFrame.mTimeStamp-mTimeStampLost>time_recently_lost)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                    else
                    {
                        // TODO fix relocalization
                        // 如果跟踪状态不成功,那么就只能重定位了
                        // BOW搜索，EPnP求解位姿
                        bOK = Relocalization();
                        if(!bOK)
                        {
                            mState = LOST;
                            Verbose::PrintMess("Track Lost...", Verbose::VERBOSITY_NORMAL);
                            bOK=false;
                        }
                    }
                }
                else if (mState == LOST)
                {
                    Verbose::PrintMess("A new map is started...", Verbose::VERBOSITY_NORMAL);

                    if (pCurrentMap->KeyFramesInMap()<10)
                    {
                        mpSystem->ResetActiveMap();
                        cout << "Reseting current map..." << endl;
                    }else
                        CreateMapInAtlas();

                    if(mpLastKeyFrame)
                        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

                    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

                    return;
                }
            }


#ifdef SAVE_TIMES
        std::chrono::steady_clock::time_point timeEndPosePredict = std::chrono::steady_clock::now();

        mTime_PosePred = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(timeEndPosePredict - timeStartPosePredict).count();
#endif

        }
        else
        {
            // Localization Mode: Local Mapping is deactivated (TODO Not available in inertial mode)
            // Step 2：只进行跟踪tracking，局部地图不工作
            if(mState==LOST)
            {
                if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                    Verbose::PrintMess("IMU. State LOST", Verbose::VERBOSITY_NORMAL);
                // Step 2.1 如果跟丢了，只能重定位
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // Step 2.2 如果跟踪正常，使用恒速模型 或 参考关键帧跟踪
                    // In last frame we tracked enough MapPoints in the map
                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        // 如果恒速模型不被满足,那么就只能够通过参考关键帧来定位
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    // mbVO为true，表明此帧匹配了很少（小于10）的地图点，要跪的节奏，既做跟踪又做重定位

                    //MM=Motion Model,通过运动模型进行跟踪的结果
                    bool bOKMM = false;

                    //通过重定位方法来跟踪的结果
                    bool bOKReloc = false;

                    //运动模型中构造的地图点
                    vector<MapPoint*> vpMPsMM;

                    //在追踪运动模型后发现的外点
                    vector<bool> vbOutMM;

                    //运动模型得到的位姿
                    cv::Mat TcwMM;

                    // Step 2.3 当运动模型有效的时候,根据运动模型计算位姿
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();

                        // 将恒速模型跟踪结果暂存到这几个变量中，因为后面重定位会改变这些变量
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }

                    // Step 2.4 使用重定位的方法来得到当前帧的位姿
                    bOKReloc = Relocalization();

                    // Step 2.5 根据前面的恒速模型、重定位结果来更新状态
                    if(bOKMM && !bOKReloc)
                    {
                        // 恒速模型成功、重定位失败，重新使用之前暂存的恒速模型结果
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        // 如果当前帧匹配的3D点很少，增加当前可视地图点的被观测次数
                        if(mbVO)
                        {
                            // 更新当前帧的地图点被观测次数
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                //如果这个特征点形成了地图点,并且也不是外点的时候
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    //增加能观测到该地图点的帧数
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        // 只要重定位成功整个跟踪过程正常进行（重定位与跟踪，更相信重定位）
                        mbVO = false;
                    }
                    
                    //有一个成功我们就认为执行成功了
                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            // 将最新的关键帧作为当前帧的参考关键帧
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        // Step 3：在跟踪得到当前帧初始姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
        // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
        if(!mbOnlyTracking)
        {
            if(bOK)
            {
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point time_StartTrackLocalMap = std::chrono::steady_clock::now();
#endif
                bOK = TrackLocalMap();
#ifdef SAVE_TIMES
                std::chrono::steady_clock::time_point time_EndTrackLocalMap = std::chrono::steady_clock::now();

                mTime_LocalMapTrack = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndTrackLocalMap - time_StartTrackLocalMap).count();
#endif


            }
            if(!bOK)
                cout << "Fail to track local map!" << endl;
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            // 重定位成功
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        //根据上面的操作来判断是否追踪成功
        if(bOK)
            mState = OK;
        else if (mState == OK)
        {
            if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            {
                Verbose::PrintMess("Track lost for less than one second...", Verbose::VERBOSITY_NORMAL);
                if(!pCurrentMap->isImuInitialized() || !pCurrentMap->GetIniertialBA2())
                {
                    cout << "IMU is not or recently initialized. Reseting active map..." << endl;
                    mpSystem->ResetActiveMap();
                }

                mState=RECENTLY_LOST;
            }
            else
                mState=LOST; // visual to lost

            if(mCurrentFrame.mnId>mnLastRelocFrameId+mMaxFrames)
            {
                mTimeStampLost = mCurrentFrame.mTimeStamp;
            }
        }

        // Save frame if recent relocalization, since they are used for IMU reset (as we are making copy, it shluld be once mCurrFrame is completely modified)
        if((mCurrentFrame.mnId<(mnLastRelocFrameId+mnFramesToResetIMU)) && (mCurrentFrame.mnId > mnFramesToResetIMU) && ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && pCurrentMap->isImuInitialized())
        {
            // TODO check this situation
            Verbose::PrintMess("Saving pointer to frame. imu needs reset...", Verbose::VERBOSITY_NORMAL);
            Frame* pF = new Frame(mCurrentFrame);
            pF->mpPrevFrame = new Frame(mLastFrame);

            // Load preintegration
            pF->mpImuPreintegratedFrame = new IMU::Preintegrated(mCurrentFrame.mpImuPreintegratedFrame);
        }

        if(pCurrentMap->isImuInitialized())
        {
            if(bOK)
            {
                if(mCurrentFrame.mnId==(mnLastRelocFrameId+mnFramesToResetIMU))
                {
                    cout << "RESETING FRAME!!!" << endl;
                    ResetFrameIMU();
                }
                else if(mCurrentFrame.mnId>(mnLastRelocFrameId+30))
                    mLastBias = mCurrentFrame.mImuBias;
            }
        }

        // Update drawer
        // Step 4：更新显示线程中的图像、特征点、地图点等信息
        mpFrameDrawer->Update(this);
        if(!mCurrentFrame.mTcw.empty())
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        if(bOK || mState==RECENTLY_LOST)
        {
            // Update motion model
            // Step 5：跟踪成功，更新恒速运动模型
            if(!mLastFrame.mTcw.empty() && !mCurrentFrame.mTcw.empty())
            {
                // 更新恒速运动模型 TrackWithMotionModel 中的mVelocity
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                //否则速度为空
                mVelocity = cv::Mat();

            if(mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
                //更新显示中的位姿
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            // Step 6：清除观测不到的地图点   
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            // Step 7：清除恒速模型跟踪中 UpdateLastFrame中为当前帧临时添加的MapPoints（仅双目和rgbd）
            // 步骤6中只是在当前帧中将这些MapPoints剔除，这里从MapPoints数据库中删除
            // 临时地图点仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }

            // 这里不仅仅是清除mlpTemporalPoints，通过delete pMP还删除了指针指向的MapPoint
            // 不能够直接执行这个是因为其中存储的都是指针,之前的操作都是为了避免内存泄露
            mlpTemporalPoints.clear();

#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point timeStartNewKF = std::chrono::steady_clock::now();
#endif
            bool bNeedKF = NeedNewKeyFrame();
#ifdef SAVE_TIMES
            std::chrono::steady_clock::time_point timeEndNewKF = std::chrono::steady_clock::now();

            mTime_NewKF_Dec = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(timeEndNewKF - timeStartNewKF).count();
#endif



            // Check if we need to insert a new keyframe
            // Step 8：检测并插入关键帧，对于双目或RGB-D会产生新的地图点
            if(bNeedKF && (bOK|| (mState==RECENTLY_LOST && (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO))))
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame. Only has effect if lastframe is tracked
            // 作者这里说允许在BA中被Huber核函数判断为外点的传入新的关键帧中，让后续的BA来审判他们是不是真正的外点
            // 但是估计下一帧位姿的时候我们不想用这些外点，所以删掉

            //  Step 9 删除那些在bundle adjustment中检测为outlier的地图点
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        // Step 10 如果初始化后不久就跟踪失败，并且relocation也没有搞定，只能重新Reset
        if(mState==LOST)
        {
            //如果地图中的关键帧信息过少的话,直接重新进行初始化了
            if(pCurrentMap->KeyFramesInMap()<=5)
            {
                mpSystem->ResetActiveMap();
                return;
            }
            if ((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO))
                if (!pCurrentMap->isImuInitialized())
                {
                    Verbose::PrintMess("Track lost before IMU initialisation, reseting...", Verbose::VERBOSITY_QUIET);
                    mpSystem->ResetActiveMap();
                    return;
                }

            CreateMapInAtlas();
        }

        //确保已经设置了参考关键帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // 保存上一帧的数据,当前帧变上一帧
        mLastFrame = Frame(mCurrentFrame);
    }




    if(mState==OK || mState==RECENTLY_LOST)
    {
        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        // Step 11：记录位姿信息，用于最后保存所有的轨迹
        if(!mCurrentFrame.mTcw.empty())
        {
            // 计算相对姿态Tcr = Tcw * Twr, Twr = Trw^-1
            cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
            
            //保存各种状态
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mCurrentFrame.mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            // 如果跟踪失败，则相对位姿使用上一次值
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState==LOST);
        }

    }
}

/*
 * @brief 双目和rgbd的地图初始化，比单目简单很多
 *
 * 由于具有深度信息，直接生成MapPoints
 */
void Tracking::StereoInitialization()
{
    // 初始化要求当前帧的特征点超过500
    if(mCurrentFrame.N>500)
    {
        if (mSensor == System::IMU_STEREO)
        {
            if (!mCurrentFrame.mpImuPreintegrated || !mLastFrame.mpImuPreintegrated)
            {
                cout << "not IMU meas" << endl;
                return;
            }

            if (cv::norm(mCurrentFrame.mpImuPreintegratedFrame->avgA-mLastFrame.mpImuPreintegratedFrame->avgA)<0.5)
            {
                cout << "not enough acceleration" << endl;
                return;
            }

            if(mpImuPreintegratedFromLastKF)
                delete mpImuPreintegratedFromLastKF;

            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
            mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;
        }

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        if (mSensor == System::IMU_STEREO)
        {
            cv::Mat Rwb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).colRange(0,3).clone();
            cv::Mat twb0 = mCurrentFrame.mImuCalib.Tcb.rowRange(0,3).col(3).clone();
            mCurrentFrame.SetImuPoseVelocity(Rwb0, twb0, cv::Mat::zeros(3,1,CV_32F));
        }
        else
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        // 将当前帧构造为初始关键帧
        // mCurrentFrame的数据类型为Frame
        // KeyFrame包含Frame、地图3D点、以及BoW
        // KeyFrame里有一个mpMap，Tracking里有一个mpMap，而KeyFrame里的mpMap都指向Tracking里的这个mpMap
        // KeyFrame里有一个mpKeyFrameDB，Tracking里有一个mpKeyFrameDB，而KeyFrame里的mpMap都指向Tracking里的这个mpKeyFrameDB
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

        // Insert KeyFrame in the map
        // KeyFrame中包含了地图、反过来地图中也包含了KeyFrame，相互包含
        // 在地图中添加该初始关键帧
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        // 为每个特征点构造MapPoint
        if(!mpCamera2){
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                //只有具有正深度的点才会被构造地图点
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    // 通过反投影得到该特征点的世界坐标系下3D坐标
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap());
                    
                    // 为该MapPoint添加属性：
                    // a.观测到该MapPoint的关键帧
                    // b.该MapPoint的描述子
                    // c.该MapPoint的平均观测方向和深度范围

                    // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
                    pNewMP->AddObservation(pKFini,i);
                    pKFini->AddMapPoint(pNewMP,i);

                    // b.从众多观测到该MapPoint的特征点中挑选区分度最高的描述子
                    pNewMP->ComputeDistinctiveDescriptors();

                    // c.更新该MapPoint平均观测方向以及观测距离的范围
                    pNewMP->UpdateNormalAndDepth();

                    // 在地图中添加该MapPoint
                    mpAtlas->AddMapPoint(pNewMP);

                    // 将该MapPoint添加到当前帧的mvpMapPoints中
                    // 为当前Frame的特征点与MapPoint之间建立索引
                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }
        } else{
            for(int i = 0; i < mCurrentFrame.Nleft; i++){
                int rightIndex = mCurrentFrame.mvLeftToRightMatch[i];
                if(rightIndex != -1){
                    cv::Mat x3D = mCurrentFrame.mvStereo3Dpoints[i];

                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpAtlas->GetCurrentMap());

                    pNewMP->AddObservation(pKFini,i);
                    pNewMP->AddObservation(pKFini,rightIndex + mCurrentFrame.Nleft);

                    pKFini->AddMapPoint(pNewMP,i);
                    pKFini->AddMapPoint(pNewMP,rightIndex + mCurrentFrame.Nleft);

                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    mCurrentFrame.mvpMapPoints[rightIndex + mCurrentFrame.Nleft]=pNewMP;
                }
            }
        }

        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);

        // 在局部地图中添加该初始关键帧
        mpLocalMapper->InsertKeyFrame(pKFini);

        // 更新当前帧为上一帧
        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        mnLastRelocFrameId = mCurrentFrame.mnId;

        mvpLocalKeyFrames.push_back(pKFini);

        //这个点只是暂时被保存在了 Tracking 线程之中， 所以称之为 local 
        // 初始化之后，通过双目图像生成的地图点，都应该被认为是局部地图点
        mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        // 把当前（最新的）局部MapPoints作为ReferenceMapPoints
        // ReferenceMapPoints是DrawMapPoints函数画图的时候用的
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        //追踪成功
        mState=OK;
    }
}

/*
 * @brief 单目的地图初始化
 *
 * 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
 * 得到初始两帧的匹配、相对运动、初始MapPoints
 * 
 * Step 1：（未创建）得到用于初始化的第一帧，初始化需要两帧
 * Step 2：（已创建）如果当前帧特征点数大于100，则得到用于单目初始化的第二帧
 * Step 3：在mInitialFrame与mCurrentFrame中找匹配的特征点对
 * Step 4：如果初始化的两帧之间的匹配点太少，重新初始化
 * Step 5：通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
 * Step 6：删除那些无法进行三角化的匹配点
 * Step 7：将三角化得到的3D点包装成MapPoints
 */
void Tracking::MonocularInitialization()
{
    // Step 1 如果单目初始器还没有被创建，则创建。后面如果重新初始化时会清掉这个
    if(!mpInitializer)
    {
        // Set Reference Frame
        // 单目初始帧的特征点数必须大于100
        if(mCurrentFrame.mvKeys.size()>100)
        {
            // 初始化需要两帧，分别是mInitialFrame，mCurrentFrame
            mInitialFrame = Frame(mCurrentFrame);

            // 用当前帧更新上一帧
            mLastFrame = Frame(mCurrentFrame);

            // mvbPrevMatched  记录"上一帧"所有特征点
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            // 删除前判断一下，来避免出现段错误。不过在这里是多余的判断
            // 不过在这里是多余的判断，因为前面已经判断过了
            if(mpInitializer)
                delete mpInitializer;

            // 由当前帧构造初始器 sigma:1.0 iterations:200s
            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            // 初始化为-1 表示没有任何匹配。这里面存储的是匹配的点的id
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            if (mSensor == System::IMU_MONOCULAR)
            {
                if(mpImuPreintegratedFromLastKF)
                {
                    delete mpImuPreintegratedFromLastKF;
                }
                mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
                mCurrentFrame.mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            }
            return;
        }
    }
    else    //如果单目初始化器已经被创建
    {
        // Try to initialize
        // Step 2 如果当前帧特征点数太少（不超过100），则重新构造初始器
        // NOTICE 只有连续两帧的特征点个数都大于100时，才能继续进行初始化过程
        if (((int)mCurrentFrame.mvKeys.size()<=100)||((mSensor == System::IMU_MONOCULAR)&&(mLastFrame.mTimeStamp-mInitialFrame.mTimeStamp>1.0)))
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }

        // Find correspondences
        // Step 3 在mInitialFrame与mCurrentFrame中找匹配的特征点对
        ORBmatcher matcher(
            0.9,        //最佳的和次佳特征点评分的比值阈值，这里是比较宽松的，跟踪时一般是0.7
            true);      //检查特征点的方向
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpCamera->ReconstructWithTwoViews(mInitialFrame.mvKeysUn,mCurrentFrame.mvKeysUn,mvIniMatches,Rcw,tcw,mvIniP3D,vbTriangulated))
        {
            //初始化成功后，删除那些无法进行三角化的匹配点
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            //将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            // 由Rcw和tcw构造Tcw,并赋值给mTcw，mTcw为世界坐标系到相机坐标系的变换矩阵
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            // Initialize函数会得到mvIniP3D，
            // mvIniP3D是cv::Point3f类型的一个容器，是个存放3D点的临时变量，
            // CreateInitialMapMonocular将3D点包装成MapPoint类型存入KeyFrame和Map中
            CreateInitialMapMonocular();

            // Just for video
            // bStepByStep = true;
        }
    }
}


/**
 * @brief 单目相机成功初始化后用三角化得到的点生成MapPoints
 * 
 */
void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames 认为单目初始化时候的参考帧和当前帧都是关键帧
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);// 第一帧
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);// 第二帧
    
    if(mSensor == System::IMU_MONOCULAR)
        // Step 1 将初始关键帧,当前关键帧的描述子转为BoW
        pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);

    
    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    // Step 2 将关键帧插入到地图
    mpAtlas->AddKeyFrame(pKFini);
    mpAtlas->AddKeyFrame(pKFcur);

    // Step 3 用初始化得到的3D点来生成地图点MapPoints
    //  mvIniMatches[i] 表示初始化两帧特征点匹配关系。
    //  具体解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值,没有匹配关系的话，vMatches12[i]值为 -1
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        // 用三角化点初始化为空间点的世界坐标
        cv::Mat worldPos(mvIniP3D[i]);

        // Step 3.1 用3D点构造MapPoint
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpAtlas->GetCurrentMap());


        // Step 3.2 为该MapPoint添加属性：
        // a.观测到该MapPoint的关键帧
        // b.该MapPoint的描述子
        // c.该MapPoint的平均观测方向和深度范围
        // 表示该KeyFrame的2D特征点和对应的3D地图点
        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        // a.表示该MapPoint可以被哪个KeyFrame的哪个特征点观测到
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        // b.从众多观测到该MapPoint的特征点中挑选最有代表性的描述子
        pMP->ComputeDistinctiveDescriptors();

        // c.更新该MapPoint平均观测方向以及观测距离的范围
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        //mvIniMatches下标i表示在初始化参考帧中的特征点的序号
        //mvIniMatches[i]是初始化当前帧中的特征点的序号
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpAtlas->AddMapPoint(pMP);
    }


    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    std::set<MapPoint*> sMPs;
    sMPs = pKFini->GetMapPoints();

    // Bundle Adjustment
    Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points", Verbose::VERBOSITY_QUIET);
    // Step 4 全局BA优化，同时优化所有位姿和三维点
    Optimizer::GlobalBundleAdjustemnt(mpAtlas->GetCurrentMap(),20);

    pKFcur->PrintPointDistribution();

    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth;
    if(mSensor == System::IMU_MONOCULAR)
        invMedianDepth = 4.0f/medianDepth; // 4.0f
    else
        invMedianDepth = 1.0f/medianDepth;

    //两个条件,一个是平均深度要大于0,另外一个是在当前帧中被观测到的地图点的数目应该大于100
    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50) // TODO Check, originally 100 tracks
    {
        Verbose::PrintMess("Wrong initialization, reseting...", Verbose::VERBOSITY_NORMAL);
        mpSystem->ResetActiveMap();
        return;
    }

    // Step 6 将两帧之间的变换归一化到平均深度1的尺度下
    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();

    // x/z y/z 将z归一化到1 
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    // Step 7 把3D点的尺度也归一化到1
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
            pMP->UpdateNormalAndDepth();
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
    {
        pKFcur->mPrevKF = pKFini;
        pKFini->mNextKF = pKFcur;
        pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
    }

    //  Step 8 将关键帧插入局部地图，更新归一化后的位姿、局部地图点
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;
    mnLastRelocFrameId = mInitialFrame.mnId;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);

    // 单目初始化之后，得到的初始地图中的所有点都是局部地图点
    mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    // Compute here initial velocity
    vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

    cv::Mat deltaT = vKFs.back()->GetPose()*vKFs.front()->GetPoseInverse();
    mVelocity = cv::Mat();
    Eigen::Vector3d phi = LogSO3(Converter::toMatrix3d(deltaT.rowRange(0,3).colRange(0,3)));


    double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
    phi *= aux;

    mLastFrame = Frame(mCurrentFrame);

    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;// 初始化成功，至此，初始化过程完成

    initID = pKFcur->mnId;
}


void Tracking::CreateMapInAtlas()
{
    mnLastInitFrameId = mCurrentFrame.mnId;
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();
    mbSetInit=false;

    mnInitialFrameId = mCurrentFrame.mnId+1;
    mState = NO_IMAGES_YET;

    // Restart the variable with information about the last KF
    mVelocity = cv::Mat();
    mnLastRelocFrameId = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
    Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId+1), Verbose::VERBOSITY_NORMAL);
    mbVO = false; // Init value for know if there are enough MapPoints in the last KF
    if(mSensor == System::MONOCULAR || mSensor == System::IMU_MONOCULAR)
    {
        if(mpInitializer)
            delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO ) && mpImuPreintegratedFromLastKF)
    {
        delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(),*mpImuCalib);
    }

    if(mpLastKeyFrame)
        mpLastKeyFrame = static_cast<KeyFrame*>(NULL);

    if(mpReferenceKF)
        mpReferenceKF = static_cast<KeyFrame*>(NULL);

    mLastFrame = Frame();
    mCurrentFrame = Frame();
    mvIniMatches.clear();

    mbCreatedMap = true;

}

/*
 * @brief 检查上一帧中的地图点是否需要被替换
 * 
 * Local Mapping线程可能会将关键帧中某些地图点进行替换，由于tracking中需要用到上一帧地图点，所以这里检查并更新上一帧中被替换的地图点
 * @see LocalMapping::SearchInNeighbors()
 */
void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        //如果这个地图点存在
        if(pMP)
        {
            // 获取其是否被替换,以及替换后的点
            // 这也是程序不直接删除这个地图点删除的原因
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                //然后替换一下
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

/*
 * @brief 对参考关键帧的MapPoints进行跟踪（关键帧模式）
 * 
 * 1. 计算当前帧的词包，将当前帧的特征点分到特定层的nodes上
 * 2. 对属于同一node的描述子进行匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return 如果匹配数大于10，返回true
 * 
 * Step 1：将当前帧的描述子转化为BoW向量
 * Step 2：通过特征点的BoW加快当前帧与参考帧之间的特征点匹配
 * Step 3:将上一帧的位姿态作为当前帧位姿的初始值
 * Step 4:通过优化3D-2D的重投影误差来获得位姿
 * Step 5：剔除优化后的outlier匹配点（MapPoints）
 */
bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    // Step 1：将当前帧的描述子转化为BoW向量
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    // Step 2：通过特征点的BoW加快当前帧与参考帧之间的特征点匹配
    // 特征点的匹配关系由MapPoints进行维护
    int nmatches = matcher.SearchByBoW(
        mpReferenceKF,  //参考关键帧
        mCurrentFrame,  //当前帧
        vpMapPointMatches); //存储匹配关系

    //这里的匹配数超过15才继续
    if(nmatches<15)
    {
        cout << "TRACK_REF_KF: Less than 15 matches!!\n";
        return false;
    }

    // Step 3:将上一帧的位姿态作为当前帧位姿的初始值
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw); // 用上一次的Tcw设置初值，在PoseOptimization可以收敛快一些

    //mCurrentFrame.PrintPointDistribution();


    // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
    // Step 4:通过优化3D-2D的重投影误差来获得位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    // Step 5：剔除优化后的outlier匹配点（MapPoints）
    //之所以在优化之后才剔除外点，是因为在优化的过程中就有了对这些外点的标记
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        //if(i >= mCurrentFrame.Nleft) break;
        if(mCurrentFrame.mvpMapPoints[i])
        {
            //如果对应到的某个特征点是外点
            if(mCurrentFrame.mvbOutlier[i])
            {
                //清除它在当前帧中存在过的痕迹
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                //匹配的内点计数++
                nmatchesMap++;
        }
    }

    // TODO check these conditions
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;
    else
        return nmatchesMap>=10;
}

/**
 * @brief 更新上一帧位姿，在上一帧深度值中生成临时地图点
 * 单目情况：只计算了上一帧的世界坐标系位姿
 * 双目和rgbd情况：选取有有深度值的并且没有被选为地图点的点生成新的临时地图点，提高跟踪鲁棒性
 */
void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    // Step 1：计算上一帧在世界坐标系下的位姿
    // 上一普通帧的参考关键帧，注意这里用的是参考关键帧（位姿准）而不是上上一帧的普通帧
    KeyFrame* pRef = mLastFrame.mpReferenceKF;

    // ref_keyframe 到 lastframe的位姿变换
    cv::Mat Tlr = mlRelativeFramePoses.back();

    // 将上一帧的世界坐标系下的位姿计算出来
    // l:last, r:reference, w:world
    // Tlw = Tlr*Trw 
    mLastFrame.SetPose(Tlr*pRef->GetPose());

    // 如果上一帧为关键帧，或者单目的情况，则退出
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR || !mbOnlyTracking)
        return;
    
    // Step 2：对于双目或rgbd摄像头，为上一帧临时生成新的地图点
    // 注意这些地图点只是用来跟踪，不加入到Map中，tracking用完后会删除
    // 跟踪过程中将上一帧地图点投影到当前帧可以缩小匹配范围，加快特征点匹配

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    // Step 2.1：得到上一帧有深度值的特征点（不一定是地图点）
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            // vDepthIdx第一个元素是某个点的深度,第二个元素是对应的特征点id
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    // 如果上一帧中没有有效深度的点,那么就直接退出
    if(vDepthIdx.empty())
        return;

    // 按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    // Step 2.2：从中找出不是地图点的生成临时地图点 
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;
        // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            // 地图点被创建后就没有被观测，也需要创建
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            // 新建地图点只是为了提高双目和RGBD的跟踪成功率，并没有添加复杂属性，因为后面会扔掉
            // 反投影到世界坐标系中
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(
                x3D,    // 世界坐标系坐标
                mpAtlas->GetCurrentMap(),   // 跟踪的全局地图
                &mLastFrame,    // 存在这个特征点的帧(上一帧)
                i); // 特征点id

            // 加入上一帧的地图点中
            mLastFrame.mvpMapPoints[i]=pNewMP;

            // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删除
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            // 因为从近到远排序，记录其中不需要创建地图点的个数
            nPoints++;
        }

        // 停止新增临时地图点必须同时满足以下条件：
        // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
        // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
        {
            break;
        }
    }
}

/**
 * @brief 根据匀速度模型对上一帧的地图点进行跟踪
 * Step 1：更新上一帧的位姿；对于双目或rgbd摄像头，还会根据深度值生成临时地图点
 * Step 2：根据上一帧特征点对应地图点进行投影匹配
 * Step 3：优化位姿
 * Step 4：优化位姿后剔除地图点中外点
 * @return 如果匹配数大于10，认为跟踪成功，返回true
 */
bool Tracking::TrackWithMotionModel()
{
    // 最小距离 < 0.9*次小距离 匹配成功，检查旋转
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    // Step 1：更新上一帧的位姿；对于双目或rgbd摄像头，还会根据深度值生成临时地图点
    UpdateLastFrame();



    if (mpAtlas->isImuInitialized() && (mCurrentFrame.mnId>mnLastRelocFrameId+mnFramesToResetIMU))
    {
        // Predict ste with IMU if it is initialized and it doesnt need reset
        PredictStateIMU();
        return true;
    }
    else
    {
        // 根据恒速模型估计当前帧初始位姿
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    }

    //清空当前帧的地图点
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    // 设置匹配过程中的搜索半径
    int th;

    if(mSensor==System::STEREO)
        th=7;//双目
    else
        th=15;//单目

    // Step 2：根据上一帧特征点对应地图点进行投影匹配
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);

    // If few matches, uses a wider window search
    // 如果匹配点太少，则扩大搜索半径再来一次
    // 如果就算是这样还是不能够获得足够的匹配点,那么就认为跟踪失败
    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR || mSensor==System::IMU_MONOCULAR);
        Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);

    }

    if(nmatches<20)
    {
        Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
        if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
            return true;
        else
            return false;
    }

    // Optimize frame pose with all matches
    // Step 3：优化当前帧位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    // Step 4：优化位姿后剔除地图点中外点
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                if(i < mCurrentFrame.Nleft){
                    pMP->mbTrackInView = false;
                }
                else{
                    pMP->mbTrackInViewR = false;
                }
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                //累加成功匹配到的地图点数目
                nmatchesMap++;
        }
    }

    if(mbOnlyTracking)
    {
        // 如果在纯定位过程中追踪的地图点非常少,那么这里的 mbVO 标志就会置位
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
        return true;
    else
        // Step 5：匹配超过10个点就认为成功
        return nmatchesMap>=10;
}

/**
 * @brief 对Local Map的MapPoints进行跟踪
 * 
 * 1. 更新局部地图，包括局部关键帧和关键点
 * 2. 对局部MapPoints进行投影匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return true if success
 * 
 * Step 1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints 
 * Step 2：在局部地图中查找与当前帧匹配的MapPoints, 其实也就是对局部地图点进行跟踪
 * Step 3：更新局部所有MapPoints后对位姿再次优化
 * Step 4：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
 * Step 5：决定是否跟踪成功
 */
bool Tracking::TrackLocalMap()
{

    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    mTrackedFr++;

    // Update Local KeyFrames and Local Points
    // Step 1：更新局部关键帧 mvpLocalKeyFrames 和局部地图点 mvpLocalMapPoints
    UpdateLocalMap();

    // Step 2：在局部地图中查找与当前帧匹配的MapPoints, 其实也就是用局部地图点进行跟踪
    SearchLocalPoints();

    // TOO check outliers before PO
    int aux1 = 0, aux2=0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    int inliers;
    if (!mpAtlas->isImuInitialized())
        // Optimize Pose
        // 在这个函数之前，在 Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel 中都有位姿优化，
        // Step 3：更新局部所有MapPoints后对位姿再次优化
        Optimizer::PoseOptimization(&mCurrentFrame);
    else
    {
        if(mCurrentFrame.mnId<=mnLastRelocFrameId+mnFramesToResetIMU)
        {
            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurrentFrame);
        }
        else
        {
            // if(!mbMapUpdated && mState == OK) //  && (mnMatchesInliers>30))
            if(!mbMapUpdated) //  && (mnMatchesInliers>30))
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
            else
            {
                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame); // , !mpLastKeyFrame->GetMap()->GetIniertialBA1());
            }
        }
    }

    aux1 = 0, aux2 = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
        if( mCurrentFrame.mvpMapPoints[i])
        {
            aux1++;
            if(mCurrentFrame.mvbOutlier[i])
                aux2++;
        }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    // Step 4：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                // 由于当前帧的MapPoints可以被当前帧观测到，其被观测统计量加1
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                // 找到该点的帧数mnFound 加 1
                if(!mbOnlyTracking)
                {
                    // 如果该地图点被相机观测数目nObs大于0，匹配内点计数+1
                    // nObs： 被观测到的相机数目，单目+1，双目或RGB-D则+2
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    // 记录当前帧跟踪到的MapPoints，用于统计跟踪效果
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    // Step 5：根据跟踪匹配数目及回环情况决定是否跟踪成功
    // 如果最近刚刚发生了重定位,那么至少成功匹配50个点才认为是成功跟踪
    mpLocalMapper->mnMatchesInliers=mnMatchesInliers;
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    //如果是正常的状态话只要跟踪的地图点大于30个就认为成功了
    if((mnMatchesInliers>10)&&(mState==RECENTLY_LOST))
        return true;


    if (mSensor == System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else if (mSensor == System::IMU_STEREO)
    {
        if(mnMatchesInliers<15)
        {
            return false;
        }
        else
            return true;
    }
    else
    {
        if(mnMatchesInliers<30)
            return false;
        else
            return true;
    }
}

/**
 * @brief 判断当前帧是否需要插入关键帧
 * 
 * Step 1：纯VO模式下不插入关键帧，如果局部地图被闭环检测使用，则不插入关键帧
 * Step 2：如果距离上一次重定位比较近，或者关键帧数目超出最大限制，不插入关键帧
 * Step 3：得到参考关键帧跟踪到的地图点数量
 * Step 4：查询局部地图管理器是否繁忙,也就是当前能否接受新的关键帧
 * Step 5：对于双目或RGBD摄像头，统计可以添加的有效地图点总数 和 跟踪到的地图点数量
 * Step 6：决策是否需要插入关键帧
 * @return true         需要
 * @return false        不需要
 */
bool Tracking::NeedNewKeyFrame()
{
    if(((mSensor == System::IMU_MONOCULAR) || (mSensor == System::IMU_STEREO)) && !mpAtlas->GetCurrentMap()->isImuInitialized())
    {
        if (mSensor == System::IMU_MONOCULAR && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else if (mSensor == System::IMU_STEREO && (mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.25)
            return true;
        else
            return false;
    }

    // Step 1：纯VO模式下不插入关键帧
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    // Step 2：如果局部地图线程被闭环检测使用，则不插入关键帧
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    {
        return false;
    }

    // Return false if IMU is initialazing
    if (mpLocalMapper->IsInitializing())
        return false;
    const int nKFs = mpAtlas->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    // mCurrentFrame.mnId是当前帧的ID
    // mnLastRelocFrameId是最近一次重定位帧的ID
    // mMaxFrames等于图像输入的帧率
    //  Step 3：如果距离上一次重定位比较近，或者关键帧数目超出最大限制，不插入关键帧
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    // Step 4：得到参考关键帧跟踪到的地图点数量
    // UpdateLocalKeyFrames 函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧 

    // 地图点的最小观测次数
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    
    // 参考关键帧地图点中观测的数目>= nMinObs的地图点数目
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    // Step 5：查询局部地图管理器是否繁忙,也就是当前能否接受新的关键帧
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    // Step 6：对于双目或RGBD摄像头，统计有效深度值中跟踪到的地图点和未跟踪成功的点的数量
    int nNonTrackedClose = 0;   //双目，RGB-D中没有跟踪到的三维点
    int nTrackedClose= 0;   //双目，RGB-D中跟踪到的比较近的三维点

    if(mSensor!=System::MONOCULAR && mSensor!=System::IMU_MONOCULAR)
    {
        int N = (mCurrentFrame.Nleft == -1) ? mCurrentFrame.N : mCurrentFrame.Nleft;
        for(int i =0; i<N; i++)
        {
            // 深度值在有效范围内
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                // 是否有对应地图点
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
    }

    // 双目或RGBD情况下：跟踪到的地图点太少 同时 没有跟踪到的三维点太多，可以插入关键帧了
    // 单目时，为false
    bool bNeedToInsertClose;
    bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Step 7：决策是否需要插入关键帧
    // Thresholds
    // Step 7.1：设定比例阈值，当前帧和参考关键帧跟踪到点的比例，比例越大，越倾向于增加关键帧
    float thRefRatio = 0.75f;

    // 关键帧只有一帧，那么插入关键帧的阈值设置的低一点，可以多插入一些
    if(nKFs<2)
        thRefRatio = 0.4f;

    //单目情况下插入关键帧的频率很高
    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    if(mpCamera2) thRefRatio = 0.75f;

    if(mSensor==System::IMU_MONOCULAR)
    {
        if(mnMatchesInliers>350) // Points tracked from the local map
            thRefRatio = 0.75f;
        else
            thRefRatio = 0.90f;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    // Step 7.2：很长时间没有插入关键帧，可以插入
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    // Step 7.3：满足插入关键帧的最小间隔并且localMapper处于空闲状态，可以插入
    const bool c1b = ((mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames) && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    // Step 7.4：在双目，RGB-D的情况下当前帧跟踪到的点比参考关键帧的0.25倍还少，或者满足bNeedToInsertClose
    const bool c1c = mSensor!=System::MONOCULAR &&  
            mSensor!=System::IMU_MONOCULAR && 
            mSensor!=System::IMU_STEREO && 
            (mnMatchesInliers<nRefMatches*0.25 || 
            bNeedToInsertClose) ;   //需要插入
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    // Step 7.5：和参考帧相比当前跟踪到的点太少 或者满足bNeedToInsertClose；同时跟踪到的内点还不能太少
    const bool c2 = (((mnMatchesInliers<nRefMatches*thRefRatio || bNeedToInsertClose)) && mnMatchesInliers>15);

    // Temporal condition for Inertial cases
    bool c3 = false;
    if(mpLastKeyFrame)
    {
        if (mSensor==System::IMU_MONOCULAR)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
        else if (mSensor==System::IMU_STEREO)
        {
            if ((mCurrentFrame.mTimeStamp-mpLastKeyFrame->mTimeStamp)>=0.5)
                c3 = true;
        }
    }

    bool c4 = false;
    if ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR))) // MODIFICATION_2, originally ((((mnMatchesInliers<75) && (mnMatchesInliers>15)) || mState==RECENTLY_LOST) && ((mSensor == System::IMU_MONOCULAR)))
        c4=true;
    else
        c4=false;

    if(((c1a||c1b||c1c) && c2)||c3 ||c4)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        // Step 7.6：local mapping空闲时可以直接插入，不空闲的时候要根据情况插入
        if(bLocalMappingIdle)
        {
            //可以插入关键帧
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR  && mSensor!=System::IMU_MONOCULAR)
            {
                // 队列里不能阻塞太多关键帧
                // tracking插入关键帧不是直接插入，而且先插入到mlNewKeyFrames中，
                // 然后localmapper再逐个pop出来插入到mspKeyFrames
                if(mpLocalMapper->KeyframesInQueue()<3)
                    //队列中的关键帧数目不是很多,可以插入
                    return true;
                else
                    //队列中缓冲的关键帧数目太多,暂时不能插入
                    return false;
            }
            else
                return false;
        }
    }
    else
        //对于单目情况,就直接无法插入关键帧了
        //? 为什么这里对单目情况的处理不一样?
        //回答：可能是单目关键帧相对比较密集
        return false;
}

/**
 * @brief 创建新的关键帧
 * 对于非单目的情况，同时创建新的MapPoints
 * 
 * Step 1：将当前帧构造成关键帧
 * Step 2：将当前关键帧设置为当前帧的参考关键帧
 * Step 3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
 */
void Tracking::CreateNewKeyFrame()
{
    // 如果不能保持局部建图器开启的状态,就无法顺利插入关键帧
    if(mpLocalMapper->IsInitializing())
        return;

    // 如果不能保持局部建图器开启的状态,就无法顺利插入关键帧
    if(!mpLocalMapper->SetNotStop(true))
        return;

    // Step 1：将当前帧构造成关键帧
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpAtlas->GetCurrentMap(),mpKeyFrameDB);

    if(mpAtlas->isImuInitialized())
        pKF->bImu = true;

    pKF->SetNewBias(mCurrentFrame.mImuBias);

    // Step 2：将当前关键帧设置为当前帧的参考关键帧
    // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mpLastKeyFrame)
    {
        pKF->mPrevKF = mpLastKeyFrame;
        mpLastKeyFrame->mNextKF = pKF;
    }
    else
        Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

    // Reset preintegration from last KF (Create new object)
    if (mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO)
    {
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
    }

    if(mSensor!=System::MONOCULAR && mSensor != System::IMU_MONOCULAR) // TODO check if incluide imu_stereo
    {
        // 根据Tcw计算mRcw、mtcw和mRwc、mOw
        mCurrentFrame.UpdatePoseMatrices();

        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        // Step 3.1：得到当前帧有深度值的特征点（不一定是地图点）
        int maxPoint = 100;
        if(mSensor == System::IMU_STEREO)
            maxPoint = 100;

        vector<pair<float,int> > vDepthIdx;
        int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                // 第一个元素是深度,第二个元素是对应的特征点的id
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            // Step 3.2：按照深度从小到大排序
            sort(vDepthIdx.begin(),vDepthIdx.end());
            // Step 3.3：从中找出不是地图点的生成临时地图点 
            //处理的近点的个数
            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;
                // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }
                // 如果需要就新建地图点，这里的地图点不是临时的，是全局地图中新建地图点，用于跟踪
                if(bCreateNew)
                {
                    cv::Mat x3D;

                    if(mCurrentFrame.Nleft == -1){
                        x3D = mCurrentFrame.UnprojectStereo(i);
                    }
                    else{
                        x3D = mCurrentFrame.UnprojectStereoFishEye(i);
                    }

                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
                    // 这些添加属性的操作是每次创建MapPoint后都要做的
                    pNewMP->AddObservation(pKF,i);

                    //Check if it is a stereo observation in order to not
                    //duplicate mappoints
                    if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
                        mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
                        pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                        pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
                    }

                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    // 因为从近到远排序，记录其中不需要创建地图点的个数
                    nPoints++; // TODO check ???
                }
                // Step 3.4：停止新建地图点必须同时满足以下条件：
                // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
                // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
                if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
                {
                    break;
                }
            }

            Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);

        }
    }

    // Step 4：插入关键帧
    // 关键帧插入到列表 mlNewKeyFrames中，等待local mapping线程临幸
    mpLocalMapper->InsertKeyFrame(pKF);

    // 插入好了，允许局部建图停止
    mpLocalMapper->SetNotStop(false);

    // 当前帧成为新的关键帧，更新
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
    //cout  << "end creating new KF" << endl;
}

/**
 * @brief 对 Local MapPoints 进行跟踪
 * 
 * 在局部地图中查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配
 */
void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    // Step 1：遍历当前帧的mvpMapPoints，标记这些MapPoints不参与之后的搜索
    // 因为当前的mvpMapPoints一定在当前帧的视野中
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                // 更新能观测到该点的帧数加1(被当前帧看到了)
                pMP->IncreaseVisible();
                // 标记该点被当前帧观测到
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                // 标记该点将来不被投影，因为已经匹配过(指的是使用恒速运动模型进行投影)
                pMP->mbTrackInView = false;
                pMP->mbTrackInViewR = false;
            }
        }
    }//遍历当前帧中所有的地图点

    //准备进行投影匹配的点的数目
    int nToMatch=0;

    // Project points in frame and check its visibility
    // Step 2：将所有 局部地图点 投影到当前帧，判断是否在视野范围内
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        // 已经被当前帧观测到的MapPoint不再需要判断是否能被当前帧观测到
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        
        //跳过局部地图中的坏点
        if(pMP->isBad())
            continue;
        
        // Project (this fills MapPoint variables for matching)
        // 判断LocalMapPoints中的点是否在在视野内
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            // 观测到该点的帧数加1，该MapPoint在某些帧的视野范围内
            pMP->IncreaseVisible();

            // 只有在视野范围内的MapPoints才参与之后的投影匹配
            nToMatch++;
        }
        if(pMP->mbTrackInView)
        {
            mCurrentFrame.mmProjectPoints[pMP->mnId] = cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY);
        }
    }

    // Step 3：如果需要进行投影匹配的点的数目大于0，就进行投影匹配
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)   //RGBD相机输入的时候,搜索的阈值会变得稍微大一些
            th=3;
        if(mpAtlas->isImuInitialized())
        {
            if(mpAtlas->GetCurrentMap()->GetIniertialBA2())
                th=2;
            else
                th=3;
        }
        else if(!mpAtlas->isImuInitialized() && (mSensor==System::IMU_MONOCULAR || mSensor==System::IMU_STEREO))
        {
            th=10;
        }

        // If the camera has been relocalised recently, perform a coarser search
        // 如果不久前进行过重定位，那么进行一个更加宽泛的搜索，阈值需要增大
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;

        if(mState==LOST || mState==RECENTLY_LOST) // Lost for less than 1 second
            th=15; // 15

        // 对视野范围内的MapPoints通过投影进行特征点匹配
        int matches = matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th, mpLocalMapper->mbFarPoints, mpLocalMapper->mThFarPoints);
    }
}

/**
 * @brief 更新LocalMap
 *
 * 局部地图包括： 
 * 1、K1个关键帧、K2个临近关键帧和参考关键帧
 * 2、由这些关键帧观测到的MapPoints
 */
void Tracking::UpdateLocalMap()
{
    // This is for visualization
    // 设置参考地图点用于绘图显示局部地图点（红色）
    mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    // 用共视图来更新局部关键帧和局部地图点
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

/*
 * @brief 更新局部关键点,就是先把局部地图清空，然后将局部关键帧的有效地图点添加到局部地图中
 */
void Tracking::UpdateLocalPoints()
{
    // Step 1：清空局部MapPoints
    mvpLocalMapPoints.clear();

    int count_pts = 0;

    // Step 2：遍历局部关键帧 mvpLocalKeyFrames
    for(vector<KeyFrame*>::const_reverse_iterator itKF=mvpLocalKeyFrames.rbegin(), itEndKF=mvpLocalKeyFrames.rend(); itKF!=itEndKF; ++itKF)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        // step 2：将局部关键帧的MapPoints添加到mvpLocalMapPoints
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {

            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            // 用该地图点的成员变量mnTrackReferenceForFrame 记录当前帧的id
            // 表示它已经是当前帧的局部地图点了，可以防止重复添加局部地图点
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                count_pts++;
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

/**
 * @brief 更新局部关键帧
 * 方法是遍历当前帧的MapPoints，将观测到这些MapPoints的关键帧和相邻的关键帧及其父子关键帧，作为mvpLocalKeyFrames
 * Step 1：遍历当前帧的MapPoints，记录所有能观测到当前帧MapPoints的关键帧 
 * Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有三个策略
 * Step 2.1 策略1：能观测到当前帧MapPoints的关键帧作为局部关键帧 （将邻居拉拢入伙）
 * Step 2.2 策略2：遍历策略1得到的局部关键帧里共视程度很高的关键帧，将他们的家人和邻居作为局部关键帧
 * Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
 */
void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    // Step 1：遍历当前帧的MapPoints，记录所有能观测到当前帧MapPoints的关键帧
    map<KeyFrame*,int> keyframeCounter;
    if(!mpAtlas->isImuInitialized() || (mCurrentFrame.mnId<mnLastRelocFrameId+2))
    {
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    // 观测到该MapPoint的KF和该MapPoint在KF中的索引
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    // 这里的操作非常精彩！
                    // map[key] = value，当要插入的键存在时，会覆盖键对应的原来的值。如果键不存在，则添加一组键值对
                    // it->first 是地图点看到的关键帧，同一个关键帧看到的地图点会累加到该关键帧计数
                    // 所以最后keyframeCounter 第一个参数表示某个关键帧，第2个参数表示该关键帧看到了多少当前帧(mCurrentFrame)的地图点，也就是共视程度
                    keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }
    else
    {
        for(int i=0; i<mLastFrame.N; i++)
        {
            // Using lastframe since current frame has not matches yet
            if(mLastFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mLastFrame.mvpMapPoints[i];
                if(!pMP)
                    continue;
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,tuple<int,int>> observations = pMP->GetObservations();
                    for(map<KeyFrame*,tuple<int,int>>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    // MODIFICATION
                    mLastFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
    }


    // 存储具有最多观测次数（max）的关键帧
    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    // Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有三个策略
    // 先清空局部关键帧
    mvpLocalKeyFrames.clear();

    // 先申请3倍内存，不够后面再加
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    // Step 2.1 策略1：能观测到当前帧MapPoints的关键帧作为局部关键帧 （将邻居拉拢入伙）
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        // 如果设定为要删除的，跳过
        if(pKF->isBad())
            continue;

        //寻找具有最大观测数目的关键帧
        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        // 添加到局部关键帧的列表里
        mvpLocalKeyFrames.push_back(pKF);
    
        // 用该关键帧的成员变量mnTrackReferenceForFrame 记录当前帧的id
        // 表示它已经是当前帧的局部关键帧了，可以防止重复添加局部关键帧
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    // Step 2.2 策略2：遍历策略1得到的局部关键帧里共视程度很高的（前10个）关键帧，将他们的家人和邻居作为局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        // 处理的局部关键帧不超过80帧
        if(mvpLocalKeyFrames.size()>80) // 80
            break;

        KeyFrame* pKF = *itKF;

        // 策略2.1:最佳共视的10帧; 如果共视帧不足10帧,那么就返回所有具有共视关系的关键帧（将邻居的邻居拉拢入伙）
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        // vNeighs 是按照共视程度从大到小排列
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                // mnTrackReferenceForFrame防止重复添加局部关键帧
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 策略2.2:将自己的子关键帧作为局部关键帧（将邻居的子孙们拉拢入伙）
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 策略2.3:自己的父关键帧（将邻居的父母们拉拢入伙）
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            // mnTrackReferenceForFrame防止重复添加局部关键帧
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }
    }

    // Add 10 last temporal KFs (mainly for IMU)
    if((mSensor == System::IMU_MONOCULAR || mSensor == System::IMU_STEREO) &&mvpLocalKeyFrames.size()<80)
    {
        //cout << "CurrentKF: " << mCurrentFrame.mnId << endl;
        KeyFrame* tempKeyFrame = mCurrentFrame.mpLastKeyFrame;

        const int Nd = 20;
        for(int i=0; i<Nd; i++){
            if (!tempKeyFrame)
                break;
            //cout << "tempKF: " << tempKeyFrame << endl;
            if(tempKeyFrame->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(tempKeyFrame);
                tempKeyFrame->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                tempKeyFrame=tempKeyFrame->mPrevKF;
            }
        }
    }

    // Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

/**
 * @details 重定位过程
 * @return true 
 * @return false 
 * 
 * Step 1：计算当前帧特征点的Bow
 * Step 2：找到与当前帧相似的候选关键帧
 * Step 3：通过BoW进行匹配
 * Step 4：通过EPnP算法估计姿态
 * Step 5：通过PoseOptimization对姿态进行优化求解
 * Step 6：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
 */
bool Tracking::Relocalization()
{
    Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
    // Compute Bag of Words Vector
    // Step 1： 计算当前帧特征点的Bow映射
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    // Step 2：找到与当前帧相似的候选关键帧组
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame, mpAtlas->GetCurrentMap());

    // 如果没有候选关键帧，则退出
    if(vpCandidateKFs.empty()) {
        Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    //每个关键帧的解算器
    vector<MLPnPsolver*> vpMLPnPsolvers;
    vpMLPnPsolvers.resize(nKFs);

    //每个关键帧和当前帧中特征点的匹配关系
    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    //放弃某个关键帧的标记
    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    //有效的候选关键帧数目
    int nCandidates=0;

    // Step 3：遍历所有的候选关键帧，通过BoW进行快速匹配，用匹配结果初始化PnP Solver
    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            // 当前帧和候选关键帧用BoW进行快速匹配，匹配结果记录在vvpMapPointMatches，nmatches表示匹配的数目
            
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            // 如果和当前帧的匹配数小于15,那么只能放弃这个关键帧
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                // 如果匹配数目够用，用匹配结果初始化EPnPsolver
                MLPnPsolver* pSolver = new MLPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(
                    0.99,   //用于计算RANSAC迭代次数理论值的概率
                    10,     //最小内点数, 但是要注意在程序中实际上是min(给定最小内点数,最小集,内点数理论值),不一定使用这个
                    300,    //最大迭代次数
                    6,      //最小集(求解这个问题在一次采样中所需要采样的最少的点的个数,对于Sim3是3,EPnP是4),参与到最小内点数的确定过程中
                    0.5,    //这个是表示(最小内点数/样本总数);实际上的RANSAC正常退出的时候所需要的最小内点数其实是根据这个量来计算得到的
                    5.991);  //This solver needs at least 6 points // 自由度为2的卡方检验的阈值,程序中还会根据特征点所在的图层对这个阈值进行缩放
                vpMLPnPsolvers[i] = pSolver;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    // 这里的 P4P RANSAC是Epnp，每次迭代需要4个点
    // 是否已经找到相匹配的关键帧的标志
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    // Step 4: 通过一系列操作,直到找到能够匹配上的关键帧
    while(nCandidates>0 && !bMatch)
    {
        //遍历当前所有的候选关键帧
        for(int i=0; i<nKFs; i++)
        {
            // 忽略放弃的
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            //内点标记
            vector<bool> vbInliers;
            int nInliers;

            // 表示RANSAC已经没有更多的迭代次数可用 -- 也就是说数据不够好，RANSAC也已经尽力了。。。
            bool bNoMore;

            // Step 4.1：通过EPnP算法估计姿态，迭代5次
            MLPnPsolver* pSolver = vpMLPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            // bNoMore 为true 表示已经超过了RANSAC最大迭代次数，就放弃当前关键帧
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                //  Step 4.2：如果EPnP 计算出了位姿，对内点进行BA优化
                Tcw.copyTo(mCurrentFrame.mTcw);

                // EPnP 里RANSAC后的内点的集合
                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                //遍历所有内点
                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                // 只优化位姿,不优化地图点的坐标，返回的是内点的数量
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                // 如果优化之后的内点数目不多，跳过了当前候选关键帧,但是却没有放弃当前帧的重定位
                if(nGood<10)
                    continue;

                // 删除外点对应的地图点
                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                // Step 4.3：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
                // 前面的匹配关系是用词袋匹配过程得到的
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(
                        mCurrentFrame,      //当前帧
                        vpCandidateKFs[i],  //关键帧
                        sFound, //已经找到的地图点集合，不会用于PNP
                        10,     //窗口阈值，会乘以金字塔尺度
                        100);   //匹配的ORB描述子距离应该小于这个阈值

                    // 如果通过投影过程新增了比较多的匹配特征点对
                    if(nadditional+nGood>=50)
                    {
                        // 根据投影匹配的结果，再次采用3D-2D pnp BA优化位姿
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        // Step 4.4：如果BA后内点数还是比较少(<50)但是还不至于太少(>30)，可以挽救一下, 最后垂死挣扎 
                        // 重新执行上一步 4.3的过程，只不过使用更小的搜索窗口
                        // 这里的位姿已经使用了更多的点进行了优化,应该更准，所以使用更小的窗口搜索
                        if(nGood>30 && nGood<50)
                        {
                            // 用更小窗口、更严格的描述子阈值，重新进行投影搜索匹配
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(
                                mCurrentFrame,      //当前帧
                                vpCandidateKFs[i],  //候选的关键帧
                                sFound,  //已经找到的地图点，不会用于PNP
                                3,      //新的窗口阈值，会乘以金字塔尺度
                                64);    //匹配的ORB描述子距离应该小于这个阈值

                            // Final optimization
                            // 如果成功挽救回来，匹配数目达到要求，最后BA优化一下
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                                //更新地图点
                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                            //如果还是不能够满足就放弃了
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                // 如果对于当前的候选关键帧已经有足够的内点(50个)了,那么就认为重定位成功
                if(nGood>=50)
                {
                    bMatch = true;
                    // 只要有一个候选关键帧重定位成功，就退出循环，不考虑其他候选关键帧了
                    break;
                }
            }
        }//一直运行,知道已经没有足够的关键帧,或者是已经有成功匹配上的关键帧
    }

    // 折腾了这么久还是没有匹配上，重定位失败
    if(!bMatch)
    {
        return false;
    }
    else
    {
        // 如果匹配上了,说明当前帧重定位成功了(当前帧已经有了自己的位姿)
        // 记录成功重定位帧的id，防止短时间多次重定位
        mnLastRelocFrameId = mCurrentFrame.mnId;
        cout << "Relocalized!!" << endl;
        return true;
    }

}

//整个追踪线程执行复位操作
void Tracking::Reset(bool bLocMap)
{
    Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);

    //基本上是挨个请求各个线程终止
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }


    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestReset();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clear();
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearAtlas();
    mpAtlas->CreateNewMap();
    if (mSensor==System::IMU_STEREO || mSensor == System::IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();
    mnInitialFrameId = 0;

    //然后复位各种变量
    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }
    mbSetInit=false;

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
    mCurrentFrame = Frame();
    mnLastRelocFrameId = 0;
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

void Tracking::ResetActiveMap(bool bLocMap)
{
    Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    Map* pMap = mpAtlas->GetCurrentMap();

    if (!bLocMap)
    {
        Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
        mpLocalMapper->RequestResetActiveMap(pMap);
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
    }

    // Reset Loop Closing
    Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
    mpLoopClosing->RequestResetActiveMap(pMap);
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear BoW Database
    Verbose::PrintMess("Reseting Database", Verbose::VERBOSITY_NORMAL);
    mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
    Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

    // Clear Map (this erase MapPoints and KeyFrames)
    mpAtlas->clearMap();


    //KeyFrame::nNextId = mpAtlas->GetLastInitKFid();
    //Frame::nNextId = mnLastInitFrameId;
    mnLastInitFrameId = Frame::nNextId;
    mnLastRelocFrameId = mnLastInitFrameId;
    mState = NO_IMAGES_YET; //NOT_INITIALIZED;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    list<bool> lbLost;
    // lbLost.reserve(mlbLost.size());
    unsigned int index = mnFirstFrameId;
    cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
    for(Map* pMap : mpAtlas->GetAllMaps())
    {
        if(pMap->GetAllKeyFrames().size() > 0)
        {
            if(index > pMap->GetLowerKFID())
                index = pMap->GetLowerKFID();
        }
    }

    //cout << "First Frame id: " << index << endl;
    int num_lost = 0;
    cout << "mnInitialFrameId = " << mnInitialFrameId << endl;

    for(list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++)
    {
        if(index < mnInitialFrameId)
            lbLost.push_back(*ilbL);
        else
        {
            lbLost.push_back(true);
            num_lost += 1;
        }

        index++;
    }
    cout << num_lost << " Frames set to lost" << endl;

    mlbLost = lbLost;

    mnInitialFrameId = mCurrentFrame.mnId;
    mnLastRelocFrameId = mCurrentFrame.mnId;

    mCurrentFrame = Frame();
    mLastFrame = Frame();
    mpReferenceKF = static_cast<KeyFrame*>(NULL);
    mpLastKeyFrame = static_cast<KeyFrame*>(NULL);
    mvIniMatches.clear();

    if(mpViewer)
        mpViewer->Release();

    Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
}

vector<MapPoint*> Tracking::GetLocalMapMPS()
{
    return mvpLocalMapPoints;
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    //     |fx  0   cx|
    // K = |0   fy  cy|
    //     |0   0   1 |
    //构造相机内参矩阵
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    // 图像矫正系数
    // [k1 k2 p1 p2 k3]
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];

    //有些相机的畸变系数中会没有k3项
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    // 双目摄像头baseline * fx 50
    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame)
{
    Map * pMap = pCurrentKeyFrame->GetMap();
    unsigned int index = mnFirstFrameId;
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mlpReferences.begin();
    list<bool>::iterator lbL = mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mlRelativeFramePoses.begin(),lend=mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        while(pKF->isBad())
        {
            pKF = pKF->GetParent();
        }

        if(pKF->GetMap() == pMap)
        {
            (*lit).rowRange(0,3).col(3)=(*lit).rowRange(0,3).col(3)*s;
        }
    }

    mLastBias = b;

    mpLastKeyFrame = pCurrentKeyFrame;

    mLastFrame.SetNewBias(mLastBias);
    mCurrentFrame.SetNewBias(mLastBias);

    cv::Mat Gz = (cv::Mat_<float>(3,1) << 0, 0, -IMU::GRAVITY_VALUE);

    cv::Mat twb1;
    cv::Mat Rwb1;
    cv::Mat Vwb1;
    float t12;

    while(!mCurrentFrame.imuIsPreintegrated())
    {
        usleep(500);
    }


    if(mLastFrame.mnId == mLastFrame.mpLastKeyFrame->mnFrameId)
    {
        mLastFrame.SetImuPoseVelocity(mLastFrame.mpLastKeyFrame->GetImuRotation(),
                                      mLastFrame.mpLastKeyFrame->GetImuPosition(),
                                      mLastFrame.mpLastKeyFrame->GetVelocity());
    }
    else
    {
        twb1 = mLastFrame.mpLastKeyFrame->GetImuPosition();
        Rwb1 = mLastFrame.mpLastKeyFrame->GetImuRotation();
        Vwb1 = mLastFrame.mpLastKeyFrame->GetVelocity();
        t12 = mLastFrame.mpImuPreintegrated->dT;

        mLastFrame.SetImuPoseVelocity(Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mLastFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    if (mCurrentFrame.mpImuPreintegrated)
    {
        twb1 = mCurrentFrame.mpLastKeyFrame->GetImuPosition();
        Rwb1 = mCurrentFrame.mpLastKeyFrame->GetImuRotation();
        Vwb1 = mCurrentFrame.mpLastKeyFrame->GetVelocity();
        t12 = mCurrentFrame.mpImuPreintegrated->dT;

        mCurrentFrame.SetImuPoseVelocity(Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaRotation(),
                                      twb1 + Vwb1*t12 + 0.5f*t12*t12*Gz+ Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaPosition(),
                                      Vwb1 + Gz*t12 + Rwb1*mCurrentFrame.mpImuPreintegrated->GetUpdatedDeltaVelocity());
    }

    mnFirstImuFrameId = mCurrentFrame.mnId;
}


cv::Mat Tracking::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = Converter::tocvSkewMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}


void Tracking::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    const vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpLastKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpLastKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpLastKeyFrame->GetCameraCenter();

    const float &fx1 = mpLastKeyFrame->fx;
    const float &fy1 = mpLastKeyFrame->fy;
    const float &cx1 = mpLastKeyFrame->cx;
    const float &cy1 = mpLastKeyFrame->cy;
    const float &invfx1 = mpLastKeyFrame->invfx;
    const float &invfy1 = mpLastKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpLastKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF2 = vpKFs[i];
        if(pKF2==mpLastKeyFrame)
            continue;

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if((mSensor!=System::MONOCULAR)||(mSensor!=System::IMU_MONOCULAR))
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpLastKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpLastKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpLastKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpLastKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpLastKeyFrame->mb/2,mpLastKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpLastKeyFrame->UnprojectStereo(idx1);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpLastKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpLastKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpLastKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpLastKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpLastKeyFrame,mpAtlas->GetCurrentMap());

            pMP->AddObservation(mpLastKeyFrame,idx1);
            pMP->AddObservation(pKF2,idx2);

            mpLastKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            nnew++;
        }
    }
    TrackReferenceKeyFrame();
}

void Tracking::NewDataset()
{
    mnNumDataset++;
}

int Tracking::GetNumberDataset()
{
    return mnNumDataset;
}

int Tracking::GetMatchesInliers()
{
    return mnMatchesInliers;
}

} //namespace ORB_SLAM
