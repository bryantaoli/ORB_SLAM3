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

/**
 * @file System.h
 * @brief 这个头文件定义了ORB-SLAM2的主线程（或者称之为系统）结构，其他的各个模块都是由这里开始被调用的。
 * 
 */
#ifndef SYSTEM_H
#define SYSTEM_H

//#define SAVE_TIMES
//一些公用库的支持，字符串操作，多线程操作，以及opencv库等
#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

//下面则是本ORB-SLAM3系统中的其他模块
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"


namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};

//要用到的其他类的前视声明
class Viewer;
class FrameDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;

//本类的定义
class System
{
public:
    // Input sensor
    //这个枚举类型用于 表示本系统所使用的传感器类型
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4
    };

    // File type
    enum eFileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    //构造函数，用来初始化整个系统。
    System(const string &strVocFile,   //指定ORB字典文件的路径
           const string &strSettingsFile,   //指定配置文件的路径
           const eSensor sensor,  //指定所使用的传感器类型
           const bool bUseViewer = true,   //指定是否使用可视化界面
           const int initFr = 0, 
           const string &strSequence = std::string(), 
           const string &strLoadingFile = std::string());

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    // NOTE 注意这里英文注释的说法，双目图像有同步和校准的概念。
    cv::Mat TrackStereo(const cv::Mat &imLeft,  //左目图像
                        const cv::Mat &imRight,  //右目图像
                        const double &timestamp,  //时间戳
                        const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(),  //IMU测量
                        string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    // NOTE 而在这里对RGBD图像的说法则是“配准”
    cv::Mat TrackRGBD(const cv::Mat &im,  //彩色图像
                      const cv::Mat &depthmap,  //深度图像
                      const double &timestamp,  //时间戳
                      string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im,  //图像
                           const double &timestamp,  //时间戳
                           const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(),  //IMU测量
                           string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    //使能定位模式，此时仅有运动追踪部分在工作，局部建图功能则不工作
    void ActivateLocalizationMode();

    // This resumes local mapping thread and performs SLAM again.
    //反之同上
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    // 获取从上次调用本函数后是否发生了比较大的地图变化
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    // 复位 系统
    void Reset();
    
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    //关闭系统，这将会关闭所有线程并且丢失曾经的各种数据
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    // 以TUM格式保存相机的运动轨迹，这个函数将会在Shutdown函数中被首先调用
    void SaveTrajectoryTUM(const string &filename); //指定文件名

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    // 以TUM格式保存关键帧位姿。
    void SaveKeyFrameTrajectoryTUM(const string &filename); //指定文件名

    // 以EuRoC格式保存相机的运动轨迹
    void SaveTrajectoryEuRoC(const string &filename);

    // 以EuRoC格式保存关键帧运动轨迹
    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    // Save data used for initialization debug
    // 保存用于初始化调试的数据
    void SaveDebugData(const int &iniIdx);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    // 以KITTI格式保存相机的运行轨迹
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    //获取最近的运动追踪状态、地图点追踪状态、特征点追踪状态（）
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    //void SaveAtlas(int type);

private:

    //bool LoadAtlas(string filename, int type);

    //string CalculateCheckSum(string filename, int type);

    // Input sensor
    // 传感器类型
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    // 一个指针指向ORB字典
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    // 关键帧数据库的指针，这个数据库用于重定位和回环检测
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap; //指向地图（数据库）的指针
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    // 追踪器，除了进行运动追踪外还要负责创建关键帧、创建新地图点和进行重定位的工作。详细信息还得看相关文件
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    //局部建图器。局部BA由它进行。
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    // 回环检测器，它会执行位姿图优化并且开一个新的线程进行全局BA
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    // 查看器，可视化 界面
    Viewer* mpViewer;

    //帧绘制器
    FrameDrawer* mpFrameDrawer;

    //地图绘制器
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    //系统除了在主进程中进行运动追踪工作外，会创建局部建图线程、回环检测线程和查看器线程。
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    //复位标志，注意这里目前还不清楚为什么要定义为std::mutex类型
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    //模式改变标志
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    // 追踪状态标志，注意前三个的类型和上面的函数类型相互对应
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
