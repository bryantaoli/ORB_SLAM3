/**
 * @file Sysytem.cc
 * @brief 系统
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

//主进程的实现文件
//包含了一些自建库和共享库
#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;
//系统的构造函数，将会启动其他的线程
System::System(const string &strVocFile, //词典文件路径
              const string &strSettingsFile, //配置文件路径
              const eSensor sensor, //传感器类型
               const bool bUseViewer,  //是否使用可视化界面
               const int initFr, //是否是初值帧
               const string &strSequence, 
               const string &strLoadingFile):
    mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbResetActiveMap(false),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";
    // 输出当前传感器类型
    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(),  //将配置文件名转换成为字符串
                                cv::FileStorage::READ);  //只读
    //如果打开失败，就输出调试信息
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       //然后退出
       exit(-1);
    }

    bool loadedAtlas = false;

    //----
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    //建立一个新的ORB字典
    mpVocabulary = new ORBVocabulary();
    //获取字典加载状态
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    //如果加载失败，就输出调试信息
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        //然后退出
        exit(-1);
    }
    //否则则说明加载成功
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database 创建关键帧数据库
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Atlas
    //mpMap = new Map();
    //创建新地图集
    mpAtlas = new Atlas(0);
    //----

    /*if(strLoadingFile.empty())
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Atlas
        //mpMap = new Map();
        mpAtlas = new Atlas(0);
    }
    else
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        cout << "Load File" << endl;

        // Load the file with an earlier session
        //clock_t start = clock();
        bool isRead = LoadAtlas(strLoadingFile,BINARY_FILE);

        if(!isRead)
        {
            cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
            exit(-1);
        }
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
        mpAtlas->SetORBVocabulary(mpVocabulary);
        mpAtlas->PostLoad();
        //cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " << mpKeyFrameDatabase->mnNumWords << endl;

        loadedAtlas = true;

        mpAtlas->CreateNewMap();

        //clock_t timeElapsed = clock() - start;
        //unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
        //cout << "Binary file read in " << msElapsed << " ms" << endl;

        //usleep(10*1000*1000);
    }*/


    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR)
        mpAtlas->SetInertialSensor();

    //Create Drawers. These are used by the Viewer
    //这里的帧绘制器和地图绘制器将会被可视化的Viewer所使用
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    //在本主进程中初始化追踪线程
    cout << "Seq. Name: " << strSequence << endl;
    mpTracker = new Tracking(this, 
                            mpVocabulary, //字典
                            mpFrameDrawer, //帧绘制器
                            mpMapDrawer,//地图绘制器
                            mpAtlas, //地图
                            mpKeyFrameDatabase, //关键帧地图
                            strSettingsFile, //设置文件路径
                            mSensor, //传感器类型iomanip
                            strSequence);

    //初始化局部建图线程并运行
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, 
                                    mpAtlas, 
                                    mSensor==MONOCULAR || mSensor==IMU_MONOCULAR, mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO, 
                                    strSequence);
    //运行这个局部建图线程
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, //这个线程会调用的函数
                                 mpLocalMapper); //这个调用函数的参数
    mpLocalMapper->mInitFr = initFr;
    mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if(mpLocalMapper->mThFarPoints!=0)
    {
        cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    //Initialize the Loop Closing thread and launch
    // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR); // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        //如果指定了，程序的运行过程中需要运行可视化部分
        //新建viewer
        mpViewer = new Viewer(this, 
                            mpFrameDrawer, //帧绘制器
                            mpMapDrawer, //地图绘制器
                            mpTracker, //追踪器
                            strSettingsFile); //配置文件的访问路径
        //新建viewer线程
        mptViewer = new thread(&Viewer::Run, mpViewer);
        //给运动追踪器设置其查看器
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
    }

    //Set pointers between threads
    //设置进程间的指针
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

}

//双目输入时的追踪器接口
cv::Mat System::TrackStereo(const cv::Mat &imLeft,  //左侧图像
                            const cv::Mat &imRight,  //右侧图像
                            const double &timestamp,  //时间戳
                            const vector<IMU::Point>& vImuMeas,  //IMU测量
                            string filename) //文件名
{
    //检查输入数据类型是否合法
    if(mSensor!=STEREO && mSensor!=IMU_STEREO)
    {
        //不合法那就退出
        cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << endl;
        exit(-1);
    }   
    //检查是否有运行模式的改变
    // Check mode change
    {
        //锁住这个变量，防止其他的线程对它的更改 
        unique_lock<mutex> lock(mMutexMode);
        //如果激活定位模式
        if(mbActivateLocalizationMode)
        {
            //调用局部建图器的请求停止函数
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }
            //运行到这里的时候，局部建图部分就真正地停止了
            //告知追踪器，现在 只有追踪工作
            mpTracker->InformOnlyTracking(true); // 定位时，只跟踪
            //同时清除定位标记
            mbActivateLocalizationMode = false; // 防止重复执行
        }//如果激活定位模式
        if(mbDeactivateLocalizationMode)
        {
            //如果取消定位模式
        	//告知追踪器，现在地图构建部分也要开始工作了
            mpTracker->InformOnlyTracking(false);
            //局部建图器要开始工作
            mpLocalMapper->Release();
            //清楚标志
            mbDeactivateLocalizationMode = false;// 防止重复执行
        }//如果取消定位模式
    }//检查是否有模式的改变

    // Check reset，检查是否有复位的操作
    {
        //上锁
        unique_lock<mutex> lock(mMutexReset);
        //是否有复位请求？
        if(mbReset)
        {
            //有，追踪器复位
            mpTracker->Reset();
            cout << "Reset stereo..." << endl;
            //清除标志
            mbReset = false;
            mbResetActiveMap = false;
        }//是否有复位请求
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    // std::cout << "start GrabImageStereo" << std::endl;
    //用矩阵Tcw来保存估计的相机 位姿，运动追踪器的GrabImageStereo函数才是真正进行运动估计的函数
    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp,filename);

    // std::cout << "out grabber" << std::endl;
    //给运动追踪状态上锁
    unique_lock<mutex> lock2(mMutexState);
    //获取运动追踪状态
    mTrackingState = mpTracker->mState;
    //获取当前帧追踪到的地图点向量指针
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    //获取当前帧追踪到的关键帧特征点向量的指针
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    //返回获得的相机运动估计
    return Tcw;
}

//当输入图像 为RGBD时进行的追踪
cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, string filename)
{
    //判断输入数据类型是否合法
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    //检查模式改变
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    //检查是否有复位请求
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    //获得相机位姿的估计
    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

//同理，输入为单目图像时的追踪器接口
cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp, const vector<IMU::Point>& vImuMeas, string filename)
{
    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << endl;
        exit(-1);
    }

    // Check mode change
    {
        // 独占锁，主要是为了mbActivateLocalizationMode和mbDeactivateLocalizationMode不会发生混乱
        unique_lock<mutex> lock(mMutexMode);
        // mbActivateLocalizationMode为true会关闭局部地图线程
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }
            // 局部地图关闭以后，只进行追踪的线程，只计算相机的位姿，没有对局部地图进行更新
            // 设置mbOnlyTracking为真
            mpTracker->InformOnlyTracking(true);
            // 关闭线程可以使得别的线程得到更多的资源
            mbActivateLocalizationMode = false;
        }
        // 如果mbDeactivateLocalizationMode是true，局部地图线程就被释放, 关键帧从局部地图中删除.
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            cout << "SYSTEM-> Reseting active map in monocular case" << endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);
    //获取相机位姿的估计结果
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp,filename);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}


//激活定位模式
void System::ActivateLocalizationMode()
{
    //上锁
    unique_lock<mutex> lock(mMutexMode);
    //设置标志
    mbActivateLocalizationMode = true;
}

//取消定位模式
void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

//判断是否地图有较大的改变
bool System::MapChanged()
{
    static int n=0;
    //其实整个函数功能实现的重点还是在这个GetLastBigChangeIdx函数上
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

//准备执行复位
void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}


void System::ResetActiveMap()
{
    unique_lock<mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

//退出
void System::Shutdown()
{
    //对局部建图线程和回环检测线程发送终止请求
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    //如果使用了可视化窗口查看器
    if(mpViewer)
    {
        //向查看器发送终止请求
        mpViewer->RequestFinish();
        //等到，知道真正地停止
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        if(!mpLocalMapper->isFinished())
            cout << "mpLocalMapper is not finished" << endl;
        if(!mpLoopCloser->isFinished())
            cout << "mpLoopCloser is not finished" << endl;
        if(mpLoopCloser->isRunningGBA()){
            cout << "mpLoopCloser is running GBA" << endl;
            cout << "break anyway..." << endl;
            break;
        }
        usleep(5000);
    }

    //如果使用了可视化的窗口查看器执行这个
    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}


//按照TUM格式保存相机运行轨迹并保存到指定的文件中
void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    //只有在传感器为双目或者RGBD时才可以工作
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    //从地图中获取所有的关键帧
    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    //根据关键帧生成的先后顺序（id）进行排序
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    // 到原点的转换，获取这个转换矩阵
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    //文件写入的准备工作
    ofstream f;
    f.open(filename.c_str());
    //这个可以理解为，在输出浮点数的时候使用0.3141592654这样的方式而不是使用科学计数法
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.
    // 之前的帧位姿都是基于其参考关键帧的，现在我们把它恢复

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    //参考关键帧列表
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    //所有帧对应的时间戳列表
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    //每帧的追踪状态组成的列表
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //如果该帧追踪失败，不管它，进行下一个
        if(*lbL)
            continue;

        //获取其对应的参考关键帧
        KeyFrame* pKF = *lRit;
        //变换矩阵的初始化，初始化为一个单位阵
        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled（剔除）, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            //更新关键帧变换矩阵的初始值，
            Trw = Trw*pKF->mTcp;
            //并且更新到原关键帧的父关键帧
            pKF = pKF->GetParent();
        }//查看当前使用的参考关键帧是否为bad

        //最后一个Two是原点校正
        //最终得到的是参考关键帧相对于世界坐标系的变换
        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        //然后分解出旋转矩阵
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        //以及平移向量
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
        //用四元数表示旋转
        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    // cout << endl << "trajectory saved!" << endl;
}

//保存关键帧的轨迹
void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    //获取关键帧vector并按照生成时间对其进行排序
    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    //本来这里需要进行原点校正，但是实际上没有做
    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    //文件写入的准备操作
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    //对于每个关键帧
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        //获取该 关键帧
        KeyFrame* pKF = vpKFs[i];

        //原本有个原点校正，这里注释掉了
        //如果这个关键帧是bad那么就跳过
       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        //抽取旋转部分和平移部分，前者使用四元数表示
        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        //按照给定的格式输出到文件中
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    //关闭文件
    f.close();
}

//按照EuRoC格式保存相机运行轨迹并保存到指定的文件中
void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    /*if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
        return;
    }*/

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }
    //从地图中获取所有的关键帧
    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    //根据关键帧生成的先后顺序（id）进行排序
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    // 到原点的转换，获取这个转换矩阵
    cv::Mat Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    //文件写入的准备工作
    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.
    // 之前的帧位姿都是基于其参考关键帧的，现在我们把它恢复

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    //参考关键帧列表
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    //所有帧对应的时间戳列表
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    //每帧的追踪状态组成的列表    
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        //cout << "1" << endl;
        //如果该帧追踪失败，不管它，进行下一个
        if(*lbL)
            continue;

        //获取其对应的参考关键帧
        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;
        //变换矩阵的初始化，初始化为一个单位阵
        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        /*cout << "2" << endl;
        cout << "KF id: " << pKF->mnId << endl;*/

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //更新关键帧变换矩阵的初始值，
            //cout << " 2.bad" << endl;
            Trw = Trw*pKF->mTcp;
            //并且更新到原关键帧的父关键帧
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }//查看当前使用的参考关键帧是否为bad

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            /*if(pKF)
                cout << "--Parent KF " << pKF->mnId << " is from another map " << pKF->GetMap()->GetId() << endl;*/
            continue;
        }

        //cout << "3" << endl;
        //最后一个Two是原点校正
        //最终得到的是参考关键帧相对于世界坐标系的变换
        Trw = Trw*pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        // cout << "4" << endl;

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO)
        {
            cv::Mat Tbw = pKF->mImuCalib.Tbc*(*lit)*Trw;
            cv::Mat Rwb = Tbw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twb = -Rwb*Tbw.rowRange(0,3).col(3);
            vector<float> q = Converter::toQuaternion(Rwb);
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        else
        {
            //在此基础上得到相机当前帧相对于世界坐标系的变换
            cv::Mat Tcw = (*lit)*Trw;
            //然后分解出旋转矩阵
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            //以及平移向量
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            //用四元数表示旋转
            vector<float> q = Converter::toQuaternion(Rwc);
            //然后按照给定的格式输出到文件中
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }

        // cout << "5" << endl;
    }//对于每一个mlRelativeFramePoses中的帧lit所进行的操作
    //cout << "end saving trajectory" << endl;

    //操作完毕，关闭文件并且输出调试信息
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

//保存关键帧的轨迹
void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }
    //获取关键帧vector并按照生成时间对其进行排序
    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //文件写入的准备操作
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    //对于每个关键帧
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        //获取该 关键帧
        KeyFrame* pKF = vpKFs[i];

        //原本有个原点校正，这里注释掉了
        //pKF->SetPose(pKF->GetPose()*Two);

        //如果这个关键帧是bad那么就跳过
        if(pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO)
        {
            //抽取旋转部分和平移部分，前者使用四元数表示
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            //按照给定的格式输出到文件中
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " << twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
        else
        {
            //抽取旋转部分和平移部分，前者使用四元数表示
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            //按照给定的格式输出到文件中
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    }
    //关闭文件
    f.close();
}

//按照KITTI数据集的格式将相机的运动轨迹保存到文件中
void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
}


void System::SaveDebugData(const int &initIdx)
{
    // 0. Save initialization trajectory
    SaveTrajectoryEuRoC("init_FrameTrajectoy_" +to_string(mpLocalMapper->mInitSect)+ "_" + to_string(initIdx)+".txt");

    // 1. Save scale
    ofstream f;
    f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mScale << endl;
    f.close();

    // 2. Save gravity direction
    f.open("init_GDir_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mRwg(0,0) << "," << mpLocalMapper->mRwg(0,1) << "," << mpLocalMapper->mRwg(0,2) << endl;
    f << mpLocalMapper->mRwg(1,0) << "," << mpLocalMapper->mRwg(1,1) << "," << mpLocalMapper->mRwg(1,2) << endl;
    f << mpLocalMapper->mRwg(2,0) << "," << mpLocalMapper->mRwg(2,1) << "," << mpLocalMapper->mRwg(2,2) << endl;
    f.close();

    // 3. Save computational cost
    f.open("init_CompCost_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mCostTime << endl;
    f.close();

    // 4. Save biases
    f.open("init_Biases_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
    f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
    f.close();

    // 5. Save covariance matrix
    f.open("init_CovMatrix_" +to_string(mpLocalMapper->mInitSect)+ "_" +to_string(initIdx)+".txt", ios_base::app);
    f << fixed;
    for(int i=0; i<mpLocalMapper->mcovInertial.rows(); i++)
    {
        for(int j=0; j<mpLocalMapper->mcovInertial.cols(); j++)
        {
            if(j!=0)
                f << ",";
            f << setprecision(15) << mpLocalMapper->mcovInertial(i,j);
        }
        f << endl;
    }
    f.close();

    // 6. Save initialization time
    f.open("init_Time_" +to_string(mpLocalMapper->mInitSect)+ ".txt", ios_base::app);
    f << fixed;
    f << mpLocalMapper->mInitTime << endl;
    f.close();
}


int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit()
{
    double aux = mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    if ((aux>0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs;
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if ((mpTracker->mState==Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }
}


bool System::isFinished()
{
    return (GetTimeFromIMUInit()>0.1);
}

void System::ChangeDataset()
{
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

/*void System::SaveAtlas(int type){
    cout << endl << "Enter the name of the file if you want to save the current Atlas session. To exit press ENTER: ";
    string saveFileName;
    getline(cin,saveFileName);
    if(!saveFileName.empty())
    {
        //clock_t start = clock();

        // Save the current session
        mpAtlas->PreSave();
        mpKeyFrameDatabase->PreSave();

        string pathSaveFileName = "./";
        pathSaveFileName = pathSaveFileName.append(saveFileName);
        pathSaveFileName = pathSaveFileName.append(".osa");

        string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);
        std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
        string strVocabularyName = mStrVocabularyFilePath.substr(found+1);

        if(type == TEXT_FILE) // File text
        {
            cout << "Starting to write the save text file " << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::text_oarchive oa(ofs);

            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            oa << mpKeyFrameDatabase;
            cout << "End to write the save text file" << endl;
        }
        else if(type == BINARY_FILE) // File binary
        {
            cout << "Starting to write the save binary file" << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            oa << mpKeyFrameDatabase;
            cout << "End to write save binary file" << endl;
        }

        //clock_t timeElapsed = clock() - start;
        //unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
        //cout << "Binary file saved in " << msElapsed << " ms" << endl;
    }
}

bool System::LoadAtlas(string filename, int type)
{
    string strFileVoc, strVocChecksum;
    bool isRead = false;

    if(type == TEXT_FILE) // File text
    {
        cout << "Starting to read the save text file " << endl;
        std::ifstream ifs(filename, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        //ia >> mpKeyFrameDatabase;
        cout << "End to load the save text file " << endl;
        isRead = true;
    }
    else if(type == BINARY_FILE) // File binary
    {
        cout << "Starting to read the save binary file"  << endl;
        std::ifstream ifs(filename, std::ios::binary);
        if(!ifs.good())
        {
            cout << "Load file not found" << endl;
            return false;
        }
        boost::archive::binary_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        //ia >> mpKeyFrameDatabase;
        cout << "End to load the save binary file" << endl;
        isRead = true;
    }

    if(isRead)
    {
        //Check if the vocabulary is the same
        string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);

        if(strInputVocabularyChecksum.compare(strVocChecksum) != 0)
        {
            cout << "The vocabulary load isn't the same which the load session was created " << endl;
            cout << "-Vocabulary name: " << strFileVoc << endl;
            return false; // Both are differents
        }

        return true;
    }
    return false;
}

string System::CalculateCheckSum(string filename, int type)
{
    string checksum = "";

    unsigned char c[MD5_DIGEST_LENGTH];

    std::ios_base::openmode flags = std::ios::in;
    if(type == BINARY_FILE) // Binary file
        flags = std::ios::in | std::ios::binary;

    ifstream f(filename.c_str(), flags);
    if ( !f.is_open() )
    {
        cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
        return checksum;
    }

    MD5_CTX md5Context;
    char buffer[1024];

    MD5_Init (&md5Context);
    while ( int count = f.readsome(buffer, sizeof(buffer)))
    {
        MD5_Update(&md5Context, buffer, count);
    }

    f.close();

    MD5_Final(c, &md5Context );

    for(int i = 0; i < MD5_DIGEST_LENGTH; i++)
    {
        char aux[10];
        sprintf(aux,"%02x", c[i]);
        checksum = checksum + aux;
    }

    return checksum;
}*/

} //namespace ORB_SLAM


