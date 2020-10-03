/**
 * @file Viewer.cc
 * @brief 查看器的实现
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


#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM3
{

//查看器的构造函数
Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    //从文件中读取相机的帧频
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool is_correct = ParseViewerParamFile(fSettings);

    if(!is_correct)
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

    mbStopTrack = false;
}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    
    //计算出每一帧所持续的时间
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

// pangolin库的文档：http://docs.ros.org/fuerte/api/pangolin_wrapper/html/namespacepangolin.html
//查看器的主进程看来是外部函数所调用的
void Viewer::Run()
{
    //这个变量配合SetFinish函数用于指示该函数是否执行完毕
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    // 启动深度测试，OpenGL只绘制最前面的一层，绘制时检查当前像素前面是否有别的像素，如果别的像素挡住了它，那它就不会绘制
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    // 在OpenGL中使用颜色混合
    glEnable (GL_BLEND);

    // 选择混合选项
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
    pangolin::Var<bool> menuTopView("menu.Top View",false,false);
    // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuStepByStep("menu.Step By Step",false,true);  // false, true
    pangolin::Var<bool> menuStep("menu.Step",false,false);

    // Define Camera Render Object (for view / scene browsing)
    // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
    // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
    //                观测目标位置：(0, 0, 0)
    //                观测的方位向量：(0.0,-1.0, 0.0)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
    // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
    // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
    // 最后一个参数（-1024.0f/768.0f）为显示长宽比
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    //创建一个欧式变换矩阵,存储当前的相机位姿
    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();
    pangolin::OpenGlMatrix Twwp; // Oriented with g in the z axis, but y and x from camera
    Twwp.SetIdentity();

    //创建当前帧图像查看器,谢晓佳在泡泡机器人的第35课中讲过这个;需要先声明窗口,再创建;否则就容易出现窗口无法刷新的情况
    cv::namedWindow("ORB-SLAM3: Current Frame");

    //ui设置
    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bStepByStep = false;
    bool bCameraView = true;

    if(mpTracker->mSensor == mpSystem->MONOCULAR || mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->RGBD)
    {
        menuShowGraph = true;
    }

    //更新绘制的内容
    while(1)
    {
        // 清除缓冲区中的当前可写的颜色缓冲 和 深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // step1：得到最新的相机位姿
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc,Ow,Twwp);

        if(mbStopTrack)
        {
            menuStepByStep = true;
            mbStopTrack = false;
        }

        // step2：根据相机的位姿调整视角
        // menuFollowCamera为按钮的状态，bFollow为真实的状态
        if(menuFollowCamera && bFollow)
        {
            if(bCameraView)
            //当之前也在跟踪相机时
                s_cam.Follow(Twc);
            else
                s_cam.Follow(Ow);
        }
        else if(menuFollowCamera && !bFollow)
        {
            if(bCameraView)
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
            }
            else
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,10, 0,0,0,0.0,0.0, 1.0));
                s_cam.Follow(Ow);
            }
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            //之前跟踪相机,但是现在菜单命令不要跟踪相机时
            bFollow = false;
        }

        if(menuCamView)
        {
            menuCamView = false;
            bCameraView = true;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
        }

        //更新定位模式或者是SLAM模式
        if(menuTopView && mpMapDrawer->mpAtlas->isImuInitialized())
        {
            menuTopView = false;
            bCameraView = false;
            /*s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,1000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,10, 0,0,0,0.0,0.0, 1.0));*/
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,50, 0,0,0,0.0,0.0, 1.0));
            s_cam.Follow(Ow);
        }

        /*if(menuSideView && mpMapDrawer->mpAtlas->isImuInitialized())
        {
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0.0,0.1,30.0,0,0,0,0.0,0.0,1.0));
            s_cam.Follow(Twwp);
        }*/


        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        if(menuStepByStep && !bStepByStep)
        {
            mpTracker->SetStepByStep(true);
            bStepByStep = true;
        }
        else if(!menuStepByStep && bStepByStep)
        {
            mpTracker->SetStepByStep(false);
            bStepByStep = false;
        }

        if(menuStep)
        {
            mpTracker->mbStep = true;
            menuStep = false;
        }

        // step 3：绘制地图和图像(3D部分)
        // 设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        //绘制当前相机
        mpMapDrawer->DrawCurrentCamera(Twc);

        //绘制关键帧和共视图
        if(menuShowKeyFrames || menuShowGraph || menuShowInertialGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph, menuShowInertialGraph);
        
        //绘制地图点
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        cv::Mat toShow;

        // step 4:绘制当前帧图像和特征点提取匹配结果
        cv::Mat im = mpFrameDrawer->DrawFrame(true);

        if(both){
            cv::Mat imRight = mpFrameDrawer->DrawRightFrame();
            cv::hconcat(im,imRight,toShow);
        }
        else{
            toShow = im;
        }

        cv::imshow("ORB-SLAM3: Current Frame",toShow);
        //NOTICE 注意对于我所遇到的问题,ORB-SLAM3是这样子来处理的
        cv::waitKey(mT);

        // step 5 相应其他请求
        //复位按钮
        if(menuReset)
        {
            //将所有的GUI控件恢复初始状态
            menuShowGraph = true;
            menuShowInertialGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            //相关变量也恢复到初始状态
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            //告知系统复位
            //mpSystem->Reset();
            mpSystem->ResetActiveMap();
            //按钮本身状态复位
            menuReset = false;
        }

        //如果有停止更新的请求
        if(Stop())
        {
            //就不再绘图了,并且在这里每隔三秒检查一下是否结束
            while(isStopped())
            {
                usleep(3000);
            }
        }

        //满足的时候退出这个线程循环,这里应该是查看终止请求
        if(CheckFinish())
            break;
    }

    //终止查看器,主要是设置状态,执行完成退出这个函数后,查看器进程就已经被销毁了
    SetFinish();
}

//外部函数调用,用来请求当前进程结束
void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

//检查是否有结束当前进程的请求
bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

//设置变量:当前进程已经结束
void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

//判断当前进程是否已经结束
bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

//请求当前查看器停止更新
void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

//查看当前查看器是否已经停止更新
bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

//当前查看器停止更新
bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

//释放查看器进程,因为如果停止查看器的话,查看器进程会处于死循环状态.这个就是为了释放那个标志
void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}

}
