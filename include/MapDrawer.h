/**
 * @file MapDrawer.h
 * @brief 绘制地图点
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


#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Atlas.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ORB_SLAM3
{

class MapDrawer
{
public:
    /**
     * @brief 构造函数
     * 
     * @param[in] pMap              地图句柄
     * @param[in] strSettingPath    配置文件的路径
     */
    MapDrawer(Atlas* pAtlas, const string &strSettingPath);
    
    //地图句柄
    Atlas* mpAtlas;

    /** @brief 绘制地图点 */
    void DrawMapPoints();

    /**
     * @brief 绘制关键帧
     * 
     * @param[in] bDrawKF       是否绘制关键帧
     * @param[in] bDrawGraph    是否绘制共视图
     * @param[in] bDrawInertialGraph    是否绘制IMU共视图
     */
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph);

    /**
     * @brief 绘制当前相机
     * 
     * @param[in] Twc 相机的位姿矩阵
     */
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    /**
     * @brief 设置当前帧的相机位姿
     * 
     * @param[in] Tcw 位姿矩阵
     */
    void SetCurrentCameraPose(const cv::Mat &Tcw);

    /**
     * @brief 设置参考关键帧
     * 
     * @param[in] pKF 参考关键帧的句柄
     */
    void SetReferenceKeyFrame(KeyFrame *pKF);

    /**
     * @brief 将相机位姿mCameraPose由Mat类型转化为OpenGlMatrix类型
     * 
     * @param[out] M 旋转+平移
     * @param[out] MOw 平移
     */
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);

    /**
     * @brief 将相机位姿mCameraPose由Mat类型转化为OpenGlMatrix类型
     * 
     * @param[out] M 旋转+平移
     * @param[out] MOw 单位阵+平移
     * @param[out] MTwwp 单位阵+平移
     */
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw, pangolin::OpenGlMatrix &MTwwp);

private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    //绘制这些部件的参数
    ///关键帧-大小
    float mKeyFrameSize;

    ///关键帧-线宽
    float mKeyFrameLineWidth;

    ///共视图的线宽
    float mGraphLineWidth;

    ///地图点的大小
    float mPointSize;

    ///绘制的相机的大小
    float mCameraSize;

    ///绘制相机的线宽
    float mCameraLineWidth;

    ///相机位置
    cv::Mat mCameraPose;

    ///线程互斥量
    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
