/**
 * @file LocalMapping.h
 * @brief 局部建图线程
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


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Initializer.h"

#include <mutex>


namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;

/** @brief 局部建图线程类 */
class LocalMapping
{
public:

    /**
     * @brief 构造函数
     * @param[in] pMap          局部地图的句柄
     * @param[in] bMonocular    当前系统是否是单目输入
     */
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName=std::string());

    /**
     * @brief 设置回环检测线程句柄
     * @param[in] pLoopCloser 回环检测线程句柄
     */
    void SetLoopCloser(LoopClosing* pLoopCloser);

    /**
     * @brief 设置追踪线程句柄
     * @param[in] pTracker 追踪线程句柄
     */
    void SetTracker(Tracking* pTracker);

    // Main function
    /** @brief 线程主函数 */
    void Run();

    /**
     * @brief 插入关键帧,由外部线程调用
     * @details 将关键帧插入到地图中，以便将来进行局部地图优化 \n
     * NOTICE 这里仅仅是将关键帧插入到列表中进行等待
     * @param pKF KeyFrame
     */
    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    /** @brief 外部线程调用,请求停止当前线程的工作 */
    void RequestStop();
    /** @brief 请求当前线程复位,由外部线程调用,堵塞的 */
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);

    /**
     * @brief 检查是否要把当前的局部建图线程停止,如果当前线程没有那么检查请求标志,如果请求标志被置位那么就设置为停止工作.由run函数调用
     * @return true 
     * @return false 
     */
    bool Stop();

    /** @brief 释放当前还在缓冲区中的关键帧指针  */
    void Release();

    /** @brief 检查mbStopped是否被置位了 */
    bool isStopped();

    /** @brief 是否有终止当前线程的请求 */
    bool stopRequested();
    bool AcceptKeyFrames();

    /**
     * @brief 设置"允许接受关键帧"的状态标志
     * @param[in] flag 是或者否
     */
    void SetAcceptKeyFrames(bool flag);

    /** @brief 设置 mbnotStop标志的状态 */
    bool SetNotStop(bool flag);

    /** @brief 外部线程调用,终止BA */
    void InterruptBA();

    /** @brief 请求终止当前线程 */
    void RequestFinish();

    /** @brief 当前线程的run函数是否已经终止 */
    bool isFinished();

    //查看队列中等待插入的关键帧数目
    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();
    KeyFrame* GetCurrKF();

    std::mutex mMutexImuInit;

    Eigen::MatrixXd mcovInertial;
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
    double mInitTime;
    double mCostTime;
    bool mbNewInit;
    unsigned int mInitSect;
    unsigned int mIdxInit;
    unsigned int mnKFs;
    double mFirstTs;
    int mnMatchesInliers;

    // For debugging (erase in normal mode)
    int mInitFr;
    int mIdxIteration;
    string strSequence;

    bool mbNotBA1;
    bool mbNotBA2;
    bool mbBadImu;

    bool mbWriteStats;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;
protected:


    /**
     * @brief 查看列表中是否有等待被插入的关键帧
     * @return 如果存在，返回true
     */
    bool CheckNewKeyFrames();

    /**
     * @brief 处理列表中的关键帧
     * 
     * - 计算Bow，加速三角化新的MapPoints
     * - 关联当前关键帧至MapPoints，并更新MapPoints的平均观测方向和观测距离范围
     * - 插入关键帧，更新Covisibility图和Essential图
     * @see VI-A keyframe insertion
     */
    void ProcessNewKeyFrame();

    /** @brief 相机运动过程中和共视程度比较高的关键帧通过三角化恢复出一些MapPoints */
    void CreateNewMapPoints();

    /**
     * @brief 剔除ProcessNewKeyFrame和CreateNewMapPoints函数中引入的质量不好的MapPoints
     * @see VI-B recent map points culling
     */
    void MapPointCulling();

    /** @brief 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints */
    void SearchInNeighbors();

    /**
     * @brief 关键帧剔除
     * @detials 在Covisibility Graph中的关键帧，其90%以上的MapPoints能被其他关键帧（至少3个）观测到，则认为该关键帧为冗余关键帧。
     * @see VI-E Local Keyframe Culling
     */
    void KeyFrameCulling();

    /**
     * 根据两关键帧的姿态计算两个关键帧之间的基本矩阵
     * @param  pKF1 关键帧1
     * @param  pKF2 关键帧2
     * @return      基本矩阵
     */
    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    /**
     * @brief 计算三维向量v的反对称矩阵
     * @param[in] v     三维向量
     * @return cv::Mat  反对称矩阵
     */
    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    System *mpSystem;

    /// 当前系统输入数单目还是双目RGB-D的标志
    bool mbMonocular;

    /// 当前系统输入是否有IMU的标志
    bool mbInertial;

    /** @brief 检查当前是否有复位线程的请求 */
    void ResetIfRequested();

    /// 当前系统是否收到了请求复位的信号
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;

    /// 和复位信号有关的互斥量
    std::mutex mMutexReset;

    /** @brief 检查是否已经有外部线程请求终止当前线程 */
    bool CheckFinish();

    /** @brief 设置当前线程已经真正地结束了,由本线程run函数调用 */
    void SetFinish();

    /// 当前线程是否收到了请求终止的信号
    bool mbFinishRequested;

    /// 当前线程的主函数是否已经终止
    bool mbFinished;

    // 和"线程真正结束"有关的互斥锁
    std::mutex mMutexFinish;

    // 指向地图的句柄
    Atlas* mpAtlas;

    // 回环检测线程句柄
    LoopClosing* mpLoopCloser;

    // 追踪线程句柄
    Tracking* mpTracker;

    /// 存储当前关键帧生成的地图点,也是等待检查的地图点列表
    std::list<KeyFrame*> mlNewKeyFrames;

    /// 当前正在处理的关键帧
    KeyFrame* mpCurrentKeyFrame;

    /// 存储当前关键帧生成的地图点,也是等待检查的地图点列表
    std::list<MapPoint*> mlpRecentAddedMapPoints;

    /// 操作关键帧列表时使用的互斥量 
    std::mutex mMutexNewKFs;

    /// 终止BA的标志
    bool mbAbortBA;

    /// 当前线程是否已经真正地终止了
    bool mbStopped;

    /// 终止当前线程的请求
    bool mbStopRequested;

    /// 标志这当前线程还不能够停止工作,优先级比那个"mbStopRequested"要高.只有这个和mbStopRequested都满足要求的时候,线程才会进行一系列的终止操作
    bool mbNotStop;

    /// 和终止线程相关的互斥锁
    std::mutex mMutexStop;

    /// 当前局部建图线程是否允许关键帧输入
    bool mbAcceptKeyFrames;

    /// 和操作上面这个变量有关的互斥量
    std::mutex mMutexAccept;

    /**
     * 初始化IMU
     * @param  priorG 先验G的信息info
     * @param  priorA 先验A的信息info
     * @param  bFirst 是否是第一次做BA
     * @return   void
     */
    void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false);

    /** @brief 尺度改善 */
    void ScaleRefinement();

    /// 是否初始化
    bool bInitializing;

    /// 惯性的信息
    Eigen::MatrixXd infoInertial;
    int mNumLM;
    int mNumKFCulling;

    float mTinit;

    ///尺度改善的次数
    int countRefinement;

    //DEBUG
    ofstream f_lm;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
