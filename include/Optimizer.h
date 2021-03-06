/**
 * @file Optimizer.h
 * @brief 优化器，所有用到的优化函数的声明都在这个文件中
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


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include <math.h>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace ORB_SLAM3
{

class LoopClosing;

/** @brief 优化器,所有的优化相关的函数都在这个类中; 并且这个类只有成员函数没有成员变量,相对要好分析一点 */
class Optimizer
{
public:

    /**
     * @brief bundle adjustment Optimization
     * 
     * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n
     * 
     * 1. Vertex: g2o::VertexSE3Expmap()，即当前帧的Tcw
     *            g2o::VertexSBAPointXYZ()，MapPoint的mWorldPos
     * 2. Edge:
     *     - g2o::EdgeSE3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：待优化当前帧的Tcw
     *         + Vertex：待优化MapPoint的mWorldPos
     *         + measurement：MapPoint在当前帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *         
     * @param   vpKFs    关键帧 
     *          vpMP     MapPoints
     *          nIterations 迭代次数（20次）
     *          pbStopFlag  是否强制暂停
     *          nLoopKF  关键帧的个数 -- 但是我觉得形成了闭环关系的当前关键帧的id
     *          bRobust  是否使用核函数
     */
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);

    /**
     * @brief 进行全局BA优化，但主要功能还是调用 BundleAdjustment,这个函数相当于加了一个壳.
     * @param[in] pMap          地图对象的指针
     * @param[in] nIterations   迭代次数
     * @param[in] pbStopFlag    外界给的控制GBA停止的标志位
     * @param[in] nLoopKF       当前回环关键帧的id，其实也就是参与GBA的关键帧个数
     * @param[in] bRobust       是否使用鲁棒核函数
     */
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static FullInertialBA(Map *pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);


    /**
     * @brief Local Bundle Adjustment
     *
     * 1. Vertex:
     *     - g2o::VertexSE3Expmap()，LocalKeyFrames，即当前关键帧的位姿、与当前关键帧相连的关键帧的位姿
     *     - g2o::VertexSE3Expmap()，FixedCameras，即能观测到LocalMapPoints的关键帧（并且不属于LocalKeyFrames）的位姿，在优化中这些关键帧的位姿不变
     *     - g2o::VertexSBAPointXYZ()，LocalMapPoints，即LocalKeyFrames能观测到的所有MapPoints的位置
     * 2. Edge:
     *     - g2o::EdgeSE3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Tcw，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *     - g2o::EdgeStereoSE3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Tcw，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(ul,v,ur)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *         
     * @param pKF        KeyFrame
     * @param pbStopFlag 是否停止优化的标志
     * @param pMap       在优化后，更新状态时需要用到Map的互斥量mMutexMapUpdate
     * @note 由局部建图线程调用,对局部地图进行优化的函数
     */
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, vector<KeyFrame*> &vpNonEnoughOptKFs);

    /**
     * @brief Local Bundle Adjustment
     *
     * 1. Vertex:
     *     - g2o::VertexSE3Expmap()，LocalKeyFrames，即当前关键帧的位姿、与当前关键帧相连的关键帧的位姿
     *     - g2o::VertexSE3Expmap()，FixedCameras，即能观测到LocalMapPoints的关键帧（并且不属于LocalKeyFrames）的位姿，在优化中这些关键帧的位姿不变
     *     - g2o::VertexSBAPointXYZ()，LocalMapPoints，即LocalKeyFrames能观测到的所有MapPoints的位置
     * 2. Edge:
     *     - g2o::EdgeSE3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Tcw，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *     - g2o::EdgeStereoSE3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Tcw，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(ul,v,ur)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *         
     * @param pKF        KeyFrame
     * @param pbStopFlag 是否停止优化的标志
     * @param pMap       在优化后，更新状态时需要用到Map的互斥量mMutexMapUpdate
     * @param num_fixedKF 局部关键帧中的局部地图点
     * @note 由局部建图线程调用,对局部地图进行优化的函数
     */
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF);

    void static MergeBundleAdjustmentVisual(KeyFrame* pCurrentKF, vector<KeyFrame*> vpWeldingKFs, vector<KeyFrame*> vpFixedKFs, bool *pbStopFlag);

    /**
     * @brief Pose Only Optimization
     * 
     * 3D-2D 最小化重投影误差 e = (u,v) - project(Tcw*Pw) \n
     * 只优化Frame的Tcw，不优化MapPoints的坐标
     * 
     * 1. Vertex: g2o::VertexSE3Expmap()，即当前帧的Tcw
     * 2. Edge:
     *     - g2o::EdgeSE3ProjectXYZOnlyPose()，BaseUnaryEdge
     *         + Vertex：待优化当前帧的Tcw
     *         + measurement：MapPoint在当前帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *     - g2o::EdgeStereoSE3ProjectXYZOnlyPose()，BaseUnaryEdge
     *         + Vertex：待优化当前帧的Tcw
     *         + measurement：MapPoint在当前帧中的二维位置(ul,v,ur)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *
     * @param   pFrame Frame
     * @return  inliers数量
     */
    int static PoseOptimization(Frame* pFrame);

    int static PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
    int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    /**
     * @brief 闭环检测后，EssentialGraph优化
     *
     * 1. Vertex:
     *     - g2o::VertexSim3Expmap，Essential graph中关键帧的位姿
     * 2. Edge:
     *     - g2o::EdgeSim3()，BaseBinaryEdge
     *         + Vertex：关键帧的Tcw，MapPoint的Pw
     *         + measurement：经过CorrectLoop函数步骤2，Sim3传播校正后的位姿
     *         + InfoMatrix: 单位矩阵     
     *
     * @param pMap               全局地图
     * @param pLoopKF            闭环匹配上的关键帧
     * @param pCurKF             当前关键帧
     * @param NonCorrectedSim3   未经过Sim3传播调整过的关键帧位姿
     * @param CorrectedSim3      经过Sim3传播调整过的关键帧位姿
     * @param LoopConnections    因闭环时MapPoints调整而新生成的边
     */
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);
    void static OptimizeEssentialGraph6DoF(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs, vector<KeyFrame*> &vpFixedCorrectedKFs,
                                           vector<KeyFrame*> &vpNonFixedKFs, vector<MapPoint*> &vpNonCorrectedMPs, double scale);
    void static OptimizeEssentialGraph(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs, vector<KeyFrame*> &vpFixedCorrectedKFs,
                                       vector<KeyFrame*> &vpNonFixedKFs, vector<MapPoint*> &vpNonCorrectedMPs);
    void static OptimizeEssentialGraph(KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3);
    // For inetial loopclosing
    void static OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (OLD)
    // 闭环刚刚形成的时候,对当前关键帧和闭环关键帧之间的sim3变换的优化
    /**
     * @brief 形成闭环时,对当前关键帧和闭环关键帧的Sim3位姿进行优化
     *
     * 1. Vertex:
     *     - g2o::VertexSim3Expmap()，两个关键帧的位姿
     *     - g2o::VertexSBAPointXYZ()，两个关键帧共有的MapPoints
     * 2. Edge:
     *     - g2o::EdgeSim3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Sim3，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *     - g2o::EdgeInverseSim3ProjectXYZ()，BaseBinaryEdge
     *         + Vertex：关键帧的Sim3，MapPoint的Pw
     *         + measurement：MapPoint在关键帧中的二维位置(u,v)
     *         + InfoMatrix: invSigma2(与特征点所在的尺度有关)
     *         
     * @param pKF1        KeyFrame
     * @param pKF2        KeyFrame
     * @param vpMatches1  两个关键帧的匹配关系
     * @param g2oS12      两个关键帧间的Sim3变换
     * @param th2         核函数阈值
     * @param bFixScale   是否优化尺度，弹目进行尺度优化，双目不进行尺度优化
     */
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono) (NEW)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale,
                            Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints=false);
    static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, vector<KeyFrame*> &vpMatches1KF,
                     g2o::Sim3 &g2oS12, const float th2, const bool bFixScale, Eigen::Matrix<double,7,7> &mAcumHessian,
                     const bool bAllPoints = false);

    // For inertial systems

    void static LocalInertialBA(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, bool bLarge = false, bool bRecInit = false);

    void static MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool *pbStopFlag, Map *pMap, LoopClosing::KeyFrameAndPose &corrPoses);

    // Local BA in welding area when two maps are merged
    void static LocalBundleAdjustment(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag);

    // Marginalize block element (start:end,start:end). Perform Schur complement.
    // Marginalized elements are filled with zeros.
    /**
     * @brief 边缘化块元素(start:end，start:end)。执行舒尔补。 边缘元素用零填充。
     *
     * @param H        信息矩阵
     * @param start       H开始的行，列
     * @param end         H结束的行，列
     */
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);
    
    // Condition block element (start:end,start:end). Fill with zeros.
    /**
     * @brief 条件块元素(start:end，start:end)。 用零填充。
     *
     * @param H        信息矩阵
     * @param start       H开始的行，列
     * @param end         H结束的行，列
     */
    static Eigen::MatrixXd Condition(const Eigen::MatrixXd &H, const int &start, const int &end);

    // Remove link between element 1 and 2. Given elements 1,2 and 3 must define the whole matrix.
    /**
     * @brief 删除块元素(start:end，start:end)。 用零填充。
     *
     * @param H        信息矩阵
     * @param start1       开始行
     * @param end1         结束行
     * @param start2       开始列
     * @param end2         结束列
     */
    static Eigen::MatrixXd Sparsify(const Eigen::MatrixXd &H, const int &start1, const int &end1, const int &start2, const int &end2);

    // Inertial pose-graph
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel=false, bool bGauss=false, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(vector<KeyFrame*> vpKFs, Eigen::Vector3d &bg, Eigen::Vector3d &ba, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);
};

} //namespace ORB_SLAM3

#endif // OPTIMIZER_H
