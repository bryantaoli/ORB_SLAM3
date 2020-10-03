/**
 * @file ORBextractor.h
 * @brief ORB特征点提取
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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM3
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:

    //定义一个枚举类型用于表示使用HARRIS响应值还是使用FAST响应值
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /**
     * @brief 构造函数
     * @detials 之所以会有两种响应值的阈值，原因是，程序先使用初始的默认FAST响应值阈值提取图像cell中的特征点；如果提取到的
     * 特征点数目不足，那么就降低要求，使用较小FAST响应值阈值进行再次提取，以获得尽可能多的FAST角点。
     * @param[in] nfeatures         指定要提取出来的特征点数目
     * @param[in] scaleFactor       图像金字塔的缩放系数
     * @param[in] nlevels           指定需要提取特征点的图像金字塔层
     * @param[in] iniThFAST         初始的默认FAST响应值阈值
     * @param[in] minThFAST         较小的FAST响应值阈值
     */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation. 函数中实际上并没有用到MASK。
    /**
     * @brief 使用八叉树的方法将提取到的ORB特征点尽可能均匀地分布在整个图像中
     * @details 这里是重载了这个ORBextractor类的括号运算符;函数中实际上并没有用到MASK这个参数。
     * 
     * @param[in] _image         要操作的图像
     * @param[in] _mask          图像掩膜，辅助进行图片处理，可以参考[https://www.cnblogs.com/skyfsm/p/6894685.html]
     * @param[out] _keypoints    保存提取出来的特征点的向量
     * @param[out] _descriptors  输出用的保存特征点描述子的cv::Mat
     * @param[out] vLappingArea  提取特征点的范围
     */
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    //下面的这些内联函数都是用来直接获取类的成员变量的
	
    /**
     * @brief 获取图像金字塔的层数
     * @return int 图像金字塔的层数
     */
    int inline GetLevels(){
        return nlevels;}

    /**
     * @brief 获取当前提取器所在的图像的缩放因子，这个不带s的因子表示是相临近层之间的
     * @return float 当前提取器所在的图像的缩放因子，相邻层之间
     */
    float inline GetScaleFactor(){
        return scaleFactor;}

    /**
     * @brief 获取图像金字塔中每个图层相对于底层图像的缩放因子
     * @return std::vector<float> 图像金字塔中每个图层相对于底层图像的缩放因子
     */
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    /**
     * @brief 获取上面的那个缩放因子s的倒数
     * @return std::vector<float> 倒数
     */
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    /**
     * @brief 获取sigma^2，就是每层图像相对于初始图像缩放因子的平方，参考cpp文件中类构造函数的操作
     * @return std::vector<float> sigma^2
     */
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    /**
     * @brief 获取上面sigma平方的倒数
     * @return std::vector<float> 
     */
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    ///这个是用来存储图像金字塔的变量，一个元素存储一层图像
    std::vector<cv::Mat> mvImagePyramid;

protected:

	//下面这些都是保护成员

    /**
     * @brief 针对给出的一张图像，计算其图像金字塔
     * @param[in] image 给出的图像
     */
    void ComputePyramid(cv::Mat image);

    /**
     * @brief 以八叉树分配特征点的方式，计算图像金字塔中的特征点
     * @detials 这里两层vector的意思是，第一层存储的是某张图片中的所有特征点，而第二层则是存储图像金字塔中所有图像的vectors of keypoints
     * @param[out] allKeypoints 提取得到的所有特征点
     */    
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    

    /**
     * @brief 对于某一图层，分配其特征点，通过八叉树的方式
     * @param[in] vToDistributeKeys         等待分配的特征点
     * @param[in] minX                      分发的图像范围
     * @param[in] maxX                      分发的图像范围
     * @param[in] minY                      分发的图像范围
     * @param[in] maxY                      分发的图像范围
     * @param[in] nFeatures                 设定的、本图层中想要提取的特征点数目
     * @param[in] level                     要提取的图像所在的金字塔层
     * @return std::vector<cv::KeyPoint>        
     */
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

