/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class Initializer
{
    typedef pair<int,int> Match;

public:

    // Fix the reference frame
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // 两种方法
    // 计算 基础矩阵 F 和 单应矩阵 H       2D-2D点对映射关系
    // Xc = H * Xr               p2转置 * F * p1 = 0
    // Selects a model and tries to recover the motion and the structure from motion
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);


private:
    // 计算单应矩阵 H,随机采样8点对,调用ComputeH21计算单应,CheckHomography 计算得分, 迭代求 得分最高的 H
    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    // 计算基础矩阵 F,随机采样8点对,调用 ComputeF21计算基础矩阵, CheckFundamental 计算得分, 迭代求 得分最高的 F
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);
    // 计算单应矩阵 H
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    // 计算基础矩阵 F
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    // 计算单应矩阵 得分
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
    // 计算 基础矩阵 得分
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);


    // 基础矩阵 恢复  R  t -----F ----> 本质矩阵E, 从本质矩阵恢复  R  t
    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
    // 单应矩阵 恢复  R  t 
    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
    // 三角化计算 深度 获取3D点坐标
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);
    
    // 标准化点坐标
    // 标准化矩阵 * 点坐标 = 标准化后的的坐标
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);
    // 从本质矩阵 恢复 R t
    // E = t^R = U C V   ,U & V 为正交矩阵, C 为奇异值矩阵, C =  diag(1, 1, 0)
    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference to Current
    vector<Match> mvMatches12;
    vector<bool> mvbMatched1;

    // Calibration
    cv::Mat mK;

    // Standard Deviation and Variance
    float mSigma, mSigma2;

    // Ransac max iterations
    int mMaxIterations;

    // Ransac sets
    vector<vector<size_t> > mvSets;   

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
