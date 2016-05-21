#ifndef GETGOODFRAME_HPP
#define GETGOODFRAME_HPP

#include "FaceAR_core.h"
#include "polyfit.hpp"
#include "solve.h"
#include "PoseEstimation.h"

#define PI (22/7.0)
using namespace clarkPoseEsti;

class GoodFrame : public PoseEsti
{
public:
    GoodFrame(int &frame_id, cv::Vec6d &diff_curr_pre, cv::Mat &captured_image,
              cv::Mat_<double> &detected_landmarks, cv::Mat_<double> &shape_3D)
    {
        this->frame_id = frame_id;
//        this->RT_Mat = RT_Mat.clone();
//        this->facear_model = facear_model;
        this->diff_curr_pre = diff_curr_pre;
        this->captured_image = captured_image.clone();
        this->goodFrm = true;
        this->detected_landmarks = detected_landmarks.clone();
        this->shape_3D = shape_3D.clone();
        std::cout << "GoodFrame" << std::endl;
    }
    /// frame id
    int frame_id;
//    /// Rt Mat
//    cv::Mat RT_Mat;
//    cv::Mat getRT_Mat() {
//        return RT_Mat;
//    }
    ///
    cv::Vec6d diff_curr_pre;
    cv::Vec6d getDiff_curr_pre() {
        return diff_curr_pre;
    }
    ///
//    FaceARTracker::FaceAR facear_model;
    ///
    cv::Mat captured_image;
    cv::Mat getCapturesImage() {
        return captured_image;
    }
    /// 2d 3d points
    cv::Mat_<double> detected_landmarks;
    cv::Mat_<double> getDetected_landmarks() {
        return detected_landmarks;
    }
    cv::Mat_<double> shape_3D;
    cv::Mat_<double> getShap3D() {
       return shape_3D;
    }
    ////////
    bool goodFrm;

    ///////
    /// \brief getRT
    /// \param modelPoints_min
    /// \return
    /// solvePnP
    cv::Mat getRT(std::vector<cv::Point3f> &modelPoints_min);
    cv::Mat getRT(PoseEsti *poseEsti);
    cv::Mat getRT_Clark(PoseEsti* poseEsti);
};

cv::Mat_<double> getDetectedLandmarks3D(std::vector<GoodFrame> &AllFrame, int &frame_id);

cv::Mat_<double> getDetectedLandmarks2D(std::vector<GoodFrame> &AllFrame, int &frame_id);

cv::Mat getImage(std::vector<GoodFrame> &AllFrame, int &frame_id);


cv::Vec6d getDiffCurrPre(std::vector<GoodFrame> &AllFrame, int &frame_id);

double getDiffCurrPreId(std::vector<GoodFrame> &AllFrame, int &frame_id, int &id);

double getVariance(std::vector<double> &resultSet);

std::vector<int> getGoodFrame(std::vector<GoodFrame> &AllFrame, int &count);

///////
/// \brief getGoodFrameLess
/// \param AllFrame
/// \param count
/// \return
/// 检测到的所有的结果相对较好的帧中间隔存储
std::vector<int> getGoodFrameLess(std::vector<GoodFrame> &AllFrame, int &count);

std::pair<int, int> find_similar(std::vector<int> &good_frame, int &id);


/////
/// Keegan.Ren
/// 2016.4.5
/// fuck bug
bool getTempGoodFrame(std::vector<GoodFrame> &AllFrame, std::vector<int> &GoodFrameID, int &tempGoodFrameID, int &tempID);

////////
/// \brief getGoodFrameLessBeta
/// \param AllFrame
/// \param count
/// \return
/// 检测到相对较好的帧中，根据拟合的结果，对最左端到最右端的一段进行间隔存储
bool getGoodFrameLessBeta(std::vector<GoodFrame> &AllFrame, int &count, std::vector<int> &good_frame_less);

#endif // GETGOODFRAME_HPP
