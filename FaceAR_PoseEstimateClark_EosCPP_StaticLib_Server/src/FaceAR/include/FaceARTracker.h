///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __FACEAR_TRACKER_h_
#define __FACEAR_TRACKER_h_

#include <FaceARParameters.h>
#include <FaceAR_utils.h>
#include <FaceAR.h>

using namespace std;
using namespace cv;

namespace FaceARTracker
{
    bool DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, FaceAR& facear_model, FaceARParameters& params);
    bool DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, const Mat_<float> &depth_image, FaceAR& facear_model, FaceARParameters& params);
    bool DetectLandmarksInVideo_Clark(const Mat &original_image, const Mat_<uchar> &grayscale_image, const Mat_<float> &depth_image, FaceAR& facear_model, FaceARParameters& params, Mat &saved_image);

    bool DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, const Rect_<double> bounding_box, FaceAR& facear_model, FaceARParameters& params);
    bool DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, const Mat_<float> &depth_image, const Rect_<double> bounding_box, FaceAR& facear_model, FaceARParameters& params);

    //================================================================================================================
    bool DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, FaceAR& facear_model, FaceARParameters& params);
    bool DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, const Rect_<double> bounding_box, FaceAR& facear_model, FaceARParameters& params);

    //================================================
    bool DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, const Mat_<float> depth_image, FaceAR& facear_model, FaceARParameters& params);
    bool DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, const Mat_<float> depth_image, const Rect_<double> bounding_box, FaceAR& facear_model, FaceARParameters& params);

    //================================================================
    Vec6d GetPoseCamera(const FaceAR& facear_model, double fx, double fy, double cx, double cy, const FaceARParameters& params);
    Vec6d GetPoseCameraPlane(const FaceAR& facear_model, double fx, double fy, double cx, double cy, const FaceARParameters& params);
    Vec6d GetCorrectedPoseCamera(const FaceAR& facear_model, double fx, double fy, double cx, double cy, const FaceARParameters& params);
    Vec6d GetCorrectedPoseCameraPlane(const FaceAR& facear_model, double fx, double fy, double cx, double cy, const FaceARParameters& params);

    //===========================================================================

}
#endif

