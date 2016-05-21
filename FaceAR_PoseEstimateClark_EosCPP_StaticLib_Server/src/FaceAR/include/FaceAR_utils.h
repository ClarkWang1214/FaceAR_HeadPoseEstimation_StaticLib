///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __FACEAR_UTILS_h_
#define __FACEAR_UTILS_h_

#include "FaceAR.h"

using namespace std;
using namespace cv;

namespace FaceARTracker
{
	//=============================================================================================
	void get_video_input_output_params(vector<string> &input_video_file, vector<string> &depth_dir,
		vector<string> &output_pose_file, vector<string> &output_video_file, vector<string> &output_landmark_files, vector<string> &output_3D_landmark_files, bool& camera_plane_pose, vector<string> &arguments);

    void get_camera_params(cv::String/*int*/ &device, float &fx, float &fy, float &cx, float &cy, vector<string> &arguments);

	void get_image_input_output_params(vector<string> &input_image_files, vector<string> &input_depth_files, vector<string> &output_feature_files, vector<string> &output_image_files,
		vector<Rect_<double>> &input_bounding_boxes, vector<string> &arguments);

	void matchTemplate_m( const Mat_<float>& input_img, Mat_<double>& img_dft, cv::Mat& _integral_img, cv::Mat& _integral_img_sq, const Mat_<float>&  templ, map<int, Mat_<double> >& templ_dfts, Mat_<float>& result, int method );

	Matx22d AlignShapesKabsch2D(const Mat_<double>& align_from, const Mat_<double>& align_to );

	Matx22d AlignShapesWithScale(cv::Mat_<double>& src, cv::Mat_<double> dst);
	void Project(Mat_<double>& dest, const Mat_<double>& mesh, double fx, double fy, double cx, double cy);
	void DrawBox(Mat image, Vec6d pose, Scalar color, int thickness, float fx, float fy, float cx, float cy);
    vector<pair<Point, Point> > CalculateBox(Vec6d pose, float fx, float fy, float cx, float cy);
    void DrawBox(vector<pair<Point, Point> > lines, Mat image, Scalar color, int thickness);

	vector<Point2d> CalculateLandmarks(const Mat_<double>& shape2D, Mat_<int>& visibilities);
	vector<Point2d> CalculateLandmarks(const Mat_<double>& shape2D);
    vector<Point2d> CalculateLandmarks(FaceAR& facear_model);
	void DrawLandmarks(cv::Mat img, vector<Point> landmarks);

	void Draw(cv::Mat img, const Mat_<double>& shape2D, Mat_<int>& visibilities);
	void Draw(cv::Mat img, const Mat_<double>& shape2D);
    void Draw(cv::Mat img, FaceAR& facear_model);

	Matx33d Euler2RotationMatrix(const Vec3d& eulerAngles);
	Vec3d RotationMatrix2Euler(const Matx33d& rotation_matrix);

	Vec3d Euler2AxisAngle(const Vec3d& euler);

	Vec3d AxisAngle2Euler(const Vec3d& axis_angle);

	Matx33d AxisAngle2RotationMatrix(const Vec3d& axis_angle);

	Vec3d RotationMatrix2AxisAngle(const Matx33d& rotation_matrix);

	// Face detection using Haar cascade classifier
	bool DetectFaces(vector<Rect_<double> >& o_regions, const Mat_<uchar>& intensity);
	bool DetectFaces(vector<Rect_<double> >& o_regions, const Mat_<uchar>& intensity, CascadeClassifier& classifier);
	bool DetectSingleFace(Rect_<double>& o_region, const Mat_<uchar>& intensity, CascadeClassifier& classifier, const cv::Point preference = Point(-1,-1));

	// Face detection using HOG-SVM classifier
	bool DetectFacesHOG(vector<Rect_<double> >& o_regions, const Mat_<uchar>& intensity, std::vector<double>& confidences);
	bool DetectFacesHOG(vector<Rect_<double> >& o_regions, const Mat_<uchar>& intensity, dlib::frontal_face_detector& classifier, std::vector<double>& confidences);
	bool DetectSingleFaceHOG(Rect_<double>& o_region, const Mat_<uchar>& intensity, dlib::frontal_face_detector& classifier, double& confidence, const cv::Point preference = Point(-1,-1));

	void ReadMatBin(std::ifstream& stream, Mat &output_mat);
	void ReadMat(std::ifstream& stream, Mat& output_matrix);

	void SkipComments(std::ifstream& stream);
}
#endif

