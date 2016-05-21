///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __FACEAR_h_
#define __FACEAR_h_

#include "PDM.h"
#include "Patch_experts.h"
#include "DetectionValidator.h"
#include "FaceARParameters.h"

using namespace std;
using namespace cv;

namespace FaceARTracker
{

class FaceAR{

public:

//    ////////////////////////////////////////////////////////////////////////////
//    /// TODO : TFY
//    cv::Rect_<double> search_rect;
    Mat_<double> pre_landmarks;
//    ////////////////////////////////////////////////////////////////////////////

	//===========================================================================
    PDM					pdm;
	Patch_experts		patch_experts;
	Mat_<double>    params_local;
    Vec6d           params_global;
    vector<FaceAR>						hierarchical_models;
	vector<string>					hierarchical_model_names;
    vector<vector<pair<int,int> > >	hierarchical_mapping;
    vector<FaceARParameters>			hierarchical_params;

	// Haar cascade classifier for face detection
	CascadeClassifier face_detector_HAAR;
	string			  face_detector_location;

	// A HOG SVM-struct based face detector
	dlib::frontal_face_detector face_detector_HOG;

	DetectionValidator	landmark_validator; 
	bool				detection_success; 
	bool				tracking_initialised;
	double				detection_certainty; 
	vector<Mat_<int> >	triangulations;
	
	//===========================================================================
	// Lastly detect 2D model shape [x1,x2,...xn,y1,...yn]
    Mat_<double>		detected_landmarks;
	double				model_likelihood;
	Mat_<double>		landmark_likelihoods;
	int failures_in_a_row;
	Mat_<uchar> face_template;
	cv::Point_<double> preference_det;

	// A default constructor
    FaceAR();
    FaceAR(string fname);
    FaceAR(const FaceAR& other);

    FaceAR & operator= (const FaceAR& other);

    ~FaceAR(){}

	// Move constructor
    FaceAR(const FaceAR && other);
    FaceAR & operator= (const FaceAR && other);

    bool DetectLandmarks(const Mat_<uchar> &image, const Mat_<float> &depth, FaceARParameters& params);
	Mat_<double> GetShape(double fx, double fy, double cx, double cy) const;
	Rect_<double> GetBoundingBox() const;

	void Reset();
	void Reset(double x, double y);
	void Read(string name);

	// Helper reading function
    void Read_FaceAR(string facear_location);
	
private:

	map<int, Mat_<float> >		kde_resp_precalc; 
    bool Fit(const Mat_<uchar>& intensity_image, const Mat_<float>& depth_image, const std::vector<int>& window_sizes, const FaceARParameters& parameters);
	void NonVectorisedMeanShift_precalc_kde(Mat_<float>& out_mean_shifts, const vector<Mat_<float> >& patch_expert_responses, const Mat_<float> &dxs, const Mat_<float> &dys, int resp_size, float a, int scale, int view_id, map<int, Mat_<float> >& mean_shifts);
    double NU_RLMS(Vec6d& final_global, Mat_<double>& final_local, const vector<Mat_<float> >& patch_expert_responses, const Vec6d& initial_global, const Mat_<double>& initial_local,
                  const Mat_<double>& base_shape, const Matx22d& sim_img_to_ref, const Matx22f& sim_ref_to_img, int resp_size, int view_idx, bool rigid, int scale, Mat_<double>& landmark_lhoods, const FaceARParameters& parameters);
	bool RemoveBackground(Mat_<float>& out_depth_image, const Mat_<float>& depth_image);
    void GetWeightMatrix(Mat_<float>& WeightMatrix, int scale, int view_id, const FaceARParameters& parameters);

	// Mean shift computation	
	void NonVectorisedMeanShift(Mat_<double>& out_mean_shifts, const vector<Mat_<float> >& patch_expert_responses, const Mat_<double> &dxs, const Mat_<double> &dys, int resp_size, double a, int scale, int view_id);
	void VectorisedMeanShift(Mat_<double>& meanShifts, const vector<Mat_<float> >& patch_expert_responses, const Mat_<double> &iis, const Mat_<double> &jjs, const Mat_<double> &dxs, const Mat_<double> &dys, const Size patchSize, double sigma, int scale, int view_id);		

  };
  //===========================================================================
}
#endif

