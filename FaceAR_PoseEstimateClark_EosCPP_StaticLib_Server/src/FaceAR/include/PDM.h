///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __PDM_h_
#define __PDM_h_

#include "FaceARParameters.h"

using namespace cv;

namespace FaceARTracker
{
//===========================================================================
class PDM{
	public:    
		cv::Mat_<double> mean_shape;	
		cv::Mat_<double> princ_comp;	
		cv::Mat_<double> eigen_values;	

		PDM(){;}
		
		// A copy constructor
		PDM(const PDM& other){
			this->mean_shape = other.mean_shape.clone();
			this->princ_comp = other.princ_comp.clone();
			this->eigen_values = other.eigen_values.clone();
		}
			
		void Read(string location);
		inline int NumberOfPoints() const {return mean_shape.rows/3;}
		inline int NumberOfModes() const {return princ_comp.cols;}

        void Clamp(Mat_<float>& params_local, Vec6d& params_global, const FaceARParameters& params);
		void CalcShape3D(Mat_<double>& out_shape, const Mat_<double>& params_local) const;
		void CalcShape2D(Mat_<double>& out_shape, const Mat_<double>& params_local, const Vec6d& params_global) const;
		void CalcParams(Vec6d& out_params_global, const Rect_<double>& bounding_box, const Mat_<double>& params_local, const Vec3d rotation = Vec3d(0.0));
		void CalcParams(Vec6d& out_params_global, const Mat_<double>& out_params_local, const Mat_<double>& landmark_locations, const Vec3d rotation = Vec3d(0.0));
		void CalcBoundingBox(Rect& out_bounding_box, const Vec6d& params_global, const Mat_<double>& params_local);
		void ComputeRigidJacobian(const Mat_<float>& params_local, const Vec6d& params_global, Mat_<float> &Jacob, const Mat_<float> W, cv::Mat_<float> &Jacob_t_w);
		void ComputeJacobian(const Mat_<float>& params_local, const Vec6d& params_global, Mat_<float> &Jacobian, const Mat_<float> W, cv::Mat_<float> &Jacob_t_w);
		void UpdateModelParameters(const Mat_<float>& delta_p, Mat_<float>& params_local, Vec6d& params_global);

  };
  //===========================================================================
}
#endif

