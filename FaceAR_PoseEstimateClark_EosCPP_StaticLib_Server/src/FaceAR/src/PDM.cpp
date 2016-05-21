///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include <PDM.h>
#include <FaceAR_utils.h>

using namespace FaceARTracker;
//===========================================================================
void Orthonormalise(cv::Matx33d &R)
{

	cv::SVD svd(R,cv::SVD::MODIFY_A);
	cv::Mat_<double> X = svd.u*svd.vt;
  
	cv::Mat_<double> W = Mat_<double>::eye(3,3); 
	double d = determinant(X);
	W(2,2) = determinant(X);
	Mat Rt = svd.u*W*svd.vt;

	Rt.copyTo(R);

}

//===========================================================================
void PDM::Clamp(cv::Mat_<float>& local_params, Vec6d& params_global, const FaceARParameters& parameters)
{
	double n_sigmas = 3;
	cv::MatConstIterator_<double> e_it  = this->eigen_values.begin();
	cv::MatIterator_<float> p_it =  local_params.begin();

	double v;
	for(; p_it != local_params.end(); ++p_it, ++e_it)
	{
		v = n_sigmas*sqrt(*e_it);
		if(fabs(*p_it) > v)
		{
			if(*p_it > 0.0)
			{
				*p_it=v;
			}
			else
			{
				*p_it=-v;
			}
		}
	}
	if(parameters.limit_pose)
	{
		if(params_global[1] > M_PI / 2)
			params_global[1] = M_PI/2;
		if(params_global[1] < -M_PI / 2)
			params_global[1] = -M_PI/2;
		if(params_global[2] > M_PI / 2)
			params_global[2] = M_PI/2;
		if(params_global[2] < -M_PI / 2)
			params_global[2] = -M_PI/2;
		if(params_global[3] > M_PI / 2)
			params_global[3] = M_PI/2;
		if(params_global[3] < -M_PI / 2)
			params_global[3] = -M_PI/2;
	}
	

}
//===========================================================================
void PDM::CalcShape3D(cv::Mat_<double>& out_shape, const Mat_<double>& p_local) const
{
	out_shape.create(mean_shape.rows, mean_shape.cols);
	out_shape = mean_shape + princ_comp*p_local;
}

//===========================================================================
void PDM::CalcShape2D(Mat_<double>& out_shape, const Mat_<double>& params_local, const Vec6d& params_global) const
{

	int n = this->NumberOfPoints();

    double s = params_global[0];
    double tx = params_global[4];
    double ty = params_global[5];

	Vec3d euler(params_global[1], params_global[2], params_global[3]);
	Matx33d currRot = Euler2RotationMatrix(euler);
	
	Mat_<double> Shape_3D = mean_shape + princ_comp * params_local;
	if((out_shape.rows != mean_shape.rows) || (out_shape.cols = 1))
	{
		out_shape.create(2*n,1);
	}
	for(int i = 0; i < n; i++)
	{
		out_shape.at<double>(i  ,0) = s * ( currRot(0,0) * Shape_3D.at<double>(i, 0) + currRot(0,1) * Shape_3D.at<double>(i+n  ,0) + currRot(0,2) * Shape_3D.at<double>(i+n*2,0) ) + tx;
		out_shape.at<double>(i+n,0) = s * ( currRot(1,0) * Shape_3D.at<double>(i, 0) + currRot(1,1) * Shape_3D.at<double>(i+n  ,0) + currRot(1,2) * Shape_3D.at<double>(i+n*2,0) ) + ty;
	}
}

//===========================================================================
void PDM::CalcParams(Vec6d& out_params_global, const Rect_<double>& bounding_box, const Mat_<double>& params_local, const Vec3d rotation)
{
	Mat_<double> current_shape(mean_shape.size());

	CalcShape3D(current_shape, params_local);
	Matx33d rotation_matrix = Euler2RotationMatrix(rotation);

	Mat_<double> reshaped = current_shape.reshape(1, 3);

	Mat rotated_shape = (Mat(rotation_matrix) * reshaped);
	double min_x;
	double max_x;
	cv::minMaxLoc(rotated_shape.row(0), &min_x, &max_x);	

	double min_y;
	double max_y;
	cv::minMaxLoc(rotated_shape.row(1), &min_y, &max_y);

	double width = abs(min_x - max_x);
	double height = abs(min_y - max_y);

	double scaling = ((bounding_box.width / width) + (bounding_box.height / height)) / 2;
	double tx = bounding_box.x + bounding_box.width / 2;
	double ty = bounding_box.y + bounding_box.height / 2;
	tx = tx - scaling * (min_x + max_x)/2;
    ty = ty - scaling * (min_y + max_y)/2;

	out_params_global = Vec6d(scaling, rotation[0], rotation[1], rotation[2], tx, ty);
}

//===========================================================================
void PDM::CalcBoundingBox(Rect& out_bounding_box, const Vec6d& params_global, const Mat_<double>& params_local)
{
	Mat_<double> current_shape;
	CalcShape2D(current_shape, params_local, params_global);
	double min_x;
	double max_x;
	cv::minMaxLoc(current_shape(Rect(0, 0, 1, this->NumberOfPoints())), &min_x, &max_x);

	double min_y;
	double max_y;
	cv::minMaxLoc(current_shape(Rect(0, this->NumberOfPoints(), 1, this->NumberOfPoints())), &min_y, &max_y);

	double width = abs(min_x - max_x);
	double height = abs(min_y - max_y);

	out_bounding_box = Rect((int)min_x, (int)min_y, (int)width, (int)height);
}

//=========================================================================== 
void PDM::ComputeRigidJacobian(const Mat_<float>& p_local, const Vec6d& params_global, cv::Mat_<float> &Jacob, const Mat_<float> W, cv::Mat_<float> &Jacob_t_w)
{
	int n = this->NumberOfPoints();
  
	Jacob.create(n * 2, 6);

	float X,Y,Z;

	float s = (float)params_global[0];
  	
	Mat_<double> shape_3D_d;
	Mat_<double> p_local_d;
	p_local.convertTo(p_local_d, CV_64F);
	this->CalcShape3D(shape_3D_d, p_local_d);
	
	Mat_<float> shape_3D;
	shape_3D_d.convertTo(shape_3D, CV_32F);

	Vec3d euler(params_global[1], params_global[2], params_global[3]);
	Matx33d currRot = Euler2RotationMatrix(euler);
	
	float r11 = (float) currRot(0,0);
	float r12 = (float) currRot(0,1);
	float r13 = (float) currRot(0,2);
	float r21 = (float) currRot(1,0);
	float r22 = (float) currRot(1,1);
	float r23 = (float) currRot(1,2);
	float r31 = (float) currRot(2,0);
	float r32 = (float) currRot(2,1);
	float r33 = (float) currRot(2,2);

	cv::MatIterator_<float> Jx = Jacob.begin();
	cv::MatIterator_<float> Jy = Jx + n * 6;

	for(int i = 0; i < n; i++)
	{
    
		X = shape_3D.at<float>(i,0);
		Y = shape_3D.at<float>(i+n,0);
		Z = shape_3D.at<float>(i+n*2,0);    

		*Jx++ =  (X  * r11 + Y * r12 + Z * r13);
		*Jy++ =  (X  * r21 + Y * r22 + Z * r23);
		
		*Jx++ = (s * (Y * r13 - Z * r12) );
		*Jy++ = (s * (Y * r23 - Z * r22) );
		*Jx++ = (-s * (X * r13 - Z * r11));
		*Jy++ = (-s * (X * r23 - Z * r21));
		*Jx++ = (s * (X * r12 - Y * r11) );
		*Jy++ = (s * (X * r22 - Y * r21) );

		*Jx++ = 1.0f;
		*Jy++ = 0.0f;
		*Jx++ = 0.0f;
		*Jy++ = 1.0f;

	}

	Mat Jacob_w = Mat::zeros(Jacob.rows, Jacob.cols, Jacob.type());
	
	Jx =  Jacob.begin();
	Jy =  Jx + n*6;

	cv::MatIterator_<float> Jx_w =  Jacob_w.begin<float>();
	cv::MatIterator_<float> Jy_w =  Jx_w + n*6;

	for(int i = 0; i < n; i++)
	{
		float w_x = W.at<float>(i, i);
		float w_y = W.at<float>(i+n, i+n);

		for(int j = 0; j < Jacob.cols; ++j)
		{
			*Jx_w++ = *Jx++ * w_x;
			*Jy_w++ = *Jy++ * w_y;
		}		
	}

	Jacob_t_w = Jacob_w.t();
}

//===========================================================================
void PDM::ComputeJacobian(const Mat_<float>& params_local, const Vec6d& params_global, Mat_<float> &Jacobian, const Mat_<float> W, cv::Mat_<float> &Jacob_t_w)
{ 
	int n = this->NumberOfPoints();
	int m = this->NumberOfModes();

	Jacobian.create(n * 2, 6 + m);
	
	float X,Y,Z;
	
	float s = (float) params_global[0];
  	
	Mat_<double> shape_3D_d;
	Mat_<double> p_local_d;
	params_local.convertTo(p_local_d, CV_64F);
	this->CalcShape3D(shape_3D_d, p_local_d);
	
	Mat_<float> shape_3D;
	shape_3D_d.convertTo(shape_3D, CV_32F);

	Vec3d euler(params_global[1], params_global[2], params_global[3]);
	Matx33d currRot = Euler2RotationMatrix(euler);
	
	float r11 = (float) currRot(0,0);
	float r12 = (float) currRot(0,1);
	float r13 = (float) currRot(0,2);
	float r21 = (float) currRot(1,0);
	float r22 = (float) currRot(1,1);
	float r23 = (float) currRot(1,2);
	float r31 = (float) currRot(2,0);
	float r32 = (float) currRot(2,1);
	float r33 = (float) currRot(2,2);

	cv::MatIterator_<float> Jx =  Jacobian.begin();
	cv::MatIterator_<float> Jy =  Jx + n * (6 + m);
	cv::MatConstIterator_<double> Vx =  this->princ_comp.begin();
	cv::MatConstIterator_<double> Vy =  Vx + n*m;
	cv::MatConstIterator_<double> Vz =  Vy + n*m;

	for(int i = 0; i < n; i++)
	{
    
		X = shape_3D.at<float>(i,0);
		Y = shape_3D.at<float>(i+n,0);
		Z = shape_3D.at<float>(i+n*2,0);    
    
		*Jx++ = (X  * r11 + Y * r12 + Z * r13);
		*Jy++ = (X  * r21 + Y * r22 + Z * r23);

		*Jx++ = (s * (Y * r13 - Z * r12) );
		*Jy++ = (s * (Y * r23 - Z * r22) );
		*Jx++ = (-s * (X * r13 - Z * r11));
		*Jy++ = (-s * (X * r23 - Z * r21));
		*Jx++ = (s * (X * r12 - Y * r11) );
		*Jy++ = (s * (X * r22 - Y * r21) );

		*Jx++ = 1.0f;
		*Jy++ = 0.0f;
		*Jx++ = 0.0f;
		*Jy++ = 1.0f;

		for(int j = 0; j < m; j++,++Vx,++Vy,++Vz)
		{
			*Jx++ = (float) ( s*(r11*(*Vx) + r12*(*Vy) + r13*(*Vz)) );
			*Jy++ = (float) ( s*(r21*(*Vx) + r22*(*Vy) + r23*(*Vz)) );
		}
	}	

	Mat Jacob_w = Jacobian.clone();
	
	if(cv::trace(W)[0] != W.rows) 
	{
		Jx =  Jacobian.begin();
		Jy =  Jx + n*(6+m);

		cv::MatIterator_<float> Jx_w =  Jacob_w.begin<float>();
		cv::MatIterator_<float> Jy_w =  Jx_w + n*(6+m);

		for(int i = 0; i < n; i++)
		{
			float w_x = W.at<float>(i, i);
			float w_y = W.at<float>(i+n, i+n);

			for(int j = 0; j < Jacobian.cols; ++j)
			{
				*Jx_w++ = *Jx++ * w_x;
				*Jy_w++ = *Jy++ * w_y;
			}
		}
	}
	Jacob_t_w = Jacob_w.t();

}

//===========================================================================
void PDM::UpdateModelParameters(const Mat_<float>& delta_p, Mat_<float>& params_local, Vec6d& params_global)
{
	params_global[0] += (double)delta_p.at<float>(0,0);
	params_global[4] += (double)delta_p.at<float>(4,0);
	params_global[5] += (double)delta_p.at<float>(5,0);

	Vec3d eulerGlobal(params_global[1], params_global[2], params_global[3]);
	Matx33d R1 = Euler2RotationMatrix(eulerGlobal);

	Matx33d R2 = Matx33d::eye();

	R2(1,2) = -1.0*(R2(2,1) = (double)delta_p.at<float>(1,0));
	R2(2,0) = -1.0*(R2(0,2) = (double)delta_p.at<float>(2,0));
	R2(0,1) = -1.0*(R2(1,0) = (double)delta_p.at<float>(3,0));
	
	Orthonormalise(R2);

	Matx33d R3 = R1 *R2;

	Vec3d axis_angle = RotationMatrix2AxisAngle(R3);	
	Vec3d euler = AxisAngle2Euler(axis_angle);

	params_global[1] = euler[0];
	params_global[2] = euler[1];
	params_global[3] = euler[2];

	if(delta_p.rows > 6)
	{
		params_local = params_local + delta_p(cv::Rect(0,6,1, this->NumberOfModes()));
	}

}

void PDM::CalcParams(Vec6d& out_params_global, const Mat_<double>& out_params_local, const Mat_<double>& landmark_locations, const Vec3d rotation)
{
		
	int m = this->NumberOfModes();
	int n = this->NumberOfPoints();

	Mat_<int> visi_ind_2D(n * 2, 1, 1);
	Mat_<int> visi_ind_3D(3 * n , 1, 1);

	for(size_t i = 0; i < n; ++i)
	{
		if(landmark_locations.at<double>(i) == 0)
		{
			visi_ind_2D.at<int>(i) = 0;
			visi_ind_2D.at<int>(i+n) = 0;
			visi_ind_3D.at<int>(i) = 0;
			visi_ind_3D.at<int>(i+n) = 0;
			visi_ind_3D.at<int>(i+2*n) = 0;
		}
	}

	Mat_<double> M(0, mean_shape.cols, 0.0);
	Mat_<double> V(0, princ_comp.cols, 0.0);

	for(size_t i = 0; i < n * 3; ++i)
	{
		if(visi_ind_3D.at<int>(i) == 1)
		{
			cv::vconcat(M, this->mean_shape.row(i), M);
			cv::vconcat(V, this->princ_comp.row(i), V);
		}
	}

	Mat_<double> m_old = this->mean_shape.clone();
	Mat_<double> v_old = this->princ_comp.clone();

	this->mean_shape = M;
	this->princ_comp = V;
	n  = M.rows / 3;
	Mat_<double> landmark_locs_vis(n*2, 1, 0.0);
	int k = 0;
	for(size_t i = 0; i < visi_ind_2D.rows; ++i)
	{
		if(visi_ind_2D.at<int>(i) == 1)
		{
			landmark_locs_vis.at<double>(k) = landmark_locations.at<double>(i);
			k++;
		}		
	}

	double min_x;
	double max_x;
	cv::minMaxLoc(landmark_locations(Rect(0, 0, 1, this->NumberOfPoints())), &min_x, &max_x);

	double min_y;
	double max_y;
	cv::minMaxLoc(landmark_locations(Rect(0, this->NumberOfPoints(), 1, this->NumberOfPoints())), &min_y, &max_y);

	double width = abs(min_x - max_x);
	double height = abs(min_y - max_y);

	Rect model_bbox;
	CalcBoundingBox(model_bbox, Vec6d(1.0, 0.0, 0.0, 0.0, 0.0, 0.0), cv::Mat_<double>(this->NumberOfModes(), 1, 0.0));

	Rect bbox((int)min_x, (int)min_y, (int)width, (int)height);

	double scaling = ((width / model_bbox.width) + (height / model_bbox.height)) / 2;
        
    Vec3d rotation_init = rotation;
	Matx33d R = Euler2RotationMatrix(rotation_init);
    Vec2d translation((min_x + max_x) / 2.0, (min_y + max_y) / 2.0);
    
	Mat_<float> loc_params(this->NumberOfModes(),1, 0.0);
	Vec6d glob_params(scaling, rotation_init[0], rotation_init[1], rotation_init[2], translation[0], translation[1]);

	Mat_<double> loc_params_d;
	loc_params.convertTo(loc_params_d, CV_64F);
	Mat_<double> shape_3D = M + V * loc_params_d;

	Mat_<double> curr_shape(2*n, 1);
	
	for(int i = 0; i < n; i++)
	{
		curr_shape.at<double>(i  ,0) = scaling * ( R(0,0) * shape_3D.at<double>(i, 0) + R(0,1) * shape_3D.at<double>(i+n  ,0) + R(0,2) * shape_3D.at<double>(i+n*2,0) ) + translation[0];
		curr_shape.at<double>(i+n,0) = scaling * ( R(1,0) * shape_3D.at<double>(i, 0) + R(1,1) * shape_3D.at<double>(i+n  ,0) + R(1,2) * shape_3D.at<double>(i+n*2,0) ) + translation[1];
	}
		    
    double currError = cv::norm(curr_shape - landmark_locs_vis);

	Mat_<float> regularisations = Mat_<double>::zeros(1, 6 + m);

	double reg_factor = 1;

	Mat(reg_factor / this->eigen_values).copyTo(regularisations(Rect(6, 0, m, 1)));
	Mat_<double> regTerm_d = Mat::diag(regularisations.t());
	regTerm_d.convertTo(regularisations, CV_32F);    
    
	Mat_<float> WeightMatrix = Mat_<float>::eye(n*2, n*2);

    for (size_t i = 0; i < 1000; ++i)
	{
		Mat_<double> loc_params_d;
		loc_params.convertTo(loc_params_d, CV_64F);
		shape_3D = M + V * loc_params_d;

		shape_3D = shape_3D.reshape(1, 3);

		Matx23d R_2D(R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2)); 

		Mat_<double> curr_shape_2D = scaling * shape_3D.t() * Mat(R_2D).t();
        curr_shape_2D.col(0) = curr_shape_2D.col(0) + translation(0);
		curr_shape_2D.col(1) = curr_shape_2D.col(1) + translation(1);

		curr_shape_2D = Mat(curr_shape_2D.t()).reshape(1, n * 2);
		
		Mat_<float> error_resid;
		Mat(landmark_locs_vis - curr_shape_2D).convertTo(error_resid, CV_32F);
        
		Mat_<float> J, J_w_t;
		this->ComputeJacobian(loc_params, glob_params, J, WeightMatrix, J_w_t);
        
		Mat_<float> J_w_t_m = J_w_t * error_resid;

		J_w_t_m(Rect(0,6,1, m)) = J_w_t_m(Rect(0,6,1, m)) - regularisations(Rect(6,6, m, m)) * loc_params;

		Mat_<float> Hessian = J_w_t * J;

		Hessian = Hessian + regularisations;

		Mat_<float> param_update;
		solve(Hessian, J_w_t_m, param_update, CV_CHOLESKY);

		param_update = 0.5 * param_update;

		UpdateModelParameters(param_update, loc_params, glob_params);		
        
        scaling = glob_params[0];
		rotation_init[0] = glob_params[1];
		rotation_init[1] = glob_params[2];
		rotation_init[2] = glob_params[3];

		translation[0] = glob_params[4];
		translation[1] = glob_params[5];
        
		R = Euler2RotationMatrix(rotation_init);

		R_2D(0,0) = R(0,0);R_2D(0,1) = R(0,1); R_2D(0,2) = R(0,2);
		R_2D(1,0) = R(1,0);R_2D(1,1) = R(1,1); R_2D(1,2) = R(1,2); 

		curr_shape_2D = scaling * shape_3D.t() * Mat(R_2D).t();
        curr_shape_2D.col(0) = curr_shape_2D.col(0) + translation(0);
		curr_shape_2D.col(1) = curr_shape_2D.col(1) + translation(1);

		curr_shape_2D = Mat(curr_shape_2D.t()).reshape(1, n * 2);
        
        double error = cv::norm(curr_shape_2D - landmark_locs_vis);
        
        if(0.999 * currError < error)
		{
            break;
		}
        
        currError = error;
        
	}

	out_params_global = glob_params;
	loc_params.convertTo(out_params_local, CV_64F);
    	
	this->mean_shape = m_old;
	this->princ_comp = v_old;


}

void PDM::Read(string location)
{
  	
	ifstream pdmLoc(location, ios_base::in);

    FaceARTracker::SkipComments(pdmLoc);

    FaceARTracker::ReadMat(pdmLoc,mean_shape);
    //// TODO
    //std::cout << mean_shape.size() << std::endl;
	
    FaceARTracker::SkipComments(pdmLoc);

    FaceARTracker::ReadMat(pdmLoc,princ_comp);
	
    FaceARTracker::SkipComments(pdmLoc);

    FaceARTracker::ReadMat(pdmLoc,eigen_values);

}

