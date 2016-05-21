///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include "SVR_patch_expert.h"
#include "FaceAR_utils.h"

using namespace FaceARTracker;

//===========================================================================
void Grad(const cv::Mat& im, cv::Mat& grad)
{
	int x,y,h = im.rows,w = im.cols;
	float vx,vy;

	grad.create(im.size(), CV_32F);
	grad.setTo(0.0f);

	cv::MatIterator_<float> gp  = grad.begin<float>() + w+1;
	cv::MatConstIterator_<float> px1 = im.begin<float>()   + w+2;
	cv::MatConstIterator_<float> px2 = im.begin<float>()   + w;
	cv::MatConstIterator_<float> py1 = im.begin<float>()   + 2*w+1;
	cv::MatConstIterator_<float> py2 = im.begin<float>()   + 1;

	for(y = 1; y < h-1; y++)
	{ 
		for(x = 1; x < w-1; x++)
		{
			vx = *px1++ - *px2++;
			vy = *py1++ - *py2++;
			*gp++ = vx*vx + vy*vy;
		}
		px1 += 2;
		px2 += 2;
		py1 += 2;
		py2 += 2;
		gp += 2;
	}

}

//===========================================================================
void SVR_patch_expert::Read(ifstream &stream)
{
	int read_type;
	stream >> read_type;
	assert(read_type == 2);
  
	stream >> type >> confidence >> scaling >> bias;
    FaceARTracker::ReadMat(stream, weights);
	
	weights = weights.t();

}

//===========================================================================
void SVR_patch_expert::Response(const Mat_<float>& area_of_interest, Mat_<float>& response)
{

	int response_height = area_of_interest.rows - weights.rows + 1;
	int response_width = area_of_interest.cols - weights.cols + 1;

	cv::Mat_<float> normalised_area_of_interest;
  
	if(response.rows != response_height || response.cols != response_width)
	{
		response.create(response_height, response_width);
	}

	if(type == 0)
	{
		cv::Scalar mean;
		cv::Scalar std;

		cv::meanStdDev(area_of_interest, mean, std);
		if(std[0] == 0)
		{
			std[0] = 1;
		}
		normalised_area_of_interest = (area_of_interest - mean[0]) / std[0];
	}
	else if(type == 1)
	{
		Grad(area_of_interest, normalised_area_of_interest);
	}
  	else
	{
		printf("ERROR(%s,%d): Unsupported patch type %d!\n", __FILE__,__LINE__, type);
		abort();
	}
	
	Mat_<float> svr_response;
	Mat_<double> empty_matrix_0(0,0,0.0);
	Mat_<float> empty_matrix_1(0,0,0.0);
	Mat_<float> empty_matrix_2(0,0,0.0);
	matchTemplate_m(normalised_area_of_interest, empty_matrix_0, empty_matrix_1, empty_matrix_2, weights, weights_dfts, svr_response, CV_TM_CCOEFF_NORMED); 
	
	response.create(svr_response.size());
	MatIterator_<float> p = response.begin();

    cv::MatIterator_<float> q1 = svr_response.begin();
	cv::MatIterator_<float> q2 = svr_response.end();

	while(q1 != q2)
	{
		*p++ = 1.0/(1.0 + exp( -(*q1++ * scaling + bias )));
	}

}

void SVR_patch_expert::ResponseDepth(const Mat_<float>& area_of_interest, cv::Mat_<float> &response)
{
	int response_height = area_of_interest.rows - weights.rows + 1;
	int response_width = area_of_interest.cols - weights.cols + 1;
	Mat_<float> normalised_area_of_interest;
  
	if(response.rows != response_height || response.cols != response_width)
	{
		response.create(response_height, response_width);
	}

	if(type == 0)
	{
		cv::Scalar mean;
		cv::Scalar std;
		cv::Mat_<uchar> mask = area_of_interest > 0;
		cv::meanStdDev(area_of_interest, mean, std, mask);

		if(std[0] == 0)
		{
			std[0] = 1;
		}

		normalised_area_of_interest = (area_of_interest - mean[0]) / std[0];
		normalised_area_of_interest.setTo(0, mask == 0);
	}
	else
	{
		printf("ERROR(%s,%d): Unsupported patch type %d!\n", __FILE__,__LINE__,type);
		abort();
	}
  
	Mat_<float> svr_response;
	Mat_<double> empty_matrix_0(0,0,0.0);
	Mat_<float> empty_matrix_1(0,0,0.0);
	Mat_<float> empty_matrix_2(0,0,0.0);

	matchTemplate_m(normalised_area_of_interest, empty_matrix_0, empty_matrix_1, empty_matrix_2, weights, weights_dfts, svr_response, CV_TM_CCOEFF); 
	
	response.create(svr_response.size());
	MatIterator_<float> p = response.begin();

    cv::MatIterator_<float> q1 = svr_response.begin();
	cv::MatIterator_<float> q2 = svr_response.end();

	while(q1 != q2)
	{
		*p++ = 1.0/(1.0 + exp( -(*q1++ * scaling + bias )));
	}	
}

//===========================================================================
void Multi_SVR_patch_expert::Read(ifstream &stream)
{
	int type;
	stream >> type;
	assert(type == 3);
	int number_modalities;

	stream >> width >> height >> number_modalities;
	
	svr_patch_experts.resize(number_modalities);
	for(int i = 0; i < number_modalities; i++)
		svr_patch_experts[i].Read(stream);

}
//===========================================================================
void Multi_SVR_patch_expert::Response(const Mat_<float> &area_of_interest, Mat_<float> &response)
{
	
	int response_height = area_of_interest.rows - height + 1;
	int response_width = area_of_interest.cols - width + 1;

	if(response.rows != response_height || response.cols != response_width)
	{
		response.create(response_height, response_width);
	}

	if(svr_patch_experts.size() == 1)
	{
		svr_patch_experts[0].Response(area_of_interest, response);		
	}
	else
	{
		response.setTo(1.0);
		
		Mat_<float> modality_resp(response_height, response_width);

		for(size_t i = 0; i < svr_patch_experts.size(); i++)
		{			
			svr_patch_experts[i].Response(area_of_interest, modality_resp);			
			response = response.mul(modality_resp);	
		}	
		
	}

}

void Multi_SVR_patch_expert::ResponseDepth(const Mat_<float>& area_of_interest, Mat_<float>& response)
{
	int response_height = area_of_interest.rows - height + 1;
	int response_width = area_of_interest.cols - width + 1;

	if(response.rows != response_height || response.cols != response_width)
	{
		response.create(response_height, response_width);
	}
	
	svr_patch_experts[0].ResponseDepth(area_of_interest, response);
}
//===========================================================================

