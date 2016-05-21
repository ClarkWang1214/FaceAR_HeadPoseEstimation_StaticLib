///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "CCNF_patch_expert.h"
#include "FaceAR_utils.h"

using namespace FaceARTracker;

void CCNF_patch_expert::ComputeSigmas(std::vector<Mat_<float> > sigma_components, int window_size)
{
	for(size_t i=0; i < window_sizes.size(); ++i)
	{
		if( window_sizes[i] == window_size)
			return;
	}
	int n_betas = sigma_components.size();
	float sum_alphas = 0;
	int n_alphas = this->neurons.size();
	for(int a = 0; a < n_alphas; ++a)
	{
		sum_alphas = sum_alphas + this->neurons[a].alpha;
	}

	Mat_<float> q1 = sum_alphas * Mat_<float>::eye(window_size*window_size, window_size*window_size);

	Mat_<float> q2 = Mat_<float>::zeros(window_size*window_size, window_size*window_size);
	for (int b=0; b < n_betas; ++b)
	{			
		q2 = q2 + ((float)this->betas[b]) * sigma_components[b];
	}

	Mat_<float> SigmaInv = 2 * (q1 + q2);
	
	Mat Sigma_f;
	invert(SigmaInv, Sigma_f, DECOMP_CHOLESKY);

	window_sizes.push_back(window_size);
	Sigmas.push_back(Sigma_f);

}

//===========================================================================
void CCNF_neuron::Read(ifstream &stream)
{
	int read_type;
	stream.read ((char*)&read_type, 4);
	assert(read_type == 2);

	stream.read ((char*)&neuron_type, 4);
	stream.read ((char*)&norm_weights, 8);
	stream.read ((char*)&bias, 8);
	stream.read ((char*)&alpha, 8);
	
    FaceARTracker::ReadMatBin(stream, weights);

}

//===========================================================================
void CCNF_neuron::Response(Mat_<float> &im, Mat_<double> &im_dft, Mat &integral_img, Mat &integral_img_sq, Mat_<float> &resp)
{

	int h = im.rows - weights.rows + 1;
	int w = im.cols - weights.cols + 1;

	Mat_<float> I;    

	if(neuron_type == 3)
	{
		Scalar mean;
		Scalar std;
		Mat_<uchar> mask = im > 0;
		meanStdDev(im, mean, std, mask);
		if(std[0] != 0)
		{
			I = (im - mean[0]) / std[0];
		}
		else
		{
			I = (im - mean[0]);
		}

		I.setTo(0, mask == 0);
	}
	else
	{
		if(neuron_type == 0)
		{
			I = im;
		}
		else
		{
			printf("ERROR(%s,%d): Unsupported patch type %d!\n", __FILE__,__LINE__,neuron_type);
			abort();
		}
	}
  
	if(resp.empty())
	{		
		resp.create(h, w);
	}

	if(neuron_type == 3)
	{
		matchTemplate_m(I, im_dft, integral_img, integral_img_sq, weights, weights_dfts, resp, CV_TM_CCOEFF); // the linear multiplication, efficient calc of response
	}
	else
	{
		matchTemplate_m(I, im_dft, integral_img, integral_img_sq, weights, weights_dfts, resp, CV_TM_CCOEFF_NORMED); // the linear multiplication, efficient calc of response
	}

	MatIterator_<float> p = resp.begin();

    MatIterator_<float> q1 = resp.begin();
	MatIterator_<float> q2 = resp.end();
	while(q1 != q2)
	{
		*p++ = (2 * alpha) * 1.0 /(1.0 + exp( -(*q1++ * norm_weights + bias )));
	}

}

//===========================================================================
void CCNF_patch_expert::Read(ifstream &stream, std::vector<int> window_sizes, std::vector<std::vector<Mat_<float> > > sigma_components)
{
	int read_type;

	stream.read ((char*)&read_type, 4);
	assert(read_type == 5);

	int num_neurons;
	stream.read ((char*)&width, 4);
	stream.read ((char*)&height, 4);
	stream.read ((char*)&num_neurons, 4);

	if(num_neurons == 0)
	{
		stream.read ((char*)&num_neurons, 4);
		return;
	}

	neurons.resize(num_neurons);
	for(int i = 0; i < num_neurons; i++)
		neurons[i].Read(stream);

	int n_sigmas = window_sizes.size();

	int n_betas = 0;

	if(n_sigmas > 0)
	{
		n_betas = sigma_components[0].size();

		betas.resize(n_betas);

		for (int i=0; i < n_betas;  ++i)
		{
			stream.read ((char*)&betas[i], 8);
		}
	}	

	stream.read ((char*)&patch_confidence, 8);

}

//===========================================================================
void CCNF_patch_expert::Response(Mat_<float> &area_of_interest, Mat_<float> &response)
{
	
	int response_height = area_of_interest.rows - height + 1;
	int response_width = area_of_interest.cols - width + 1;

	if(response.rows != response_height || response.cols != response_width)
	{
		response.create(response_height, response_width);
	}
		
	response.setTo(0);
	
	Mat_<double> area_of_interest_dft;
	Mat integral_image, integral_image_sq;
	
	Mat_<float> neuron_response;

	for(size_t i = 0; i < neurons.size(); i++)
	{		
		if(neurons[i].alpha > 1e-4)
		{
			neurons[i].Response(area_of_interest, area_of_interest_dft, integral_image, integral_image_sq, neuron_response);
			response = response + neuron_response;						
		}
	}

	int s_to_use = -1;

	for(size_t i=0; i < window_sizes.size(); ++i)
	{
		if(window_sizes[i] == response_height)
		{
			s_to_use = i;			
			break;
		}
	}

	Mat_<float> resp_vec_f = response.reshape(1, response_height * response_width);

	Mat out = Sigmas[s_to_use] * resp_vec_f;
	
	response = out.reshape(1, response_height);

	double min;

	minMaxIdx(response, &min, 0);
	if(min < 0)
	{
		response = response - min;
	}

}

