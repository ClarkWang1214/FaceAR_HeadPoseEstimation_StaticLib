///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __CCNF_PATCH_EXPERT_h_
#define __CCNF_PATCH_EXPERT_h_

using namespace cv;

namespace FaceARTracker
{

//===========================================================================
/** 
	A single Neuron response
*/
class CCNF_neuron{

public:

	int     neuron_type; 
    double  norm_weights;
	double  bias;
	cv::Mat_<float> weights; 
	std::map<int, cv::Mat_<double> > weights_dfts;
	double alpha; 

	CCNF_neuron(){;}
	// Copy constructor
	CCNF_neuron(const CCNF_neuron& other): weights(other.weights.clone())
	{
		this->neuron_type = other.neuron_type;
		this->norm_weights = other.norm_weights;
		this->bias = other.bias;
		this->alpha = other.alpha;

		for(std::map<int, Mat_<double> >::const_iterator it = other.weights_dfts.begin(); it!= other.weights_dfts.end(); it++)
		{
			this->weights_dfts.insert(std::pair<int, Mat>(it->first, it->second.clone()));
		}
	}

	void Read(std::ifstream &stream);
	void Response(Mat_<float> &im, Mat_<double> &im_dft, Mat &integral_img, Mat &integral_img_sq, Mat_<float> &resp);

};

//===========================================================================
/**
A CCNF patch expert
*/
class CCNF_patch_expert{
public:
    
	int width;
	int height;             
	std::vector<CCNF_neuron> neurons;
	std::vector<int>				window_sizes;
	std::vector<cv::Mat_<float> >	Sigmas;
	std::vector<double>				betas;
	double   patch_confidence;
	CCNF_patch_expert(){;}

	CCNF_patch_expert(const CCNF_patch_expert& other): neurons(other.neurons), window_sizes(other.window_sizes), betas(other.betas)
	{
		this->width = other.width;
		this->height = other.height;
		this->patch_confidence = other.patch_confidence;

		for(std::vector<Mat_<float> >::const_iterator it = other.Sigmas.begin(); it!= other.Sigmas.end(); it++)
		{
			this->Sigmas.push_back(it->clone());
		}

	}


	void Read(std::ifstream &stream, std::vector<int> window_sizes, std::vector<std::vector<Mat_<float> > > sigma_components);
	void Response(Mat_<float> &area_of_interest, Mat_<float> &response);    
	void ComputeSigmas(std::vector<Mat_<float> > sigma_components, int window_size);
	
};
  //===========================================================================
}
#endif

