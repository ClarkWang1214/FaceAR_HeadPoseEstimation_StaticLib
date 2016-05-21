///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __SVR_PATCH_EXPERT_h_
#define __SVR_PATCH_EXPERT_h_

using namespace cv;

namespace FaceARTracker
{
  //===========================================================================

class SVR_patch_expert{
	public:
		int     type;					
		double  scaling;
		double  bias;
		Mat_<float> weights;
		std::map<int, Mat_<double> > weights_dfts;
		double  confidence;

		SVR_patch_expert(){;}
		
		// A copy constructor
		SVR_patch_expert(const SVR_patch_expert& other): weights(other.weights.clone())
		{
			this->type = other.type;
			this->scaling = other.scaling;
			this->bias = other.bias;
			this->confidence = other.confidence;

			for(std::map<int, Mat_<double> >::const_iterator it = other.weights_dfts.begin(); it!= other.weights_dfts.end(); it++)
			{
				this->weights_dfts.insert(std::pair<int, Mat>(it->first, it->second.clone()));
			}
		}
		void Read(std::ifstream &stream);
		void Response(const Mat_<float> &area_of_interest, Mat_<float> &response);    
		void ResponseDepth(const Mat_<float> &area_of_interest, Mat_<float> &response);

};
//===========================================================================
/**
    A Multi-patch Expert that can include different patch types. Raw pixel values or image gradients
*/
class Multi_SVR_patch_expert{
	public:
		
		int width;
		int height;						

		std::vector<SVR_patch_expert> svr_patch_experts;	

		Multi_SVR_patch_expert(){;}			
		Multi_SVR_patch_expert(const Multi_SVR_patch_expert& other): svr_patch_experts(other.svr_patch_experts)
		{
			this->width = other.width;
			this->height = other.height;
		}

		void Read(std::ifstream &stream);
		void Response(const Mat_<float> &area_of_interest, Mat_<float> &response);
		void ResponseDepth(const Mat_<float> &area_of_interest, Mat_<float> &response);

};
}
#endif

