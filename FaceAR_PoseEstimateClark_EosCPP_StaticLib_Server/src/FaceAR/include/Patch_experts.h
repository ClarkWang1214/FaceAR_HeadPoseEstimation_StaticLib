///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __Patch_experts_h_
#define __Patch_experts_h_

#include "SVR_patch_expert.h"
#include "CCNF_patch_expert.h"
#include "PDM.h"

namespace FaceARTracker
{
//===========================================================================
/** 
    Combined class for all of the patch experts
*/
class Patch_experts
{

public:

	vector<vector<vector<Multi_SVR_patch_expert> > >	svr_expert_intensity;
	vector<vector<vector<Multi_SVR_patch_expert> > >	svr_expert_depth;
	vector<vector<vector<CCNF_patch_expert> > >			ccnf_expert_intensity;
	vector<vector<cv::Mat_<float> > >					sigma_components;
	vector<double>							patch_scaling;
	vector<vector<cv::Vec3d> >               centers;
    vector<vector<cv::Mat_<int> > >          visibilities;
    //
	Patch_experts(){;}

	// A copy constructor
	Patch_experts(const Patch_experts& other): patch_scaling(other.patch_scaling), centers(other.centers), svr_expert_intensity(other.svr_expert_intensity), svr_expert_depth(other.svr_expert_depth), ccnf_expert_intensity(other.ccnf_expert_intensity)
	{
		this->sigma_components.resize(other.sigma_components.size());
		for(size_t i = 0; i < other.sigma_components.size(); ++i)
		{
			this->sigma_components[i].resize(other.sigma_components[i].size());

			for(size_t j = 0; j < other.sigma_components[i].size(); ++j)
			{
				this->sigma_components[i][j] = other.sigma_components[i][j].clone();
			}
		}

		this->visibilities.resize(other.visibilities.size());
		for(size_t i = 0; i < other.visibilities.size(); ++i)
		{
			this->visibilities[i].resize(other.visibilities[i].size());

			for(size_t j = 0; j < other.visibilities[i].size(); ++j)
			{
				this->visibilities[i][j] = other.visibilities[i][j].clone();
			}
		}
	}

	void Response(vector<cv::Mat_<float> >& patch_expert_responses, Matx22f& sim_ref_to_img, Matx22d& sim_img_to_ref, const Mat_<uchar>& grayscale_image, const Mat_<float>& depth_image,
							 const PDM& pdm, const Vec6d& params_global, const Mat_<double>& params_local, int window_size, int scale);

	int GetViewIdx(const Vec6d& params_global, int scale);
    inline int nViews(int scale=0){return centers[scale].size();};
	void Read(vector<string> intensity_svr_expert_locations, vector<string> depth_svr_expert_locations, vector<string> intensity_ccnf_expert_locations);

private:
	void Read_SVR_patch_experts(string expert_location, std::vector<cv::Vec3d>& centers, std::vector<cv::Mat_<int> >& visibility, std::vector<std::vector<Multi_SVR_patch_expert> >& patches, double& scale);
	void Read_CCNF_patch_experts(string patchesFileLocation, std::vector<cv::Vec3d>& centers, std::vector<cv::Mat_<int> >& visibility, std::vector<std::vector<CCNF_patch_expert> >& patches, double& patchScaling);
};
 
}
#endif

