///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include "Patch_experts.h"
#include "FaceAR_utils.h"

using namespace cv;

using namespace FaceARTracker;

void Patch_experts::Response(vector<cv::Mat_<float> >& patch_expert_responses, Matx22f& sim_ref_to_img, Matx22d& sim_img_to_ref, const Mat_<uchar>& grayscale_image, const Mat_<float>& depth_image,
							 const PDM& pdm, const Vec6d& params_global, const Mat_<double>& params_local, int window_size, int scale)
{

	int view_id = GetViewIdx(params_global, scale);		

	int n = pdm.NumberOfPoints();

	Mat_<double> landmark_locations;

	pdm.CalcShape2D(landmark_locations, params_local, params_global);

	Mat_<double> reference_shape;

	Vec6d global_ref(patch_scaling[scale], 0, 0, 0, 0, 0);

	pdm.CalcShape2D(reference_shape, params_local, global_ref);

	Mat_<double> reference_shape_2D = (reference_shape.reshape(1, 2).t());
	Mat_<double> image_shape_2D = landmark_locations.reshape(1, 2).t();

	sim_img_to_ref = AlignShapesWithScale(image_shape_2D, reference_shape_2D);
	Matx22d sim_ref_to_img_d = sim_img_to_ref.inv(DECOMP_LU);

	double a1 = sim_ref_to_img_d(0,0);
	double b1 = -sim_ref_to_img_d(0,1);
		
	sim_ref_to_img(0,0) = (float)sim_ref_to_img_d(0,0);
	sim_ref_to_img(0,1) = (float)sim_ref_to_img_d(0,1);
	sim_ref_to_img(1,0) = (float)sim_ref_to_img_d(1,0);
	sim_ref_to_img(1,1) = (float)sim_ref_to_img_d(1,1);

	Mat_<uchar> mask;
	if(!depth_image.empty())
	{
		mask = depth_image > 0;			
		mask = mask / 255;
	}		
	

	bool use_ccnf = !this->ccnf_expert_intensity.empty();

	if(use_ccnf)
	{
		vector<Mat_<float> > sigma_components;

		for( size_t w_size = 0; w_size < this->sigma_components.size(); ++w_size)
		{
			if(!this->sigma_components[w_size].empty())
			{
				if(window_size*window_size == this->sigma_components[w_size][0].rows)
				{
					sigma_components = this->sigma_components[w_size];
				}
			}
		}			

		for( int lmark = 0; lmark < n; lmark++)
		{
			if(visibilities[scale][view_id].at<int>(lmark,0))
			{
				ccnf_expert_intensity[scale][view_id][lmark].ComputeSigmas(sigma_components, window_size);
			}
		}

	}

/////////////////////////////////
/// Keegan.Ren
/// TODO : TBB
#ifdef _OPENMP
#pragma omp parallel for
#endif
    tbb::parallel_for(0, (int)n, [&](int i){
//    for(int i = 0; i < n; i++)
	{	
		if(visibilities[scale][view_id].rows == n)
		{
			if(visibilities[scale][view_id].at<int>(i,0) != 0)
			{
				int area_of_interest_width;
				int area_of_interest_height;

				if(use_ccnf)
				{
					area_of_interest_width = window_size + ccnf_expert_intensity[scale][view_id][i].width - 1; 
					area_of_interest_height = window_size + ccnf_expert_intensity[scale][view_id][i].height - 1;				
				}
				else
				{
					area_of_interest_width = window_size + svr_expert_intensity[scale][view_id][i].width - 1; 
					area_of_interest_height = window_size + svr_expert_intensity[scale][view_id][i].height - 1;
				}
				Mat sim = (Mat_<float>(2,3) << a1, -b1, landmark_locations.at<double>(i,0), b1, a1, landmark_locations.at<double>(i+n,0));
				Mat_<float> area_of_interest(area_of_interest_height, area_of_interest_width);
				CvMat area_of_interest_o = area_of_interest;
				CvMat sim_o = sim;
				IplImage im_o = grayscale_image;			
				cvGetQuadrangleSubPix(&im_o, &area_of_interest_o, &sim_o);	
				patch_expert_responses[i] = Mat_<float>(window_size, window_size);
				if(!ccnf_expert_intensity.empty())
				{				

					ccnf_expert_intensity[scale][view_id][i].Response(area_of_interest, patch_expert_responses[i]);
				}
				else
				{
					svr_expert_intensity[scale][view_id][i].Response(area_of_interest, patch_expert_responses[i]);
				}
				if(!svr_expert_depth.empty() && !depth_image.empty() && visibilities[scale][view_id].at<int>(i,0))
				{

					Mat_<float> dProb = patch_expert_responses[i].clone();
					Mat_<float> depthWindow(area_of_interest_height, area_of_interest_width);
			

					CvMat dimg_o = depthWindow;
					Mat maskWindow(area_of_interest_height, area_of_interest_width, CV_32F);
					CvMat mimg_o = maskWindow;

					IplImage d_o = depth_image;
					IplImage m_o = mask;

					cvGetQuadrangleSubPix(&d_o,&dimg_o,&sim_o);
				
					cvGetQuadrangleSubPix(&m_o,&mimg_o,&sim_o);

					depthWindow.setTo(0, maskWindow < 1);

					svr_expert_depth[scale][view_id][i].ResponseDepth(depthWindow, dProb);

					double sum = cv::sum(patch_expert_responses[i])[0];
					if(sum == 0)
					{
						sum = 1;
					}

					patch_expert_responses[i] /= sum;

					sum = cv::sum(dProb)[0];
					if(sum == 0)
					{
						sum = 1;
					}

					dProb /= sum;

					patch_expert_responses[i] = patch_expert_responses[i] + dProb;

				}
			}
		}
	}
////////////////////////////
///TODO: TBB
    });

}

//=============================================================================
int Patch_experts::GetViewIdx(const Vec6d& params_global, int scale)
{	
	int idx = 0;
	
	double dbest;

	for(int i = 0; i < this->nViews(scale); i++)
	{
		double v1 = params_global[1] - centers[scale][i][0]; 
		double v2 = params_global[2] - centers[scale][i][1];
		double v3 = params_global[3] - centers[scale][i][2];
			
		double d = v1*v1 + v2*v2 + v3*v3;

		if(i == 0 || d < dbest)
		{
			dbest = d;
			idx = i;
		}
	}
	return idx;
}


//===========================================================================
void Patch_experts::Read(vector<string> intensity_svr_expert_locations, vector<string> depth_svr_expert_locations, vector<string> intensity_ccnf_expert_locations)
{
	int num_intensity_svr = intensity_svr_expert_locations.size();
	centers.resize(num_intensity_svr);
	visibilities.resize(num_intensity_svr);
	patch_scaling.resize(num_intensity_svr);
	
	svr_expert_intensity.resize(num_intensity_svr);

	for(int scale = 0; scale < num_intensity_svr; ++scale)
	{		
		string location = intensity_svr_expert_locations[scale];
        //cout << "Reading the intensity SVR patch experts from: " << location << "....";
		Read_SVR_patch_experts(location,  centers[scale], visibilities[scale], svr_expert_intensity[scale], patch_scaling[scale]);
	}

	int num_intensity_ccnf = intensity_ccnf_expert_locations.size();

	if(num_intensity_ccnf > 0)
	{
		centers.resize(num_intensity_ccnf);
		visibilities.resize(num_intensity_ccnf);
		patch_scaling.resize(num_intensity_ccnf);
		ccnf_expert_intensity.resize(num_intensity_ccnf);
	}

	for(int scale = 0; scale < num_intensity_ccnf; ++scale)
	{		
		string location = intensity_ccnf_expert_locations[scale];
        //cout << "Reading the intensity CCNF patch experts from: " << location << "....";
		Read_CCNF_patch_experts(location,  centers[scale], visibilities[scale], ccnf_expert_intensity[scale], patch_scaling[scale]);
	}

	int num_depth_scales = depth_svr_expert_locations.size();
	int num_intensity_scales = centers.size();
	
	if(num_depth_scales > 0 && num_intensity_scales != num_depth_scales)
	{
        cout << "Intensity and depth patch experts have a different number of scales, can't read depth" << endl;
		return;
	}

	vector<vector<cv::Vec3d> > centers_depth(num_depth_scales);
	vector<vector<cv::Mat_<int> > > visibilities_depth(num_depth_scales);
	vector<double> patch_scaling_depth(num_depth_scales);
	
	svr_expert_depth.resize(num_depth_scales);	
	for(int scale = 0; scale < num_depth_scales; ++scale)
	{		
		string location = depth_svr_expert_locations[scale];
        //cout << "Reading the depth SVR patch experts from: " << location << "....";
		Read_SVR_patch_experts(location,  centers_depth[scale], visibilities_depth[scale], svr_expert_depth[scale], patch_scaling_depth[scale]);

		if(patch_scaling_depth[scale] != patch_scaling[scale])
		{
            //cout << "Intensity and depth patch experts have a different scales, can't read depth" << endl;
			svr_expert_depth.clear();
			return;			
		}

		int num_views_intensity = centers[scale].size();
		int num_views_depth = centers_depth[scale].size();

		if(num_views_intensity != num_views_depth)
		{
			cout << "Intensity and depth patch experts have a different number of scales, can't read depth" << endl;
			svr_expert_depth.clear();
			return;			
		}

		for(int view = 0; view < num_views_depth; ++view)
		{
			if(cv::countNonZero(centers_depth[scale][view] != centers[scale][view]) || cv::countNonZero(visibilities[scale][view] != visibilities_depth[scale][view]))
			{
				cout << "Intensity and depth patch experts have different visibilities or centers" << endl;
				svr_expert_depth.clear();
				return;		
			}
		}
	}

}

void Patch_experts::Read_SVR_patch_experts(string expert_location, std::vector<cv::Vec3d>& centers, std::vector<cv::Mat_<int> >& visibility, std::vector<std::vector<Multi_SVR_patch_expert> >& patches, double& scale)
{

	ifstream patchesFile(expert_location.c_str(), ios_base::in);

	if(patchesFile.is_open())
	{
        FaceARTracker::SkipComments(patchesFile);

		patchesFile >> scale;

        FaceARTracker::SkipComments(patchesFile);

		int numberViews;		

		patchesFile >> numberViews; 

		centers.resize(numberViews);
		visibility.resize(numberViews);
  
		patches.resize(numberViews);

        FaceARTracker::SkipComments(patchesFile);

		for(size_t i = 0; i < centers.size(); i++)
		{
			cv::Mat center;
            FaceARTracker::ReadMat(patchesFile, center);
			center.copyTo(centers[i]);
			centers[i] = centers[i] * M_PI / 180.0;
		}

        FaceARTracker::SkipComments(patchesFile);

		for(size_t i = 0; i < visibility.size(); i++)
		{
            FaceARTracker::ReadMat(patchesFile, visibility[i]);
		}

		int numberOfPoints = visibility[0].rows;

        FaceARTracker::SkipComments(patchesFile);

		for(size_t i = 0; i < patches.size(); i++)
		{
			patches[i].resize(numberOfPoints);
			for(int j = 0; j < numberOfPoints; j++)
			{
				patches[i][j].Read(patchesFile);
			}
		}
	
        //cout << "Done" << endl;
	}
	else
	{
		cout << "Can't find/open the patches file" << endl;
	}
}

void Patch_experts::Read_CCNF_patch_experts(string patchesFileLocation, std::vector<cv::Vec3d>& centers, std::vector<cv::Mat_<int> >& visibility, std::vector<std::vector<CCNF_patch_expert> >& patches, double& patchScaling)
{

	ifstream patchesFile(patchesFileLocation.c_str(), ios::in | ios::binary);

	if(patchesFile.is_open())
	{
		patchesFile.read ((char*)&patchScaling, 8);
		
		int numberViews;		
		patchesFile.read ((char*)&numberViews, 4);
		centers.resize(numberViews);
		visibility.resize(numberViews);
  
		patches.resize(numberViews);
		for(size_t i = 0; i < centers.size(); i++)
		{
			cv::Mat center;
            FaceARTracker::ReadMatBin(patchesFile, center);
			center.copyTo(centers[i]);
			centers[i] = centers[i] * M_PI / 180.0;
		}

		for(size_t i = 0; i < visibility.size(); i++)
		{
            FaceARTracker::ReadMatBin(patchesFile, visibility[i]);
		}
		int numberOfPoints = visibility[0].rows;
		int num_win_sizes;
		int num_sigma_comp;
		patchesFile.read ((char*)&num_win_sizes, 4);

		vector<int> windows;
		windows.resize(num_win_sizes);

		vector<vector<cv::Mat_<float> > > sigma_components;
		sigma_components.resize(num_win_sizes);

		for (int w=0; w < num_win_sizes; ++w)
		{
			patchesFile.read ((char*)&windows[w], 4);

			patchesFile.read ((char*)&num_sigma_comp, 4);

			sigma_components[w].resize(num_sigma_comp);

			for(int s=0; s < num_sigma_comp; ++s)
			{
                FaceARTracker::ReadMatBin(patchesFile, sigma_components[w][s]);
			}
		}
		
		this->sigma_components = sigma_components;

		for(size_t i = 0; i < patches.size(); i++)
		{
			patches[i].resize(numberOfPoints);
			for(int j = 0; j < numberOfPoints; j++)
			{
				patches[i][j].Read(patchesFile, windows, sigma_components);
			}
		}
        //cout << "Done" << endl;
	}
	else
	{
		cout << "Can't find/open the patches file" << endl;
	}
}


