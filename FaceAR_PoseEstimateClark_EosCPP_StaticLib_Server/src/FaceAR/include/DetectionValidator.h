///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __DValid_h_
#define __DValid_h_

#include "PAW.h"

using namespace std;
using namespace cv;

namespace FaceARTracker
{
//===========================================================================
class DetectionValidator
{
		
public:    

	int validator_type;
	vector<cv::Vec3d> orientations;
	vector<PAW>     paws;

	//==========================================
	vector<double>  bs;
	vector<Mat_<double> > ws;
	vector<vector<Mat_<double> > > ws_nn;
	vector<int> activation_fun;
	vector<int> output_fun;

	//==========================================
	vector<vector<vector<vector<Mat_<float> > > > > cnn_convolutional_layers;
	vector<vector<vector<vector<pair<int, Mat_<double> > > > > > cnn_convolutional_layers_dft;
	vector<vector<vector<float > > > cnn_convolutional_layers_bias;
	vector< vector<int> > cnn_subsampling_layers;
	vector< vector<Mat_<float> > > cnn_fully_connected_layers;
	vector< vector<float > > cnn_fully_connected_layers_bias;
	vector<vector<int> > cnn_layer_types;
	
	//==========================================
	vector<Mat_<double> > mean_images;
	vector<Mat_<double> > standard_deviations;

	// Default constructor
	DetectionValidator(){;}

	// Copy constructor
	DetectionValidator(const DetectionValidator& other): orientations(other.orientations), bs(other.bs), paws(other.paws),
		cnn_subsampling_layers(other.cnn_subsampling_layers),cnn_layer_types(other.cnn_layer_types), cnn_fully_connected_layers_bias(other.cnn_fully_connected_layers_bias),
		cnn_convolutional_layers_bias(other.cnn_convolutional_layers_bias), cnn_convolutional_layers_dft(other.cnn_convolutional_layers_dft)
	{
	
		this->validator_type = other.validator_type;

		this->activation_fun = other.activation_fun;
		this->output_fun = other.output_fun;

		this->ws.resize(other.ws.size());
		for(size_t i = 0; i < other.ws.size(); ++i)
		{
			this->ws[i] = other.ws[i].clone();
		}

		this->ws_nn.resize(other.ws_nn.size());
		for(size_t i = 0; i < other.ws_nn.size(); ++i)
		{
			this->ws_nn[i].resize(other.ws_nn[i].size());

			for(size_t k = 0; k < other.ws_nn[i].size(); ++k)
			{
				this->ws_nn[i][k] = other.ws_nn[i][k].clone();
			}
		}

		this->cnn_convolutional_layers.resize(other.cnn_convolutional_layers.size());
		for(size_t v = 0; v < other.cnn_convolutional_layers.size(); ++v)
		{
			this->cnn_convolutional_layers[v].resize(other.cnn_convolutional_layers[v].size());

			for(size_t l = 0; l < other.cnn_convolutional_layers[v].size(); ++l)
			{
				this->cnn_convolutional_layers[v][l].resize(other.cnn_convolutional_layers[v][l].size());

				for(size_t i = 0; i < other.cnn_convolutional_layers[v][l].size(); ++i)
				{
					this->cnn_convolutional_layers[v][l][i].resize(other.cnn_convolutional_layers[v][l][i].size());

					for(size_t k = 0; k < other.cnn_convolutional_layers[v][l][i].size(); ++k)
					{
						this->cnn_convolutional_layers[v][l][i][k] = other.cnn_convolutional_layers[v][l][i][k].clone();
					}
					
				}
			}
		}

		this->cnn_fully_connected_layers.resize(other.cnn_fully_connected_layers.size());
		for(size_t v = 0; v < other.cnn_fully_connected_layers.size(); ++v)
		{
			this->cnn_fully_connected_layers[v].resize(other.cnn_fully_connected_layers[v].size());

			for(size_t l = 0; l < other.cnn_fully_connected_layers[v].size(); ++l)
			{
				this->cnn_fully_connected_layers[v][l] = other.cnn_fully_connected_layers[v][l].clone();
			}
		}

		this->mean_images.resize(other.mean_images.size());
		for(size_t i = 0; i < other.mean_images.size(); ++i)
		{
			this->mean_images[i] = other.mean_images[i].clone();
		}

		this->standard_deviations.resize(other.standard_deviations.size());
		for(size_t i = 0; i < other.standard_deviations.size(); ++i)
		{
			this->standard_deviations[i] = other.standard_deviations[i].clone();
		}
	
	}

	double Check(const Vec3d& orientation, const Mat_<uchar>& intensity_img, Mat_<double>& detected_landmarks);
	void Read(string location);
	int GetViewId(const cv::Vec3d& orientation) const;

private:

	double CheckSVR(const Mat_<double>& warped_img, int view_id);
	double CheckNN(const Mat_<double>& warped_img, int view_id);
	double CheckCNN(const Mat_<double>& warped_img, int view_id);
	void NormaliseWarpedToVector(const Mat_<double>& warped_img, Mat_<double>& feature_vec, int view_id);

};

}
#endif

