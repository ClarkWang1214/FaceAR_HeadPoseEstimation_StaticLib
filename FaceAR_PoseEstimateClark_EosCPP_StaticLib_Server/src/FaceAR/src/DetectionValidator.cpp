///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include "DetectionValidator.h"
#include "FaceAR_utils.h"

using namespace FaceARTracker;

//===========================================================================
void DetectionValidator::Read(string location)
{

	ifstream detection_validator_stream (location, ios::in|ios::binary);
	if (detection_validator_stream.is_open())	
	{				
		detection_validator_stream.seekg (0, ios::beg);

		detection_validator_stream.read ((char*)&validator_type, 4);

		int n;
		detection_validator_stream.read ((char*)&n, 4);
	
		orientations.resize(n);

		for(int i = 0; i < n; i++)
		{
			Mat_<double> orientation_tmp;
            FaceARTracker::ReadMatBin(detection_validator_stream, orientation_tmp);
		
			orientations[i] = Vec3d(orientation_tmp.at<double>(0), orientation_tmp.at<double>(1), orientation_tmp.at<double>(2));

			orientations[i] = orientations[i] * M_PI / 180.0;
		}

		paws.resize(n);

		if( validator_type == 0)
		{
			bs.resize(n);
			ws.resize(n);
		}
		else if(validator_type == 1)
		{
			ws_nn.resize(n);

			activation_fun.resize(n);
			output_fun.resize(n);
		}
		else if(validator_type == 2)
		{
			cnn_convolutional_layers.resize(n);
			cnn_convolutional_layers_dft.resize(n);
			cnn_subsampling_layers.resize(n);
			cnn_fully_connected_layers.resize(n);
			cnn_layer_types.resize(n);
			cnn_fully_connected_layers_bias.resize(n);
			cnn_convolutional_layers_bias.resize(n);
		}			

		mean_images.resize(n);
		standard_deviations.resize(n);

		for(int i = 0; i < n; i++)
		{
            FaceARTracker::ReadMatBin(detection_validator_stream, mean_images[i]);
			mean_images[i] = mean_images[i].t();
	
            FaceARTracker::ReadMatBin(detection_validator_stream, standard_deviations[i]);
			standard_deviations[i] = standard_deviations[i].t();

			if(validator_type == 0)
			{
				detection_validator_stream.read ((char*)&bs[i], 8);
                FaceARTracker::ReadMatBin(detection_validator_stream, ws[i]);
	
			}
			else if(validator_type == 1)
			{
				int num_depth_layers;
				detection_validator_stream.read ((char*)&num_depth_layers, 4);
				detection_validator_stream.read ((char*)&activation_fun[i], 4);
				detection_validator_stream.read ((char*)&output_fun[i], 4);

				ws_nn[i].resize(num_depth_layers);
				for(int layer = 0; layer < num_depth_layers; layer++)
				{
                    FaceARTracker::ReadMatBin(detection_validator_stream, ws_nn[i][layer]);
					ws_nn[i][layer] = ws_nn[i][layer].t();
				}
			}
			else if(validator_type == 2)
			{
				int network_depth;
				detection_validator_stream.read ((char*)&network_depth, 4);
	
				cnn_layer_types[i].resize(network_depth);

				for(int layer = 0; layer < network_depth; ++layer)
				{

					int layer_type;
					detection_validator_stream.read ((char*)&layer_type, 4);
					cnn_layer_types[i][layer] = layer_type;

					if(layer_type == 0)
					{
						int num_in_maps;
						detection_validator_stream.read ((char*)&num_in_maps, 4);
						int num_kernels;
						detection_validator_stream.read ((char*)&num_kernels, 4);

						vector<vector<Mat_<float> > > kernels;
						vector<vector<pair<int, Mat_<double> > > > kernel_dfts;

						kernels.resize(num_in_maps);
						kernel_dfts.resize(num_in_maps);

						vector<float> biases;
						for (int k = 0; k < num_kernels; ++k)
						{
							float bias;
							detection_validator_stream.read ((char*)&bias, 4);						
							biases.push_back(bias);
						}

						cnn_convolutional_layers_bias[i].push_back(biases);
						for (int in = 0; in < num_in_maps; ++in)
						{
							kernels[in].resize(num_kernels);
							kernel_dfts[in].resize(num_kernels);

							for (int k = 0; k < num_kernels; ++k)
							{
								ReadMatBin(detection_validator_stream, kernels[in][k]);				
								cv::flip(kernels[in][k], kernels[in][k], -1);
							}
						}

						cnn_convolutional_layers[i].push_back(kernels);
						cnn_convolutional_layers_dft[i].push_back(kernel_dfts);
					}
					else if(layer_type == 1)
					{
						int scale;
						detection_validator_stream.read ((char*)&scale, 4);

						cnn_subsampling_layers[i].push_back(scale);
					}
					else if(layer_type == 2)
					{
						float bias;
						detection_validator_stream.read ((char*)&bias, 4);
						cnn_fully_connected_layers_bias[i].push_back(bias);

						Mat_<float> weights;
						ReadMatBin(detection_validator_stream, weights);
						cnn_fully_connected_layers[i].push_back(weights);
					}
				}
			}
			
			paws[i].Read(detection_validator_stream);
		}
		
	}
	else
	{
		cout << "WARNING: Can't find the Face checker location" << endl;
	}
}

//===========================================================================
double DetectionValidator::Check(const Vec3d& orientation, const Mat_<uchar>& intensity_img, Mat_<double>& detected_landmarks)
{

	int id = GetViewId(orientation);
	Mat_<double> warped;
	Mat_<double> intensity_img_double;
	intensity_img.convertTo(intensity_img_double, CV_64F);

	paws[id].Warp(intensity_img_double, warped, detected_landmarks);	
	
	double dec;
	if(validator_type == 0)
	{
		dec = CheckSVR(warped, id);
	}
	else if(validator_type == 1)
	{
		dec = CheckNN(warped, id);
	}
	else if(validator_type == 2)
	{
		dec = CheckCNN(warped, id);
	}
	return dec;
}

double DetectionValidator::CheckNN(const Mat_<double>& warped_img, int view_id)
{
	Mat_<double> feature_vec;
	NormaliseWarpedToVector(warped_img, feature_vec, view_id);
	feature_vec = feature_vec.t();
			
	for(size_t layer = 0; layer < ws_nn[view_id].size(); ++layer)
	{
		cv::hconcat(Mat_<double>(1,1, 1.0), feature_vec, feature_vec);
		feature_vec = feature_vec * ws_nn[view_id][layer];
		int fun_type;
		if(layer != ws_nn[view_id].size() - 1)
		{
			fun_type = activation_fun[view_id];
		}
		else
		{
			fun_type = output_fun[view_id];
		}

		if(fun_type == 0)
		{
			cv::exp(-feature_vec, feature_vec);
			feature_vec = 1.0 /(1.0 + feature_vec);		
		}
		else if(fun_type == 1)
		{
			MatIterator_<double> q1 = feature_vec.begin(); // respone for each pixel
			MatIterator_<double> q2 = feature_vec.end();
			while(q1 != q2)
			{
				*q1 = 1.7159 * tanh((2.0/3.0) * (*q1));
				q1++;
			}
		}
	}
	double dec = (feature_vec.at<double>(0) - 0.5) * 2;

	return dec;

}

double DetectionValidator::CheckSVR(const Mat_<double>& warped_img, int view_id)
{

	Mat_<double> feature_vec;
	NormaliseWarpedToVector(warped_img, feature_vec, view_id);
		

	double dec = (ws[view_id].dot(feature_vec.t()) + bs[view_id]);

	return dec;

}

double DetectionValidator::CheckCNN(const Mat_<double>& warped_img, int view_id)
{

	Mat_<double> feature_vec;
	NormaliseWarpedToVector(warped_img, feature_vec, view_id);
	
	Mat_<float> img(warped_img.size(), 0.0);
	img = img.t();

	cv::Mat mask = paws[view_id].pixel_mask.t();
	cv::MatIterator_<uchar>  mask_it = mask.begin<uchar>();
	
	cv::MatIterator_<double> feature_it = feature_vec.begin();
	cv::MatIterator_<float> img_it = img.begin();		

	int wInt = img.cols;
	int hInt = img.rows;

	for(int i=0; i < wInt; ++i)
	{
		for(int j=0; j < hInt; ++j, ++mask_it, ++img_it)
		{
			if(*mask_it)
			{
				*img_it = (float)*feature_it++;
			}
		}
	}
	img = img.t();
	
	int cnn_layer = 0;
	int subsample_layer = 0;
	int fully_connected_layer = 0;

	vector<Mat_<float> > input_maps;
	input_maps.push_back(img);
	
	vector<Mat_<float> > outputs;

	for(size_t layer = 0; layer < cnn_layer_types[view_id].size(); ++layer)
	{
		int layer_type = cnn_layer_types[view_id][layer];
		
		if(layer_type == 0)
		{
			vector<Mat_<float> > outputs_kern;
			for(size_t in = 0; in < input_maps.size(); ++in)
			{
				Mat_<float> input_image = input_maps[in];
				Mat_<double> input_image_dft;
				Mat integral_image;
				Mat integral_image_sq;

				for(size_t k = 0; k < cnn_convolutional_layers[view_id][cnn_layer][in].size(); ++k)
				{
					Mat_<float> kernel = cnn_convolutional_layers[view_id][cnn_layer][in][k];					
					Mat_<float> output;
					if(cnn_convolutional_layers_dft[view_id][cnn_layer][in][k].second.empty())
					{
						std::map<int, Mat_<double> > precomputed_dft;
						
                        FaceARTracker::matchTemplate_m(input_image, input_image_dft, integral_image, integral_image_sq, kernel, precomputed_dft, output, CV_TM_CCORR);
						
						cnn_convolutional_layers_dft[view_id][cnn_layer][in][k].first = precomputed_dft.begin()->first;
						cnn_convolutional_layers_dft[view_id][cnn_layer][in][k].second = precomputed_dft.begin()->second;
					}
					else
					{
						std::map<int, Mat_<double> > precomputed_dft;
						precomputed_dft[cnn_convolutional_layers_dft[view_id][cnn_layer][in][k].first] = cnn_convolutional_layers_dft[view_id][cnn_layer][in][k].second;
                        FaceARTracker::matchTemplate_m(input_image, input_image_dft, integral_image, integral_image_sq, kernel,  precomputed_dft, output, CV_TM_CCORR);
					}

					if(in == 0)
					{
						outputs_kern.push_back(output);
					}
					else
					{
						outputs_kern[k] = outputs_kern[k] + output;
					}

				}
								
			}
			
			outputs.clear();
			for(size_t k = 0; k < cnn_convolutional_layers[view_id][cnn_layer][0].size(); ++k)
			{
				cv::exp(-outputs_kern[k] - cnn_convolutional_layers_bias[view_id][cnn_layer][k], outputs_kern[k]);
				outputs_kern[k] = 1.0 /(1.0 + outputs_kern[k]);	

				outputs.push_back(outputs_kern[k]);

			}			

			cnn_layer++;
		}				
		if(layer_type == 1)
		{
			int scale = cnn_subsampling_layers[view_id][subsample_layer];
			
			Mat kx = Mat::ones(2, 1, CV_32F)*1.0f/scale;
			Mat ky = Mat::ones(1, 2, CV_32F)*1.0f/scale;

			vector<Mat_<float>> outputs_sub;
			for(size_t in = 0; in < input_maps.size(); ++in)
			{

				Mat_<float> conv_out;

				sepFilter2D(input_maps[in], conv_out, CV_32F, kx, ky);
				conv_out = conv_out(Rect(1, 1, conv_out.cols - 1, conv_out.rows - 1));

				int res_rows = conv_out.rows / scale;
				int res_cols = conv_out.cols / scale;

				if(conv_out.rows % scale != 0)
				{
					res_rows++;
				}
				if(conv_out.cols % scale != 0)
				{
					res_cols++;
				}

				Mat_<float> sub_out(res_rows, res_cols);
				for(int w = 0; w < conv_out.cols; w+=scale)
				{
					for(int h=0; h < conv_out.rows; h+=scale)
					{
						sub_out.at<float>(h/scale, w/scale) = conv_out(h, w);
					}
				}
				outputs_sub.push_back(sub_out);
			}
			outputs = outputs_sub;
			subsample_layer++;

		}
		if(layer_type == 2)
		{
			Mat_<float> input_concat = input_maps[0].t();
			input_concat = input_concat.reshape(0, 1);

			for(size_t in = 1; in < input_maps.size(); ++in)
			{
				Mat_<float> add = input_maps[in].t();
				add = add.reshape(0,1);
				cv::hconcat(input_concat, add, input_concat);
			}
						
			input_concat = input_concat * cnn_fully_connected_layers[view_id][fully_connected_layer].t();

			cv::exp(-input_concat - cnn_fully_connected_layers_bias[view_id][fully_connected_layer], input_concat);
			input_concat = 1.0 /(1.0 + input_concat);		

			outputs.clear();
			outputs.push_back(input_concat);

			fully_connected_layer++;
		}
		input_maps = outputs;

	}
	double dec = (outputs[0].at<float>(0) - 0.5) * 2.0;

	return dec;
}

void DetectionValidator::NormaliseWarpedToVector(const Mat_<double>& warped_img, Mat_<double>& feature_vec, int view_id)
{
	Mat_<double> warped_t = warped_img.t();
	cv::MatIterator_<double> vp;	
	cv::MatIterator_<double>  cp;

	cv::Mat_<double> vec(paws[view_id].number_of_pixels,1);
	vp = vec.begin();

	cp = warped_t.begin();		

	int wInt = warped_img.cols;
	int hInt = warped_img.rows;
	
	cv::Mat maskT = paws[view_id].pixel_mask.t();

	cv::MatIterator_<uchar>  mp = maskT.begin<uchar>();

	for(int i=0; i < wInt; ++i)
	{
		for(int j=0; j < hInt; ++j, ++mp, ++cp)
		{
			if(*mp)
			{
				*vp++ = *cp;
			}
		}
	}
	cv::Scalar mean;
	cv::Scalar std;
	cv::meanStdDev(vec, mean, std);

	vec -= mean[0];
	if(std[0] == 0)
	{
		std[0] = 1;
	}
	
	vec /= std[0];
	feature_vec = (vec - mean_images[view_id])  / standard_deviations[view_id];
}

int DetectionValidator::GetViewId(const cv::Vec3d& orientation) const
{
	int id = 0;

	double dbest = -1.0;

	for(size_t i = 0; i < this->orientations.size(); i++)
	{
		double d = cv::norm(orientation, this->orientations[i]);

		if(i == 0 || d < dbest)
		{
			dbest = d;
			id = i;
		}
	}
	return id;
	
}



