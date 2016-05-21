/////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <map>
#include <vector>
#include <opencv2/core.hpp>
//#include <glm/glm.hpp>
//#include "FaceAR_core.h"
//glm::mat4 clark_dlib_facetracker_per_image( Mat image, FaceARTracker::FaceAR _facear_model, FaceARTracker::FaceARParameters clm_parameters);
std::map<int, double> clark_dlib_facetracker(const std::string input_video, std::vector<cv::Mat>& saved_images_Vec);
std::string pre_main(const std::string input_video, const std::string outtext);


