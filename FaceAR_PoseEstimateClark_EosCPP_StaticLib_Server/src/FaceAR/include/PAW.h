///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#ifndef __PAW_h_
#define __PAW_h_

using namespace cv;

namespace FaceARTracker
{
  //===========================================================================

class PAW{
public:    
    int     number_of_pixels; 
    double  min_x;
    double  min_y;
    Mat_<double> destination_landmarks;
    Mat_<double> source_landmarks;
    Mat_<int> triangulation;    
    Mat_<int> triangle_id;  
	Mat_<uchar> pixel_mask;
    Mat_<double> coefficients;
    Mat_<double> alpha;  
    Mat_<double> beta;   
    Mat_<float> map_x;   
    Mat_<float> map_y;   

	// Default constructor
    PAW(){;}
	PAW(const Mat_<double>& destination_shape, const Mat_<int>& triangulation);
	PAW(const Mat_<double>& destination_shape, const Mat_<int>& triangulation, double in_min_x, double in_min_y, double in_max_x, double in_max_y);

	// Copy constructor
	PAW(const PAW& other): destination_landmarks(other.destination_landmarks.clone()), source_landmarks(other.source_landmarks.clone()), triangulation(other.triangulation.clone()),
		triangle_id(other.triangle_id.clone()), pixel_mask(other.pixel_mask.clone()), coefficients(other.coefficients.clone()), alpha(other.alpha.clone()), beta(other.beta.clone()), map_x(other.map_x.clone()), map_y(other.map_y.clone())
	{
		this->number_of_pixels = other.number_of_pixels; 
		this->min_x = other.min_x;
		this->min_y = other.min_y;
	}

	void Read(std::ifstream &s);
    void Warp(const Mat& image_to_warp, Mat& destination_image, const Mat_<double>& landmarks_to_warp);
    void CalcCoeff();
    void WarpRegion(Mat_<float>& map_x, Mat_<float>& map_y);

    inline int NumberOfLandmarks() const {return destination_landmarks.rows/2;} ;
    inline int NumberOfTriangles() const {return triangulation.rows;} ;

    inline int constWidth() const {return pixel_mask.cols;}
    inline int Height() const {return pixel_mask.rows;}
    
private:

    int findTriangle(const cv::Point_<double>& point, const std::vector<std::vector<double> >& control_points, int guess = -1) const;

  };
  //===========================================================================
}
#endif

