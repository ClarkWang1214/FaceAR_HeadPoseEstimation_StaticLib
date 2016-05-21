///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include "PAW.h"
#include "FaceAR_utils.h"

using namespace FaceARTracker;

using namespace cv;

PAW::PAW(const Mat_<double>& destination_shape, const Mat_<int>& triangulation)
{
	this->destination_landmarks = destination_shape;
	this->triangulation = triangulation;

	int num_points = destination_shape.rows/2;

	int num_tris = triangulation.rows;
	
    alpha = Mat_<double>(num_tris, 3);
    beta = Mat_<double>(num_tris, 3);
    
    Mat_<double> xs = destination_shape(Rect(0, 0, 1, num_points));
    Mat_<double> ys = destination_shape(Rect(0, num_points, 1, num_points));
    
	vector<vector<double>> destination_points;

	for (int tri = 0; tri < num_tris; ++tri)
	{	
		int j = triangulation.at<int>(tri, 0);
		int k = triangulation.at<int>(tri, 1);
		int l = triangulation.at<int>(tri, 2);

        double c1 = ys.at<double>(l) - ys.at<double>(j);
        double c2 = xs.at<double>(l) - xs.at<double>(j);
        double c4 = ys.at<double>(k) - ys.at<double>(j);
        double c3 = xs.at<double>(k) - xs.at<double>(j);
        		
        double c5 = c3*c1 - c2*c4;

        alpha.at<double>(tri, 0) = (ys.at<double>(j) * c2 - xs.at<double>(j) * c1) / c5;
        alpha.at<double>(tri, 1) = c1/c5;
        alpha.at<double>(tri, 2) = -c2/c5;

        beta.at<double>(tri, 0) = (xs.at<double>(j) * c4 - ys.at<double>(j) * c3)/c5;
        beta.at<double>(tri, 1) = -c4/c5;
        beta.at<double>(tri, 2) = c3/c5;

		vector<double> triangle_points(10);

		triangle_points[0] = xs.at<double>(j);
		triangle_points[1] = ys.at<double>(j);
		triangle_points[2] = xs.at<double>(k);
		triangle_points[3] = ys.at<double>(k);
		triangle_points[4] = xs.at<double>(l);
		triangle_points[5] = ys.at<double>(l);
		
		Vec3d xs_three(triangle_points[0], triangle_points[2], triangle_points[4]);
		Vec3d ys_three(triangle_points[1], triangle_points[3], triangle_points[5]);

		double min_x, max_x, min_y, max_y;
		cv::minMaxIdx(xs_three, &min_x, &max_x);
		cv::minMaxIdx(ys_three, &min_y, &max_y);

		triangle_points[6] = max_x;
		triangle_points[7] = max_y;

		triangle_points[8] = min_x;
		triangle_points[9] = min_y;

		destination_points.push_back(triangle_points);

	}

	double max_x;
	double max_y;

	minMaxLoc(xs, &min_x, &max_x);
	minMaxLoc(ys, &min_y, &max_y);

	int w = (int)(max_x - min_x + 1.5);
    int h = (int)(max_y - min_y + 1.5);

    pixel_mask = Mat_<uchar>(h, w, (uchar)0);
    triangle_id = Mat_<int>(h, w, -1);
        
	int curr_tri = -1;

	for(int y = 0; y < pixel_mask.rows; y++)
	{
		for(int x = 0; x < pixel_mask.cols; x++)
		{
			curr_tri = findTriangle(Point_<double>(x + min_x, y + min_y), destination_points, curr_tri);
            if(curr_tri != -1)
			{
				triangle_id.at<int>(y, x) = curr_tri;
                pixel_mask.at<uchar>(y, x) = 1;
			}	
		}
	}
    	
	coefficients.create(num_tris, 6);
	map_x.create(pixel_mask.rows,pixel_mask.cols);
	map_y.create(pixel_mask.rows,pixel_mask.cols);


}

PAW::PAW(const Mat_<double>& destination_shape, const Mat_<int>& triangulation, double in_min_x, double in_min_y, double in_max_x, double in_max_y)
{
	this->destination_landmarks = destination_shape;
	this->triangulation = triangulation;

	int num_points = destination_shape.rows/2;

	int num_tris = triangulation.rows;

    alpha = Mat_<double>(num_tris, 3);
    beta = Mat_<double>(num_tris, 3);
    
    Mat_<double> xs = destination_shape(Rect(0, 0, 1, num_points));
    Mat_<double> ys = destination_shape(Rect(0, num_points, 1, num_points));

	vector<vector<double>> destination_points;
    
	for (int tri = 0; tri < num_tris; ++tri)
	{	
		int j = triangulation.at<int>(tri, 0);
		int k = triangulation.at<int>(tri, 1);
		int l = triangulation.at<int>(tri, 2);

        double c1 = ys.at<double>(l) - ys.at<double>(j);
        double c2 = xs.at<double>(l) - xs.at<double>(j);
        double c4 = ys.at<double>(k) - ys.at<double>(j);
        double c3 = xs.at<double>(k) - xs.at<double>(j);
        		
        double c5 = c3*c1 - c2*c4;

        alpha.at<double>(tri, 0) = (ys.at<double>(j) * c2 - xs.at<double>(j) * c1) / c5;
        alpha.at<double>(tri, 1) = c1/c5;
        alpha.at<double>(tri, 2) = -c2/c5;

        beta.at<double>(tri, 0) = (xs.at<double>(j) * c4 - ys.at<double>(j) * c3)/c5;
        beta.at<double>(tri, 1) = -c4/c5;
        beta.at<double>(tri, 2) = c3/c5;

		vector<double> triangle_points(10);

		triangle_points[0] = xs.at<double>(j);
		triangle_points[1] = ys.at<double>(j);
		triangle_points[2] = xs.at<double>(k);
		triangle_points[3] = ys.at<double>(k);
		triangle_points[4] = xs.at<double>(l);
		triangle_points[5] = ys.at<double>(l);
		
		Vec3d xs_three(triangle_points[0], triangle_points[2], triangle_points[4]);
		Vec3d ys_three(triangle_points[1], triangle_points[3], triangle_points[5]);

		double min_x, max_x, min_y, max_y;
		cv::minMaxIdx(xs_three, &min_x, &max_x);
		cv::minMaxIdx(ys_three, &min_y, &max_y);

		triangle_points[6] = max_x;
		triangle_points[7] = max_y;

		triangle_points[8] = min_x;
		triangle_points[9] = min_y;

		destination_points.push_back(triangle_points);
		
	}

	double max_x;
	double max_y;

	min_x = in_min_x;
	min_y = in_min_y;

	max_x = in_max_x;
	max_y = in_max_y;

	int w = (int)(max_x - min_x + 1.5);
    int h = (int)(max_y - min_y + 1.5);

    pixel_mask = Mat_<uchar>(h, w, (uchar)0);
    triangle_id = Mat_<int>(h, w, -1);
        
	int curr_tri = -1;

	for(int y = 0; y < pixel_mask.rows; y++)
	{
		for(int x = 0; x < pixel_mask.cols; x++)
		{
			curr_tri = findTriangle(Point_<double>(x + min_x, y + min_y), destination_points, curr_tri);
            if(curr_tri != -1)
			{
				triangle_id.at<int>(y, x) = curr_tri;
                pixel_mask.at<uchar>(y, x) = 1;
			}	
		}
	}    	
	coefficients.create(num_tris, 6);
	map_x.create(pixel_mask.rows,pixel_mask.cols);
	map_y.create(pixel_mask.rows,pixel_mask.cols);

}

//===========================================================================
void PAW::Read(std::ifstream& stream)
{

	stream.read ((char*)&number_of_pixels, 4);
	stream.read ((char*)&min_x, 8);
	stream.read ((char*)&min_y, 8);

    FaceARTracker::ReadMatBin(stream, destination_landmarks);

    FaceARTracker::ReadMatBin(stream, triangulation);

    FaceARTracker::ReadMatBin(stream, triangle_id);
	
	cv::Mat tmpMask;	
    FaceARTracker::ReadMatBin(stream, tmpMask);
	tmpMask.convertTo(pixel_mask, CV_8U);	
	
    FaceARTracker::ReadMatBin(stream, alpha);

    FaceARTracker::ReadMatBin(stream, beta);

	map_x.create(pixel_mask.rows,pixel_mask.cols);
	map_y.create(pixel_mask.rows,pixel_mask.cols);

	coefficients.create(this->NumberOfTriangles(),6);
	
	source_landmarks = destination_landmarks;
}

//=============================================================================
void PAW::Warp(const Mat& image_to_warp, Mat& destination_image, const Mat_<double>& landmarks_to_warp)
{
	source_landmarks = landmarks_to_warp.clone();
	this->CalcCoeff();
	this->WarpRegion(map_x, map_y);
	remap(image_to_warp, destination_image, map_x, map_y, CV_INTER_LINEAR);
}

//=============================================================================
void PAW::CalcCoeff()
{
	int p = this->NumberOfLandmarks();

	for(int l = 0; l < this->NumberOfTriangles(); l++)
	{
	  
		int i = triangulation.at<int>(l,0);
		int j = triangulation.at<int>(l,1);
		int k = triangulation.at<int>(l,2);

		double c1 = source_landmarks.at<double>(i    , 0);
		double c2 = source_landmarks.at<double>(j    , 0) - c1;
		double c3 = source_landmarks.at<double>(k    , 0) - c1;
		double c4 = source_landmarks.at<double>(i + p, 0);
		double c5 = source_landmarks.at<double>(j + p, 0) - c4;
		double c6 = source_landmarks.at<double>(k + p, 0) - c4;
		double *coeff = coefficients.ptr<double>(l);
		double *c_alpha = alpha.ptr<double>(l);
		double *c_beta  = beta.ptr<double>(l);

		coeff[0] = c1 + c2 * c_alpha[0] + c3 * c_beta[0];
		coeff[1] =      c2 * c_alpha[1] + c3 * c_beta[1];
		coeff[2] =      c2 * c_alpha[2] + c3 * c_beta[2];
		coeff[3] = c4 + c5 * c_alpha[0] + c6 * c_beta[0];
		coeff[4] =      c5 * c_alpha[1] + c6 * c_beta[1];
		coeff[5] =      c5 * c_alpha[2] + c6 * c_beta[2];
	}
}

//======================================================================
void PAW::WarpRegion(Mat_<float>& mapx, Mat_<float>& mapy)
{
	
	cv::MatIterator_<float> xp = mapx.begin();
	cv::MatIterator_<float> yp = mapy.begin();
	cv::MatIterator_<uchar> mp = pixel_mask.begin();
	cv::MatIterator_<int>   tp = triangle_id.begin();
	
	double * a;
	int k=-1;

	for(int y = 0; y < pixel_mask.rows; y++)
	{
		double yi = double(y) + min_y;
	
		for(int x = 0; x < pixel_mask.cols; x++)
		{
			double xi = double(x) + min_x;

			if(*mp == 0)
			{
				*xp = -1;
				*yp = -1;
			}
			else
			{
				int j = *tp;
				if(j != k)
				{
					a = coefficients.ptr<double>(j);			
					k = j;
				}  	
				double *ap = a;							
				double xo = *ap++;						
				xo += *ap++ * xi;						
				*xp = float(xo + *ap++ * yi);			
				double yo = *ap++;						
				yo += *ap++ * xi;						
				*yp = float(yo + *ap++ * yi);			

			}
			mp++; tp++; xp++; yp++;	
		}
	}
}

// ============================================================
bool sameSide(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3)
{
    
    double x = (x3-x2)*(y0-y2) - (x0-x2)*(y3-y2);
    double y = (x3-x2)*(y1-y2) - (x1-x2)*(y3-y2);

    return x*y >= 0;

}

bool pointInTriangle(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3)
{
	bool same_1 = sameSide(x0, y0, x1, y1, x2, y2, x3, y3);
	bool same_2 = sameSide(x0, y0, x2, y2, x1, y1, x3, y3);
	bool same_3 = sameSide(x0, y0, x3, y3, x1, y1, x2, y2);

	return same_1 && same_2 && same_3;

}

int PAW::findTriangle(const cv::Point_<double>& point, const std::vector<vector<double>>& control_points, int guess) const
{
    
	int num_tris = control_points.size();
	
	int tri = -1;
    
	double x0 = point.x;
	double y0 = point.y;
	if(guess != -1)
	{
		
		bool in_triangle = pointInTriangle(x0, y0, control_points[guess][0], control_points[guess][1], control_points[guess][2], control_points[guess][3], control_points[guess][4], control_points[guess][5]);
		if(in_triangle)
		{
			return guess;
		}
	}


    for (int i = 0; i < num_tris; ++i)
	{

		double max_x = control_points[i][6];
		double max_y = control_points[i][7];

		double min_x = control_points[i][8];
		double min_y = control_points[i][9];

		if( max_x < x0 || min_x > x0 || max_y < y0 || min_y > y0)
		{
			continue;
		}

		bool in_triangle = pointInTriangle(x0, y0, 
			control_points[i][0], control_points[i][1],
			control_points[i][2], control_points[i][3],
			control_points[i][4], control_points[i][5]);

        if(in_triangle)
		{
           tri = i;
           break;
		}        
	}
	return tri;
}
