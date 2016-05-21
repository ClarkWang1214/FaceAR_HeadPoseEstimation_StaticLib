#pragma once

//#include <stdafx.h>
// OpenCV stuff   opencv2/
#include <core/core.hpp>
#include <objdetect.hpp>
#include <calib3d.hpp>
#include <imgcodecs.hpp>
#include <imgproc.hpp>
#include <highgui/highgui.hpp>
#include <videoio/videoio.hpp>  // Video write  opencv2/
#include <videoio/videoio_c.h>  // Video write  opencv2/
//#include <viz/vizcore.hpp> ///*opencv2/*/

// IplImage stuff (get rid of it? TODO)
#include <core/core_c.h>
#include <imgproc/imgproc_c.h>

// C++ stuff
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <iomanip>

#define _USE_MATH_DEFINES
#include <math.h>

//// dlib stuff
//// Used for face detection
//#include <dlib/image_processing/frontal_face_detector.h>
//#include <dlib/opencv.h>

// Boost stuff
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/program_options.hpp>

#include <tbb.h>

// glm
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>

//g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace cv;
using namespace std;

namespace clarkPoseEsti
{
    class FaceGeom{
    public:
        double LeftEye_Nose_distance;
        double RightEye_Nose_distance;
        double LeftEye_RightEye_distance;
        double Nose_Mouth_distance;

        double init_LeftEye_Nose_distance;
        double init_RightEye_Nose_distance;
        double init_LeftEye_RightEye_distance;
        double init_Nose_Mouth_distance;

        double init_Mean_Feature_distance;
        double Mean_Feature_distance;

    };

    class Pose{
    public:
        double pitch, yaw, roll;
        double slant;
        double zaxis, xaxis, yaxis;

    };

    class Face{
    public:
        CvPoint2D32f 	Face;
        CvPoint2D32f	LeftEye;
        CvPoint2D32f	RightEye;
        CvPoint2D32f	Nose;
        CvPoint2D32f 	Mouth;

        CvPoint2D32f	NoseBase;
        CvPoint2D32f 	MidEyes;

        CvRect* FaceBox;
        CvRect* NoseBox;
        CvRect* EyeBox1;
        CvRect* EyeBox2;
        CvRect* MouthBox;

    };


    class PoseEsti{

        public:
            PoseEsti();
            ~PoseEsti();


        public:
//            Face *FPtr;
            //create CvPoint structures to hold the located feature coordinates
            //centre points
            CvPoint2D32f 	Face;
            CvPoint2D32f	LeftEye;
            CvPoint2D32f	RightEye;
            CvPoint2D32f	Nose;
            CvPoint2D32f 	Mouth;

            CvPoint2D32f	NoseBase;
            CvPoint2D32f 	MidEyes;

            CvRect* FaceBox;
            CvRect* NoseBox;
            CvRect* EyeBox1;
            CvRect* EyeBox2;
            CvRect* MouthBox;



//            FaceGeom *GPtr;
            double LeftEye_Nose_distance;
            double RightEye_Nose_distance;
            double LeftEye_RightEye_distance;
            double Nose_Mouth_distance;

            double init_LeftEye_Nose_distance;
            double init_RightEye_Nose_distance;
            double init_LeftEye_RightEye_distance;
            double init_Nose_Mouth_distance;

            double init_Mean_Feature_distance;
            double Mean_Feature_distance;



//            Pose *PPtr;
            double pitch_angle, yaw_angle, roll_angle;
            double pitch_radian, yaw_radian, roll_radian;
            double slant_radian;
            double zaxis, xaxis, yaxis;


            CvPoint rand_coord;

            float  R_m;
            float  R_n;
            float  R_e;
            double pi;
            float scale;

            CvMemStorage *storage;
            CvSeq *seq;

            CvPoint pointer_2d;
            CvPoint mouse;

            bool flag;

            glm::mat4 rotateMatrix;

        //function
        public:
            //Function to return distance between 2 points in image
            float FindDistance(CvPoint pt1, CvPoint pt2);

            //Function to return distance between 2 points in image
            double FindDistance2D32f(CvPoint2D32f pt1, CvPoint2D32f pt2);

            //Function to return angle between 2 points in image
            float FindAngle(CvPoint2D32f pt1, CvPoint2D32f pt2);

            //Function to find slant angle in image 'Gee & Cipolla'
            double Find_slant(int ln, int lf, float Rn, float tita);

            void draw_trail(IplImage* img, CvPoint* pt);

            void draw_crosshair(IplImage* img, CvPoint centre, int circle_radius, int line_radius, CvScalar colour);

            void draw_pin(IplImage* img, CvPoint3D32f normal, float slant, float tita, CvScalar colour);

            void print_text(IplImage* img, int counter, CvScalar colour);

            void init_geometric_model();

            glm::mat4  poseEstimation(Mat_<double> detected_landmarks);

            glm::mat4  poseEstimation(vector<cv::Point> detected_landmarks, Mat &img);

            glm::mat4  poseEstimation_g2o(vector<cv::Point> detected_landmarks, Mat &img);

            glm::mat4  poseEstimationByFaceNormal(Point3f face_normal, double roll_radian, Mat &img);

            glm::mat4  poseEstimation_CLM(Point3f face_normal, double roll_radian, Mat &img);

            void getPose();

    };
}
