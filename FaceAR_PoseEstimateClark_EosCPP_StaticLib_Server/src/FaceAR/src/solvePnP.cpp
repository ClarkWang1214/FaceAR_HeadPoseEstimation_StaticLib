
#include "solvePnP.h"

cv::Mat solvePnP_clark(cv::Mat &img, FaceARTracker::FaceAR &facear_model,
                       float &fx, float &fy, float &cx, float &cy)
{
    int max_d = MAX(img.rows,img.cols);

    cv::Mat camMatrix = (Mat_<double>(3,3) << max_d, 0,     img.cols/2.0,
                                              0,	 max_d, img.rows/2.0,
                                              0,	 0,	    1.0);
    cv::Mat_<double> shape_3D = facear_model.GetShape(fx, fy, cx, cy);
//    cv::Mat point_2d(2, 68, CV_64FC1);
//    cv::Mat point_3d(3, 68, CV_64FC1);
//    for ( size_t i = 0; i < facear_model.pdm.NumberOfPoints(); ++i )
//    {
//        point_2d.at<double>(0, i) = facear_model.detected_landmarks.at<double>(i);
//        point_2d.at<double>(1, i) = facear_model.detected_landmarks.at<double>(i+facear_model.pdm.NumberOfPoints());

//        point_3d.at<double>(0, i) = shape_3D.at<double>(i);
//        point_3d.at<double>(1, i) = shape_3D.at<double>(i+facear_model.pdm.NumberOfPoints());
//        point_3d.at<double>(2, i) = shape_3D.at<double>(i+2*facear_model.pdm.NumberOfPoints());
//    }

    std::vector<cv::Point3f> modelPoints;
    std::vector<cv::Point2f> imagePoints;
    for (size_t i = 0; i < facear_model.pdm.NumberOfPoints(); ++i)
    {
        imagePoints.push_back(cv::Point2f((float)(facear_model.detected_landmarks.at<double>(i)),
                                          (float)(facear_model.detected_landmarks.at<double>(i+facear_model.pdm.NumberOfPoints()))));
        modelPoints.push_back(cv::Point3f(shape_3D.at<double>(i),
                                          shape_3D.at<double>(i+facear_model.pdm.NumberOfPoints()),
                                          shape_3D.at<double>(i+2*facear_model.pdm.NumberOfPoints())));
    }
    cv::Mat ip(imagePoints);
    cv::Mat op(modelPoints);
    std::vector<double> rv(3), tv(3);
    cv::Mat rvec(rv),tvec(tv);
    double _dc[] = {0,0,0,0};
//    std::cout << ip << std::endl << std::endl;
//    std::cout << op << std::endl << std::endl;
//    std::cout << camMatrix << std::endl << std::endl;
    solvePnP(op, ip, camMatrix, cv::Mat(1,4,CV_64FC1,_dc), rvec, tvec, false, CV_EPNP);

    double rot[9] = {0};
    cv::Mat rotM(3, 3, CV_64FC1, rot);
    cv::Rodrigues(rvec, rotM);
    double* _r = rotM.ptr<double>();
//    printf("rotation mat: \n %.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n",_r[0],_r[1],_r[2],_r[3],_r[4],_r[5],_r[6],_r[7],_r[8]);

//    printf("trans vec: \n %.3f %.3f %.3f\n",tv[0],tv[1],tv[2]);

    cv::Mat _pm(3, 4, CV_64FC1);
    _pm.at<double>(0,0) = _r[0]; _pm.at<double>(0,1) = _r[1]; _pm.at<double>(0,2) = _r[2]; _pm.at<double>(0,3) = tv[0];
    _pm.at<double>(1,0) = _r[3]; _pm.at<double>(1,1) = _r[4]; _pm.at<double>(1,2) = _r[5]; _pm.at<double>(1,3) = tv[1];
    _pm.at<double>(2,0) = _r[6]; _pm.at<double>(2,1) = _r[7]; _pm.at<double>(2,2) = _r[8]; _pm.at<double>(2,3) = tv[2];

    return _pm;
}
