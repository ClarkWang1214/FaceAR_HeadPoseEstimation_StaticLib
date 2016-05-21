///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include <FaceARTracker.h>

using namespace FaceARTracker;
using namespace cv;

Vec6d FaceARTracker::GetPoseCamera(const FaceAR& facear_model, double fx, double fy, double cx, double cy, const FaceARParameters& params)
{
    if(!facear_model.detected_landmarks.empty() && facear_model.params_global[0] != 0)
    {
        double Z = fx / facear_model.params_global[0];

        double X = ((facear_model.params_global[4] - cx) * (1.0/fx)) * Z;
        double Y = ((facear_model.params_global[5] - cy) * (1.0/fy)) * Z;

        return Vec6d(X, Y, Z, facear_model.params_global[1], facear_model.params_global[2], facear_model.params_global[3]);
    }
    else
    {
        return Vec6d(0,0,0,0,0,0);
    }
}

Vec6d FaceARTracker::GetPoseCameraPlane(const FaceAR& facear_model, double fx, double fy, double cx, double cy, const FaceARParameters& params)
{
    if(!facear_model.detected_landmarks.empty() && facear_model.params_global[0] != 0 && facear_model.tracking_initialised)
    {
        double Z = fx / facear_model.params_global[0];

        double X = ((facear_model.params_global[4] - cx) * (1.0/fx)) * Z;
        double Y = ((facear_model.params_global[5] - cy) * (1.0/fy)) * Z;

        double z_x = cv::sqrt(X * X + Z * Z);
        double eul_x = atan2(Y, z_x);

        double z_y = cv::sqrt(Y * Y + Z * Z);
        double eul_y = -atan2(X, z_y);

        Matx33d camera_rotation = FaceARTracker::Euler2RotationMatrix(Vec3d(eul_x, eul_y, 0));
        Matx33d head_rotation = FaceARTracker::AxisAngle2RotationMatrix(Vec3d(facear_model.params_global[1], facear_model.params_global[2], facear_model.params_global[3]));

        Matx33d corrected_rotation = camera_rotation.t() * head_rotation;

        Vec3d euler_corrected = FaceARTracker::RotationMatrix2Euler(corrected_rotation);

        return Vec6d(X, Y, Z, euler_corrected[0], euler_corrected[1], euler_corrected[2]);
    }
    else
    {
        return Vec6d(0,0,0,0,0,0);
    }
}

Vec6d FaceARTracker::GetCorrectedPoseCameraPlane(const FaceAR& facear_model, double fx, double fy, double cx, double cy, const FaceARParameters& params)
{
    if(!facear_model.detected_landmarks.empty() && facear_model.params_global[0] != 0 && facear_model.tracking_initialised)
    {
        double Z = fx / facear_model.params_global[0];

        double X = ((facear_model.params_global[4] - cx) * (1.0/fx)) * Z;
        double Y = ((facear_model.params_global[5] - cy) * (1.0/fy)) * Z;

        // 2D points
        Mat_<double> landmarks_2D = facear_model.detected_landmarks;

        landmarks_2D = landmarks_2D.reshape(1, 2).t();

        // 3D points
        Mat_<double> landmarks_3D;
        facear_model.pdm.CalcShape3D(landmarks_3D, facear_model.params_local);

        landmarks_3D = landmarks_3D.reshape(1, 3).t();

        Matx33d camera_matrix(fx, 0, cx, 0, fy, cy, 0, 0, 1);

        Vec3d vec_trans(X, Y, Z);
        Vec3d vec_rot(facear_model.params_global[1], facear_model.params_global[2], facear_model.params_global[3]);

        cv::solvePnP(landmarks_3D, landmarks_2D, camera_matrix, Mat(), vec_rot, vec_trans, true);

        Vec3d euler = FaceARTracker::AxisAngle2Euler(vec_rot);

        return Vec6d(vec_trans[0], vec_trans[1], vec_trans[2], vec_rot[0], vec_rot[1], vec_rot[2]);
    }
    else
    {
        return Vec6d(0,0,0,0,0,0);
    }
}

Vec6d FaceARTracker::GetCorrectedPoseCamera(const FaceAR& facear_model, double fx, double fy, double cx, double cy, const FaceARParameters& params)
{
    if(!facear_model.detected_landmarks.empty() && facear_model.params_global[0] != 0)
    {

        double Z = fx / facear_model.params_global[0];

        double X = ((facear_model.params_global[4] - cx) * (1.0/fx)) * Z;
        double Y = ((facear_model.params_global[5] - cy) * (1.0/fy)) * Z;

        Mat_<double> landmarks_3D;
        facear_model.pdm.CalcShape3D(landmarks_3D, facear_model.params_local);

        landmarks_3D = landmarks_3D.reshape(1, 3).t();

        Mat_<double> landmarks_2D = facear_model.detected_landmarks;

        landmarks_2D = landmarks_2D.reshape(1, 2).t();

        Matx33d camera_matrix(fx, 0, cx, 0, fy, cy, 0, 0, 1);

        Vec3d vec_trans(X, Y, Z);
        Vec3d vec_rot(facear_model.params_global[1], facear_model.params_global[2], facear_model.params_global[3]);

        cv::solvePnP(landmarks_3D, landmarks_2D, camera_matrix, Mat(), vec_rot, vec_trans, true);

        double z_x = cv::sqrt(vec_trans[0] * vec_trans[0] + vec_trans[2] * vec_trans[2]);
        double eul_x = atan2(vec_trans[1], z_x);

        double z_y = cv::sqrt(vec_trans[1] * vec_trans[1] + vec_trans[2] * vec_trans[2]);
        double eul_y = -atan2(vec_trans[0], z_y);

        Matx33d camera_rotation = FaceARTracker::Euler2RotationMatrix(Vec3d(eul_x, eul_y, 0));
        Matx33d head_rotation = FaceARTracker::AxisAngle2RotationMatrix(vec_rot);

        Matx33d corrected_rotation = camera_rotation * head_rotation;

        Vec3d euler_corrected = FaceARTracker::RotationMatrix2Euler(corrected_rotation);

        return Vec6d(vec_trans[0], vec_trans[1], vec_trans[2], euler_corrected[0], euler_corrected[1], euler_corrected[2]);
    }
    else
    {
        return Vec6d(0,0,0,0,0,0);
    }
}

void UpdateTemplate_Clark(const Mat &original_image, const Mat_<uchar> &grayscale_image, FaceAR& facear_model, Mat &saved_image )
{
    Rect bounding_box;
    facear_model.pdm.CalcBoundingBox(bounding_box, facear_model.params_global, facear_model.params_local);
    bounding_box = bounding_box & Rect(0, 0, grayscale_image.cols, grayscale_image.rows);

    facear_model.face_template = grayscale_image(bounding_box).clone();
    saved_image = original_image(bounding_box).clone();

//    imshow("bounding_box", saved_image);
//    imshow("original_image", original_image);
//    imshow("grayscale_image", grayscale_image);
//    waitKey(1);
}

void UpdateTemplate(const Mat_<uchar> &grayscale_image, FaceAR& facear_model)
{
    Rect bounding_box;
    facear_model.pdm.CalcBoundingBox(bounding_box, facear_model.params_global, facear_model.params_local);
    bounding_box = bounding_box & Rect(0, 0, grayscale_image.cols, grayscale_image.rows);

    facear_model.face_template = grayscale_image(bounding_box).clone();
    imshow("bounding_box", grayscale_image(bounding_box));
    waitKey(1);
}

void CorrectGlobalParametersVideo(const Mat_<uchar> &grayscale_image, FaceAR& facear_model, const FaceARParameters& params)
{
    Rect init_box;
    facear_model.pdm.CalcBoundingBox(init_box, facear_model.params_global, facear_model.params_local);

    Rect roi(init_box.x - init_box.width/2, init_box.y - init_box.height/2, init_box.width * 2, init_box.height * 2);
    roi = roi & Rect(0, 0, grayscale_image.cols, grayscale_image.rows);

    int off_x = roi.x;
    int off_y = roi.y;

    double scaling = params.face_template_scale / facear_model.params_global[0];
    Mat_<uchar> image;
    if(scaling < 1)
    {
        cv::resize(facear_model.face_template, facear_model.face_template, Size(), scaling, scaling);
        cv::resize(grayscale_image(roi), image, Size(), scaling, scaling);
    }
    else
    {
        scaling = 1;
        image = grayscale_image(roi).clone();
    }
    Mat corr_out;
    cv::matchTemplate(image, facear_model.face_template, corr_out, CV_TM_CCOEFF_NORMED);

    int max_loc[2];

    cv::minMaxIdx(corr_out, NULL, NULL, NULL, max_loc);

    Rect_<double> out_bbox(max_loc[1]/scaling + off_x, max_loc[0]/scaling + off_y, facear_model.face_template.rows / scaling, facear_model.face_template.cols / scaling);

    double shift_x = out_bbox.x - (double)init_box.x;
    double shift_y = out_bbox.y - (double)init_box.y;

    facear_model.params_global[4] = facear_model.params_global[4] + shift_x;
    facear_model.params_global[5] = facear_model.params_global[5] + shift_y;

}

bool FaceARTracker::DetectLandmarksInVideo_Clark(const Mat &original_image, const Mat_<uchar> &grayscale_image, const Mat_<float> &depth_image, FaceAR& facear_model, FaceARParameters& params, Mat &saved_image)
{
    bool initial_detection = !facear_model.tracking_initialised;

    if(facear_model.tracking_initialised)
    {
        if(!facear_model.detection_success)
        {
            params.window_sizes_current = params.window_sizes_init;
        }
        else
        {
            params.window_sizes_current = params.window_sizes_small;
        }

//        cout<<"params.use_face_template: "<<params.use_face_template<<endl;  //   0
//        cout<<"facear_model.face_template.empty(): "<<facear_model.face_template.empty()<<endl;  //  0
//        cout<<"facear_model.detection_success: "<<facear_model.detection_success<<endl; //   1
        if(params.use_face_template && !facear_model.face_template.empty() && facear_model.detection_success)
        {
            CorrectGlobalParametersVideo(grayscale_image, facear_model, params);
        }

        bool track_success = facear_model.DetectLandmarks(grayscale_image, depth_image, params);
        if(!track_success)
        {
            facear_model.failures_in_a_row++;
        }
        else
        {
//            cout<<"track_success"<<endl;
            facear_model.failures_in_a_row = -1;


            // TODO:
            // update bounding box area
//            UpdateTemplate(grayscale_image, facear_model);

            saved_image.release();
            UpdateTemplate_Clark(original_image, grayscale_image, facear_model, saved_image);
        }
    }

    if((!facear_model.tracking_initialised && (facear_model.failures_in_a_row + 1) % (params.reinit_video_every * 6) == 0)
        || (facear_model.tracking_initialised && !facear_model.detection_success && params.reinit_video_every > 0 && facear_model.failures_in_a_row % params.reinit_video_every == 0))
    {

        Rect_<double> bounding_box;
        if(facear_model.face_detector_HAAR.empty())
        {
            cout<<"face_detector_HAAR"<<endl;
            facear_model.face_detector_HAAR.load(params.face_detector_location);
            facear_model.face_detector_location = params.face_detector_location;
        }

        Point preference_det(-1, -1);
        cout<<"facear_model.preference_det.x"<<facear_model.preference_det.x<<endl;
        cout<<"facear_model.preference_det.y"<<facear_model.preference_det.y<<endl;
        if(facear_model.preference_det.x != -1 && facear_model.preference_det.y != -1)
        {

            preference_det.x = facear_model.preference_det.x * grayscale_image.cols;
            preference_det.y = facear_model.preference_det.y * grayscale_image.rows;
            facear_model.preference_det = Point(-1, -1);
        }

        bool face_detection_success;
        if(params.curr_face_detector == FaceARParameters::HOG_SVM_DETECTOR)
        {
            double confidence;
            face_detection_success = FaceARTracker::DetectSingleFaceHOG(bounding_box, grayscale_image, facear_model.face_detector_HOG, confidence, preference_det);
        }
        else if(params.curr_face_detector == FaceARParameters::HAAR_DETECTOR)
        {
            face_detection_success = FaceARTracker::DetectSingleFace(bounding_box, grayscale_image, facear_model.face_detector_HAAR, preference_det);
        }

        cout<<"face_detection_success: "<<face_detection_success<<endl; //   1
        if(face_detection_success)
        {
            facear_model.tracking_initialised = true;
            Vec6d params_global_init = facear_model.params_global;
            Mat_<double> params_local_init = facear_model.params_local.clone();
            double likelihood_init = facear_model.model_likelihood;
            Mat_<double> detected_landmarks_init = facear_model.detected_landmarks.clone();
            Mat_<double> landmark_likelihoods_init = facear_model.landmark_likelihoods.clone();
            facear_model.params_local.setTo(0);
            facear_model.pdm.CalcParams(facear_model.params_global, bounding_box, facear_model.params_local);
            params.window_sizes_current = params.window_sizes_init;
            bool landmark_detection_success = facear_model.DetectLandmarks(grayscale_image, depth_image, params);
            if(!initial_detection && !landmark_detection_success)
            {
                facear_model.params_global = params_global_init;
                facear_model.params_local = params_local_init.clone();
                facear_model.pdm.CalcShape2D(facear_model.detected_landmarks, facear_model.params_local, facear_model.params_global);
                facear_model.model_likelihood = likelihood_init;
                facear_model.detected_landmarks = detected_landmarks_init.clone();
                facear_model.landmark_likelihoods = landmark_likelihoods_init.clone();

                return false;
            }
            else
            {
                facear_model.failures_in_a_row = -1;

                /// TODO :
//                UpdateTemplate(grayscale_image, facear_model);

                saved_image.release();
                UpdateTemplate_Clark(original_image, grayscale_image, facear_model, saved_image);

                return true;
            }
        }
    }

    if(!facear_model.tracking_initialised)
    {
        facear_model.failures_in_a_row++;
    }

    if(	facear_model.failures_in_a_row > 100)
    {
        facear_model.tracking_initialised = false;
    }

    return facear_model.detection_success;

}


bool FaceARTracker::DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, const Mat_<float> &depth_image, FaceAR& facear_model, FaceARParameters& params)
{
    bool initial_detection = !facear_model.tracking_initialised;

    if(facear_model.tracking_initialised)
    {
        if(!facear_model.detection_success)
        {
            params.window_sizes_current = params.window_sizes_init;
        }
        else
        {
            params.window_sizes_current = params.window_sizes_small;
        }

        cout<<"params.use_face_template: "<<params.use_face_template<<endl;  //   0
        cout<<"facear_model.face_template.empty(): "<<facear_model.face_template.empty()<<endl;  //  0
        cout<<"facear_model.detection_success: "<<facear_model.detection_success<<endl; //   1
        if(params.use_face_template && !facear_model.face_template.empty() && facear_model.detection_success)
        {
            CorrectGlobalParametersVideo(grayscale_image, facear_model, params);
        }

        bool track_success = facear_model.DetectLandmarks(grayscale_image, depth_image, params);
        if(!track_success)
        {
            facear_model.failures_in_a_row++;
        }
        else
        {
            cout<<"track_success"<<endl;
            facear_model.failures_in_a_row = -1;


            // TODO:
            // update bounding box area
            UpdateTemplate(grayscale_image, facear_model);


        }
    }

    if((!facear_model.tracking_initialised && (facear_model.failures_in_a_row + 1) % (params.reinit_video_every * 6) == 0)
        || (facear_model.tracking_initialised && !facear_model.detection_success && params.reinit_video_every > 0 && facear_model.failures_in_a_row % params.reinit_video_every == 0))
    {

        Rect_<double> bounding_box;
        if(facear_model.face_detector_HAAR.empty())
        {
            cout<<"face_detector_HAAR"<<endl;
            facear_model.face_detector_HAAR.load(params.face_detector_location);
            facear_model.face_detector_location = params.face_detector_location;
        }

        Point preference_det(-1, -1);
        cout<<"facear_model.preference_det.x"<<facear_model.preference_det.x<<endl;
        cout<<"facear_model.preference_det.y"<<facear_model.preference_det.y<<endl;
        if(facear_model.preference_det.x != -1 && facear_model.preference_det.y != -1)
        {

            preference_det.x = facear_model.preference_det.x * grayscale_image.cols;
            preference_det.y = facear_model.preference_det.y * grayscale_image.rows;
            facear_model.preference_det = Point(-1, -1);
        }

        bool face_detection_success;
        if(params.curr_face_detector == FaceARParameters::HOG_SVM_DETECTOR)
        {
            double confidence;
            face_detection_success = FaceARTracker::DetectSingleFaceHOG(bounding_box, grayscale_image, facear_model.face_detector_HOG, confidence, preference_det);
        }
        else if(params.curr_face_detector == FaceARParameters::HAAR_DETECTOR)
        {
            face_detection_success = FaceARTracker::DetectSingleFace(bounding_box, grayscale_image, facear_model.face_detector_HAAR, preference_det);
        }

        cout<<"face_detection_success: "<<face_detection_success<<endl; //   1
        if(face_detection_success)
        {
            facear_model.tracking_initialised = true;
            Vec6d params_global_init = facear_model.params_global;
            Mat_<double> params_local_init = facear_model.params_local.clone();
            double likelihood_init = facear_model.model_likelihood;
            Mat_<double> detected_landmarks_init = facear_model.detected_landmarks.clone();
            Mat_<double> landmark_likelihoods_init = facear_model.landmark_likelihoods.clone();
            facear_model.params_local.setTo(0);
            facear_model.pdm.CalcParams(facear_model.params_global, bounding_box, facear_model.params_local);
            params.window_sizes_current = params.window_sizes_init;
            bool landmark_detection_success = facear_model.DetectLandmarks(grayscale_image, depth_image, params);
            if(!initial_detection && !landmark_detection_success)
            {
                facear_model.params_global = params_global_init;
                facear_model.params_local = params_local_init.clone();
                facear_model.pdm.CalcShape2D(facear_model.detected_landmarks, facear_model.params_local, facear_model.params_global);
                facear_model.model_likelihood = likelihood_init;
                facear_model.detected_landmarks = detected_landmarks_init.clone();
                facear_model.landmark_likelihoods = landmark_likelihoods_init.clone();

                return false;
            }
            else
            {
                facear_model.failures_in_a_row = -1;

                /// TODO :
                UpdateTemplate(grayscale_image, facear_model);

                return true;
            }
        }
    }

    if(!facear_model.tracking_initialised)
    {
        facear_model.failures_in_a_row++;
    }

    if(	facear_model.failures_in_a_row > 100)
    {
        facear_model.tracking_initialised = false;
    }

    return facear_model.detection_success;

}

bool FaceARTracker::DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, const Mat_<float> &depth_image, const Rect_<double> bounding_box, FaceAR& facear_model, FaceARParameters& params)
{
    if(bounding_box.width > 0)
    {
        facear_model.params_local.setTo(0);
        facear_model.pdm.CalcParams(facear_model.params_global, bounding_box, facear_model.params_local);
        facear_model.tracking_initialised = true;
    }

    return DetectLandmarksInVideo(grayscale_image, depth_image, facear_model, params);

}

bool FaceARTracker::DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, FaceAR& facear_model, FaceARParameters& params)
{
    return DetectLandmarksInVideo(grayscale_image, Mat_<float>(), facear_model, params);
}

bool FaceARTracker::DetectLandmarksInVideo(const Mat_<uchar> &grayscale_image, const Rect_<double> bounding_box, FaceAR& facear_model, FaceARParameters& params)
{
    return DetectLandmarksInVideo(grayscale_image, Mat_<float>(), facear_model, params);
}

//================================================================================================================
bool FaceARTracker::DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, const Mat_<float> depth_image, const Rect_<double> bounding_box, FaceAR& facear_model, FaceARParameters& params)
{
    vector<Vec3d> rotation_hypotheses;

    if(params.multi_view)
    {
        rotation_hypotheses.push_back(Vec3d(0,0,0));
        rotation_hypotheses.push_back(Vec3d(0,0.5236,0));
        rotation_hypotheses.push_back(Vec3d(0,-0.5236,0));
        rotation_hypotheses.push_back(Vec3d(0.5236,0,0));
        rotation_hypotheses.push_back(Vec3d(-0.5236,0,0));
    }
    else
    {
        rotation_hypotheses.push_back(Vec3d(0,0,0));
    }
    params.window_sizes_current = params.window_sizes_init;
    double best_likelihood;
    Vec6d best_global_parameters;
    Mat_<double> best_local_parameters;
    Mat_<double> best_detected_landmarks;
    Mat_<double> best_landmark_likelihoods;
    bool best_success;

    for(size_t hypothesis = 0; hypothesis < rotation_hypotheses.size(); ++hypothesis)
    {
        facear_model.params_local.setTo(0.0);
        facear_model.pdm.CalcParams(facear_model.params_global, bounding_box, facear_model.params_local, rotation_hypotheses[hypothesis]);

        bool success = facear_model.DetectLandmarks(grayscale_image, depth_image, params);

        if(hypothesis == 0 || best_likelihood < facear_model.model_likelihood)
        {
            best_likelihood = facear_model.model_likelihood;
            best_global_parameters = facear_model.params_global;
            best_local_parameters = facear_model.params_local.clone();
            best_detected_landmarks = facear_model.detected_landmarks.clone();
            best_landmark_likelihoods = facear_model.landmark_likelihoods.clone();
            best_success = success;

        }

    }
    facear_model.model_likelihood = best_likelihood;
    facear_model.params_global = best_global_parameters;
    facear_model.params_local = best_local_parameters.clone();
    facear_model.detected_landmarks = best_detected_landmarks.clone();
    facear_model.detection_success = best_success;
    facear_model.landmark_likelihoods = best_landmark_likelihoods.clone();
    return best_success;
}

bool FaceARTracker::DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, const Mat_<float> depth_image, FaceAR& facear_model, FaceARParameters& params)
{

    Rect_<double> bounding_box;
    if(facear_model.face_detector_HAAR.empty())
    {
        facear_model.face_detector_HAAR.load(params.face_detector_location);
        facear_model.face_detector_location = params.face_detector_location;
    }

    if(params.curr_face_detector == FaceARParameters::HOG_SVM_DETECTOR)
    {
        double confidence;
        FaceARTracker::DetectSingleFaceHOG(bounding_box, grayscale_image, facear_model.face_detector_HOG, confidence);
    }
    else if(params.curr_face_detector == FaceARParameters::HAAR_DETECTOR)
    {
        FaceARTracker::DetectSingleFace(bounding_box, grayscale_image, facear_model.face_detector_HAAR);
    }

    return DetectLandmarksInImage(grayscale_image, depth_image, bounding_box, facear_model, params);

}

bool FaceARTracker::DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, const Rect_<double> bounding_box, FaceAR& facear_model, FaceARParameters& params)
{
    return DetectLandmarksInImage(grayscale_image, Mat_<float>(), bounding_box, facear_model, params);
}

bool FaceARTracker::DetectLandmarksInImage(const Mat_<uchar> &grayscale_image, FaceAR& facear_model, FaceARParameters& params)
{
    return DetectLandmarksInImage(grayscale_image, Mat_<float>(), facear_model, params);
}


