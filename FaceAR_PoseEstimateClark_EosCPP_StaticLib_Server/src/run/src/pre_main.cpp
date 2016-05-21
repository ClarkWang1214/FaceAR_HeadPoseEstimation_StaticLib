/////////////////////////////////////////////////////////////////////////////////

#include "pre_main.h"
#include "FaceAR_core.h"
#include "solvePnP.h"
#include "LandmarksImproved.h"
#include "PoseEstimation.h"
#include "eos_run.h"
#include "GetGoodFrame.h"

//#define PI (22/7.0)

using namespace std;
using namespace cv;
using namespace clarkPoseEsti;


map<int, double> clark_dlib_facetracker(const std::string input_video, vector<Mat>& saved_images_Vec)
{
    PoseEsti* poseEsti = new PoseEsti();

    float fx = 500, fy = 500, cx = 0, cy = 0;
    vector<string> arguments;
    arguments.push_back("");
    FaceARTracker::FaceARParameters clm_parameters(arguments);


    cv::String device = cv::String(input_video);
    FaceARTracker::get_camera_params(device, fx, fy, cx, cy, arguments);
    FaceARTracker::FaceAR facear_model(clm_parameters.model_location);


    map<int, double> tmp_map;
    VideoCapture video_capture;
    video_capture = VideoCapture( device );
    if( !video_capture.isOpened() )
    {
        std::cout << "Failed to open video source" << std::endl;
        abort();
    }

    Mat captured_image;
    video_capture >> captured_image;



    cx = captured_image.cols / 2.0f;
    cy = captured_image.rows / 2.0f;

    int frame_count = 0;
    int improve = 0;
    std::cout << "======Starting tracking======" << std::endl;


    // For measuring the timings
    int64 t1,t0 = cv::getTickCount();
    double fps = 10;
    while(!captured_image.empty())
    {
        Mat_<float> depth_image;
        Mat_<uchar> grayscale_image;

        if(captured_image.channels() == 3)
            cvtColor(captured_image, grayscale_image, CV_BGR2GRAY);
        else
            grayscale_image = captured_image.clone();


        //FaceARTracker::DetectLandmarksInVideo(grayscale_image, depth_image, facear_model, clm_parameters);
        Mat saved_image = captured_image.clone();
//        FaceARTracker::DetectLandmarksInVideo(grayscale_image, depth_image, facear_model, clm_parameters);
        FaceARTracker::DetectLandmarksInVideo_Clark(captured_image, grayscale_image, depth_image, facear_model, clm_parameters, saved_image);

        saved_images_Vec.push_back(saved_image);


        // Vec6d(vec_trans[0], vec_trans[1], vec_trans[2], euler_corrected[0], euler_corrected[1], euler_corrected[2])
//        Vec6d pose_estimate_FaceAR;
        FaceARTracker::GetCorrectedPoseCamera(facear_model, fx, fy, cx, cy, clm_parameters);

//        cout<<"============s[0]: "<<facear_model.params_global[0]<<endl;
//        cout<<"============euler[1]: "<<facear_model.params_global[1]<<endl;
//        cout<<"============euler[2]: "<<facear_model.params_global[2]<<endl;
//        cout<<"============euler[3]: "<<facear_model.params_global[3]<<endl;
//        cout<<"============trans[4]: "<<facear_model.params_global[4]<<endl;
//        cout<<"============trans[5]: "<<facear_model.params_global[5]<<endl;

//        cout<<"============vec_trans[0]: "<<pose_estimate_FaceAR[0]<<endl;
//        cout<<"============vec_trans[1]: "<<pose_estimate_FaceAR[1]<<endl;
//        cout<<"============vec_trans[2]: "<<pose_estimate_FaceAR[2]<<endl;
//        cout<<"============euler_corrected[3]: "<<pose_estimate_FaceAR[3]<<endl;
//        cout<<"============euler_corrected[4]: "<<pose_estimate_FaceAR[4]<<endl;
//        cout<<"============euler_corrected[5]: "<<pose_estimate_FaceAR[5]<<endl;

        // Visualising the results
        double detection_certainty = facear_model.detection_certainty;
        double visualisation_boundary = 0.2;
        if(detection_certainty < visualisation_boundary)
        {
            /// TODO : landmarks improve
            landmarksImproved(facear_model, frame_count, improve);

            /// TODO : plot the face point and point number in the image
//            FaceARTracker::Draw(captured_image, facear_model);


            if(detection_certainty > 1)
                detection_certainty = 1;
            if(detection_certainty < -1)
                detection_certainty = -1;

            ///////////////////////////////////////////////////////////////////////
            improve++;
            ///////////////////////////////////////////////////////////////////////
            //// eos getgoodframe
            ///3D points
//            Mat_<double> shape_3D = facear_model.GetShape(fx, fy, cx, cy);
//            GoodFrame tempFrame(frame_count, diff_curr_pre, captured_image, facear_model.detected_landmarks, shape_3D);
//            AllFrame.push_back(tempFrame);


            ////////////////////////////////////////////////////////////////////////////clark  2016.5.12
            Mat_<double> _detected_landmarks = facear_model.detected_landmarks.clone();
            glm::mat4 rotateMatrix = poseEsti->poseEstimation(_detected_landmarks);
            tmp_map[frame_count] = rotateMatrix[3][1];

            frame_count++;
        }

        // Work out the framerate
        if(frame_count % 10 == 0)
        {
            t1 = cv::getTickCount();
            fps = 10.0 / (double(t1-t0)/cv::getTickFrequency());
            t0 = t1;
        }

        // Write out the framerate on the image before displaying it
        char fpsC[255];
        sprintf(fpsC, "%d", (int)fps);
        string fpsSt("FPS:");
        fpsSt += fpsC;
        cv::putText(captured_image, fpsSt, cv::Point(10,20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0));

        /// show image
        imshow("tracking_result", captured_image);
        cv::waitKey(1);
        ////////////////////////////////////////////////////////////////////////////
        video_capture >> captured_image;

    }

    return tmp_map;
}


std::string pre_main(const std::string input_video, const std::string outtext)
{
    std::string txt_file_output;
    cv::String device = cv::String(input_video);
    boost::filesystem::path input_video_root = boost::filesystem::path(device).parent_path();
    boost::filesystem::path input_video_parent_root = boost::filesystem::path(device).parent_path().parent_path();
    boost::filesystem::path input_video_name = boost::filesystem::path(device).filename();
    string input_video_basename = boost::filesystem::basename(input_video_name);
    string tmp_root = "poseInfoTxT";
    string tmp_obj_root = "poseInfoTxT/obj";
    string txt_file_suf = ".txt";
    
    string obj_output_keegan = ( input_video_parent_root / tmp_obj_root / input_video_basename ).string();

    //    int device = 0;
    float fx = 500, fy = 500, cx = 0, cy = 0;
    bool done = false;
    vector<string> arguments;
    arguments.push_back("/home/ren/MyGit/CLM_Server_Upload/FaceAR_PoseEstimateClark_EosCPP_StaticLib_Server_Run/bin"); //string(argv[0])
    FaceARTracker::FaceARParameters clm_parameters(arguments);
    FaceARTracker::get_camera_params(device, fx, fy, cx, cy, arguments);
    FaceARTracker::FaceAR facear_model(clm_parameters.model_location);
    PoseEsti* poseEsti = new PoseEsti();

    /// TODO : eos_init
    /// read eos file
    fs::path modelfile("../model/share/sfm_shape_3448.bin");
    fs::path mappingsfile("../model/share/ibug2did.txt");
    fs::path outputfile("out");
    fs::path isomapfile/* imagefile, landmarksfile,*/;

    while(!done)
    {
        VideoCapture video_capture;
        video_capture = VideoCapture( device );
        if( !video_capture.isOpened() )
        {
            std::cout << "Failed to open video source" << std::endl;
            abort();
        }

        Mat captured_image, captured_image_PnP;;
        video_capture >> captured_image;

        cx = captured_image.cols / 2.0f;
        cy = captured_image.rows / 2.0f;

        int frame_count = 0;
        int improve = 0;
        std::cout << "Starting tracking" << std::endl;
        /////////////////
        /// eos getgoodframe
        ///
        double XYZ_Pose_Pre, XYZ_Pose, XYZ_Pose_min;
        FaceARTracker::FaceAR facear_model_min, facear_model_first; ///
        cv::Mat facear_Mat;
        int facear_count = 0;
        cv::Vec6d vec6d_curr, vec6d_pre;
        cv::Vec6d diff_curr_pre;
        std::vector<GoodFrame> AllFrame;
        ////////////////end

        while(!captured_image.empty())
        {
            captured_image.copyTo(captured_image_PnP);
            Mat_<float> depth_image;
            Mat_<uchar> grayscale_image;
            if(captured_image.channels() == 3) {
                cvtColor(captured_image, grayscale_image, CV_BGR2GRAY);
            } else {
                grayscale_image = captured_image.clone();
            }

            bool detection_success = FaceARTracker::DetectLandmarksInVideo(grayscale_image, depth_image, facear_model, clm_parameters);
            Vec6d pose_estimate_FaceAR;

            // Vec6d(vec_trans[0], vec_trans[1], vec_trans[2], euler_corrected[0], euler_corrected[1], euler_corrected[2])
            pose_estimate_FaceAR = FaceARTracker::GetCorrectedPoseCamera(facear_model, fx, fy, cx, cy, clm_parameters);

            cout<<"============s[0]: "<<facear_model.params_global[0]<<endl;
            cout<<"============euler[1]: "<<facear_model.params_global[1]<<endl;
            cout<<"============euler[2]: "<<facear_model.params_global[2]<<endl;
            cout<<"============euler[3]: "<<facear_model.params_global[3]<<endl;
            cout<<"============trans[4]: "<<facear_model.params_global[4]<<endl;
            cout<<"============trans[5]: "<<facear_model.params_global[5]<<endl;

            cout<<"============vec_trans[0]: "<<pose_estimate_FaceAR[0]<<endl;
            cout<<"============vec_trans[1]: "<<pose_estimate_FaceAR[1]<<endl;
            cout<<"============vec_trans[2]: "<<pose_estimate_FaceAR[2]<<endl;
            cout<<"============euler_corrected[3]: "<<pose_estimate_FaceAR[3]<<endl;
            cout<<"============euler_corrected[4]: "<<pose_estimate_FaceAR[4]<<endl;
            cout<<"============euler_corrected[5]: "<<pose_estimate_FaceAR[5]<<endl;



            //            std::cout << "pose_estimate_FaceAR = " << pose_estimate_FaceAR[0] << pose_estimate_FaceAR[1] << pose_estimate_FaceAR[2]
            //                      << pose_estimate_FaceAR[3]*180.0/PI << pose_estimate_FaceAR[4]*180.0/PI << pose_estimate_FaceAR[5]*180.0/PI << std::endl;
            if(0 == frame_count)
            {
                XYZ_Pose = (0.2*pose_estimate_FaceAR[3] + 0.6*pose_estimate_FaceAR[4] + 0.2*pose_estimate_FaceAR[5])*180.0/PI;
                //cv::Vec3d(pose_estimate_FaceAR[3]*180.0/3.1415926, pose_estimate_FaceAR[4]*180.0/3.1415926, pose_estimate_FaceAR[5]*180.0/3.1415926);
                XYZ_Pose_min = XYZ_Pose;
                facear_model_min = facear_model;
                facear_model_first = facear_model;
                captured_image_PnP.copyTo(facear_Mat);
                facear_count = frame_count;
                ///
                vec6d_curr = pose_estimate_FaceAR;
            }
            else
            {
                XYZ_Pose_Pre = XYZ_Pose;
                XYZ_Pose = (std::abs(0.1*pose_estimate_FaceAR[3]) + std::abs(0.8*pose_estimate_FaceAR[4]) + std::abs(0.1*pose_estimate_FaceAR[5]))*180.0/PI;
                if (XYZ_Pose_min > XYZ_Pose)
                {
                    XYZ_Pose_min = XYZ_Pose;
                    facear_model_min = facear_model;
                    captured_image_PnP.copyTo(facear_Mat);
                    facear_count = frame_count;
                }
                ///
                vec6d_pre = vec6d_curr;
                vec6d_curr = pose_estimate_FaceAR;
                diff_curr_pre = vec6d_curr - vec6d_pre;
                std::cout <<  "{" << frame_count << "," << diff_curr_pre[0] << "," << diff_curr_pre[1] << "," << diff_curr_pre[2] << ","
                           << diff_curr_pre[3]*180.0/PI << "," << diff_curr_pre[4]*180.0/PI << "," << diff_curr_pre[5]*180.0/PI << "}," << std::endl;
                //                if( std::abs(diff_curr_pre[4]*180.0/PI) > 5 || std::abs(diff_curr_pre[0]) > 5 || std::abs(diff_curr_pre[1]) > 5 || std::abs(diff_curr_pre[2]) > 5 )
            }
            // std::cout << "XYZ_Pose = " << XYZ_Pose << std::endl;
            // std::cout << "XYZ_Pose = " << XYZ_Pose << std::endl;

            // Visualising the results
            double detection_certainty = facear_model.detection_certainty;
            double visualisation_boundary = 0.2;
            if(detection_certainty < visualisation_boundary)
            {
                /// TODO : landmarks improve
                landmarksImproved(facear_model, frame_count, improve);
                /// TODO : plot the face point and point number in the image
                FaceARTracker::Draw(captured_image, facear_model);

                if(detection_certainty > 1)
                    detection_certainty = 1;
                if(detection_certainty < -1)
                    detection_certainty = -1;
                //                /// TODO : RT [R t]   R_3*3  t_3*1  RT_3*4
                //                cv::Mat RT_Mat = solvePnP_clark(captured_image_PnP, facear_model, fx, fy, cx, cy);
                //                cout << "RT_Mat = " << RT_Mat << endl;
                ///////////////////////////////////////////////////////////////////
                /// Pose Estimate
                ///
                //cv::resize(captured_image, captured_image, cv::Size(960, 640), 0, 0, CV_INTER_LINEAR);
                //                cv::cvtColor(captured_image, captured_image, CV_BGR2RGBA);
                //                cv::flip(captured_image, captured_image, 0);
                //cout<<"TODO : poseEsti->poseEstimation"<<endl;
                //glm::mat4 rotateMatrix = poseEsti->poseEstimation(facear_model.detected_landmarks);
                //cout<<"rotateMatrix"<<endl<<
                //                            rotateMatrix[0][0]<<" "<<rotateMatrix[0][1]<<" "<<rotateMatrix[0][2]<<endl<<
                //                            rotateMatrix[1][0]<<" "<<rotateMatrix[1][1]<<" "<<rotateMatrix[1][2]<<endl<<
                //                            rotateMatrix[2][0]<<" "<<rotateMatrix[2][1]<<" "<<rotateMatrix[2][2]<<endl<<
                //                            rotateMatrix[3][0]<<" "<<rotateMatrix[3][1]<<" "<<rotateMatrix[3][2]<<endl;
                //cout<<"Mean_Feature_distance: "<<(float)poseEsti->Mean_Feature_distance<<endl;
                //cout<<"init_Mean_Feature_distance: "<<(float)poseEsti->init_Mean_Feature_distance<<endl;
                //cout<<"scale: "<<poseEsti->scale<<endl;

                ///////////////////////////////////////////////////////////////////////
                improve++;
                ///////////////////////////////////////////////////////////////////////
                //// eos getgoodframe
                ///3D points
                Mat_<double> shape_3D = facear_model.GetShape(fx, fy, cx, cy);
                GoodFrame tempFrame(frame_count, diff_curr_pre, captured_image,
                                    facear_model.detected_landmarks, shape_3D);
                AllFrame.push_back(tempFrame);
                /// end
                std::cout << "fuck__while" << std::endl;
            }
            std::cout << "fuck" << std::endl;
            /// show image
            //            namedWindow("tracking_result",1);
            imshow("tracking_result", captured_image);
            cv::waitKey(1);
            ////////////////////////////////////////////////////////////////////////////
            video_capture >> captured_image;
            // detect key presses
            //char character_press = cv::waitKey(1);
            //if(character_press == 'r')// restart the tracker
            //{
            //   facear_model.Reset();
            //}
            //else if(character_press=='q')// quit the application
            //{
            //return(0);
            //    break;
            //}
            frame_count++;
        }

        /// TODO : eos
        std::cout << "eos" << std::endl;
        render::Mesh mesh;
        getModel(modelfile, mappingsfile, outputfile, isomapfile, facear_model_min, facear_Mat, facear_count, mesh, obj_output_keegan);
        std::vector<cv::Point3f> modelPoints_min;
        Mat_<double> shape_3D_min = facear_model_min.GetShape(fx, fy, cx, cy);
        for (size_t i = 0; i < facear_model_min.pdm.NumberOfPoints(); ++i)
        {
            modelPoints_min.push_back(cv::Point3f(shape_3D_min.at<double>(i),
                                                  shape_3D_min.at<double>(i+facear_model.pdm.NumberOfPoints()),
                                                  shape_3D_min.at<double>(i+2*facear_model.pdm.NumberOfPoints())));
        }
        //save
        std::stringstream strImage_eos;
        strImage_eos << "data/images_eos/images_eos.jpg";
        imwrite(strImage_eos.str(), facear_Mat);
        std::ofstream landmark2dPointTxt_eos;
        std::stringstream strPoint2dTxtTitle_eos;
        strPoint2dTxtTitle_eos << "data/images_eos/landmarks_2d_eos.txt";
        landmark2dPointTxt_eos.open(strPoint2dTxtTitle_eos.str(), ios_base::out);
        for (int i = 0; i < 68; ++i)
        {
            landmark2dPointTxt_eos << facear_model_min.detected_landmarks.at<double>(i) << " "
                                   << facear_model_min.detected_landmarks.at<double>(i+68) << " "
                                   << endl;
        }
        landmark2dPointTxt_eos.close();
        std::ofstream landmark3dPointTxt_eos;
        std::stringstream strPoint3dTxtTitle_eos;
        strPoint3dTxtTitle_eos << "data/images_eos/landmarks_3d_eos.txt";
        landmark3dPointTxt_eos.open(strPoint3dTxtTitle_eos.str(), ios_base::out);
        Mat_<double> shape_3D_min_eos = facear_model_min.GetShape(fx, fy, cx, cy);
        for (int i = 0; i < 68; ++i)
        {
            landmark3dPointTxt_eos << shape_3D_min_eos.at<double>(i) << " "
                                   << shape_3D_min_eos.at<double>(i+68) << " "
                                   << shape_3D_min_eos.at<double>(i+2*68)
                                   << endl;
        }
        landmark3dPointTxt_eos.close();

        ///TODO : good Frame
        int frame_r = 5;

        ////////////////////////////////////////////////////////////////////////////clark  2016.5.11
        //        std::vector<int> GoodFrameID ;
        //        bool getgoodframe_OK = getGoodFrameLessBeta(AllFrame, frame_r, GoodFrameID);
        //        if ( !getgoodframe_OK ){
        //             cout<<"haha"<<endl;
        //             return output_RTtxt;
        //        }




        //        cout << "ID------------------------------------------------------------------------------------------------"<< endl;
        std::ofstream RT_matTxt_total;
        std::stringstream strRT_matTxtTitle;
        txt_file_output = outtext + input_video_basename + ".txt";
        //        strRT_matTxtTitle << txt_file_output; //"data/RT/RT_total.txt";
        RT_matTxt_total.open(txt_file_output, ios_base::out);


        ////////////////////////////////////////////////////////////////////////////clark  2016.5.11

        cout<<"AllFrame.size(): "<<AllFrame.size()<<endl;
        if(AllFrame.size() == 0)
            return "-1";

        for(int i=0; i<AllFrame.size(); ++i){
            //            cout<<"    i: "<<i<<endl;
            GoodFrame tempGoodFrame = AllFrame.at(i);
            //            cout<<"Clark Test 1!!!!!!!!: "<<i<<endl;
            /// SolvePnP
            //            cv::Mat RTMmat = tempGoodFrame.getRT(modelPoints_min);
            //            cout << "for_begin======RTMmat" << endl;
            cv::Mat RTMmat = tempGoodFrame.getRT(poseEsti);
            //            cout << "ClarkClarkClarkClarkClarkClarkClarkClarkClarkClarkClarkClark" << endl;
            //            cout << "for_end======RTMmat" << endl;
            //            cout << "GoodFrameID" << GoodFrameID.at(i) << endl;
            RT_matTxt_total << i << endl;
            for (int j = 0; j < 4; ++j)
            {
                RT_matTxt_total << RTMmat.at<double>(j, 0) << " " << RTMmat.at<double>(j, 1) << " "
                                << RTMmat.at<double>(j, 2) << " " << RTMmat.at<double>(j, 3) << endl;
            }
        }
        cout<<"AllFrame.size(): "<<AllFrame.size()<<endl;

        //        cout << "txt_file_output" <<  txt_file_output <<endl;


        ////////////////////////////////////////////////////////////////////////////clark  2016.5.11
        //        cout << "for_begin" << GoodFrameID.size() << endl;
        //        for (int i = 0; i < GoodFrameID.size(); ++i)
        //        {
        //            //////// RT mat
        ////            cout << GoodFrameID.at(i) << endl;

        //            /// fuck bug
        //            /// error in Keegan.Ren 2016.4.5
        //            int tempGoodFrameID;
        //            getTempGoodFrame(AllFrame, GoodFrameID, tempGoodFrameID, i);
        ////            cout << "tempGoodFrameID =====" << tempGoodFrameID << endl;
        //            GoodFrame tempGoodFrame = AllFrame.at(tempGoodFrameID);
        //            ////GoodFrame tempGoodFrame = AllFrame.at(GoodFrameID.at(i));

        //            /// SolvePnP
        //            //            cv::Mat RTMmat = tempGoodFrame.getRT(modelPoints_min);
        ////            cout << "for_begin======RTMmat" << endl;
        //            cv::Mat RTMmat = tempGoodFrame.getRT(poseEsti);
        //            //cout << RTMmat << endl;
        ////            cout << "for_end======RTMmat" << endl;
        ////            cout << "GoodFrameID" << GoodFrameID.at(i) << endl;
        //            RT_matTxt_total << GoodFrameID.at(i) << endl;
        //            for (int j = 0; j < 4; ++j)
        //            {
        //                RT_matTxt_total << RTMmat.at<double>(j, 0) << " " << RTMmat.at<double>(j, 1) << " "
        //                                << RTMmat.at<double>(j, 2) << " " << RTMmat.at<double>(j, 3) << endl;
        //            }
        ////            cout << "for_for_j_end" << endl;
        //        }

        if (RT_matTxt_total.is_open())
            RT_matTxt_total.close();

        frame_count = 0;
        facear_model.Reset();
        done = true;
    }
    cout << "txt_file_output" <<  txt_file_output <<endl;
    return txt_file_output;
}


