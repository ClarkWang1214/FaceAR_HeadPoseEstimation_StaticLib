#ifndef LANDMARKSIMPROVED_H
#define LANDMARKSIMPROVED_H

#include "FaceAR_core.h"

/////////////////////////////////////////////////////////////////
/// TFY
double distance_TFY(cv::Point pre,cv::Point now);
//{
//    return sqrt((now.x-pre.x)*(now.x-pre.x)+(now.y-pre.y)*(now.y-pre.y));
//}

bool landmarksImproved(FaceARTracker::FaceAR &facear_model, int &frame_count, int &improve);
//{
//    ///TODO:TFY
//    ///stabilize the points
//    if(frame_count==0 || improve==0)
//    {
//        facear_model.pre_landmarks=facear_model.detected_landmarks;
//    }
//    else
//    {
//        double sumnm=0,sume=0;//nose,mouse,brow
//        for(int q=17;q<68;q++)
//        {
//            Point now=Point(facear_model.detected_landmarks.at<double>(q, 0),facear_model.detected_landmarks.at<double>(q+68, 0));
//            Point pre=Point(facear_model.pre_landmarks.at<double>(q, 0),facear_model.pre_landmarks.at<double>(q+68, 0));
//            double p=distance(pre,now);

//            if((q>26&&q<36)||(q>48&&q<59))
//            {
//                sumnm=sumnm+p;
//            }
//            if(q>35&&q<48)
//            {
//                sume=sume+p;
//            }
//            //                     double k=p/(p+0.25);
//            //                     now.x=now.x-k*(now.x-pre.x);
//            //                     now.y=now.y-k*(now.y-pre.y);
//            if(p<2)
//            {
//                facear_model.detected_landmarks.at<double>(q, 0)=pre.x;
//                facear_model.detected_landmarks.at<double>(q+68, 0)=pre.y;
//            }
//        }
//        if(sumnm/21<2)//(nose,mouse)'s p is smaller than 2,then both of them with chin are equal to the pre points.
//        {
//            for(int k=27;k<36;k++)
//            {
//                facear_model.detected_landmarks.at<double>(k, 0)=facear_model.pre_landmarks.at<double>(k, 0);
//                facear_model.detected_landmarks.at<double>(k+68, 0)=facear_model.pre_landmarks.at<double>(k+68, 0);
//            }
//            for(int k=48;k<68;k++)
//            {
//                facear_model.detected_landmarks.at<double>(k, 0)=facear_model.pre_landmarks.at<double>(k, 0);
//                facear_model.detected_landmarks.at<double>(k+68, 0)=facear_model.pre_landmarks.at<double>(k+68, 0);
//            }
//            for(int k=0;k<17;k++)
//            {
//                facear_model.detected_landmarks.at<double>(k, 0)=facear_model.pre_landmarks.at<double>(k, 0);
//                facear_model.detected_landmarks.at<double>(k+68, 0)=facear_model.pre_landmarks.at<double>(k+68, 0);
//            }
//        }
//        if(sume/12<2)
//        {
//            for(int k=17;k<27;k++)
//            {
//                facear_model.detected_landmarks.at<double>(k, 0)=facear_model.pre_landmarks.at<double>(k, 0);
//                facear_model.detected_landmarks.at<double>(k+68, 0)=facear_model.pre_landmarks.at<double>(k+68, 0);
//            }
//            for(int k=36;k<48;k++)
//            {
//                facear_model.detected_landmarks.at<double>(k, 0)=facear_model.pre_landmarks.at<double>(k, 0);
//                facear_model.detected_landmarks.at<double>(k+68, 0)=facear_model.pre_landmarks.at<double>(k+68, 0);
//            }
//        }
//    }
//    facear_model.pre_landmarks=facear_model.detected_landmarks.clone();
//    ///stabilize the points finished-------------------
//    return 1;
//}

#endif // LANDMARKSIMPROVED_H
