#include "PoseEstimation.h"


using namespace clarkPoseEsti;

PoseEsti::PoseEsti(){
    R_m = 0.5;
    R_n = 0.5;
    R_e = 0.91;
    pi = 3.141592653589;

    flag = false;

    storage = cvCreateMemStorage(0);
    seq = cvCreateSeq(CV_SEQ_FLAG_CLOSED | CV_SEQ_KIND_CURVE | CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint), storage);

}

PoseEsti::~PoseEsti(){

}

//Function to return distance between 2 points in image
float PoseEsti::FindDistance(CvPoint pt1, CvPoint pt2){
    int x,y;
    double z;
    x = pt1.x - pt2.x;
    y = pt1.y - pt2.y;
    z = (x*x)+(y*y);

    return (float)sqrt(z);
}

//Function to return distance between 2 points in image
double PoseEsti::FindDistance2D32f(CvPoint2D32f pt1, CvPoint2D32f pt2){
    double x,y,z;

    x = pt1.x - pt2.x;
    y = pt1.y - pt2.y;
    z = (x*x)+(y*y);

    return sqrt(z);
}

//Function to return angle between 2 points in image // the angle between x axis through z
float PoseEsti::FindAngle(CvPoint2D32f pt1, CvPoint2D32f pt2){
    float angle;
    angle = cvFastArctan(pt2.y - pt1.y, pt2.x - pt1.x);

    return 360-angle;
}

//Function to find slant angle in image 'Gee & Cipolla'
double PoseEsti::Find_slant(int ln, int lf, float Rn, float tita){
    float dz=0;
    double slant;
    float m1 = ((float)ln*ln)/((float)lf*lf);
    float m2 = (cos(tita))*(cos(tita));

    if (m2 == 1)
    {
        dz = sqrt(	(Rn*Rn)/(m1 + (Rn*Rn))	);
    }
    if (m2>=0 && m2<1)
    {
        dz = sqrt(	((Rn*Rn) - m1 - 2*m2*(Rn*Rn) + sqrt(	((m1-(Rn*Rn))*(m1-(Rn*Rn))) + 4*m1*m2*(Rn*Rn)	))/ (2*(1-m2)*(Rn*Rn))	);
    }
    slant = acos(dz);
    return slant;
}

void PoseEsti::draw_trail(IplImage* img, CvPoint* pt){
    static int counter;
    const int length = 30;

    static CvPoint p_1 = cvPoint(0,0);
    static CvPoint p = cvPoint(0,0);
    static float dist;
    CvPoint* p1,*p2;


    p 	= cvPoint(pt->x,pt->y);
    dist = FindDistance(p_1,p);

    printf("dist = %f\n",dist);

    if(dist > 20){
        p_1	= cvPoint(p.x,p.y);
        cvSeqPush(seq, pt);

        if (counter < length )
            counter ++;

        if (counter >= length){
            cvSeqPopFront(seq, NULL);
        }

    }

    for ( int i=0; i<(seq->total)-1;++i){
        //CvPoint* p = (CvPoint*)cvGetSeqElem (seq, i);
        p1 = (CvPoint*)cvGetSeqElem (seq, i);
        p2 = (CvPoint*)cvGetSeqElem (seq, i+1);
        //cvCircle(img, *p1, 1, CV_RGB(255,0,0), 1, 4, 0);
        cvLine(img, *p1, *p2, CV_RGB(255,0,255), 2, 4, 0);
    }
}

void PoseEsti::draw_crosshair(IplImage* img, CvPoint centre, int circle_radius, int line_radius, CvScalar colour){
    CvPoint pt1,pt2,pt3,pt4;

    pt1.x = centre.x;
    pt2.x = centre.x;
    pt1.y = centre.y - line_radius;
    pt2.y = centre.y + line_radius;
    pt3.x = centre.x - line_radius;
    pt4.x = centre.x + line_radius;
    pt3.y = centre.y;
    pt4.y = centre.y;


    cvCircle(img, centre, circle_radius, colour, 2, 4, 0);

    cvLine(img, pt1, pt2, colour, 1, 4, 0);
    cvLine(img, pt3, pt4, colour, 1, 4, 0);
}

void PoseEsti::draw_pin(IplImage* img, CvPoint3D32f normal, float slant, float tita, CvScalar colour){
    //define a 2d origin point for the view pointer
    const CvPoint origin = cvPoint(100,100);

    //define a 2d point for the projected 3D vector in 2d
    CvPoint projection_2d;
    projection_2d.x = origin.x + cvRound(150*(normal.x));
    projection_2d.y = origin.y + cvRound(150*(normal.y));

    //draw pin circle
    if (normal.x > 0 && normal.y < 0)
    {
        cvEllipse(img, origin, cvSize(60,abs(cvRound(60-slant*(180/(2*pi))))) , abs(180-(tita*(180/pi))), 0, 360, colour, 5,4,0);
    }
    else
    {
        cvEllipse(img, origin, cvSize(60,abs(cvRound(60-slant*(180/(2*pi))))) , abs(tita*(180/pi)), 0, 360, colour, 5,4,0);
    }

    //draw pin head
    cvLine(img, origin,			projection_2d,		CV_RGB(0,0,255), 2, 4, 0);
}

void PoseEsti::print_text(IplImage* img, int counter, CvScalar colour){
    //routines for printing text on image 'looking left or right'
    CvFont font;

    char msg1[4];
    sprintf(msg1, "%d", counter);
    //char msg2[] = "Left";

    const CvPoint text1 = cvPoint(5,200);

    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1,1,0,1,8);

    cvPutText(img, msg1, text1, &font, colour);
}

void PoseEsti::init_geometric_model(){
    init_LeftEye_Nose_distance 	= FindDistance2D32f(Nose, LeftEye);	//initial distance between Nose center and left Eye
    init_RightEye_Nose_distance 	= FindDistance2D32f(Nose, RightEye);	//initial distance between Nose center and right Eye
    init_LeftEye_RightEye_distance = FindDistance2D32f(LeftEye, RightEye);	//initial distance between left Eye and right Eye
    init_Nose_Mouth_distance 	= FindDistance2D32f(Nose, Mouth);		//initial distance between Nose center and Mouth
    init_Mean_Feature_distance 	= 50;

    LeftEye_Nose_distance 	= init_LeftEye_Nose_distance;
    RightEye_Nose_distance 	= init_RightEye_Nose_distance;
    LeftEye_RightEye_distance 	= init_LeftEye_RightEye_distance;
    Nose_Mouth_distance 		= init_Nose_Mouth_distance;
    Mean_Feature_distance 	= init_Mean_Feature_distance;

    yaw_angle = 0.;
    pitch_angle = 0.;
    roll_angle = 0.;
    yaw_radian = 0.;
    pitch_radian = 0.;
    roll_radian = 0.;

    int W = 960;
    int H = 640;
    rand_coord.x = W/2;
    rand_coord.y = H/2;
    srand( time(NULL));

    pointer_2d.x = (NoseBase.x + cvRound(500*(tan((double)yaw_radian))));
    pointer_2d.y = (NoseBase.y + cvRound(500*(tan((double)pitch_radian))));
}

glm::mat4  PoseEsti::poseEstimation(Mat_<double> detected_landmarks){

//        cout<<"detected_landmarks.at<double>(36): "<<detected_landmarks.at<double>(36)<<endl;
//        cout<<"detected_landmarks.at<double>(39): "<<detected_landmarks.at<double>(39)<<endl;
        ////////
        ///
        /// Eye
        ///
        ////////
        LeftEye.x = (detected_landmarks.at<double>(36) + detected_landmarks.at<double>(39))/2.0;
        LeftEye.y = (detected_landmarks.at<double>(36+68) + detected_landmarks.at<double>(39+68))/2.0;
//                cv::circle(captured_image, cv::Point2f(LeftEye.x, LeftEye.y), 2, cv::Scalar(0.0, 0.0, 255.0), 2);

        RightEye.x = (detected_landmarks.at<double>(42) + detected_landmarks.at<double>(45))/2.0;
        RightEye.y = (detected_landmarks.at<double>(42+68) + detected_landmarks.at<double>(45+68))/2.0;
//                cv::circle(captured_image, cv::Point2f(RightEye.x, RightEye.y), 2, cv::Scalar(0.0, 0.0, 255.0), 2);

        MidEyes.x = (LeftEye.x + RightEye.x) / 2;
        MidEyes.y = (LeftEye.y + RightEye.y) / 2;
//                cv::circle(captured_image, cv::Point2f(MidEyes.x, MidEyes.y), 4, cv::Scalar(0.0, 0.0, 255.0), 2);


        ////////
        ///
        /// Nose
        ///
        ////////
        Nose.x = (detected_landmarks.at<double>(30) + detected_landmarks.at<double>(31) + detected_landmarks.at<double>(35) )/3.0;
        Nose.y = (detected_landmarks.at<double>(30+68) + detected_landmarks.at<double>(31+68) + detected_landmarks.at<double>(35+68) )/3.0;
//                cv::circle(captured_image, cv::Point2f(Nose.x, Nose.y), 2, cv::Scalar(255.0, 0.0, 0.0), 2);


        ////////
        ///
        /// Mouth
        ///
        ////////
        Mouth.x = (detected_landmarks.at<double>(48) + detected_landmarks.at<double>(54))/2.0;
        Mouth.y = (detected_landmarks.at<double>(48+68) + detected_landmarks.at<double>(54+68))/2.0;
//                cv::circle(captured_image, cv::Point2f(Mouth.x, Mouth.y), 2, cv::Scalar(0.0, 255.0, 255.0), 2);


        NoseBase.x = Mouth.x + (MidEyes.x - Mouth.x)*(R_m); // the middle point between mouth and nosebase
        NoseBase.y = Mouth.y + (MidEyes.y - Mouth.y)*(R_m);


//        cout<<"init_geometric_model"<<endl;
        if(flag == false){
            init_geometric_model();
            flag = true;
        }


        //distance between Nose center and left Eye
        LeftEye_Nose_distance 	= FindDistance2D32f(Nose, LeftEye);

        //distance between Nose center and right Eye
        RightEye_Nose_distance 	= FindDistance2D32f(Nose, RightEye);

        //distance between left Eye and right Eye
        LeftEye_RightEye_distance 	= FindDistance2D32f(LeftEye, RightEye);

        //distance between Nose center and Mouth
        Nose_Mouth_distance 		= FindDistance2D32f(Nose, Mouth);

        Mean_Feature_distance 	= (LeftEye_Nose_distance + RightEye_Nose_distance
                                           + LeftEye_RightEye_distance + Nose_Mouth_distance)/4;

        scale = (float)Mean_Feature_distance/ (float)init_Mean_Feature_distance;

        //distance between Nose base and Nose center
        float Image_Facial_Normal_length = FindDistance2D32f(NoseBase, Nose);
//        cout<<"Image_Facial_Normal_length = "<<Image_Facial_Normal_length<<endl;

        //distance between Eye center and Mouth center
        float Eye_Mouth_distance = FindDistance2D32f(MidEyes, Mouth);
//        cout<<"Eye_Mouth_distance = "<<Eye_Mouth_distance<<endl;

        //roll angle - angle between left Eye and right Eye // the angle between x axis through z
        roll_angle = FindAngle(LeftEye, RightEye);
        if (roll_angle > 180){
            roll_angle = roll_angle-360;
        }
        roll_radian = - roll_angle * pi / 180;

        //symm angle - angle between the symmetry axis and the 'x' axis
        float symm_angle = FindAngle(NoseBase, MidEyes);
        //tilt angle - angle between normal in image and 'x' axis
        float tilt_angle = FindAngle(NoseBase, Nose);
        //tita angle - angle between the symmetry axis and the image normal
        float tita_radian = (abs(tilt_angle-symm_angle))*(pi/180);

        //slant angle - angle between the facial normal and the image normal
        slant_radian = Find_slant(Image_Facial_Normal_length, Eye_Mouth_distance, R_n, tita_radian);

        //define a 3D vector for the facial normal
        CvPoint3D32f normal;
        normal.x = (sin(slant_radian))*(cos((360-tilt_angle)*(pi/180)));
        normal.y = (sin(slant_radian))*(sin((360-tilt_angle)*(pi/180)));
        normal.z = -cos(slant_radian);

        //find pitch and yaw
        //kpitch_pre = pitch;
        pitch_radian = acos(sqrt((normal.x*normal.x + normal.z*normal.z)/(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z)));
        if((Nose.y - NoseBase.y)< 0 )
            pitch_radian = -pitch_radian;
        pitch_angle = pitch_radian * 180 / pi;


        //pitch[frame_number] = pitch*(180/pi);
        yaw_radian = acos((abs(normal.z))/(sqrt(normal.x*normal.x + normal.z*normal.z)));
        if((Nose.x - NoseBase.x)< 0 )
            yaw_radian = -yaw_radian;
        yaw_angle = yaw_radian * 180 / pi;

        //cout<<"roll, pitch, yaw: "<<roll<<", "<<pitch<<", "<<yaw<<endl;


        // Eular Angle To Rotate Matrix
        // create quaternion from  eular angle roll pitch yaw

        // radians
        double theta_x =  pitch_radian,
                theta_y = roll_radian,
                theta_z = yaw_radian;

        // angles
//        cout<<"pitch_radian = "<<pitch_radian<<endl;
//        cout<<"roll_radian = "<<roll_radian<<endl;
//        cout<<"yaw_radian = "<<yaw_radian<<endl;

//        cout<<"pitch_angle = "<<pitch_angle<<endl;
//        cout<<"roll_angle = "<<roll_angle<<endl;
//        cout<<"yaw_angle = "<<yaw_angle<<endl;


        float cx = cos(theta_x / 2);
        float sx = sin(theta_x / 2);
        float cy = cos(theta_y / 2);
        float sy = sin(theta_y / 2);
        float cz = cos(theta_z / 2);
        float sz = sin(theta_z / 2);

        float qw = cx * cy * cz + sx * sy * sz;
        float qx = sx * cy * cz - cx * sy * sz;
        float qy = cx * sy * cz + sx * cy * sz;
        float qz = cx * cy * sz + sx * sy * cz;

        float qxx = qx * qx;
        float qyy = qy * qy;
        float qzz = qz * qz;
        float qxy = qx * qy;
        float qwz = qw * qz;
        float qwy = qw * qy;
        float qxz = qx * qz;
        float qyz = qy * qz;
        float qwx = qw * qx;

        float m11 = 1.0f - 2 * (qyy + qzz);
        float m12 = 2 * (qxy - qwz);
        float m13 = 2 * (qwy + qxz);
        float m14 = MidEyes.x;

        float m21 = 2 * (qxy + qwz);
        float m22 = 1.0f - 2 * (qxx + qzz);
        float m23 = 2 * (qyz - qwx);
        float m24 = MidEyes.y;

        float m31 = 2 * (qxy - qwy);
        float m32 = 2 * (qyz + qwx);
        float m33 = 1.0f - 2 * (qxx + qyy);
        float m34 = scale;

        float m41 = pitch_radian;
        float m42 = roll_radian;
        float m43 = yaw_radian;
        float m44 = 1.0f;

//        cout<< m11 << ", " << m12 << ", " << m13 << ", " << m14 <<endl;
//        cout<< m21 << ", " << m22 << ", " << m23 << ", " << m24 <<endl;
//        cout<< m31 << ", " << m32 << ", " << m33 << ", " << m34 <<endl;
//        cout<< m41 << ", " << m42 << ", " << m43 << ", " << m44 <<endl;

        rotateMatrix = glm::mat4( glm::vec4(m11, m12, m13, m14),
                                            glm::vec4(m21, m22, m23, m24),
                                            glm::vec4(m31, m32, m33, m34),
                                            glm::vec4(m41, m42, m43, m44));

        return rotateMatrix;
}

glm::mat4  PoseEsti::poseEstimation(vector<cv::Point> detected_landmarks, Mat &img){

    ////////
    ///
    /// Eye
    ///
    ////////
    LeftEye.x = (detected_landmarks[3].x + detected_landmarks[4].x)/2.0;
    LeftEye.y = (detected_landmarks[3].y + detected_landmarks[4].y)/2.0;
//                cv::circle(captured_image, cv::Point2f(LeftEye.x, LeftEye.y), 2, cv::Scalar(0.0, 0.0, 255.0), 2);

    RightEye.x = (detected_landmarks[5].x + detected_landmarks[6].x)/2.0;
    RightEye.y = (detected_landmarks[5].y + detected_landmarks[6].y)/2.0;
//                cv::circle(captured_image, cv::Point2f(RightEye.x, RightEye.y), 2, cv::Scalar(0.0, 0.0, 255.0), 2);

    MidEyes.x = (LeftEye.x + RightEye.x) / 2;
    MidEyes.y = (LeftEye.y + RightEye.y) / 2;
//                cv::circle(captured_image, cv::Point2f(MidEyes.x, MidEyes.y), 4, cv::Scalar(0.0, 0.0, 255.0), 2);


    ////////
    ///
    /// Nose
    ///
    ////////
    Nose.x = (detected_landmarks[0].x + detected_landmarks[1].x+ detected_landmarks[2].x )/3.0;
    Nose.y = (detected_landmarks[0].y + detected_landmarks[1].y+ detected_landmarks[2].y )/3.0;
//                cv::circle(captured_image, cv::Point2f(Nose.x, Nose.y), 2, cv::Scalar(255.0, 0.0, 0.0), 2);


    ////////
    ///
    /// Mouth
    ///
    ////////
    Mouth.x = (detected_landmarks[7].x + detected_landmarks[8].x)/2.0;
    Mouth.y = (detected_landmarks[7].y + detected_landmarks[8].y)/2.0;
//                cv::circle(captured_image, cv::Point2f(Mouth.x, Mouth.y), 2, cv::Scalar(0.0, 255.0, 255.0), 2);


    NoseBase.x = Mouth.x + (MidEyes.x - Mouth.x)*(R_m);
    NoseBase.y = Mouth.y + (MidEyes.y - Mouth.y)*(R_m);


//    cout<<"init_geometric_model"<<endl;
    if(flag == false){
        init_geometric_model();
        flag = true;
    }


    //distance between Nose center and left Eye
    LeftEye_Nose_distance 	= FindDistance2D32f(Nose, LeftEye);

    //distance between Nose center and right Eye
    RightEye_Nose_distance 	= FindDistance2D32f(Nose, RightEye);

    //distance between left Eye and right Eye
    LeftEye_RightEye_distance 	= FindDistance2D32f(LeftEye, RightEye);

    //distance between Nose center and Mouth
    Nose_Mouth_distance 		= FindDistance2D32f(Nose, Mouth);

    Mean_Feature_distance 	= (LeftEye_Nose_distance + RightEye_Nose_distance
                                       + LeftEye_RightEye_distance + Nose_Mouth_distance)/4;

    scale = (float)Mean_Feature_distance/ (float)init_Mean_Feature_distance;

    //distance between Nose base and Nose center
    float Image_Facial_Normal_length = FindDistance2D32f(NoseBase, Nose);
//    cout<<"Image_Facial_Normal_length = "<<Image_Facial_Normal_length<<endl;

    //distance between Eye center and Mouth center
    float Eye_Mouth_distance = FindDistance2D32f(MidEyes, Mouth);
//    cout<<"Eye_Mouth_distance = "<<Eye_Mouth_distance<<endl;

    //roll angle - angle between left Eye and right Eye
    roll_angle = FindAngle(LeftEye, RightEye);
    if (roll_angle > 180){
        roll_angle = roll_angle-360;
    }
    roll_radian = - roll_angle * pi / 180;

    //symm angle - angle between the symmetry axis and the 'x' axis
    float symm_angle = FindAngle(NoseBase, MidEyes);
    //tilt angle - angle between normal in image and 'x' axis
    float tilt_angle = FindAngle(NoseBase, Nose);
    //tita angle - angle between the symmetry axis and the image normal
    float tita_radian = (abs(tilt_angle-symm_angle))*(pi/180);

    //slant angle - angle between the facial normal and the image normal
    slant_radian = Find_slant(Image_Facial_Normal_length, Eye_Mouth_distance, R_n, tita_radian);

    //define a 3D vector for the facial normal
    CvPoint3D32f normal;
    normal.x = (sin(slant_radian))*(cos((360-tilt_angle)*(pi/180)));
    normal.y = (sin(slant_radian))*(sin((360-tilt_angle)*(pi/180)));
    normal.z = -cos(slant_radian);

    //find pitch and yaw
    //kpitch_pre = pitch;
    pitch_radian = acos(sqrt((normal.x*normal.x + normal.z*normal.z)/(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z)));
    if((Nose.y - NoseBase.y)< 0 )
        pitch_radian = -pitch_radian;
    pitch_angle = pitch_radian * 180 / pi;


    //pitch[frame_number] = pitch*(180/pi);
    yaw_radian = acos((abs(normal.z))/(sqrt(normal.x*normal.x + normal.z*normal.z)));
    if((Nose.x - NoseBase.x)< 0 )
        yaw_radian = -yaw_radian;
    yaw_angle = yaw_radian * 180 / pi;

    //cout<<"roll, pitch, yaw: "<<roll<<", "<<pitch<<", "<<yaw<<endl;


    // Eular Angle To Rotate Matrix
    // create quaternion from  eular angle roll pitch yaw

    // radians
    double theta_x =  pitch_radian,
            theta_y = roll_radian,
            theta_z = yaw_radian;





    // angles
//    cout<<"\n\npitch_radian = "<<pitch_radian<<endl;
//    cout<<"roll_radian = "<<roll_radian<<endl;
//    cout<<"yaw_radian = "<<yaw_radian<<endl;

//    cout<<"pitch_angle = "<<pitch_angle<<endl;
//    cout<<"roll_angle = "<<roll_angle<<endl;
//    cout<<"yaw_angle = "<<yaw_angle<<endl<<endl<<endl;


    float cx = cos(theta_x / 2);
    float sx = sin(theta_x / 2);
    float cy = cos(theta_y / 2);
    float sy = sin(theta_y / 2);
    float cz = cos(theta_z / 2);
    float sz = sin(theta_z / 2);

    float qw = cx * cy * cz + sx * sy * sz;
    float qx = sx * cy * cz - cx * sy * sz;
    float qy = cx * sy * cz + sx * cy * sz;
    float qz = cx * cy * sz + sx * sy * cz;

    float qxx = qx * qx;
    float qyy = qy * qy;
    float qzz = qz * qz;
    float qxy = qx * qy;
    float qwz = qw * qz;
    float qwy = qw * qy;
    float qxz = qx * qz;
    float qyz = qy * qz;
    float qwx = qw * qx;

    float m11 = 1.0f - 2 * (qyy + qzz);
    float m12 = 2 * (qxy - qwz);
    float m13 = 2 * (qwy + qxz);
    float m14 = MidEyes.x;

    float m21 = 2 * (qxy + qwz);
    float m22 = 1.0f - 2 * (qxx + qzz);
    float m23 = 2 * (qyz - qwx);
    float m24 = MidEyes.y;

    float m31 = 2 * (qxy - qwy);
    float m32 = 2 * (qyz + qwx);
    float m33 = 1.0f - 2 * (qxx + qyy);
    float m34 = scale;

    float m41 = normal.x;
    float m42 = normal.y;
    float m43 = normal.z;
    float m44 = roll_radian;

//    float m41 = 0.0f;
//    float m42 = 0.0f;
//    float m43 = 0.0f;
//    float m44 = 1.0f;

//    cout<<"MidEyes.x = "<<MidEyes.x<<endl;
//    cout<<"MidEyes.y = "<<MidEyes.y<<endl;
//    cout<<"scale = "<<scale<<endl;

//    cout<< "Eular Angle To Rotate Matrix : " <<endl;
//    cout<< m11 << ", " << m12 << ", " << m13 << ", " << m14 <<endl;
//    cout<< m21 << ", " << m22 << ", " << m23 << ", " << m24 <<endl;
//    cout<< m31 << ", " << m32 << ", " << m33 << ", " << m34 <<endl;
//    cout<< m41 << ", " << m42 << ", " << m43 << ", " << m44 <<endl;

    rotateMatrix = glm::mat4( glm::vec4(m11, m12, m13, m14),
                                        glm::vec4(m21, m22, m23, m24),
                                        glm::vec4(m31, m32, m33, m34),
                                        glm::vec4(m41, m42, m43, m44));

//    rotateMatrix = glm::eulerAngleYXZ(theta_y, theta_x, theta_z);

//    rotateMatrix[0][3] = MidEyes.x;
//    rotateMatrix[1][3] = MidEyes.y;
//    rotateMatrix[2][3] = scale;
//    cout<< "glm::eulerAngleYXZ : " <<endl;
//    cout<< rotateMatrix[0][0] << ", " << rotateMatrix[0][1] << ", " << rotateMatrix[0][2] << ", " << rotateMatrix[0][3] <<endl;
//    cout<< rotateMatrix[1][0] << ", " << rotateMatrix[1][1] << ", " << rotateMatrix[1][2] << ", " << rotateMatrix[1][3] <<endl;
//    cout<< rotateMatrix[2][0] << ", " << rotateMatrix[2][1] << ", " << rotateMatrix[2][2] << ", " << rotateMatrix[2][3] <<endl;
//    cout<< rotateMatrix[3][0] << ", " << rotateMatrix[3][1] << ", " << rotateMatrix[3][2] << ", " << rotateMatrix[3][3] <<endl;


//    //draw lines
//    //cvPoint(LeftEye.x,LeftEye.y)
//    line(img, cv::Point(Nose.x,Nose.y),		cv::Point(Mouth.x,Mouth.y),		cv::Scalar(255,0,0), 1, 4, 0);
//    line(img, cv::Point(Nose.x,Nose.y),		cv::Point(LeftEye.x,LeftEye.y),	cv::Scalar(255,0,0), 1, 4, 0);
//    line(img, cv::Point(Nose.x,Nose.y),		cv::Point(RightEye.x,RightEye.y),	cv::Scalar(255,0,0), 1, 4, 0);
//    line(img, cv::Point(RightEye.x,RightEye.y),	cv::Point(LeftEye.x,LeftEye.y),	cv::Scalar(0,0,255), 1, 4, 0);
//    line(img, cv::Point(MidEyes.x,MidEyes.y),		cv::Point(Mouth.x,Mouth.y),		cv::Scalar(0,0,255), 1, 4, 0);
//    line(img, cv::Point(NoseBase.x,NoseBase.y),	cv::Point(Nose.x,Nose.y),		cv::Scalar(0,0,255), 1, 4, 0);
//    line(img, cv::Point(Nose.x,Nose.y),		cv::Point(MidEyes.x,MidEyes.y),	cv::Scalar(255,0,0), 1, 4, 0);

//    IplImage _img = img;
//    draw_pin(&_img, normal, slant_radian, tita_radian, CV_RGB(255,0,0));

    return rotateMatrix;
}


glm::mat4  PoseEsti::poseEstimationByFaceNormal(Point3f face_normal, double roll_radian,  Mat &img){
    roll_angle = - roll_radian * 180 / pi;

    //find pitch and yaw
    //kpitch_pre = pitch;
    pitch_radian = acos(sqrt((face_normal.x*face_normal.x + face_normal.z*face_normal.z)/(face_normal.x*face_normal.x + face_normal.y*face_normal.y + face_normal.z*face_normal.z)));
    if((Nose.y - NoseBase.y)< 0 )
        pitch_radian = -pitch_radian;
    pitch_angle = pitch_radian * 180 / pi;


    //pitch[frame_number] = pitch*(180/pi);
    yaw_radian = acos((abs(face_normal.z))/(sqrt(face_normal.x*face_normal.x + face_normal.z*face_normal.z)));
    if((Nose.x - NoseBase.x)< 0 )
        yaw_radian = -yaw_radian;
    yaw_angle = yaw_radian * 180 / pi;

    //cout<<"roll, pitch, yaw: "<<roll<<", "<<pitch<<", "<<yaw<<endl;


    // Eular Angle To Rotate Matrix
    // create quaternion from  eular angle roll pitch yaw

    // radians
    double theta_x =  pitch_radian,
            theta_y = roll_radian,
            theta_z = yaw_radian;





    // angles
//    cout<<"======================================pitch_radian = "<<pitch_radian<<endl;
//    cout<<"======================================roll_radian = "<<roll_radian<<endl;
//    cout<<"======================================yaw_radian = "<<yaw_radian<<endl<<endl;

//    cout<<"======================================pitch_angle = "<<pitch_angle<<endl;
//    cout<<"======================================roll_angle = "<<roll_angle<<endl;
//    cout<<"======================================yaw_angle = "<<yaw_angle<<endl<<endl<<endl;


    float cx = cos(theta_x / 2);
    float sx = sin(theta_x / 2);
    float cy = cos(theta_y / 2);
    float sy = sin(theta_y / 2);
    float cz = cos(theta_z / 2);
    float sz = sin(theta_z / 2);

    float qw = cx * cy * cz + sx * sy * sz;
    float qx = sx * cy * cz - cx * sy * sz;
    float qy = cx * sy * cz + sx * cy * sz;
    float qz = cx * cy * sz + sx * sy * cz;

    float qxx = qx * qx;
    float qyy = qy * qy;
    float qzz = qz * qz;
    float qxy = qx * qy;
    float qwz = qw * qz;
    float qwy = qw * qy;
    float qxz = qx * qz;
    float qyz = qy * qz;
    float qwx = qw * qx;

    float m11 = 1.0f - 2 * (qyy + qzz);
    float m12 = 2 * (qxy - qwz);
    float m13 = 2 * (qwy + qxz);
    float m14 = MidEyes.x;

    float m21 = 2 * (qxy + qwz);
    float m22 = 1.0f - 2 * (qxx + qzz);
    float m23 = 2 * (qyz - qwx);
    float m24 = MidEyes.y;

    float m31 = 2 * (qxy - qwy);
    float m32 = 2 * (qyz + qwx);
    float m33 = 1.0f - 2 * (qxx + qyy);
    float m34 = scale;

    float m41 = face_normal.x;
    float m42 = face_normal.y;
    float m43 = face_normal.z;
    float m44 = 1.0f;

//    float m41 = 0.0f;
//    float m42 = 0.0f;
//    float m43 = 0.0f;
//    float m44 = 1.0f;

//    cout<<"MidEyes.x = "<<MidEyes.x<<endl;
//    cout<<"MidEyes.y = "<<MidEyes.y<<endl;
//    cout<<"scale = "<<scale<<endl;

//    cout<< "Eular Angle To Rotate Matrix : " <<endl;
//    cout<< m11 << ", " << m12 << ", " << m13 << ", " << m14 <<endl;
//    cout<< m21 << ", " << m22 << ", " << m23 << ", " << m24 <<endl;
//    cout<< m31 << ", " << m32 << ", " << m33 << ", " << m34 <<endl;
//    cout<< m41 << ", " << m42 << ", " << m43 << ", " << m44 <<endl;

    rotateMatrix = glm::mat4( glm::vec4(m11, m12, m13, m14),
                                        glm::vec4(m21, m22, m23, m24),
                                        glm::vec4(m31, m32, m33, m34),
                                        glm::vec4(m41, m42, m43, m44));

//    rotateMatrix = glm::eulerAngleYXZ(theta_y, theta_x, theta_z);

//    rotateMatrix[0][3] = MidEyes.x;
//    rotateMatrix[1][3] = MidEyes.y;
//    rotateMatrix[2][3] = scale;
//    cout<< "glm::eulerAngleYXZ : " <<endl;
//    cout<< rotateMatrix[0][0] << ", " << rotateMatrix[0][1] << ", " << rotateMatrix[0][2] << ", " << rotateMatrix[0][3] <<endl;
//    cout<< rotateMatrix[1][0] << ", " << rotateMatrix[1][1] << ", " << rotateMatrix[1][2] << ", " << rotateMatrix[1][3] <<endl;
//    cout<< rotateMatrix[2][0] << ", " << rotateMatrix[2][1] << ", " << rotateMatrix[2][2] << ", " << rotateMatrix[2][3] <<endl;
//    cout<< rotateMatrix[3][0] << ", " << rotateMatrix[3][1] << ", " << rotateMatrix[3][2] << ", " << rotateMatrix[3][3] <<endl;


//    //draw lines
//    //cvPoint(LeftEye.x,LeftEye.y)
//    line(img, cv::Point(Nose.x,Nose.y),		cv::Point(Mouth.x,Mouth.y),		cv::Scalar(255,0,0), 1, 4, 0);
//    line(img, cv::Point(Nose.x,Nose.y),		cv::Point(LeftEye.x,LeftEye.y),	cv::Scalar(255,0,0), 1, 4, 0);
//    line(img, cv::Point(Nose.x,Nose.y),		cv::Point(RightEye.x,RightEye.y),	cv::Scalar(255,0,0), 1, 4, 0);
//    line(img, cv::Point(RightEye.x,RightEye.y),	cv::Point(LeftEye.x,LeftEye.y),	cv::Scalar(0,0,255), 1, 4, 0);
//    line(img, cv::Point(MidEyes.x,MidEyes.y),		cv::Point(Mouth.x,Mouth.y),		cv::Scalar(0,0,255), 1, 4, 0);
//    line(img, cv::Point(NoseBase.x,NoseBase.y),	cv::Point(Nose.x,Nose.y),		cv::Scalar(0,0,255), 1, 4, 0);
//    line(img, cv::Point(Nose.x,Nose.y),		cv::Point(MidEyes.x,MidEyes.y),	cv::Scalar(255,0,0), 1, 4, 0);

    return rotateMatrix;
}


glm::mat4  PoseEsti::poseEstimation_g2o(vector<cv::Point> detected_landmarks, Mat &img){
    // Todo ...




//    // 找到对应点
//    vector<cv::Point2f> pts1, pts2;
////    if ( findCorrespondingPoints( img1, img2, pts1, pts2 ) == false )
////    {
////        cout<<"匹配点不够！"<<endl;
////        return 0;
////    }
//    cout<<"找到了"<<pts1.size()<<"组对应特征点。"<<endl;
//    // 构造g2o中的图
//    // 先构造求解器
//    g2o::SparseOptimizer    optimizer;
//    // 使用Cholmod中的线性方程求解器
//    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType> ();
//    // 6*3 的参数
//    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3( linearSolver );
//    // L-M 下降
//    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg( block_solver );

//    optimizer.setAlgorithm( algorithm );
//    optimizer.setVerbose( false );

//    // 添加节点
//    // 两个位姿节点
//    for ( int i=0; i<2; i++ )
//    {
//        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
//        v->setId(i);
//        if ( i == 0)
//            v->setFixed( true ); // 第一个点固定为零
//        // 预设值为单位Pose，因为我们不知道任何信息
//        v->setEstimate( g2o::SE3Quat() );
//        optimizer.addVertex( v );
//    }
//    // 很多个特征点的节点
//    // 以第一帧为准
//    for ( size_t i=0; i<pts1.size(); i++ )
//    {
//        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
//        v->setId( 2 + i );
//        // 由于深度不知道，只能把深度设置为1了
//        double z = 1;
//        double x = ( pts1[i].x - cx ) * z / fx;
//        double y = ( pts1[i].y - cy ) * z / fy;
//        v->setMarginalized(true);
//        v->setEstimate( Eigen::Vector3d(x,y,z) );
//        optimizer.addVertex( v );
//    }

//    // 准备相机参数
//    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
//    camera->setId(0);
//    optimizer.addParameter( camera );

//    // 准备边
//    // 第一帧
//    vector<g2o::EdgeProjectXYZ2UV*> edges;
//    for ( size_t i=0; i<pts1.size(); i++ )
//    {
//        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
//        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
//        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
//        edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
//        edge->setInformation( Eigen::Matrix2d::Identity() );
//        edge->setParameterId(0, 0);
//        // 核函数
//        edge->setRobustKernel( new g2o::RobustKernelHuber() );
//        optimizer.addEdge( edge );
//        edges.push_back(edge);
//    }
//    // 第二帧
//    for ( size_t i=0; i<pts2.size(); i++ )
//    {
//        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
//        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
//        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
//        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
//        edge->setInformation( Eigen::Matrix2d::Identity() );
//        edge->setParameterId(0,0);
//        // 核函数
//        edge->setRobustKernel( new g2o::RobustKernelHuber() );
//        optimizer.addEdge( edge );
//        edges.push_back(edge);
//    }

//    cout<<"开始优化"<<endl;
//    optimizer.setVerbose(true);
//    optimizer.initializeOptimization();
//    optimizer.optimize(10);
//    cout<<"优化完毕"<<endl;

//    //我们比较关心两帧之间的变换矩阵
//    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
//    Eigen::Isometry3d pose = v->estimate();
//    cout<<"Pose="<<endl<<pose.matrix()<<endl;

//    // 以及所有特征点的位置
//    for ( size_t i=0; i<pts1.size(); i++ )
//    {
//        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
//        cout<<"vertex id "<<i+2<<", pos = ";
//        Eigen::Vector3d pos = v->estimate();
//        cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
//    }

//    // 估计inlier的个数
//    int inliers = 0;
//    for ( auto e:edges )
//    {
//        e->computeError();
//        // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
//        if ( e->chi2() > 1 )
//        {
//            cout<<"error = "<<e->chi2()<<endl;
//        }
//        else
//        {
//            inliers++;
//        }
//    }

//    cout<<"inliers in total points: "<<inliers<<"/"<<pts1.size()+pts2.size()<<endl;
//    optimizer.save("ba.g2o");
    return rotateMatrix;
}

void PoseEsti::getPose(){
//    return
}
