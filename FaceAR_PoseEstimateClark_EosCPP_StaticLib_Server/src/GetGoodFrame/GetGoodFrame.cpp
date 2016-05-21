
#include <GetGoodFrame.h>

cv::Mat GoodFrame::getRT(std::vector<cv::Point3f> &modelPoints_min)
{
    cv::Mat img = this->getCapturesImage();
    std::vector<cv::Point2f> imagePoints;
    for (size_t i = 0; i < 68; ++i)
    {
        imagePoints.push_back(cv::Point2f((float)(this->getDetected_landmarks().at<double>(i)),
                                          (float)(this->getDetected_landmarks().at<double>(i+68))));
    }

    /////
    int max_d = MAX(img.rows,img.cols);
    cv::Mat camMatrix = (Mat_<double>(3,3) << max_d, 0,     img.cols/2.0,
                         0,	 max_d, img.rows/2.0,
                         0,	 0,	    1.0);

    cv::Mat ip(imagePoints);
    cv::Mat op(modelPoints_min);
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

cv::Mat GoodFrame::getRT_Clark(PoseEsti* poseEsti)
{
    glm::mat4 rotateMatrix = poseEsti->poseEstimation(this->getDetected_landmarks());

    cv::Mat _pm(4, 4, CV_64FC1);
    _pm.at<double>(0,0) = rotateMatrix[0][0];
    _pm.at<double>(0,1) = rotateMatrix[0][1];
    _pm.at<double>(0,2) = rotateMatrix[0][2];
    _pm.at<double>(0,3) = rotateMatrix[0][3];

    _pm.at<double>(1,0) = rotateMatrix[1][0];
    _pm.at<double>(1,1) = rotateMatrix[1][1];
    _pm.at<double>(1,2) = rotateMatrix[1][2];
    _pm.at<double>(1,3) = rotateMatrix[1][3];

    _pm.at<double>(2,0) = rotateMatrix[2][0];
    _pm.at<double>(2,1) = rotateMatrix[2][1];
    _pm.at<double>(2,2) = rotateMatrix[2][2];
    _pm.at<double>(2,3) = rotateMatrix[2][3];

    _pm.at<double>(3,0) = rotateMatrix[3][0];
    _pm.at<double>(3,1) = rotateMatrix[3][1];
    _pm.at<double>(3,2) = rotateMatrix[3][2];
    _pm.at<double>(3,3) = rotateMatrix[3][3];

    return _pm;
}

cv::Mat GoodFrame::getRT(PoseEsti* poseEsti)
{
//    cout<<"getRT======TODO : poseEsti->poseEstimation"<<endl;
    poseEsti->poseEstimation(this->getDetected_landmarks());
    glm::mat4 rotateMatrix = poseEsti->rotateMatrix;
//    cout<<"=================rotateMatrix"<<endl<<
//                                rotateMatrix[0][0]<<" "<<rotateMatrix[0][1]<<" "<<rotateMatrix[0][2]<<" "<<rotateMatrix[0][3]<<endl<<
//                                rotateMatrix[1][0]<<" "<<rotateMatrix[1][1]<<" "<<rotateMatrix[1][2]<<" "<<rotateMatrix[1][3]<<endl<<
//                                rotateMatrix[2][0]<<" "<<rotateMatrix[2][1]<<" "<<rotateMatrix[2][2]<<" "<<rotateMatrix[2][3]<<endl<<
//                                rotateMatrix[3][0]<<" "<<rotateMatrix[3][1]<<" "<<rotateMatrix[3][2]<<" "<<rotateMatrix[3][3]<<endl;
    //cout<<"Mean_Feature_distance: "<<(float)poseEsti->Mean_Feature_distance<<endl;
    //cout<<"init_Mean_Feature_distance: "<<(float)poseEsti->init_Mean_Feature_distance<<endl;
    //cout<<"scale: "<<poseEsti->scale<<endl;

    //cv::Mat _pm(4, 3, CV_64FC1);
    //_pm.at<double>(0,0) = rotateMatrix[0][0]; _pm.at<double>(0,1) = rotateMatrix[0][1]; _pm.at<double>(0,2) = rotateMatrix[0][2];
    //_pm.at<double>(1,0) = rotateMatrix[1][0]; _pm.at<double>(1,1) = rotateMatrix[1][1]; _pm.at<double>(1,2) = rotateMatrix[1][2];
    //_pm.at<double>(2,0) = rotateMatrix[2][0]; _pm.at<double>(2,1) = rotateMatrix[2][1]; _pm.at<double>(2,2) = rotateMatrix[2][2];
    //_pm.at<double>(3,0) = rotateMatrix[3][0]; _pm.at<double>(3,1) = rotateMatrix[3][1]; _pm.at<double>(3,2) = rotateMatrix[3][2];


    cv::Mat _pm(4, 4, CV_64FC1);
    _pm.at<double>(0,0) = rotateMatrix[0][0];
    _pm.at<double>(0,1) = rotateMatrix[0][1]; 
    _pm.at<double>(0,2) = rotateMatrix[0][2]; 
    _pm.at<double>(0,3) = rotateMatrix[0][3];

    _pm.at<double>(1,0) = rotateMatrix[1][0]; 
    _pm.at<double>(1,1) = rotateMatrix[1][1]; 
    _pm.at<double>(1,2) = rotateMatrix[1][2];
    _pm.at<double>(1,3) = rotateMatrix[1][3];

    _pm.at<double>(2,0) = rotateMatrix[2][0]; 
    _pm.at<double>(2,1) = rotateMatrix[2][1]; 
    _pm.at<double>(2,2) = rotateMatrix[2][2];
    _pm.at<double>(2,3) = rotateMatrix[2][3];

    _pm.at<double>(3,0) = rotateMatrix[3][0]; 
    _pm.at<double>(3,1) = rotateMatrix[3][1]; 
    _pm.at<double>(3,2) = rotateMatrix[3][2];
    _pm.at<double>(3,3) = rotateMatrix[3][3];

//    cout<<"getRT====== end"<<endl;

    return _pm;
}

cv::Mat_<double> getDetectedLandmarks3D(std::vector<GoodFrame> &AllFrame, int &frame_id)
{
    //    if (frame_id > AllFrame.size())
    //        std::cerr << "out of vector" << std::endl;
    //    GoodFrame tempFrame = AllFrame.at(frame_id-1);
    //    return tempFrame.getShap3D();
    for (int i = 0; i < AllFrame.size(); ++i)
    {
        GoodFrame tempFrame = AllFrame.at(i);
        if (tempFrame.frame_id == frame_id)
            return tempFrame.getShap3D();
    }
}

cv::Mat_<double> getDetectedLandmarks2D(std::vector<GoodFrame> &AllFrame, int &frame_id)
{
    //    if (frame_id > AllFrame.size())
    //        std::cerr << "out of vector" << std::endl;
    //    GoodFrame tempFrame = AllFrame.at(frame_id-1);
    //    return tempFrame.getDetected_landmarks();
    for (int i = 0; i < AllFrame.size(); ++i)
    {
        GoodFrame tempFrame = AllFrame.at(i);
        if (tempFrame.frame_id == frame_id)
            return tempFrame.getDetected_landmarks();
    }
}

cv::Mat getImage(std::vector<GoodFrame> &AllFrame, int &frame_id)
{
    //    if (frame_id > AllFrame.size())
    //        std::cerr << "out of vector" << std::endl;
    //    GoodFrame tempFrame = AllFrame.at(frame_id-1);
    //    return tempFrame.getCapturesImage();
    for (int i = 0; i < AllFrame.size(); ++i)
    {
        GoodFrame tempFrame = AllFrame.at(i);
        if (tempFrame.frame_id == frame_id)
            return tempFrame.getCapturesImage();
    }
}

cv::Vec6d getDiffCurrPre(std::vector<GoodFrame> &AllFrame, int &frame_id)
{
    //    if (frame_id > AllFrame.size())
    //        std::cerr << "out of vector" << std::endl;
    //    GoodFrame tempFrame = AllFrame.at(frame_id-1);
    //    return tempFrame.getDiff_curr_pre();
    for (int i = 0; i < AllFrame.size(); ++i)
    {
        GoodFrame tempFrame = AllFrame.at(i);
        if (tempFrame.frame_id == frame_id)
            return tempFrame.getDiff_curr_pre();
    }
}
double getDiffCurrPreId(std::vector<GoodFrame> &AllFrame, int &frame_id, int &id)
{
//    std::cout << "test_begin" << frame_id << "_____" << id << std::endl;
    cv::Vec6d tempVec6d = getDiffCurrPre(AllFrame, frame_id);
//    std::cout << "test" << frame_id << "_____" << id << std::endl;
    if ( id < 6 ) {
        return tempVec6d[id];
    } else {
        std::cerr << "id must less 6" << std::endl;
        return 0;
    }
}

//cv::Mat getRTMat(std::vector<GoodFrame> &AllFrame, int &frame_id)
//{
//    GoodFrame tempFrame =  AllFrame.at(frame_id);
//    return tempFrame.getRT_Mat();
//}

double getVariance(std::vector<double> &resultSet)
{
    double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
    double mean =  sum / resultSet.size(); //均值

    double accum  = 0.0;
    std::for_each (std::begin(resultSet), std::end(resultSet), [&](const double d) {
        accum  += (d-mean)*(d-mean);
    });

    //    double stdev = sqrt(accum/(resultSet.size()-1));
    double var = accum/resultSet.size();
    return var;
}

std::vector<int> getGoodFrame(std::vector<GoodFrame> &AllFrame, int &count)
{
    std::vector<int> good_frame_all;
    for (int i = 0; i < AllFrame.size(); ++i)
    {
        GoodFrame tempFrame = AllFrame.at(i);
        good_frame_all.push_back(tempFrame.frame_id);
    }
    ///
    std::vector<int> id;
    for (int i = 0; (i < AllFrame.size()) && (i + count < AllFrame.size()); ++i)
    {
        std::vector<double> tempCountVec6d_4;
        for (int j = i; j < i+count; ++j)
        {
            GoodFrame tempFrame = AllFrame.at(j);
            tempCountVec6d_4.push_back(180.0/PI*tempFrame.diff_curr_pre[4] + tempFrame.diff_curr_pre[0] /*+ tempFrame.diff_curr_pre[1]*/);
        }
        double getVar = getVariance(tempCountVec6d_4);
        //cout << "getVariance=" << i << " " << getVar << endl;
        if (getVar > 20.0 )
        {
            cout << "what" << endl;
            for (int j = i; j < i+count; ++j)
            {
                id.push_back( AllFrame.at(j).frame_id );
            }
        }
    }
    if (id.empty())
    {
        return good_frame_all;
        //        std::vector<int> good_frame_less;
        //        int cou = (int)good_frame_all.size()/18;
        //        for (int i = 0; (i < good_frame_all.size()) && ((i+cou) < good_frame_all.size()); i = i + cou)
        //            good_frame_less.push_back(good_frame_all.at(i));
        //        return good_frame_less;
    }

    std::sort(id.begin(), id.end());//sort the vector
    id.erase( std::unique( id.begin(), id.end() ), id.end());

    ///// TODO : i  frame_count
    std::vector<int> good_frame;
    for (int i = 0; i < good_frame_all.size()/*frame_count*/; ++i)
    {
        int j;
        for(j = 0; j < id.size(); ++j)
            if (id.at(j) == good_frame_all.at(i))
                break;
        if(j >= (id.size()-1))
            good_frame.push_back(good_frame_all.at(i)/*i*/);
    }

    return good_frame;
    //    ///
    //    std::vector<int> good_frame_less;
    //    int cou = (int)good_frame.size()/18;
    //    for (int i = 0; (i < good_frame.size()) && ((i+cou) < good_frame.size()); i = i + cou)
    //        good_frame_less.push_back(good_frame.at(i));
    //    return good_frame_less;
}

///////
/// \brief getGoodFrameLess
/// \param AllFrame
/// \param count
/// \return
/// 检测到的所有的结果相对较好的帧中间隔存储
std::vector<int> getGoodFrameLess(std::vector<GoodFrame> &AllFrame, int &count)
{
    std::vector<int> good_frame = getGoodFrame(AllFrame, count);
    std::vector<int> good_frame_less;
    int cou = (int)good_frame.size()/18;
    for (int i = 0; (i < good_frame.size()) && ((i+cou) < good_frame.size()); i = i + cou)
        good_frame_less.push_back(good_frame.at(i));
    return good_frame_less;
}

std::pair<int, int> find_similar(std::vector<int> &good_frame, int &id)
{
//    for (int i = 0; i < good_frame.size(); ++i)
//    {
//        std::cout << good_frame.at(i) << " ";
//    }
//    std::cout << std::endl;
    //////////////////
    int ii = 0;
    int small_id = good_frame.at(0);
    int small_id_value = std::abs(id - good_frame.at(0));
    for (int i = 1; i < good_frame.size(); ++i)
    {
//        std::cout << std::abs(id - good_frame.at(i)) << " " << small_id_value << std::endl;
        if ( std::abs(id - good_frame.at(i)) < small_id_value )
        {
            ii = i;
            small_id = good_frame.at(i);
            small_id_value = std::abs(id - good_frame.at(i));
        }
    }
    //std::cout << "ii = " << ii << std::endl;
    //std::cout << "small_id = " << small_id << std::endl;
    //std::cout << "small_id_value = " << small_id_value << std::endl;
    return std::make_pair(ii, small_id);
}

////////
/// \brief getGoodFrameLessBeta
/// \param AllFrame
/// \param count
/// \return
/// 检测到相对较好的帧中，根据拟合的结果，对最左端到最右端的一段进行间隔存储
bool getGoodFrameLessBeta(std::vector<GoodFrame> &AllFrame, int &count, std::vector<int> &good_frame_less)
{
    //std::cout << "begin" << std::endl;
    std::vector<int> good_frame = getGoodFrame(AllFrame, count);
//    std::cout << "getGoodFrame = " << good_frame.size() << std::endl;
//    std::cout << "AllFrame = " << AllFrame.size() << std::endl;
    std::vector<double> DiffCurrPreX_x, DiffCurrPreX_y;
    int id = 0; // X的变化量
    for (int i = 0; i < good_frame.size(); ++i)
    {
        DiffCurrPreX_x.push_back( (double)good_frame.at(i) );
        DiffCurrPreX_y.push_back( getDiffCurrPreId(AllFrame, good_frame.at(i), id) );
    }
//    std::cout << "fit" << std::endl;
    ///fit
    /// 
    std::vector<double> fit = polyfit( DiffCurrPreX_x, DiffCurrPreX_y, 4 );
//    cout << "fit.size()" << fit.size() << std::endl;
    if (fit.size() != 5)
    {
         return false;
    }

    double x[4];
    double a, b, c, d;
    a = fit.at(3)/fit.at(4);
    b = fit.at(2)/fit.at(4);
    c = fit.at(1)/fit.at(4);
    d = fit.at(0)/fit.at(4);
    // solve equation x^4 + a*x^3 + b*x^2 + c*x + d by Dekart-Euler method
    SolveP4(x, a, b, c, d) ;
//    cout << "solve" << endl;
//    cout << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << endl;
    std::vector<int> id_x;
    for (int i = 0; i < 4; ++i)
    {
        id_x.push_back( (int)x[i] );
    }
    std::sort(id_x.begin(), id_x.end());

//    for (int i = 0; i < id_x.size(); ++i )
//        std::cout << id_x.at(i) << " ";
//    std::cout << std::endl;

    int begin_i, begin_id, end_i, end_id;
    std::pair<int, int> begin = find_similar(good_frame, id_x.at(1));
    std::pair<int, int> end = find_similar(good_frame, id_x.at(2));
    begin_i = begin.first; begin_id = begin.second;
    end_i = end.first; end_id = end.second;
    //std::cout << "begin_i_id = " << begin_i << "_____" << begin_id << std::endl;
    //std::cout << "end_i_id = " << end_i << "_____" << end_id << std::endl;
    //////
    //std::vector<int> good_frame_less;
    int cou = (int)((end_i - begin_i) / 18);
    if (cou <= 0)
        return false;
    for (int i = begin_i; (i < end_i) && ((i+cou) < end_i); i = i + cou)
        good_frame_less.push_back(good_frame.at(i));
    //return good_frame_less;
    return true; 
}


bool getTempGoodFrame(std::vector<GoodFrame> &AllFrame, std::vector<int> &GoodFrameID, int &tempGoodFrameID, int &tempID)
{
     int temp_GoodFrameID = GoodFrameID.at(tempID);
     for (int k = 0; k < AllFrame.size(); ++k)
     {
         GoodFrame tempAllFrame = AllFrame.at(k);
         if ( tempAllFrame.frame_id == temp_GoodFrameID )
             tempGoodFrameID = k;  
         //else
         //    return false;
     }
     return true;
}
