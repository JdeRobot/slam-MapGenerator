//
// Created by ernest on 18-6-22.
//

#include "LoopConnection.h"



namespace MapGen{

    // calculate the fundamental matrix, then get the essential matrix
    LoopConnection::LoopConnection(KeyFrame * frame_a, KeyFrame * frame_b, const Camera& cam){
        frame_a_ = frame_a;
        frame_b_ = frame_b;

        T_ = Eigen::Matrix4d::Identity();

//        std::vector<cv::DMatch> macthes;
//
//        cv::BFMatcher matcher;
//        matcher.match(frame_a->getDescriptor(), frame_b->getDescriptor(), macthes);
//
//        auto raw_points_1 = frame_a->getKeypoints();
//        auto raw_points_2 = frame_b->getKeypoints();
//
//        std::vector<cv::Point2f> matched_points_1;
//        std::vector<cv::Point2f> matched_points_2;
//        for (auto x : macthes){
//            // query & training
//            matched_points_1.push_back(raw_points_1[x.queryIdx].pt);
//            matched_points_2.push_back(raw_points_2[x.trainIdx].pt);
//        }
//
//        cv::Mat F = cv::findFundamentalMat(matched_points_1,matched_points_2,cv::FM_RANSAC,1,0.99);
//
//        cv::Mat E = K.t() * F * K;
//        // get RT from the essential matrix
//        // TODO: I used K(0,0) as the focus, as this function does not support separate fx and fy
//        double focus = K.at<double>(0,0);
//        cv::Point2d pp;
//        pp.x = K.at<double>(0,2);
//        pp.y = K.at<double>(1,2);
//
//        t_ = scaling_factor * t_;
//
//        // recover pose from essential matrix
//        int num_inliners = cv::recoverPose(E,matched_points_1,matched_points_2,r_,t_,K.at<double>(0,0),pp);


//        LOG_INFO << "num_inliners: " << num_inliners << std::endl;
//        LOG_INFO << "r: " << r_ << std::endl;
//        LOG_INFO << "t: " << t_ << std::endl;

//        r_ = cv::Mat::eye(3,3,CV_64F);
//        t_ = cv::Mat::zeros(3,1,CV_64F);
    }
}