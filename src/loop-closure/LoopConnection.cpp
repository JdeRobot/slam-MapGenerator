//
// Created by ernest on 18-6-22.
//

#include "LoopConnection.h"



namespace MapGen{

    // calculate the fundamental matrix, then get the essential matrix
    LoopConnection::LoopConnection(const std::pair<KeyFrame *, KeyFrame *> &loop_closing_pair, const cv::Mat& K) {
        std::vector<cv::DMatch> macthes;

        cv::BFMatcher matcher;
        matcher.match(loop_closing_pair.first->getDescriptor(), loop_closing_pair.second->getDescriptor(), macthes);

        auto raw_points_1 = loop_closing_pair.first->getKeypoints();
        auto raw_points_2 = loop_closing_pair.second->getKeypoints();

        std::vector<cv::Point2f> matched_points_1;
        std::vector<cv::Point2f> matched_points_2;
        for (auto x : macthes){
            // query & training
            matched_points_1.push_back(raw_points_1[x.queryIdx].pt);
            matched_points_2.push_back(raw_points_2[x.trainIdx].pt);
        }

        cv::Mat F = cv::findFundamentalMat(matched_points_1,matched_points_2,cv::FM_RANSAC,1,0.99);

        cv::Mat E = K.t() * F * K;
        // get RT from the essential matrix
        // TODO: I used K(0,0) as the focus, as this function does not support separate fx and fy
        double focus = K.at<double>(0,0);
        cv::Point2d pp;
        pp.x = K.at<double>(0,2);
        pp.y = K.at<double>(1,2);

        // recover pose from essential matrix
        int num_inliners = cv::recoverPose(E,matched_points_1,matched_points_2,r_,t_,K.at<double>(0,0),pp);

    }

    cv::Mat LoopConnection::get_essential() {
        throw std::runtime_error("Not implemented yet");
    }

    cv::Mat LoopConnection::get_translation() {
        return t_;
    }

    cv::Mat LoopConnection::get_rotation(){
        return r_;
    }
}