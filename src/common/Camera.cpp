//
// Created by ernest on 18-6-27.
//

#include "Camera.h"


namespace MapGen{
    Camera::Camera(const CameraParameters &params) {
        params_ = params;
    }

    cv::Mat Camera::get_intrinsic_matrix(){
    	cv::Mat K = cv::Mat(3,3,cv::CV_32F);
    	K(0,0) = params_.fx;
    	K(1,1) = params_.fy;
    	K(0,2) = params_.cx;
    	K(1,2) = params_.cy;
    	K(2,2) = 1;

    	// TODO: for verification only
    	LOG_DEBUG << "K: " << std::endl;
    	LOG_DEBUG << K << std::endl;

    	return K;
    }
}