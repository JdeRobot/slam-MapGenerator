/**
 *
 *  Copyright (C) 2018 Jianxiong Cai <caijx AT shanghaitech.edu.cn>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "LoopDetector.h"

namespace MapGen {

    LoopDetector::LoopDetector(MapGen::Map & map, const std::string & img_dir,
                               const std::string &pretrained_voc, double loop_closure_thres) :
            img_dir_(img_dir), loop_closure_thres_(loop_closure_thres) {
        // initialize variables
        orb_ = cv::ORB::create();

        BOOST_LOG_TRIVIAL(info) << "Loading voc from " << pretrained_voc;
        voc_.loadFromTextFile(pretrained_voc);
        BOOST_LOG_TRIVIAL(info) << "Loaded voc from " << pretrained_voc;

        // compute all features
        computeAllFeatures(map, img_dir);

        // doing loop detector
        detectLoops(map);
    }


    void LoopDetector::computeAllFeatures(MapGen::Map &map, const std::string &img_dir) {

        // std::vector<std::vector<cv::Mat>> features_all;
        for (MapGen::KeyFrame *frame : map.GetAllKeyFrames()) {
            // read in the image
            std::string img_path = img_dir_ + frame->GetFilename();
            cv::Mat img = cv::imread(img_path);

            // compute ORB features
            cv::Mat mask;
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            orb_->detectAndCompute(img, mask, keypoints, descriptors);

            // change the structure of descriptors
            std::vector<cv::Mat> features;
            features.resize(descriptors.rows);
            for (int i = 0; i < descriptors.rows; ++i) {
                features.push_back(descriptors.row(i));
            }

            // convert feature to bow vector
            DBoW2::BowVector v_tmp;
            voc_.transform(features,v_tmp);
            frame->setBowVector(v_tmp);

            // append the features of current image to features_all
            // features_all.push_back(features);
        }

        // BOOST_LOG_TRIVIAL(info) << "size of features_all: " << features_all.size();
    }

    std::vector<std::pair<KeyFrame *, KeyFrame *>> LoopDetector::getLoopClosingPairs() {
        return loop_closure_pairs_;
    }


    void LoopDetector::detectLoops(MapGen::Map &map) {
        // double score_max = 0;
        auto all_keyframes = map.GetAllKeyFrames();
        for (int i = 0; i < all_keyframes.size(); i++){
            KeyFrame * kf_1 = all_keyframes[i];
            // loop closure only apply for keyframes away from each other (10 keyframes here)
            for (int j = i+11; j < all_keyframes.size(); j++){
                KeyFrame * kf_2 = all_keyframes[j];
                double score = voc_.score(kf_1->getBowVector(), kf_2->getBowVector());
                if (score > loop_closure_thres_) {
                    // std::cout << "kf_1: " << kf_1->GetId() << "  kf_2: " << kf_2->GetId()  << "    kf_1: "
                    // << kf_1->GetFilename() << "  kf_2: " << kf_2->GetFilename() << "  score: " << score << std::endl;
                    loop_closure_pairs_.push_back(std::make_pair(kf_1,kf_2));

                }
                // if (score_max < score)      score_max = score;
            }
        }

        // std::cout << "score_max: " << score_max << std::endl;
    }

}
