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

#ifndef SLAM_MAPGEN_LOOPDETECTOR_H
#define SLAM_MAPGEN_LOOPDETECTOR_H


#include <Map.h>
#include <iostream>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>

// DBoW
#include <DBoW2.h>
#include <BowVector.h>
#include "ORBVocabularyExt.h"

#include <boost/log/trivial.hpp>

using namespace MapGen;

namespace MapGen {

    class LoopDetector {
    public:
        // perform offline loop-detection from the loop detector
        // if pretrained_voc is specified, use pretrained_voc
        LoopDetector(MapGen::Map &, const std::string &, const std::string &pretrained_voc, double loop_closure_thres);

        std::vector<std::pair<KeyFrame *, KeyFrame *>> getLoopClosingPairs();

    private:
        // iterate through all images and compute BowVector for each image
        void computeAllFeatures(MapGen::Map &, const std::string &);
        void detectLoops(MapGen::Map &map);

        double loop_closure_thres_;

        // the directory to raw RGB image
        std::string img_dir_;

        // TODO: tun the parameter of orb vocabulary
        ORBVocabularyExt voc_;
        cv::Ptr<cv::ORB> orb_;

        std::vector<std::pair<KeyFrame *, KeyFrame *>> loop_closure_pairs_;
    };

}


#endif //SLAM_MAPGEN_LOOPDETECTOR_H
