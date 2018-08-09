//
// Created by ernest on 18-7-11.
//

#include "JdeRobotIO.h"


namespace MapGen{

    void JdeRobotIO::saveTrajectory(Map &map, const Camera &cam, const std::string &filename) {
        std::ofstream f(filename);
        if (!f){
            LOG_ERROR << "Failed to open file: " << filename << std::endl;
            throw std::runtime_error("");
        }

        std::string output = "%YAML:1.0\n";
        // Save camera parameters
        CameraParameters cam_param = cam.get_camera_params();
        output += "camera:\n";
        output += "  fx: " + std::to_string(cam_param.fx) + "\n";
        output += "  fy: " + std::to_string(cam_param.fy) + "\n";
        output += "  cx: " + std::to_string(cam_param.cx) + "\n";
        output += "  cy: " + std::to_string(cam_param.cy) + "\n";
        output += "  k1: " + std::to_string(cam_param.k1) + "\n";
        output += "  k2: " + std::to_string(cam_param.k2) + "\n";
        output += "  p1: " + std::to_string(cam_param.p1) + "\n";
        output += "  p2: " + std::to_string(cam_param.p2) + "\n";
        output += "  k3: " + std::to_string(cam_param.k3) + "\n";


        // Save keyframes
        auto vpKFs = map.GetAllKeyFrames();
        output += "keyframes:\n";

        for(size_t i=0; i<vpKFs.size(); i++) {
            KeyFrame* pKF = vpKFs[i];

            Eigen::Quaterniond q(pKF->GetRotation());
            Eigen::Vector3d t = pKF->GetTranslation();

            output += "  - id: " + std::to_string(pKF->GetId()) + "\n";
            output += "    filename: \"" + pKF->GetFilename() + "\"\n";
            output += "    pose:\n";
            output += "      - " + std::to_string(q.w()) + "\n";
            output += "      - " + std::to_string(q.x()) + "\n";
            output += "      - " + std::to_string(q.y()) + "\n";
            output += "      - " + std::to_string(q.z()) + "\n";
            output += "      - " + std::to_string(t(0)) + "\n";
            output += "      - " + std::to_string(t(1)) + "\n";
            output += "      - " + std::to_string(t(2)) + "\n";
        }

        f << output;

        // Save map points
        // The points are re-indexed through this stage
        f << "points:\n";
        auto map_points = map.GetAllMapPoints();
        int point_idx = 0;
        for (MapPoint * point : map_points){
            Eigen::Vector3d pos = point->GetWorldPos();

            std::string output = "";
            output += "  - id: " + std::to_string(point_idx) + "\n";
            output += "    pose:\n";
            output += "      - " + std::to_string(pos(0)) + "\n";
            output += "      - " + std::to_string(pos(1)) + "\n";
            output += "      - " + std::to_string(pos(2)) + "\n";
            output += "    observations:\n";
            for (KeyFrame * kf : point->GetObservations()){
                Eigen::Vector2d ob = kf->GetObservations()[point];
                output += "      - kf: " + std::to_string(kf->GetId()) + "\n";
                output += "        pixel:\n";
                output += "          - "+ std::to_string(ob(1)) + "\n";
                output += "          - "+ std::to_string(ob(2)) + "\n";
            }

            f << output;
            point_idx ++;
        }
        // output += "points:";
        LOG_INFO << "Trajectory saved to file: " << filename << std::endl;
    }

}