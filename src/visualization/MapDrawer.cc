#include "MapDrawer.h"

using std::vector;

namespace MapGen {

MapDrawer::MapDrawer(Map *map, NodeConfig * config):
debug_file("debug.txt",std::ofstream::out) {
    map_ = map;
    config_ = config;
    mesh_ = nullptr;
}

MapDrawer::MapDrawer(Map *map, NodeConfig * config, pcl::PolygonMeshPtr mesh):
debug_file("debug.txt",std::ofstream::out) {
    map_ = map;
    config_ = config;
    mesh_ = mesh;
}

void MapDrawer::DrawMapPoints() {
    const std::vector<MapPoint*> &points = map_->GetAllMapPoints();

    if (points.empty())
        return;

    glPointSize(Config::PointSize());
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (size_t i = 0, iend=points.size(); i < iend; i++) {
        Eigen::Vector3d pos = points[i]->GetWorldPos();
        glVertex3f(pos(0), pos(1), pos(2));
    }
    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
    pangolin::OpenGlMatrix glmatrix;
    const float &w = Config::KeyFrameSize();
    const float h = w*0.75;
    const float z = w*0.6;

    const std::vector<KeyFrame*> &keyframes = map_->GetAllKeyFrames();

    if (bDrawKF) {
        double lwidth = Config::KeyFrameLineWidth();
        for (size_t i = 0; i < keyframes.size(); i++) {
            Eigen::Matrix4d Twc = keyframes[i]->GetPose();

            // Convert to cv::Mat
            cv::Mat Twc_cv(4, 4, CV_32F);
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    Twc_cv.at<float>(i, j) = Twc(j, i);

            glPushMatrix();

            glMultMatrixf(Twc_cv.ptr<GLfloat>(0));

            glLineWidth(lwidth);
            glColor3f(0.0f, 0.0f, 0.9f);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w,-h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w,-h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);

            glVertex3f(w, h, z);
            glVertex3f(w,-h, z);

            glVertex3f(-w, h, z);
            glVertex3f(-w,-h, z);

            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);

            glVertex3f(-w,-h, z);
            glVertex3f(w,-h, z);
            glEnd();

            glPopMatrix();
        }
    }

    if (bDrawGraph) {
        glLineWidth(Config::GraphLineWidth());
        glColor4f(0.7f, 0.0f, 0.7f, 0.6f);
        glBegin(GL_LINES);

        for (size_t i = 0; i < keyframes.size(); i++) {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = keyframes[i]->GetConnectedByWeight(50);

            Eigen::Vector3d Ow = keyframes[i]->GetTranslation();
            if (!vCovKFs.empty()) {
                for (vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++) {
                    Eigen::Vector3d Ow2 = (*vit)->GetTranslation();
                    glVertex3f(Ow(0),Ow(1),Ow(2));
                    glVertex3f(Ow2(0),Ow2(1),Ow2(2));
                }
            }
        }

        glEnd();
    }
}

    void MapDrawer::DrawTriangle(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3, bool draw_border) {
        // std::cout << "[DEBUG, MapDrawer.cc] Draw once" << std::endl;

        glBegin(GL_TRIANGLES);
        glColor3f(0.0, 0.0, 1.0);

        // the first vertex
        glVertex3f(v1(0),v1(1),v1(2));
        // the second vertex
        glVertex3f(v2(0),v2(1),v2(2));
        // the third vertex
        glVertex3f(v3(0),v3(1),v3(2));
        glEnd();

        if (draw_border){
            glBegin(GL_LINES);
            glColor4f(0.0, 0.0, 0.0, 0.5);
            // the first line
            glVertex3f(v1(0),v1(1),v1(2));
            glVertex3f(v2(0),v2(1),v2(2));
            // the second line
            glVertex3f(v2(0),v2(1),v2(2));
            glVertex3f(v3(0),v3(1),v3(2));
            // the third line
            glVertex3f(v1(0),v1(1),v1(2));
            glVertex3f(v3(0),v3(1),v3(2));
            glEnd();
        }

    }

    void MapDrawer::DrawTriangle(pcl::PointXYZ pt1, pcl::PointXYZ pt2, pcl::PointXYZ pt3, bool draw_border) {
        // std::cout << "[DEBUG, MapDrawer.cc] Draw once" << std::endl;

        glBegin(GL_TRIANGLES);
        glColor3f(0.0, 0.0, 1.0);

        // the first vertex
        glVertex3f(pt1.x,pt1.y,pt1.z);
        // the second vertex
        glVertex3f(pt2.x,pt2.y,pt2.z);
        // the third vertex
        glVertex3f(pt3.x,pt3.y,pt3.z);
        glEnd();

        if (draw_border){
            glBegin(GL_LINES);
            glColor4f(0.0, 0.0, 0.0, 0.5);
            // the first line
            glVertex3f(pt1.x,pt1.y,pt1.z);
            glVertex3f(pt2.x,pt2.y,pt2.z);
            // the second line
            glVertex3f(pt2.x,pt2.y,pt2.z);
            glVertex3f(pt3.x,pt3.y,pt3.z);
            // the third line
            glVertex3f(pt1.x,pt1.y,pt1.z);
            glVertex3f(pt3.x,pt3.y,pt3.z);
            glEnd();
        }
    }


    void MapDrawer::DrawSurface() {
        if ((mesh_ == nullptr)){
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> pcl_points;
        pcl::fromPCLPointCloud2(mesh_->cloud, pcl_points);

        // if no corespondence between pcl::PointXYZ and MapPoints established, build it
        if (pt_correspondence_.size() == 0) {
            BuildCorrespondence();
            LOG_INFO << "point correspondence built, total points: " << pt_correspondence_.size() << std::endl;
        }

        auto polygons = mesh_->polygons;

        if (config_->get_double_param("SurfaceRecon","enableStitchingImage")) {
            // draw triangles with texture
            for (int polygon_idx = 0; polygon_idx < mesh_->polygons.size(); polygon_idx++) {
                DrawTriangleTexture(polygon_idx, false);
            }
        }
        else{
            // draw triangles without texture
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromPCLPointCloud2(mesh_->cloud, cloud);
            for (auto v : polygons){
                int p0_idx = v.vertices[0];
                int p1_idx = v.vertices[1];
                int p2_idx = v.vertices[2];
                DrawTriangle(cloud[p0_idx], cloud[p1_idx], cloud[p2_idx], true);
            }
        }
    }


    bool MapDrawer::DrawTriangleTexture(int polygon_idx, bool draw_border) {
        // find the corresponding map-points
        std::vector<MapPoint *> points;
        pcl::Vertices v = mesh_->polygons.at(polygon_idx);
        int p0_idx = v.vertices[0];
        int p1_idx = v.vertices[1];
        int p2_idx = v.vertices[2];
        Eigen::Vector3d pt0 = pt_correspondence_[p0_idx]->GetWorldPos();
        Eigen::Vector3d pt1 = pt_correspondence_[p1_idx]->GetWorldPos();
        Eigen::Vector3d pt2 = pt_correspondence_[p2_idx]->GetWorldPos();

        // check if the observation has been cached
        if (cached_observations_.count(polygon_idx) == 0){
            // fill in the cache
            std::vector<MapPoint *> map_points;
            map_points.push_back(pt_correspondence_[p0_idx]);
            map_points.push_back(pt_correspondence_[p1_idx]);
            map_points.push_back(pt_correspondence_[p2_idx]);
            cached_observations_[polygon_idx] = GetObservations(map_points);
        }

        // observations
        auto observations = cached_observations_[polygon_idx];
        if (observations.size() == 0){
            return false;                         // early return on invalid triangle
        }
        auto kf = observations[0].first;


        // load image if needed
        if (surf_textures_.count(kf) == 0){
            std::string img_dir = config_->get_string_param("Common", "img_dir");
            std::string img_filename = img_dir + kf->GetFilename();
            surf_textures_[kf] = SOIL_load_OGL_texture // load an image file directly as a new OpenGL texture
                    (
                            img_filename.c_str(),
                            SOIL_LOAD_AUTO,
                            SOIL_CREATE_NEW_ID,
                            SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT
                    );


            if (surf_textures_[kf] == 0){
                LOG_ERROR << "SOIL can not load image at: " << img_filename << std::endl;
                throw std::runtime_error("Failure in loading image.");
            }
        }

        // draw surface and stitch texture
        glEnable(GL_TEXTURE_2D);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glBindTexture(GL_TEXTURE_2D, surf_textures_[kf]);

        // get height and width, we need to rescale the pixel coordinate for opengl
        int width, height;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);

        glBegin(GL_TRIANGLES);
        // first vertex
        glTexCoord2f(observations[0].second[0] / width, 1 - observations[0].second[1] / height);         // stitching images
        glVertex3f(pt0[0],pt0[1],pt0[2]);                                           // 3d vertices
        // the second vertex
        glTexCoord2f(observations[1].second[0] / width, 1 - observations[1].second[1] / height);
        glVertex3f(pt1[0],pt1[1],pt1[2]);
        // the third vertex
        glTexCoord2f(observations[2].second[0] / width, 1 - observations[2].second[1] / height);
        glVertex3f(pt2[0],pt2[1],pt2[2]);
        glEnd();
        glDisable(GL_TEXTURE_2D);

        if (draw_border){
            glBegin(GL_LINES);
            glColor4f(0.0, 0.0, 0.0, 0.5);
            // the first line
            glVertex3f(pt0[0],pt0[1],pt0[2]);
            glVertex3f(pt1[0],pt1[1],pt1[2]);
            // the second line
            glVertex3f(pt1[0],pt1[1],pt1[2]);
            glVertex3f(pt2[0],pt2[1],pt2[2]);
            // the third line
            glVertex3f(pt0[0],pt0[1],pt0[2]);
            glVertex3f(pt2[0],pt2[1],pt2[2]);
            glEnd();
        }

        return true;

    }


    void MapDrawer::DrawTriangleTexture(std::vector<std::pair<pcl::PointXYZ, MapPoint *>> points_pairs,
                                        bool draw_border) {

        assert(points_pairs.size() == 3);

        std::vector<MapPoint *> v;
        v.push_back(points_pairs[0].second);
        v.push_back(points_pairs[1].second);
        v.push_back(points_pairs[2].second);

        // get the shared observation
        auto observations = GetObservations(v);
        if (observations.size() == 0)       return;          // break if no single image contains all 3 vertice
        auto kf = observations[0].first;

        // load image if needed
        if (surf_textures_.count(kf) == 0){
            std::string img_dir = config_->get_string_param("Common", "img_dir");
            std::string img_filename = img_dir + kf->GetFilename();
            surf_textures_[kf] = SOIL_load_OGL_texture // load an image file directly as a new OpenGL texture
                    (
                            img_filename.c_str(),
                            SOIL_LOAD_AUTO,
                            SOIL_CREATE_NEW_ID,
                            SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT
                    );

            if (surf_textures_[kf] == 0){
                LOG_ERROR << "SOIL can not load image at: " << img_filename << std::endl;
                throw std::runtime_error("Failure in loading image.");
            }
        }

        // draw surface and stitch texture
        std::vector<pcl::PointXYZ> points_pcl;
        pcl::PointXYZ pt1 = points_pairs[0].first;
        pcl::PointXYZ pt2 = points_pairs[1].first;
        pcl::PointXYZ pt3 = points_pairs[2].first;

        glEnable(GL_TEXTURE_2D);
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
        glBindTexture(GL_TEXTURE_2D, surf_textures_[kf]);
        glBegin(GL_TRIANGLES);
        // first vertex
        glTexCoord2f(observations[0].second[0], observations[0].second[1]);         // stitching images
        glVertex3f(pt1.x,pt1.y,pt1.z);                                              // 3d vertices
        // the second vertex
        glTexCoord2f(observations[1].second[0], observations[1].second[1]);
        glVertex3f(pt2.x,pt2.y,pt2.z);
        // the third vertex
        glTexCoord2f(observations[2].second[0], observations[2].second[1]);
        glVertex3f(pt3.x,pt3.y,pt3.z);
        glEnd();
        glDisable(GL_TEXTURE_2D);

        if (draw_border){
            glBegin(GL_LINES);
            glColor4f(0.0, 0.0, 0.0, 0.5);
            // the first line
            glVertex3f(pt1.x,pt1.y,pt1.z);
            glVertex3f(pt2.x,pt2.y,pt2.z);
            // the second line
            glVertex3f(pt2.x,pt2.y,pt2.z);
            glVertex3f(pt3.x,pt3.y,pt3.z);
            // the third line
            glVertex3f(pt1.x,pt1.y,pt1.z);
            glVertex3f(pt3.x,pt3.y,pt3.z);
            glEnd();
        }

    }


    std::vector<std::pair<KeyFrame *, Eigen::Vector2d>> MapDrawer::GetObservations(std::vector<MapPoint *> points) {
        auto observations_1 = points[0]->GetObservarionsWithPose();
        auto observations_2 = points[1]->GetObservarionsWithPose();
        auto observations_3 = points[2]->GetObservarionsWithPose();

        std::vector<std::pair<KeyFrame *, Eigen::Vector2d>> res;

        for (auto ob : observations_1){
            KeyFrame * frame = ob.first;
            if ((observations_2.count(frame) == 1) && (observations_3.count(frame) == 1)){
                // found the shared frame
                res.push_back(std::pair<KeyFrame *, Eigen::Vector2d>(frame, observations_1[frame]));
                res.push_back(std::pair<KeyFrame *, Eigen::Vector2d>(frame, observations_2[frame]));
                res.push_back(std::pair<KeyFrame *, Eigen::Vector2d>(frame, observations_3[frame]));
                LOG_INFO <<  "Found shared parent: " << frame->GetId() << std::endl;
                break;
            }
        }

//        if (res.size() == 0){
//            LOG_ERROR << "One not found" << std::endl;
//        }

        return res;
    }

    MapPoint * MapDrawer::SearchNearest(pcl::PointXYZ point) {
        MapPoint * pt_best = nullptr;
        double dist_best = 1e5;
        for (MapPoint * pt : map_->GetAllMapPoints()){
            auto pos = pt->GetWorldPos();
            double dist = pow(point.x - pos[0],2) + pow(point.y - pos[1],2) + pow(point.z - pos[2],2);
            if (dist < dist_best){
                dist_best = dist;
                pt_best = pt;
            }
        }

        if (dist_best > 1e-6) {
            LOG_ERROR << "minimum distance: " << dist_best << std::endl;
            LOG_ERROR << "It's normal only if you are using implict surface reconstruction method. " << std::endl;
        }

        return pt_best;
    }

    void MapDrawer::BuildCorrespondence() {
        // clear previous correspondence (just for sure)
        pt_correspondence_.clear();

        // pcl points
        pcl::PointCloud<pcl::PointXYZ> pcl_points;
        pcl::fromPCLPointCloud2(mesh_->cloud,pcl_points);

        // brute-force search to get the correspondence
        for (int pt_idx = 0; pt_idx < pcl_points.size(); pt_idx ++){
            auto point = SearchNearest(pcl_points[pt_idx]);
            pt_correspondence_[pt_idx] = point;
        }
    }

}  // namespace SLAM_VIEWER
