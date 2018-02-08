#include "FindCircle.h"

void FindCircle::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (image->bpp != msg->step / msg->width || image->width != msg->width || image->height != msg->height) {
        delete image;
        ROS_INFO("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.", image->width, image->height, image->bpp, msg->width, msg->height, msg->step / msg->width);
        image = new CRawImage(msg->width, msg->height, msg->step / msg->width);
    }

    memcpy(image->data, (void*) &msg->data[0], msg->step * msg->height);

    circle_detection::detection_results_array tracked_objects;
    tracked_objects.header = msg->header;
    visualization_msgs::MarkerArray marker_list;

    for (int i = 0;i<MAX_PATTERNS;i++){
        if (currentSegmentArray[i].valid){
            lastSegmentArray[i] = currentSegmentArray[i];
            currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
        }
    }

    //search for untracked (not detected in the last frame) robots
    for (int i = 0;i<MAX_PATTERNS;i++){
        if (currentSegmentArray[i].valid == false){
            lastSegmentArray[i].valid = false;
            currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
        }
        //does not make sense to search for more patterns if the last one was not found
        if (currentSegmentArray[i].valid == false) break;
    }

    // //search image for circles
    for (int i = 0; i < MAX_PATTERNS; i++) {
        objectArray[i].valid = false;

        if (currentSegmentArray[i].valid) {
            objectArray[i] = trans->transform(currentSegmentArray[i]);
            // Filter error values
            if (isnan(objectArray[i].x)||
                isnan(objectArray[i].y) ||
                isnan(objectArray[i].z) ||
                isnan(objectArray[i].roll) ||
                isnan(objectArray[i].pitch) ||
                isnan(objectArray[i].yaw))
            {
                printf("%s\n","SKIPPING NAN");
                continue;
            }

            // temp value to hold current detection
            circle_detection::detection_results objectsToAdd;

            // Convert to ROS standard Coordinate System
            objectsToAdd.pose.position.x = -objectArray[i].y;
            objectsToAdd.pose.position.y = -objectArray[i].z;
            objectsToAdd.pose.position.z = objectArray[i].x;

            // Convert YPR to Quaternion
            tf::Quaternion q;
            q.setRPY(objectArray[i].roll, objectArray[i].pitch, objectArray[i].yaw);
            objectsToAdd.pose.orientation.x = q.getX();
            objectsToAdd.pose.orientation.y = q.getY();
            objectsToAdd.pose.orientation.z = q.getZ();
            objectsToAdd.pose.orientation.w = q.getW();
            if(identify) objectsToAdd.uuid = objectArray[i].id;
            objectsToAdd.roundness = objectArray[i].roundness;
            objectsToAdd.bwratio = objectArray[i].bwratio;
            objectsToAdd.esterror = objectArray[i].esterror;

            // Hardcoded values need to be replaced
            objectsToAdd.objectsize.x = 3;
            objectsToAdd.objectsize.y = 3;
            objectsToAdd.objectsize.z = 0;

            tracked_objects.tracked_objects.push_back(objectsToAdd);

            // Generate RVIZ marker for visualisation
            visualization_msgs::Marker marker;
            marker.header = msg->header;
            marker.id = objectArray[i].id;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::MODIFY;
            marker.pose = objectsToAdd.pose;
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(0.2);
            marker_list.markers.push_back(marker);
        }
    }
    // Check if no markers were found
    if ((marker_list.markers.size() > 0) && (tracked_objects.tracked_objects.size() > 0)) {
        tracks_pub.publish(tracked_objects);
        vis_pub.publish(marker_list);
    }
    memcpy((void*) &msg->data[0], image->data, msg->step * msg->height);
    imdebug.publish(msg);
}

void FindCircle::cameraInfoCallBack(const sensor_msgs::CameraInfo &msg) {
    trans->updateParams(msg.K[2], msg.K[5], msg.K[0], msg.K[4]);
    // trans->updateParams(msg.P[2], msg.P[6], msg.P[0], msg.P[5]);
    // trans->updateParams(msg.K[2], msg.K[5], msg.K[0], msg.K[4]);
}

FindCircle::FindCircle(void) {
    nh = new ros::NodeHandle("~");
    defaultImageWidth = 1280;
    defaultImageHeight = 800;
    if (!nh->getParam("circle_diameter", circleDiameter))
        throw std::runtime_error("Private parameter \"circle_diameter\" is missing");
    if (!nh->getParam("identify", identify))
        throw std::runtime_error("Private parameter \"identify\" is missing");
    node_name = ros::this_node::getName();
}

FindCircle::~FindCircle(void) {
    delete image;
    for (int i = 0; i < MAX_PATTERNS; i++) delete detectorArray[i];
    delete trans;
}

void FindCircle::init(void) {
    image_transport::ImageTransport it(*nh);
    subinfo = nh->subscribe("/camera/camera_info", 1, &FindCircle::cameraInfoCallBack, this);
    image = new CRawImage(defaultImageWidth, defaultImageHeight, 4);
    trans = new CTransformation(circleDiameter, nh);
    for (int i = 0; i < MAX_PATTERNS; i++) detectorArray[i] = new CCircleDetect(defaultImageWidth, defaultImageHeight, identify);
    image->getSaveNumber();
    subim = it.subscribe("/camera/image_rect_color", 1, &FindCircle::imageCallback, this);
    imdebug = it.advertise("/" + node_name+ "/processedimage", 0);
    tracks_pub = nh->advertise<circle_detection::detection_results_array>("/" + node_name + "/tracking_Array", 0);
    vis_pub = nh->advertise<visualization_msgs::MarkerArray>("/" + node_name + "/rviz_marker", 0);
    lookup = new tf::TransformListener();
    ROS_DEBUG("Server running");
    ros::spin();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "circle_detector", ros::init_options::AnonymousName);
    FindCircle *detector = new FindCircle();
    //Attempt to start detector
    detector->init();
    //Clean up
    detector->~FindCircle();
    return 0;
}