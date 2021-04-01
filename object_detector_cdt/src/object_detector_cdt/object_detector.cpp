#include <object_detector_cdt/object_detector.h>


ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
{
    // Read parameters
    readParameters(nh);

    // Setup subscriber
    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe(input_image_topic_, 1, &ObjectDetector::imageCallback, this);

    // Setup publisher
    objects_pub_ = nh.advertise<cdt_msgs::ObjectList>(output_objects_topic_, 10);

    // Extrinsic calibration. This must be updated accordingly
    camera_extrinsic_x_ = 0.2;
    camera_extrinsic_y_ = 0.2;
    camera_extrinsic_z_ = 0.0;

    // Intrinsic calibration
    camera_fx_ = 381.3;
    camera_fy_ = 381.3;
    camera_cx_ = 320.5;
    camera_cy_ = 240.5;

    // Real heights of objects
    barrel_real_height_     = 1.2;   // meters
    barrow_real_height_     = 0.7;   // meters, note: includes the wheel and frame
    computer_real_height_   = 0.5;   // meters
    dog_real_height_        = 0.418; // meters, note: includes legs
}

void ObjectDetector::readParameters(ros::NodeHandle &nh)
{
    // Depending on the parameter (required or optional) the API differs:

    // input_topic is required (no default topic)
    if (!nh.getParam("input_image_topic", input_image_topic_))
    {
        ROS_ERROR("Could not read parameter `input_topic`.");
        exit(-1);
    }

    if (!nh.getParam("input_base_frame", base_frame_))
    {
        ROS_ERROR("Could not read parameter `input_base_frame`.");
        exit(-1);
    }
    if (!nh.getParam("input_fixed_frame", fixed_frame_))
    {
        ROS_ERROR("Could not read parameter `goal_frame`.");
        exit(-1);
    }

    // output topic is optional. It will use '/detected_objects' by default
    nh.param("output_objects_topic", output_objects_topic_, std::string("/detected_objects"));
}

void ObjectDetector::imageCallback(const sensor_msgs::ImageConstPtr &in_msg)
{
    ROS_DEBUG("New image received!");

    // Preallocate some variables
    cv::Mat image;
    ros::Time timestamp;

    double x, y, theta;
    getRobotPose(x, y, theta);

    // Convert message to OpenCV image
    convertMessageToImage(in_msg, image, timestamp);

    //image
    // Recognize object
    // Dog
    // TODO: This only publishes the first time we detect the dog
    // TODO: Add other objects here


    if(!wasObjectDetected("dog")) // TODO: implement a better check
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeDog(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }

    if(!wasObjectDetected("barrel")) // TODO: implement a better check
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeBarrel(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }

    if(!wasObjectDetected("barrow")) // TODO: implement a better check
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeBarrow(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }

    if(!wasObjectDetected("computer")) // TODO: implement a better check
    {
        cdt_msgs::Object new_object;
        bool valid_object = recognizeComputer(image, timestamp, x, y, theta, new_object);

        // If recognized, add to list of detected objects
        if (valid_object)
        {
            detected_objects_.objects.push_back(new_object);
        }
    }

    // Publish list of objects detected so far
    objects_pub_.publish(detected_objects_);
}

void ObjectDetector::convertMessageToImage(const sensor_msgs::ImageConstPtr &in_msg, cv::Mat &out_image, ros::Time &out_timestamp)
{
    // Convert Image message to cv::Mat using cv_bridge
    out_image = cv_bridge::toCvShare(in_msg, "bgr8")->image;
    bool result = cv::imwrite("/home/cdt2021/Desktop/real_image_debug_file.png", out_image);
    // Extract timestamp from header
    out_timestamp = in_msg->header.stamp;
}

cv::Mat ObjectDetector::applyColourFilter(const cv::Mat &in_image_bgr, const Colour &colour)
{
    assert(in_image_bgr.type() == CV_8UC3);

    // Here you should apply some binary threhsolds on the image to detect the colors
    // The output should be a binary mask indicating where the object of a given color is located
    cv::Mat mask;
    //int rows = in_image_bgr.rows;
    //int cols = in_image_bgr.cols;
    //std::cout << "Value in the midddle";
    //std::cout << in_image_bgr.at<double>(rows/2,cols/2);

    //std::cout << "M = " << std::endl << " "  << in_image_bgr << std::endl << std::endl;

    if (colour == Colour::RED) {
        //inRange(in_image_bgr, cv::Scalar(  0,  0,  160), cv::Scalar( 80, 80, 255), mask);
        cv::inRange(in_image_bgr, cv::Scalar(  0,  0,  40), cv::Scalar( 30, 30, 255), mask);
    } else if (colour == Colour::YELLOW) {
        //inRange(in_image_bgr, cv::Scalar(  0,  160,  160), cv::Scalar( 80, 255, 255), mask);
        cv::inRange(in_image_bgr, cv::Scalar(  0,  40,  40), cv::Scalar( 30, 255, 255), mask);
    } else if (colour == Colour::GREEN) {
        cv::inRange(in_image_bgr, cv::Scalar(  0,  40,  0), cv::Scalar( 30, 255, 30), mask);
    } else if (colour == Colour::BLUE) {
        cv::inRange(in_image_bgr, cv::Scalar(  40,  0,  0), cv::Scalar( 255, 30, 30), mask);
    } else {
        // Report color not implemented
        ROS_ERROR_STREAM("[ObjectDetector::colourFilter] colour (" << colour << "  not implemented!");
    }

    // Apply morphological operationOpening:
    bool result = cv::imwrite("/home/cdt2021/Desktop/mask_before_morphology.png", mask);

    cv::Mat element = cv::getStructuringElement( 0, cv::Size( 3, 3 ));
    cv::morphologyEx( mask, mask, 3, element);
    //cv::morphologyEx( mask, mask, 2, element);
    result = cv::imwrite("/home/cdt2021/Desktop/mask_after_morphology.png", mask);


    ///

    // We return the mask, that will be used later
    return mask;
}

cv::Mat ObjectDetector::applyBoundingBox(const cv::Mat1b &in_mask, double &x, double &y, double &width, double &height) {

    cv::Mat drawing = in_mask.clone();
    //cv::Mat(in_mask.rows,in_mask.cols, CV_64F, cvScalar(0.)); // it could be useful to fill this image if you want to debug

    // TODO: Compute the bounding box using the mask
    // You need to return the center of the object in image coordinates, as well as a bounding box indicating its height and width (in pixels)
    cv::Rect Min_Rect = cv::boundingRect(in_mask);
    cv::rectangle(drawing, Min_Rect,(255,255,255));


    x = (Min_Rect.width)/2 + (Min_Rect.x);
    y = (Min_Rect.height)/2 + Min_Rect.y;
    width = Min_Rect.width;
    height = Min_Rect.height;

    bool result;
    try
    {
        result = cv::imwrite("/home/cdt2021/Desktop/image_debug_box_file.png", drawing);
    }
    catch (const cv::Exception& ex)
    {
        fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
    }

    if (!result) {
      std::cout << "Image Not Saved";
    }
    return drawing;
}

bool ObjectDetector::recognizeDog(const cv::Mat &in_image, const ros::Time &in_timestamp,
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    // The values below will be filled by the following functions
    double dog_image_center_x;
    double dog_image_center_y;
    double dog_image_height;
    double dog_image_width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat in_image_red = applyColourFilter(in_image, Colour::RED);
    cv::Mat in_image_bounding_box = applyBoundingBox(in_image_red, dog_image_center_x, dog_image_center_y, dog_image_width, dog_image_height);

    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = dog_real_height_ / dog_image_height * camera_fy_;

    if (depth > 3) {

      return false;
    }

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double dog_position_camera_x = depth / camera_fx_ * (dog_image_center_x - camera_cx_);
    double dog_position_camera_y = depth / camera_fy_ * (dog_image_center_y - camera_cy_);
    double dog_position_camera_z = depth;


    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though

    double dog_position_base_x = (camera_extrinsic_x_ +  dog_position_camera_z);
    double dog_position_base_y = (camera_extrinsic_y_ + -dog_position_camera_x);

    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = "dog";
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*dog_position_base_x + sin(-robot_theta) * dog_position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*dog_position_base_x + cos(robot_theta) * dog_position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -dog_position_camera_y;

    return std::isfinite(depth);
}

// TODO: Implement similar methods for other objects
bool ObjectDetector::recognizeBarrel(const cv::Mat &in_image, const ros::Time &in_timestamp,
                                  const double& robot_x, const double& robot_y, const double& robot_theta,
                                  cdt_msgs::Object &out_new_object)
{
    // The values below will be filled by the following functions
    double obj_image_center_x;
    double obj_image_center_y;
    double obj_image_height;
    double obj_image_width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat in_image_red = applyColourFilter(in_image, Colour::YELLOW);
    cv::Mat in_image_bounding_box = applyBoundingBox(in_image_red, obj_image_center_x, obj_image_center_y, obj_image_width, obj_image_height);

    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = barrel_real_height_ / obj_image_height * camera_fy_;
    if (depth > 3) {

      return false;
    }

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double obj_position_camera_x = depth / camera_fx_ * (obj_image_center_x - camera_cx_);
    double obj_position_camera_y = depth / camera_fy_ * (obj_image_center_y - camera_cy_);
    double obj_position_camera_z = depth;


    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though

    double obj_position_base_x = (camera_extrinsic_x_ +  obj_position_camera_z);
    double obj_position_base_y = (camera_extrinsic_y_ + -obj_position_camera_x);

    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = "barrel";
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*obj_position_base_x + sin(-robot_theta) * obj_position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*obj_position_base_x + cos(robot_theta) * obj_position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -obj_position_camera_y;

    return std::isfinite(depth);
}

bool ObjectDetector::recognizeBarrow(const cv::Mat &in_image, const ros::Time &in_timestamp,
                                     const double& robot_x, const double& robot_y, const double& robot_theta,
                                     cdt_msgs::Object &out_new_object)
{
    // The values below will be filled by the following functions
    double obj_image_center_x;
    double obj_image_center_y;
    double obj_image_height;
    double obj_image_width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat in_image_red = applyColourFilter(in_image, Colour::GREEN);
    cv::Mat in_image_bounding_box = applyBoundingBox(in_image_red, obj_image_center_x, obj_image_center_y, obj_image_width, obj_image_height);

    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = barrow_real_height_ / obj_image_height * camera_fy_;

    if (depth > 3) {

      return false;
    }

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double obj_position_camera_x = depth / camera_fx_ * (obj_image_center_x - camera_cx_);
    double obj_position_camera_y = depth / camera_fy_ * (obj_image_center_y - camera_cy_);
    double obj_position_camera_z = depth;


    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though

    double obj_position_base_x = (camera_extrinsic_x_ +  obj_position_camera_z);
    double obj_position_base_y = (camera_extrinsic_y_ + -obj_position_camera_x);

    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = "barrow";
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*obj_position_base_x + sin(-robot_theta) * obj_position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*obj_position_base_x + cos(robot_theta) * obj_position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -obj_position_camera_y;

    return std::isfinite(depth);
}


bool ObjectDetector::recognizeComputer(const cv::Mat &in_image, const ros::Time &in_timestamp,
                                     const double& robot_x, const double& robot_y, const double& robot_theta,
                                     cdt_msgs::Object &out_new_object)
{
    // The values below will be filled by the following functions
    double obj_image_center_x;
    double obj_image_center_y;
    double obj_image_height;
    double obj_image_width;

    // TODO: the functions we use below should be filled to make this work
    cv::Mat in_image_red = applyColourFilter(in_image, Colour::BLUE);
    cv::Mat in_image_bounding_box = applyBoundingBox(in_image_red, obj_image_center_x, obj_image_center_y, obj_image_width, obj_image_height);

    // Note: Almost everything below should be kept as it is

    // We convert the image position in pixels into "real" coordinates in the camera frame
    // We use the intrinsics to compute the depth
    double depth = computer_real_height_ / obj_image_height * camera_fy_;

    if (depth > 3) {

      return false;
    }

    // We now back-project the center using the  pinhole camera model
    // The result is in camera coordinates. Camera coordinates are weird, see note below
    double obj_position_camera_x = depth / camera_fx_ * (obj_image_center_x - camera_cx_);
    double obj_position_camera_y = depth / camera_fy_ * (obj_image_center_y - camera_cy_);
    double obj_position_camera_z = depth;


    // Camera coordinates are different to robot and fixed frame coordinates
    // Robot and fixed frame are x forward, y left and z upward
    // Camera coordinates are x right, y downward, z forward
    // robot x -> camera  z
    // robot y -> camera -x
    // robot z -> camera -y
    // They follow x-red, y-green and z-blue in both cases though

    double obj_position_base_x = (camera_extrinsic_x_ +  obj_position_camera_z);
    double obj_position_base_y = (camera_extrinsic_y_ + -obj_position_camera_x);

    // We need to be careful when computing the final position of the object in global (fixed frame) coordinates
    // We need to introduce a correction givne by the robot orientation
    // Fill message
    out_new_object.id = "computer";
    out_new_object.header.stamp = in_timestamp;
    out_new_object.header.frame_id = fixed_frame_;
    out_new_object.position.x = robot_x +  cos(robot_theta)*obj_position_base_x + sin(-robot_theta) * obj_position_base_y;
    out_new_object.position.y = robot_y +  sin(robot_theta)*obj_position_base_x + cos(robot_theta) * obj_position_base_y;
    out_new_object.position.z = 0.0     + camera_extrinsic_z_ + -obj_position_camera_y;

    //std::cout << "Final Position Computer" << out_new_object.position.x << " " << out_new_object.position.y << " " << out_new_object.position.z << "Finish";
    return std::isfinite(depth);
}


// Utils
void ObjectDetector::getRobotPose(double &x, double &y, double &theta)
{
    // Get current pose
    tf::StampedTransform base_to_map_transform;
    tf_listener_.waitForTransform(fixed_frame_, base_frame_,  ros::Time(0), ros::Duration(0.5));
    try
    {
        tf_listener_.lookupTransform(fixed_frame_, base_frame_, ros::Time(0), base_to_map_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Extract components from robot pose
    x = base_to_map_transform.getOrigin().getX();
    y = base_to_map_transform.getOrigin().getY();

    // Extract orientation is more involved, since it is a quaternion
    // We'll get some help from Eigen
    // First we create an Eigen quaternion
    Eigen::Quaterniond q(base_to_map_transform.getRotation().getW(),
                         base_to_map_transform.getRotation().getX(),
                         base_to_map_transform.getRotation().getY(),
                         base_to_map_transform.getRotation().getZ());
    // We convert it to an Axis-Angle representation
    // This representation is given by an axis wrt to some coordinate frame, and a rotation along that axis
    Eigen::AngleAxisd axis_angle(q);

    // The value corresponding to the z component is the orientation wrt to the z axis (planar rotation)
    // We need to extract the z component of the axis and multiply it by the angle
    theta = axis_angle.axis().z() * axis_angle.angle();
}

bool ObjectDetector::wasObjectDetected(std::string object_name)
{
    bool detected = false;
    for(auto obj : detected_objects_.objects)
    {
        if(obj.id == object_name)
            detected = true;
    }

    return detected;
}
