

#include <rotatingSonarRosPlugin.h>

namespace gazebo {

    void RotatingSonarRosPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {

        RayPlugin::Load(_parent, _sdf);
        gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
        gazebo_node_->Init();
        // Get the parent ray sensor
        #if GAZEBO_MAJOR_VERSION >= 7
                parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);
        #else
                parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);
        #endif
        if (!parent_ray_sensor_) {
            gzthrow("GazeboRosSonar" << STR_Gpu << "Laser controller requires a " << STR_Gpu
                                     << "Ray Sensor as its parent");
        }

        //SDF TESTS:
        if (!_sdf->HasElement("frameName")) {
            ROS_INFO("Rotating Sonar Ros plugin missing <frameName>, defaults to /world");
            frame_name_ = "/world";
        } else {
            frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
        }
        robot_namespace_ = "/";
        if (_sdf->HasElement("robotNamespace")) {
            robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        if (!_sdf->HasElement("min_range")) {
            ROS_INFO("Rotating Sonar Ros plugin missing <min_range>, defaults to 0");
            min_range_ = 0;
        } else {
            min_range_ = _sdf->GetElement("min_range")->Get<double>();
        }

        if (!_sdf->HasElement("max_range")) {
            ROS_INFO("Rotating Sonar Ros plugin missing <max_range>, defaults to infinity");
            max_range_ = INFINITY;
        } else {
            max_range_ = _sdf->GetElement("max_range")->Get<double>();
        }

        if (!_sdf->HasElement("topicName")) {
            ROS_INFO("Rotating Sonar Ros plugin missing <topicName>, defaults to /points");
            topic_name_ = "/points";
        } else {
            topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
        }


        //STARTING WITH ROS
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                      ros::init_options::NoSigintHandler);
        }
        std::cerr << "Test if the plugin is loaded\n";
        // Create node handle
        nh_ = new ros::NodeHandle(robot_namespace_);

        if (topic_name_ != "") {
            ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
                    topic_name_, 1,
                    boost::bind(&RotatingSonarRosPlugin::ConnectCb, this),
                    boost::bind(&RotatingSonarRosPlugin::ConnectCb, this),
                    ros::VoidPtr(), &laser_queue_);
            pub_ = nh_->advertise(ao);
        }


        // Start custom queue for laser
        callback_laser_queue_thread_ = boost::thread(boost::bind(&RotatingSonarRosPlugin::laserQueueThread, this));
        #if GAZEBO_MAJOR_VERSION >= 7
                ROS_INFO("Velodyne %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->VerticalRangeCount());
                ROS_INFO("Velodyne %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->RayCount());
        #else
                ROS_INFO("Velodyne %slaser plugin ready, %i lasers", STR_GPU_, parent_ray_sensor_->GetHorizontalRangeCount());
        #endif

    }

    void RotatingSonarRosPlugin::ConnectCb()
    {
        boost::lock_guard<boost::mutex> lock(lock_);
        if (pub_.getNumSubscribers()) {
            if (!sub_) {
    #if GAZEBO_MAJOR_VERSION >= 7
                    sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(), &RotatingSonarRosPlugin::OnScan, this);
    #else
                    sub_ = gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(), &GazeboRosVelodyneLaser::OnScan, this);
    #endif
                }
                parent_ray_sensor_->SetActive(true);
            } else {
    #if GAZEBO_MAJOR_VERSION >= 7
                if (sub_) {
                    sub_->Unsubscribe();
                    sub_.reset();
                }
    #endif
                parent_ray_sensor_->SetActive(false);
            }
    }

    void RotatingSonarRosPlugin::OnScan(ConstLaserScanStampedPtr& _msg)
    {
#if GAZEBO_MAJOR_VERSION >= 7
        const ignition::math::Angle maxAngle = parent_ray_sensor_->AngleMax();
        const ignition::math::Angle minAngle = parent_ray_sensor_->AngleMin();

        const double maxRange = parent_ray_sensor_->RangeMax();
        const double minRange = parent_ray_sensor_->RangeMin();

        const int rayCount = parent_ray_sensor_->RayCount();
        const int rangeCount = parent_ray_sensor_->RangeCount();

        const int verticalRayCount = parent_ray_sensor_->VerticalRayCount();
        const int verticalRangeCount = parent_ray_sensor_->VerticalRangeCount();

        const ignition::math::Angle verticalMaxAngle = parent_ray_sensor_->VerticalAngleMax();
        const ignition::math::Angle verticalMinAngle = parent_ray_sensor_->VerticalAngleMin();
#else
        math::Angle maxAngle = parent_ray_sensor_->GetAngleMax();
  math::Angle minAngle = parent_ray_sensor_->GetAngleMin();

  const double maxRange = parent_ray_sensor_->GetRangeMax();
  const double minRange = parent_ray_sensor_->GetRangeMin();

  const int rayCount = parent_ray_sensor_->GetRayCount();
  const int rangeCount = parent_ray_sensor_->GetRangeCount();

  const int verticalRayCount = parent_ray_sensor_->GetVerticalRayCount();
  const int verticalRangeCount = parent_ray_sensor_->GetVerticalRangeCount();

  const math::Angle verticalMaxAngle = parent_ray_sensor_->GetVerticalAngleMax();
  const math::Angle verticalMinAngle = parent_ray_sensor_->GetVerticalAngleMin();
#endif

        const double yDiff = maxAngle.Radian() - minAngle.Radian();
        const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

        const double MIN_RANGE = std::max(min_range_, minRange);
        const double MAX_RANGE = std::min(max_range_, maxRange);
        const double MIN_INTENSITY = 0;

        // Populate message fields
        const uint32_t POINT_STEP = 22;
        sensor_msgs::PointCloud2 msg;
        msg.header.frame_id = frame_name_;
        msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
        msg.fields.resize(6);
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[0].count = 1;
        msg.fields[1].name = "y";
        msg.fields[1].offset = 4;
        msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[1].count = 1;
        msg.fields[2].name = "z";
        msg.fields[2].offset = 8;
        msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[2].count = 1;
        msg.fields[3].name = "intensity";
        msg.fields[3].offset = 12;
        msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[3].count = 1;
        msg.fields[4].name = "ring";
        msg.fields[4].offset = 16;
        msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
        msg.fields[4].count = 1;
        msg.fields[5].name = "time";
        msg.fields[5].offset = 18;
        msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[5].count = 1;
        msg.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

        int i, j;
        uint8_t *ptr = msg.data.data();
        for (i = 0; i < rangeCount; i++) {
            for (j = 0; j < verticalRangeCount; j++) {

                // Range
                double r = _msg->scan().ranges(i + j * rangeCount);
                // Intensity
                double intensity = _msg->scan().intensities(i + j * rangeCount);
                // Ignore points that lay outside range bands or optionally, beneath a
                // minimum intensity level.
                if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY) ) {
                    if (!organize_cloud_) {
                        continue;
                    }
                }

                // Get angles of ray to get xyz for point
                double yAngle;
                double pAngle;

                if (rangeCount > 1) {
                    yAngle = i * yDiff / (rangeCount -1) + minAngle.Radian();
                } else {
                    yAngle = minAngle.Radian();
                }

                if (verticalRayCount > 1) {
                    pAngle = j * pDiff / (verticalRangeCount -1) + verticalMinAngle.Radian();
                } else {
                    pAngle = verticalMinAngle.Radian();
                }

                // pAngle is rotated by yAngle:
                if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
                    *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle); // x
                    *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle); // y
                    *((float*)(ptr + 8)) = r * sin(pAngle); // z
                    *((float*)(ptr + 12)) = intensity; // intensity
                    *((uint16_t*)(ptr + 16)) = j; // ring
                    *((float*)(ptr + 18)) = 0.0; // time
                    ptr += POINT_STEP;
                } else if (organize_cloud_) {
                    *((float*)(ptr + 0)) = nanf(""); // x
                    *((float*)(ptr + 4)) = nanf(""); // y
                    *((float*)(ptr + 8)) = nanf(""); // x
                    *((float*)(ptr + 12)) = nanf(""); // intensity
                    *((uint16_t*)(ptr + 16)) = j; // ring
                    *((float*)(ptr + 18)) = 0.0; // time
                    ptr += POINT_STEP;
                }
            }
        }

        // Populate message with number of valid points
        msg.data.resize(ptr - msg.data.data()); // Shrink to actual size
        msg.point_step = POINT_STEP;
        msg.is_bigendian = false;
        if (organize_cloud_) {
            msg.width = verticalRangeCount;
            msg.height = msg.data.size() / POINT_STEP / msg.width;
            msg.row_step = POINT_STEP * msg.width;
            msg.is_dense = false;
        } else {
            msg.width = 1;
            msg.height = msg.data.size() / POINT_STEP;
            msg.row_step = msg.data.size();
            msg.is_dense = true;
        }

        // Publish output
        pub_.publish(msg);
    }

    void RotatingSonarRosPlugin::laserQueueThread()
    {
        while (nh_->ok()) {
            laser_queue_.callAvailable(ros::WallDuration(0.01));
        }
    }

}
