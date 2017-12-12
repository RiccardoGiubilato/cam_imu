#include "acc_integrator.h"

accInt::accInt(ros::NodeHandle& nh ): nh_(nh) {

  lpfiltx_ = new acc_filter(0.1, 159, LOWPASS);
  lpfilty_ = new acc_filter(0.1, 159, LOWPASS);
  lpfiltz_ = new acc_filter(0.1, 159, LOWPASS);

  hpfiltx_ = new acc_filter(0.1, 159, HIGHPASS);
  hpfilty_ = new acc_filter(0.1, 159, HIGHPASS);
  hpfiltz_ = new acc_filter(0.1, 159, HIGHPASS);

  // Subscribe to imu filtered topic
  imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("imu/data", 1, &accInt::ImuCallback, this);

  // Publish pose
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose", 1);

  pose_msg_.header.frame_id = "odom";

  // Initialize parameters
  pos_ = new float[3];
  pos_[0] = 0.0;
  pos_[1] = 0.0;
  pos_[2] = 0.0;

  vel_ = new float[3];
  vel_[0] = 0.0;
  vel_[1] = 0.0;
  vel_[2] = 0.0;

  acc_ = new float[3];
  accw_ = new float[3];

  gravity0_ = new float[3];
  gravityc_ = new float[3];

  quat0_ = new float[4];

  quatc_ = new float[4];
  quatcinv_ = new float[4];
  quat_ = new float[4];

  initialized_ = false;
};

void accInt::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {

  if(!initialized_) {

    /* Create butterworth filter */

    /* Average 1 s of readings to remove gravity */
    static int count_avg = 1;

    if ( count_avg == 1 ) {
      last_timestamp_ = imu_msg->header.stamp;
      gravity0_[0] = imu_msg -> linear_acceleration.x;
      gravity0_[1] = imu_msg -> linear_acceleration.y;
      gravity0_[2] = imu_msg -> linear_acceleration.z;
    }

    gravity0_[0] += imu_msg -> linear_acceleration.x;
    gravity0_[1] += imu_msg -> linear_acceleration.y;
    gravity0_[2] += imu_msg -> linear_acceleration.z;

    quat0_[0] = imu_msg -> orientation.w;
    quat0_[1] = imu_msg -> orientation.x;
    quat0_[2] = imu_msg -> orientation.y;
    quat0_[3] = imu_msg -> orientation.z;

    count_avg++;

    if ( (imu_msg->header.stamp - last_timestamp_).toSec() > 1.0) {

      gravity0_[0] /= (float) count_avg;
      gravity0_[1] /= (float) count_avg;
      gravity0_[2] /= (float) count_avg;

      initialized_ = true;
      last_timestamp_ = imu_msg->header.stamp;
      ROS_INFO("Integrator initialized..");
    }
  }

  else {

    dt = (imu_msg->header.stamp - last_timestamp_).toSec();
    last_timestamp_ = imu_msg->header.stamp;

    /* Substract gravity to acceleration */
    quatcinv_[0] = imu_msg -> orientation.w;
    quatcinv_[1] = - imu_msg -> orientation.x;
    quatcinv_[2] = - imu_msg -> orientation.y;
    quatcinv_[3] = - imu_msg -> orientation.z;

    QuaternionProduct(quatcinv_, quat0_, quat_);

    UnitQuaternionRotatePoint(quat_, gravity0_, gravityc_);

    acc_[0] = imu_msg->linear_acceleration.x - gravityc_[0];
    acc_[1] = imu_msg->linear_acceleration.y - gravityc_[1];
    acc_[2] = imu_msg->linear_acceleration.z - gravityc_[2];

    UnitQuaternionRotatePoint(quatcinv_, acc_, accw_);

    /* Filter */
    lpfiltx_->process(accw_[0]);
    lpfilty_->process(accw_[1]);
    lpfiltz_->process(accw_[2]);

    hpfiltx_->process(accw_[0]);
    hpfilty_->process(accw_[1]);
    hpfiltz_->process(accw_[2]);

    vel_[0] += accw_[0] * dt;
    vel_[1] += accw_[1] * dt;
    vel_[2] += accw_[2] * dt;

    pos_[0] += vel_[0] * dt + 0.5* accw_[0] * dt * dt;
    pos_[1] += vel_[1] * dt + 0.5* accw_[1] * dt * dt;
    pos_[2] += vel_[2] * dt + 0.5* accw_[2] * dt * dt;

    /* Publish pose message */

    pose_msg_.pose.position.x = accw_[0];
    pose_msg_.pose.position.y = accw_[1];
    pose_msg_.pose.position.z = accw_[2];

    pose_msg_.pose.orientation = imu_msg -> orientation;
    pose_msg_.header.stamp = ros::Time::now();

    pose_pub_.publish(pose_msg_);
  }

};
