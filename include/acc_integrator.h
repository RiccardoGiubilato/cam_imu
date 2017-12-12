#ifndef ACC_INTEGRATOR
#define ACC_INTEGRATOR

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2_ros/transform_broadcaster.h"

#include "filter.h"

class accInt {
  public:
    accInt(ros::NodeHandle& nh);
    ~accInt() {};

  private:

    // Callbacks
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

    // Ros parameters
    ros::NodeHandle nh_;
    ros::Time last_timestamp_;
    ros::Publisher pose_pub_;
    ros::Subscriber imu_sub_;
    geometry_msgs::PoseStamped pose_msg_;

    // State parameters
    float* quat_;
    float* quatc_;
    float* quatcinv_;
    float* quat0_;
    float* pos_;
    float* vel_;
    float* acc_;
    float* accw_;
    float* gravity0_;
    float* gravityc_;
    bool initialized_;
    double dt;

    acc_filter* lpfiltx_, *lpfilty_, *lpfiltz_;
    acc_filter* hpfiltx_, *hpfilty_, *hpfiltz_;
};

/* Quaternion operations (Ceres notation) */
template <typename T> inline
void UnitQuaternionRotatePoint(const T q[4], const T pt[3], T result[3]) {
  const T t2 =  q[0] * q[1];
  const T t3 =  q[0] * q[2];
  const T t4 =  q[0] * q[3];
  const T t5 = -q[1] * q[1];
  const T t6 =  q[1] * q[2];
  const T t7 =  q[1] * q[3];
  const T t8 = -q[2] * q[2];
  const T t9 =  q[2] * q[3];
  const T t1 = -q[3] * q[3];
  result[0] = T(2) * ((t8 + t1) * pt[0] + (t6 - t4) * pt[1] + (t3 + t7) * pt[2]) + pt[0];
  result[1] = T(2) * ((t4 + t6) * pt[0] + (t5 + t1) * pt[1] + (t9 - t2) * pt[2]) + pt[1];
  result[2] = T(2) * ((t7 - t3) * pt[0] + (t2 + t9) * pt[1] + (t5 + t8) * pt[2]) + pt[2];
}


template <typename T> inline
void QuaternionRotatePoint(const T q[4], const T pt[3], T result[3]) {
  // 'scale' is 1 / norm(q).
  const T scale = T(1) / sqrt(q[0] * q[0] +
                              q[1] * q[1] +
                              q[2] * q[2] +
                              q[3] * q[3]);

  // Make unit-norm version of q.
  const T unit[4] = {
    scale * q[0],
    scale * q[1],
    scale * q[2],
    scale * q[3],
  };

  UnitQuaternionRotatePoint(unit, pt, result);
}

template<typename T> inline
void QuaternionProduct(const T z[4], const T w[4], T zw[4]) {
  zw[0] = z[0] * w[0] - z[1] * w[1] - z[2] * w[2] - z[3] * w[3];
  zw[1] = z[0] * w[1] + z[1] * w[0] + z[2] * w[3] - z[3] * w[2];
  zw[2] = z[0] * w[2] - z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
  zw[3] = z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
}

// xy = x cross y;
template<typename T> inline
void CrossProduct(const T x[3], const T y[3], T x_cross_y[3]) {
  x_cross_y[0] = x[1] * y[2] - x[2] * y[1];
  x_cross_y[1] = x[2] * y[0] - x[0] * y[2];
  x_cross_y[2] = x[0] * y[1] - x[1] * y[0];
}

template<typename T> inline
T DotProduct(const T x[3], const T y[3]) {
  return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);
}

template<typename T>
void pop_front_and_replace(T* buf_, const T& new_) {
  buf_[0] = buf_[1];
  buf_[1] = buf_[2];
  buf_[2] = new_;
}


#endif
