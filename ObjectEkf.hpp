// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
//#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <ds_av_msgs/DataspeedObjectArray.h>

namespace ds_fusion_tracking
{
  // Define variable type 'StateVector' to be a 4x1 Eigen matrix
  typedef Eigen::Vector4d StateVector;
  typedef Eigen::Vector4d StateVector_speed;
  // Define variable type 'StateMatrix' to be a 4x4 Eigen matrix
  typedef Eigen::Matrix4d StateMatrix;
  typedef Eigen::Matrix4d StateMatrix_speed;
  class ObjectEkf {

    public:

      // Constructor arguments are the initial state values and starting time stamp
      ObjectEkf(double x_pos0, double x_vel0,
                double y_pos0, double y_vel0,
                double x_offset, double y_offset,
                const ros::Time& t0, const std::string& frame_id, const std::int32_t& id);

      // Sets the process noise standard deviations
      void setQ(double q_pos, double q_vel);

      // Sets the measurement noise standard deviation
      void setR(double r_pos, double r_vel);

      // Sets the measurement speed noise standard deviation
      void setR_speed(double r_vel);
    

      // Has it been a long time since the last time the filter has been
      // updated with a measurement sample?
      bool isStale();

      ros::Time getrostime();
      // Look up amount of time since filter was created
      double getAge();

      // Update filter without a measurement by running just the prediction step
      void updateFilterPredict(const ros::Time& current_time);

      // Full update of the filter with a cluster measurement
      void updateFilterMeasurement(const ds_av_msgs::DataspeedObject& meas);
      void updateFilterSpeed(const ds_av_msgs::DataspeedObject& meas);
      
      void updateFilterMeasurement_with_ekf(const ds_fusion_tracking::ObjectEkf& meas);

      // Create and return an Odometry output from filter state
      ds_av_msgs::DataspeedObject getEstimate();
    
      bool timetocurrent(const ros::Time& current_time);
      bool timetocurrent_output(const ros::Time& current_time);
      std::int32_t getId();
      //ds_fusion_tracking::Clustering setIdzero();
      void setId(const std::int32_t& set_id);


    private:

      // Estimate state, covariance, and current time stamp
      StateVector Offset_; 
      StateVector X_;
      StateMatrix P_;
      ros::Time estimate_stamp_;

    double b_x_ = 1;
    double b_y_ = 1;
    double b_z_ = 1;

      // Time of when the Kalman filter was created
      ros::Time spawn_stamp_;

      // Time of when the Kalman filter was last updated with a measurement sample
      ros::Time measurement_stamp_;

      // Process noise covariance
      StateMatrix Q_;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        

      // Measurement noise covariance
      Eigen::Matrix4d R_;
      Eigen::Matrix2d R_speed_;
      // Data copied from measurement and not filtered
      geometry_msgs::Vector3 dimensions_;
      geometry_msgs::Vector3 fix_axes;
      double z_;
      std::string frame_id_;
      std::int32_t id_;

      // Methods to predict states and propagate uncertainty 
      StateVector statePrediction(double dt, const StateVector& old_state);
      StateMatrix stateJacobian(double dt, const StateVector& state);
      StateMatrix covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);

       // Methods to predict states and propagate uncertainty 
      StateVector_speed statePrediction_speed(double dt, const StateVector& old_state);
      StateMatrix_speed stateJacobian_speed(double dt, const StateVector& state);
      StateMatrix_speed covPrediction_speed(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);     
  };

}
