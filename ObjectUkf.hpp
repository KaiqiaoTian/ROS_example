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
  typedef Eigen::Matrix4d StateMatrix;



 // typedef Eigen::VectorXd StateVector_UKF;


  class ObjectUkf {

    public:

      // Constructor arguments are the initial state values and starting time stamp
      ObjectUkf(double x_pos0, double x_vel0,
                double y_pos0, double y_vel0,
                double x_offset, double y_offset,
                const ros::Time& t0, const std::string& frame_id, const std::int32_t& id);

      // Sets the process noise standard deviations
      void setQ(double q_pos, double q_vel);

      // Sets the measurement noise standard deviation
      void setR_lidar(double r_pos, double r_vel);

      // Sets the measurement speed noise standard deviation
      void setR_radar(double r_vel);
    

      // Has it been a long time since the last time the filter has been
      // updated with a measurement sample?
      bool isStale();

      ros::Time getrostime();
      // Look up amount of time since filter was created
      double getAge();

      // Update filter without a measurement by running just the prediction step
      void updateFilterPredict(const ros::Time& current_time);

      // Full update of the filter with a cluster measurement
      void updateFilter_Lidar(const ds_av_msgs::DataspeedObject& meas);
      void updateFilter_Radar(const ds_av_msgs::DataspeedObject& meas);
      
 
      // Create and return an Odometry output from filter state
      ds_av_msgs::DataspeedObject getEstimate();
    
      bool timetocurrent(const ros::Time& current_time);
      bool timetocurrent_output(const ros::Time& current_time);
      std::int32_t getId();
      //ds_fusion_tracking::Clustering setIdzero();
      void setId(const std::int32_t& set_id);
      // Methods to predict states and propagate uncertainty 
      void state_cov_Prediction(double dt, const StateVector& old_state);


    private:

         //UKF Parameter

      // predicted sigma points matrix
      Eigen::MatrixXd Xsig_pred_;
     // Weights of sigma points
      Eigen::VectorXd weights_;

      // State dimension
      int n_x_; //4

      // Sigma point spreading parameter
      int lambda_;

      // Radar measurement noise covariance matrix
      Eigen::Matrix2d R_radar_;

      // Lidar measurement noise covariance matrix
      Eigen::Matrix4d R_lidar_;

      ///* the current NIS for radar
      double NIS_radar_;

      ///* the current NIS for laser
      double NIS_laser_;

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

      // Data copied from measurement and not filtered
      geometry_msgs::Vector3 dimensions_;
      geometry_msgs::Vector3 fix_axes;
      double z_;
      std::string frame_id_;
      std::int32_t id_;





  
  };

}
