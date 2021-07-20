#include "ObjectUkf.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <armadillo>

namespace ds_fusion_tracking
{

  ObjectUkf::ObjectUkf(double x_pos0, double x_vel0,
                       double y_pos0, double y_vel0,
                       double x_offset, double y_offset,
                       const ros::Time& t0, const std::string& frame_id, const std::int32_t& id)
  {
    // Initialize estimate covariance to identity
    P_.setIdentity();

    // Initialize state estimate
    X_ << x_pos0, x_vel0, y_pos0 , y_vel0;
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF initial step");
    //ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF initial value xp=%f xv=%f yp=%f yv=%f,  ", X_(0), X_(1), X_(2) , X_(3));
    //ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF initial P %f %f %f %f,  ", P_(0,0), P_(0,1), P_(0,2) , P_(0,3));
    //ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF initial P %f %f %f %f,  ", P_(1,0), P_(1,1), P_(1,2) , P_(1,3));
    //ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF initial P %f %f %f %f,  ", P_(2,0), P_(2,1), P_(2,2) , P_(2,3));
    //ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF initial P %f %f %f %f,  ", P_(3,0), P_(3,1), P_(3,2) , P_(3,3));
    //Initialize bounding box offset
    Offset_ << x_offset, y_offset, 0.0, 0.0;
    // Initialize all time stamps to the starting time
    estimate_stamp_ = t0;
    measurement_stamp_ = t0;
    spawn_stamp_ = t0;

    // Set dummy values for Q and R. These should be set from the code
    // instantiating the ObjectUkf class using setQ() and setR()
    setQ(1.0, 1.0);
    setR_lidar(1.0, 1.0);
    setR_radar(1.0);

    frame_id_ = frame_id;
    id_ = id;

    n_x_ = 4;
    lambda_ = 3 - n_x_;


    // Initialize weights_
   weights_ = Eigen::VectorXd(2*n_x_ + 1);
   weights_.fill(0.5/(lambda_ + n_x_));
  // weights_(0) = lambda_/(lambda_ + n_x_);
   weights_(0) = -0.33333;

   // sigma point prediction
   Xsig_pred_ = Eigen::MatrixXd(n_x_, 2*n_x_+1);
   Xsig_pred_.fill(0);

  }

  void ObjectUkf::updateFilterPredict(const ros::Time& current_time)
  {
    // Calculate time difference between current time and filter state
    double dt = (current_time - estimate_stamp_).toSec();
    if (fabs(dt) > 2) {
      // Large time jump detected... just reset to the current time
      spawn_stamp_ = current_time;
      estimate_stamp_ = current_time;
      measurement_stamp_ = current_time;
      return;
    }
    // Propagate estimate prediction and update estimate with result
    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~updateFilterPredict~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~`");
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF before value p=%f xv=%f yp=%f yv=%f,  ", X_(0), X_(1), X_(2) , X_(3));
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF before value P(0,0)=%f,  ", P_(0,0));
    state_cov_Prediction(dt, X_);
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF update step");
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF update value xp=%f xv=%f yp=%f yv=%f,  ", X_(0), X_(1), X_(2) , X_(3));
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF update value P(0,0)=%f,  ", P_(0,0));

    estimate_stamp_ = current_time;
  }



  void ObjectUkf::updateFilter_Radar(const ds_av_msgs::DataspeedObject& meas)
  { 
  
// Calculate time difference between measurement and filter state
    double dt = (meas.header.stamp - estimate_stamp_).toSec();

    if (fabs(dt) > 2) {
      // Large time jump detected... reset filter to this measurement
      X_ << meas.pose.pose.position.x, meas.velocity.twist.linear.x, meas.pose.pose.position.y, meas.velocity.twist.linear.y;
      P_.setIdentity();
      spawn_stamp_ = meas.header.stamp;
      estimate_stamp_ = meas.header.stamp;
      measurement_stamp_ = meas.header.stamp;
      return;
    }

     Eigen::VectorXd R_data= Eigen::VectorXd(2);
     R_data << meas.velocity.twist.linear.x, meas.velocity.twist.linear.y;

    // Prediction step

    state_cov_Prediction(dt, X_);

//extract measurement
    int n_z_ = 2; //radar data 2 dimention
    Eigen::MatrixXd Zsig =  Eigen::MatrixXd(n_z_, 2*n_x_+1);   //(2,9)
   for(int i = 0; i < 2*n_x_ + 1; i++){
     Zsig(0, i) = Xsig_pred_(2, i);
     Zsig(1, i) = Xsig_pred_(3, i);
   }

   //Predicted mean measurement
   Eigen::VectorXd z_pred_= Eigen::VectorXd(n_z_);
   z_pred_.fill(0.0);
   for(int i = 0; i < 2*n_x_+1; i++){
     z_pred_ = z_pred_ + weights_(i) * Zsig.col(i);
   }

   // calculate covariance of predicted measurement
   Eigen::MatrixXd S = Eigen::MatrixXd(n_z_, n_z_); //(2,2)
   S.fill(0.0);
   for(int i = 0; i < 2*n_x_ + 1; i++){

     Eigen::VectorXd z_diff = Zsig.col(i) - z_pred_;
     S = S + weights_(i) * z_diff * z_diff.transpose();
   }

   // adding noise 
   S = S + R_radar_;

// UKF update
   // Cross correlation matix
   Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z_);
   Tc.fill(0.0);
   for(int i = 0; i < 2*n_x_+1; i++){

     Eigen::VectorXd x_diff = Xsig_pred_.col(i) - X_;
     Eigen::VectorXd z_diff = Zsig.col(i) - z_pred_;
     Tc = Tc + weights_(i) * x_diff * z_diff.transpose();

   }

   // calculate Kalman gain K
   Eigen::MatrixXd K = Tc * S.inverse();

   // update state mean and covariance
   // residual
   Eigen::VectorXd z_diff = R_data - z_pred_;

   X_ = X_ + K*z_diff;

   P_ = P_ - K*S*K.transpose();

    // Set estimate time stamp and latest measurement time stamp to the stamp in the input argument
    estimate_stamp_ = meas.header.stamp;
    measurement_stamp_ = meas.header.stamp;
    
    if (b_x_ < meas.bounding_box_axes.x)
    {
        b_x_ = meas.bounding_box_axes.x;
    }
    if (b_y_ < meas.bounding_box_axes.y)
    {
        b_y_ = meas.bounding_box_axes.y;
    }
    if (b_z_ < meas.bounding_box_axes.z)
    {
        b_z_ = meas.bounding_box_axes.z;
    }


    //dimensions_ =  meas.bounding_box_axes;//  .dimensions;

    dimensions_.x = b_x_;
    dimensions_.y = b_y_;
    dimensions_.z = b_z_;
    
    // Copy information that is not filtered
   // dimensions_ =  meas.bounding_box_axes;//  .dimensions;

    //dimensions_.x = 5.00;
    //dimensions_.y = 3.00;
    //dimensions_.z = 2.00;

    z_ = 0.0;// meas.pose.pose.position.z;
  }





  void ObjectUkf::updateFilter_Lidar(const ds_av_msgs::DataspeedObject& meas)
  { 
  
    // Calculate time difference between measurement and filter state
    double dt = (meas.header.stamp - estimate_stamp_).toSec();

    if (fabs(dt) > 2) {
      // Large time jump detected... reset filter to this measurement
      X_ << meas.pose.pose.position.x, meas.velocity.twist.linear.x, meas.pose.pose.position.y, meas.velocity.twist.linear.y;
      P_.setIdentity();
      spawn_stamp_ = meas.header.stamp;
      estimate_stamp_ = meas.header.stamp;
      measurement_stamp_ = meas.header.stamp;
      return;
    }

     Eigen::VectorXd L_data = Eigen::VectorXd(4);
     L_data << meas.pose.pose.position.x, meas.velocity.twist.linear.x, meas.pose.pose.position.y, meas.velocity.twist.linear.y;

    // Prediction step
   //  ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ duing time time=%f",dt);
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF lidar update before");
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ before xp=%f xv=%f yp=%f yv=%f,  ", X_(0), X_(1), X_(2) , X_(3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(0,0), P_(0,1), P_(0,2) , P_(0,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(1,0), P_(1,1), P_(1,2) , P_(1,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(2,0), P_(2,1), P_(2,2) , P_(2,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(3,0), P_(3,1), P_(3,2) , P_(3,3));
    state_cov_Prediction(dt, X_);

    ROS_INFO("---------------------------------------------------------------------------------------");
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ after xp=%f xv=%f yp=%f yv=%f,  ", X_(0), X_(1), X_(2) , X_(3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(0,0), P_(0,1), P_(0,2) , P_(0,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(1,0), P_(1,1), P_(1,2) , P_(1,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(2,0), P_(2,1), P_(2,2) , P_(2,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(3,0), P_(3,1), P_(3,2) , P_(3,3));

//extract measurement
   int n_z_ = 4;
    Eigen::MatrixXd Zsig =  Eigen::MatrixXd(n_z_, 2*n_x_+1);   //(4,9)
   for(int i = 0; i < 2*n_x_ + 1; i++){
     Zsig(0, i) = Xsig_pred_(0, i);
     Zsig(1, i) = Xsig_pred_(1, i);
     Zsig(2, i) = Xsig_pred_(2, i);
     Zsig(3, i) = Xsig_pred_(3, i);
   }

   //Predicted mean measurement
   Eigen::VectorXd z_pred_= Eigen::VectorXd(n_z_);
   z_pred_.fill(0.0);
   for(int i = 0; i < 2*n_x_+1; i++){
     z_pred_ = z_pred_ + weights_(i) * Zsig.col(i);
   }

   // calculate covariance of predicted measurement
   Eigen::MatrixXd S = Eigen::MatrixXd(n_z_, n_z_);
   S.fill(0.0);
   for(int i = 0; i < 2*n_x_ + 1; i++){
   Eigen::VectorXd z_diff = Zsig.col(i) - z_pred_;
   S = S + weights_(i) * z_diff * z_diff.transpose();

   }
   // adding noise 
   S = S + R_lidar_;

// UKF update
   // Cross correlation matix
   Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z_);
   Tc.fill(0.0);
   for(int i = 0; i < 2*n_x_+1; i++){
     Eigen::VectorXd x_diff = Xsig_pred_.col(i) - X_;

     Eigen::VectorXd z_diff = Zsig.col(i) - z_pred_;

     Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
   }

   // calculate Kalman gain K
   Eigen::MatrixXd K = Tc * S.inverse();

   // update state mean and covariance
   // residual
   Eigen::VectorXd z_diff = L_data - z_pred_;

   X_ = X_ + K*z_diff;

   P_ = P_ - K*S*K.transpose();
 



    // Set estimate time stamp and latest measurement time stamp to the stamp in the input argument
    estimate_stamp_ = meas.header.stamp;
    measurement_stamp_ = meas.header.stamp;
    
    // Copy information that is not filtered

  //  double b_x = 1;
  //  double b_y = 1;
  //  double b_z = 1;

    if (b_x_ < meas.bounding_box_axes.x)
    {
        b_x_ = meas.bounding_box_axes.x;
    }
    if (b_y_ < meas.bounding_box_axes.y)
    {
        b_y_ = meas.bounding_box_axes.y;
    }
    if (b_z_ < meas.bounding_box_axes.z)
    {
        b_z_ = meas.bounding_box_axes.z;
    }


    //dimensions_ =  meas.bounding_box_axes;//  .dimensions;

    dimensions_.x = b_x_;
    dimensions_.y = b_y_;
    dimensions_.z = b_z_;

    z_ = 0.0;//= meas.pose.pose.position.z;
  }



  void ObjectUkf::state_cov_Prediction(double dt, const StateVector& old_state) {
    // TODO: Propagate the old_state argument through the discrete s tate equations and put the results in new_state
    //       The 'dt' argument of this method is the sample time to use


     // Generate X_ sigma points

   Eigen::MatrixXd Xsig = Eigen::MatrixXd(n_x_, 2*n_x_ + 1); //(4, 9)

     // create square root matrix
    // Eigen::MatrixXd L = Eigen::MatrixXd(n_x_, n_x_);
     Eigen::MatrixXd L = P_.llt().matrixL();

   //Eigen::MatrixXd L = P_.llt().matrixL();

//L.fill(0.05);
//L.setIdentity();
//L = arma::sqrtmat(P_);
/*
arma::mat A;

for(int i = 0; i < n_x_; i++)
{
  for(int j = 0; j < n_x_; i++)
  {
    A(i,j) = P_(i,j); 
  }
}

arma::cx_mat B = arma::sqrtmat(A);

for(int i = 0; i < n_x_; i++)
{
  for(int j = 0; j < n_x_; i++)
  {
    double x = std::arg(B(i,j));
    L(i,j) = x; 
  }
}

*/
   // create X_ sigma points
/*

    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(0,0), P_(0,1), P_(0,2) , P_(0,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(1,0), P_(1,1), P_(1,2) , P_(1,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(2,0), P_(2,1), P_(2,2) , P_(2,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  P %f %f %f %f,  ", P_(3,0), P_(3,1), P_(3,2) , P_(3,3));

    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  L %f %f %f %f,  ", L(0,0), L(0,1), L(0,2) , L(0,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  L %f %f %f %f,  ", L(1,0), L(1,1), L(1,2) , L(1,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  L %f %f %f %f,  ", L(2,0), L(2,1), L(2,2) , L(2,3));
    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF  L %f %f %f %f,  ", L(3,0), L(3,1), L(3,2) , L(3,3));
*/
   Xsig.col(0) = X_;
   for(int i = 0; i < n_x_; i++)
   {
     Xsig.col(i + 1) = X_ + sqrt(lambda_ + n_x_) * L.col(i);
     Xsig.col(i + 1 + n_x_) = X_ - sqrt(lambda_ + n_x_) * L.col(i);

   }
/*
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf xsig 1 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig(0,0), Xsig(1,0), Xsig(2,0) , Xsig(3,0));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf xsig 2 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig(0,1), Xsig(1,1), Xsig(2,1) , Xsig(3,1));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf xsig 3 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig(0,2), Xsig(1,2), Xsig(2,2) , Xsig(3,2));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf xsig 4 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig(0,3), Xsig(1,3), Xsig(2,3) , Xsig(3,3));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf xsig 5 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig(0,4), Xsig(1,4), Xsig(2,4) , Xsig(3,4));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf xsig 2+4 point xp=%f xv=%f yp=%f yv=%f,", Xsig(0,5), Xsig(1,5), Xsig(2,5) , Xsig(3,5));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf xsig 3+4 point xp=%f xv=%f yp=%f yv=%f,", Xsig(0,6), Xsig(1,6), Xsig(2,6) , Xsig(3,6));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf xsig 4+4 point xp=%f xv=%f yp=%f yv=%f,", Xsig(0,7), Xsig(1,7), Xsig(2,7) , Xsig(3,7));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf xsig 5+4 point xp=%f xv=%f yp=%f yv=%f,", Xsig(0,8), Xsig(1,8), Xsig(2,8) , Xsig(3,8));
*/



// x_ sigma points predict function
  for (int i = 0; i < 2*n_x_ + 1; i++)
  {
      Xsig_pred_(0, i) = Xsig(0, i) + dt*Xsig(1, i);
      Xsig_pred_(1, i) = Xsig(1, i);
      Xsig_pred_(2, i) = Xsig(2, i) + dt*Xsig(3, i);
      Xsig_pred_(3, i) = Xsig(3, i);
  }
/*
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf Xsig_pred_ 1 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig_pred_(0,0), Xsig_pred_(1,0), Xsig_pred_(2,0) , Xsig_pred_(3,0));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf Xsig_pred_ 2 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig_pred_(0,1), Xsig_pred_(1,1), Xsig_pred_(2,1) , Xsig_pred_(3,1));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf Xsig_pred_ 3 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig_pred_(0,2), Xsig_pred_(1,2), Xsig_pred_(2,2) , Xsig_pred_(3,2));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf Xsig_pred_ 4 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig_pred_(0,3), Xsig_pred_(1,3), Xsig_pred_(2,3) , Xsig_pred_(3,3));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf Xsig_pred_ 5 point xp=%f xv=%f yp=%f yv=%f,  ", Xsig_pred_(0,4), Xsig_pred_(1,4), Xsig_pred_(2,4) , Xsig_pred_(3,4));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf Xsig_pred_ 2+4 point xp=%f xv=%f yp=%f yv=%f,", Xsig_pred_(0,5), Xsig_pred_(1,5), Xsig_pred_(2,5) , Xsig_pred_(3,5));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf Xsig_pred_ 3+4 point xp=%f xv=%f yp=%f yv=%f,", Xsig_pred_(0,6), Xsig_pred_(1,6), Xsig_pred_(2,6) , Xsig_pred_(3,6));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf Xsig_pred_ 4+4 point xp=%f xv=%f yp=%f yv=%f,", Xsig_pred_(0,7), Xsig_pred_(1,7), Xsig_pred_(2,7) , Xsig_pred_(3,7));
ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf Xsig_pred_ 5+4 point xp=%f xv=%f yp=%f yv=%f,", Xsig_pred_(0,8), Xsig_pred_(1,8), Xsig_pred_(2,8) , Xsig_pred_(3,8));
*/




     // Predict state mean
     X_.fill(0.0);
   for(int i = 0; i < 2*n_x_ + 1; i++){
     X_ = X_ + weights_(i)*Xsig_pred_.col(i);
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ print wight =%f ", weights_(i));
   }

 //   ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ ukf state mean xp=%f xv=%f yp=%f yv=%f,  ", X_(0), X_(1), X_(2) , X_(3));

// Predict state covairance
     P_.fill(0.0);
   for(int i = 0; i < 2*n_x_ + 1; i++){
     Eigen::VectorXd X_diff = Xsig_pred_.col(i) - X_;
     P_ = P_ + weights_(i)*X_diff*X_diff.transpose();
   }
  //  ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF update P %f %f %f %f,  ", P_(0,0), P_(0,1), P_(0,2) , P_(0,3));
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF update P %f %f %f %f,  ", P_(1,0), P_(1,1), P_(1,2) , P_(1,3));
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF update P %f %f %f %f,  ", P_(2,0), P_(2,1), P_(2,2) , P_(2,3));
   // ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$ UKF update P %f %f %f %f,  ", P_(3,0), P_(3,1), P_(3,2) , P_(3,3));

  }




  // Has it been a long time since the last time the filter has been
  // updated with a measurement sample?
  bool ObjectUkf::isStale() {
    return (estimate_stamp_ - measurement_stamp_) > ros::Duration(0.5);
  }

  // Look up amount of time since filter was created
  double ObjectUkf::getAge() {
    return (estimate_stamp_ - spawn_stamp_).toSec();
  }

  ros::Time ObjectUkf::getrostime(){
    return (measurement_stamp_);
  }

  bool ObjectUkf::timetocurrent(const ros::Time& current_time){
    return (current_time - estimate_stamp_) > ros::Duration(0.1);
  }

  bool ObjectUkf::timetocurrent_output(const ros::Time& current_time){
    return (current_time - estimate_stamp_) > ros::Duration(0.05);
  }


  void ObjectUkf::setId(const std::int32_t& set_id){

  ObjectUkf::id_ = set_id;
  return;

  }

  std::int32_t ObjectUkf::getId(){
  std::int32_t getid = ObjectUkf::id_;
  return getid;

}



  // Sets the process noise standard deviations
  void ObjectUkf::setQ(double q_pos, double q_vel)
  {
    // TODO: Populate Q_ with q_pos and q_vel
    Q_.setZero();
    Q_.row(0) << q_pos*q_pos, 0.0, 0.0, 0.0;
    Q_.row(1) << 0.0 ,q_vel*q_vel, 0.0, 0.0;
    Q_.row(2) << 0.0, 0.0 ,q_pos*q_pos, 0.0;
    Q_.row(3) << 0.0, 0.0, 0.0, q_vel*q_vel;
  }

  // Sets the measurement noise standard deviation
  void ObjectUkf::setR_lidar(double r_pos, double r_vel)
  {
    // TODO: Populate R_ with r_pos
    R_lidar_.setIdentity();
    R_lidar_.row(0) << r_pos*r_pos, 0.0, 0.0, 0.0;
    R_lidar_.row(1) << 0.0, r_vel*r_vel, 0.0, 0.0;
    R_lidar_.row(2) << 0.0, 0.0, r_pos*r_pos, 0.0;
    R_lidar_.row(3) << 0.0, 0.0, 0.0, r_vel*r_vel;

  }

    // Sets the measurement noise standard deviation
  void ObjectUkf::setR_radar(double r_vel)
  {
    // TODO: Populate R_ with r_pos
    R_radar_.setIdentity();
    R_radar_.row(0) << r_vel*r_vel, 0.0;
    R_radar_.row(1) <<  0.0, r_vel*r_vel;

  }

  // Create and return a DetectedObject output from filter state
    
    ds_av_msgs::DataspeedObject ObjectUkf::getEstimate()
    {
    ds_av_msgs::DataspeedObject estimate_output;
    estimate_output.header.stamp = estimate_stamp_;
    estimate_output.header.frame_id = frame_id_;
    estimate_output.id = id_;
    estimate_output.pose.pose.position.z = z_;
    estimate_output.pose.pose.orientation.w = 1.0;

    //fix_axes.x = 4.0;
    //fix_axes.y = 2.0;
    //fix_axes.z = 2.0;
    estimate_output.bounding_box_axes = dimensions_; //fix_axes; //; 
   // estimate_output.bounding_box_offset.x = 0.5*dimensions_.x;
    
    // TODO: Populate output x and y position with filter estimate
    estimate_output.pose.pose.position.x = X_(0); //+ X_(3);//+ Offset_(0); //+ X_(3);   // add boundingbox offset
    estimate_output.pose.pose.position.y = X_(2); //+ X_(4);//+ Offset_(1); //+ X_(4);

    // TODO: Populate output x and y velocity with filter estimate
    estimate_output.velocity.twist.linear.x = X_(1);
    estimate_output.velocity.twist.linear.y = X_(3);
    
    return estimate_output;
  }

}
