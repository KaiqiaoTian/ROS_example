#include "ObjectEkf.hpp"

namespace ds_fusion_tracking
{

  ObjectEkf::ObjectEkf(double x_pos0, double x_vel0,
                       double y_pos0, double y_vel0,
                       double x_offset, double y_offset,
                       const ros::Time& t0, const std::string& frame_id, const std::int32_t& id)
  {
    // Initialize estimate covariance to identity
    P_.setIdentity();

    // Initialize state estimate to input arguments
    X_ << x_pos0, x_vel0, y_pos0 , y_vel0;
    //Initialize bounding box offset
    Offset_ << x_offset, y_offset, 0.0, 0.0;
    // Initialize all time stamps to the starting time
    estimate_stamp_ = t0;
    measurement_stamp_ = t0;
    spawn_stamp_ = t0;

    // Set dummy values for Q and R. These should be set from the code
    // instantiating the ObjectEkf class using setQ() and setR()
    setQ(1.0, 1.0);
    setR(1.0, 1.0);
    setR_speed(1.0);
    frame_id_ = frame_id;
    id_ = id;
  }

  void ObjectEkf::updateFilterPredict(const ros::Time& current_time)
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
    StateMatrix A = stateJacobian(dt, X_);
    X_ = statePrediction(dt, X_);
    P_ = covPrediction(A, Q_, P_);
    estimate_stamp_ = current_time;
  }



  void ObjectEkf::updateFilterSpeed(const ds_av_msgs::DataspeedObject& meas)
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

    // Prediction step
    StateMatrix A = stateJacobian(dt, X_);
    StateVector predicted_state = statePrediction(dt, X_);
    StateMatrix predicted_cov = covPrediction(A, Q_, P_);

    // Measurement update
    // TODO: Define measurement matrix
    Eigen::Matrix<double, 2, 4> C;
    C.setZero();
    C.row(0) << 0.0, 1.0, 0.0, 0.0;     
    C.row(1) << 0.0, 0.0, 0.0, 1.0;
   

    Eigen::Vector2d meas_vect;
   // meas_vect << meas.pose.pose.position.x, meas.velocity.twist.linear.x, meas.pose.pose.position.y, meas.velocity.twist.linear.y;    

    meas_vect << meas.velocity.twist.linear.x , meas.velocity.twist.linear.y;  

    // TODO: Compute expected measurement based on predicted_state
    Eigen::Vector2d expected_meas;
    expected_meas.setZero();
    expected_meas = C * predicted_state;
    // TODO: Compute residual covariance matrix
    Eigen::Matrix2d S;
    S.setZero();
    S = C * predicted_cov * C.transpose() + R_speed_;//
    // TODO: Comput Kalman gain
    Eigen::Matrix<double, 4, 2> K;
    K.setZero();
    K = predicted_cov * C.transpose() * S.inverse();
    // TODO: Update state estimate
    X_ = predicted_state + K * (meas_vect - expected_meas); 
   //  X_ << meas.pose.pose.position.x, meas.velocity.twist.linear.x, meas.pose.pose.position.y, meas.velocity.twist.linear.y;
    // TODO: Update estimate covariance
    P_ = (StateMatrix::Identity() - K * C) * predicted_cov;

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
/*
  StateVector ObjectEkf::statePrediction_speed(double dt, const StateVector& old_state) {
    // TODO: Propagate the old_state argument through the discrete s tate equations and put the results in new_state
    //       The 'dt' argument of this method is the sample time to use
    StateVector new_state = old_state;
    new_state(0) = old_state(0) + dt * old_state(1);
    new_state(1) = old_state(1);
    new_state(2) = old_state(2) + dt * old_state(3);
    new_state(3) = old_state(3);
    return new_state;
  }

  StateMatrix ObjectEkf::stateJacobian_speed(double dt, const StateVector& state) {
    // TODO: Fill in the elements of the state Jacobian
    //       The 'dt' argument of this method is the sample time to use
    StateMatrix A;
    A.row(0) << 1.0, dt*state(1),  0.0, 0.0;
    A.row(1) << 0.0, 1.0, 0.0, 0.0;
    A.row(2) << 0.0, 0.0, 1.0, dt*state(3);
    A.row(3) << 0.0, 0.0, 0.0, 1.0;
    return A;
  }

  StateMatrix ObjectEkf::covPrediction_speed(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov) {
    // TODO: Propagate the old_cov argument through the covariance prediction equation and put the result in new_cov
    StateMatrix new_cov;
    new_cov.setZero();
    new_cov = A * old_cov * A.transpose() + Q;
    return new_cov;
  }
*/




  void ObjectEkf::updateFilterMeasurement(const ds_av_msgs::DataspeedObject& meas)
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

    // Prediction step
    StateMatrix A = stateJacobian(dt, X_);
    StateVector predicted_state = statePrediction(dt, X_);
    StateMatrix predicted_cov = covPrediction(A, Q_, P_);

    // Measurement update
    // TODO: Define measurement matrix
    Eigen::Matrix<double, 4, 4> C;
    C.setZero();
    C.row(0) << 1.0, 0.0, 0.0, 0.0;
    C.row(1) << 0.0, 1.0, 0.0, 0.0;     
    C.row(2) << 0.0, 0.0, 1.0, 0.0;
    C.row(3) << 0.0, 0.0, 0.0, 1.0;    


    Eigen::Vector4d meas_vect;
    meas_vect << meas.pose.pose.position.x, meas.velocity.twist.linear.x, meas.pose.pose.position.y, meas.velocity.twist.linear.y;    

    // TODO: Compute expected measurement based on predicted_state
    Eigen::Vector4d expected_meas;
    expected_meas.setZero();
    expected_meas = C * predicted_state;
    // TODO: Compute residual covariance matrix
    Eigen::Matrix4d S;
    S.setZero();
    S = C * predicted_cov * C.transpose() + R_;
    // TODO: Comput Kalman gain
    Eigen::Matrix<double, 4, 4> K;
    K.setZero();
    K = predicted_cov * C.transpose() * S.inverse();
    // TODO: Update state estimate
    X_ = predicted_state + K * (meas_vect - expected_meas); 


    // TODO: Update estimate covariance
    P_ = (StateMatrix::Identity() - K * C) * predicted_cov;

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



  StateVector ObjectEkf::statePrediction(double dt, const StateVector& old_state) {
    // TODO: Propagate the old_state argument through the discrete s tate equations and put the results in new_state
    //       The 'dt' argument of this method is the sample time to use
    StateVector new_state = old_state;
    new_state(0) = old_state(0) + dt * old_state(1);
    new_state(1) = old_state(1);
    new_state(2) = old_state(2) + dt * old_state(3);
    new_state(3) = old_state(3);
    return new_state;
  }

  StateMatrix ObjectEkf::stateJacobian(double dt, const StateVector& state) {
    // TODO: Fill in the elements of the state Jacobian
    //       The 'dt' argument of this method is the sample time to use
    StateMatrix A;
    A.row(0) << 1.0, dt,  0.0, 0.0;
    A.row(1) << 0.0, 1.0, 0.0, 0.0;
    A.row(2) << 0.0, 0.0, 1.0, dt;
    A.row(3) << 0.0, 0.0, 0.0, 1.0;
    return A;
  }

  StateMatrix ObjectEkf::covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov) {
    // TODO: Propagate the old_cov argument through the covariance prediction equation and put the result in new_cov
    StateMatrix new_cov;
    new_cov.setZero();
    new_cov = A * old_cov * A.transpose() + Q;
    return new_cov;
  }

  // Has it been a long time since the last time the filter has been
  // updated with a measurement sample?
  bool ObjectEkf::isStale() {
    return (estimate_stamp_ - measurement_stamp_) > ros::Duration(0.5);
  }

  // Look up amount of time since filter was created
  double ObjectEkf::getAge() {
    return (estimate_stamp_ - spawn_stamp_).toSec();
  }

  ros::Time ObjectEkf::getrostime(){
    return (measurement_stamp_);
  }

  bool ObjectEkf::timetocurrent(const ros::Time& current_time){
    return (current_time - estimate_stamp_) > ros::Duration(0.1);
  }

  bool ObjectEkf::timetocurrent_output(const ros::Time& current_time){
    return (current_time - estimate_stamp_) > ros::Duration(0.05);
  }


  void ObjectEkf::setId(const std::int32_t& set_id){

  ObjectEkf::id_ = set_id;
  return;

  }

  std::int32_t ObjectEkf::getId(){
  std::int32_t getid = ObjectEkf::id_;
  return getid;

}



  // Sets the process noise standard deviations
  void ObjectEkf::setQ(double q_pos, double q_vel)
  {
    // TODO: Populate Q_ with q_pos and q_vel
    Q_.setZero();
    Q_.row(0) << q_pos*q_pos, 0.0, 0.0, 0.0;
    Q_.row(1) << 0.0 ,q_vel*q_vel, 0.0, 0.0;
    Q_.row(2) << 0.0, 0.0 ,q_pos*q_pos, 0.0;
    Q_.row(3) << 0.0, 0.0, 0.0, q_vel*q_vel;
  }

  // Sets the measurement noise standard deviation
  void ObjectEkf::setR(double r_pos, double r_vel)
  {
    // TODO: Populate R_ with r_pos
    R_.setIdentity();
    R_.row(0) << r_pos*r_pos, 0.0, 0.0, 0.0;
    R_.row(1) << 0.0, r_vel*r_vel, 0.0, 0.0;
    R_.row(2) << 0.0, 0.0, r_pos*r_pos, 0.0;
    R_.row(3) << 0.0, 0.0, 0.0, r_vel*r_vel;

  }

    // Sets the measurement noise standard deviation
  void ObjectEkf::setR_speed(double r_vel)
  {
    // TODO: Populate R_ with r_pos
    R_speed_.setIdentity();
    R_speed_.row(0) << r_vel*r_vel, 0.0;
    R_speed_.row(1) <<  0.0, r_vel*r_vel;

  }

  // Create and return a DetectedObject output from filter state
    
    ds_av_msgs::DataspeedObject ObjectEkf::getEstimate()
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
