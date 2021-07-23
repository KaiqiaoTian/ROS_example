#include "Particle_Filter.hpp"
#include <random>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>

namespace av_fusion_tracking
{

  Particle_Filter::Particle_Filter(double x_pos0, double x_vel0,
                                   double y_pos0, double y_vel0,
                                   double x_offset, double y_offset,
                                   const ros::Time &t0, const std::string &frame_id, const std::int32_t &id)
  {

    num_particles = 100;

    std::default_random_engine rand_eng;
    // Creating normal distributions

    std::normal_distribution<double> pos_x(0, stdv[0]);
    std::normal_distribution<double> pos_y(0, stdv[1]);
    std::normal_distribution<double> speed_x(0, stdv[2]);
    std::normal_distribution<double> speed_y(0, stdv[3]);

    for (int i = 0; i < num_particles; ++i)
    {

      Particle_.id_single = i;
      Particle_.x_pos0 = x_pos0; //init 
      Particle_.y_pos0 = y_pos0; //init

      Particle_.x_vel0 = x_vel0; //init 
      Particle_.y_vel0 = y_vel0; //init 


      Particle_.x_pos0 += pos_x(rand_eng); //adding noise
      Particle_.y_pos0 += pos_y(rand_eng); //adding noise
      Particle_.x_vel0 += speed_x(rand_eng); //adding noise
      Particle_.y_vel0 += speed_y(rand_eng); //adding noise


      Particle_.weight = 1.0;

      Particles.push_back(Particle_);
    }

    //Initialize bounding box offset
    Offset_ << x_offset, y_offset, 0.0, 0.0;
    // Initialize all time stamps to the starting time
    estimate_stamp_ = t0;
    measurement_stamp_ = t0;
    spawn_stamp_ = t0;

    frame_id_ = frame_id;
    id_ = id;
  }

  void Particle_Filter::updateFilterPredict(const ros::Time &current_time)
  {

    // Calculate time difference between current time and filter state
    double dt = (current_time - estimate_stamp_).toSec();
    if (fabs(dt) > 2)
    {
      // Large time jump detected... just reset to the current time
      spawn_stamp_ = current_time;
      estimate_stamp_ = current_time;
      measurement_stamp_ = current_time;
      return;
    }

    // Particle predict

    std::default_random_engine rand_eng;
    // Creating normal distributions

    std::normal_distribution<double> pos_x(0, stdv[0]);
    std::normal_distribution<double> pos_y(0, stdv[1]);
    std::normal_distribution<double> speed_x(0, stdv[2]);
    std::normal_distribution<double> speed_y(0, stdv[3]);

    for (int i = 0; i < num_particles; ++i)
    {
      Particles[i].x_vel0 += speed_x(rand_eng); //update and adding noise
      Particles[i].y_vel0 += speed_y(rand_eng); //update and adding noise

      Particles[i].x_pos0 += Particles[i].x_vel0 * dt + pos_x(rand_eng); //update and adding noise
      Particles[i].y_pos0 += Particles[i].y_vel0 * dt + pos_y(rand_eng); //update and adding noise

    }

    estimate_stamp_ = current_time;

  }

  void Particle_Filter::updateFilterWeights(const ds_av_msgs::DataspeedObject &meas)
  {

    // Update the weights of each particle using a multi-variate Gaussian distribution

    // Calculate time difference between measurement and filter state
    double dt = (meas.header.stamp - estimate_stamp_).toSec();

    if (fabs(dt) > 2)
    {
      // Large time jump detected... reset filter to this measurement
      // X_ << meas.pose.pose.position.x, meas.velocity.twist.linear.x, meas.pose.pose.position.y, meas.velocity.twist.linear.y;
      // P_.setIdentity();
      spawn_stamp_ = meas.header.stamp;
      estimate_stamp_ = meas.header.stamp;
      measurement_stamp_ = meas.header.stamp;
      return;
    }
    

    for (int i = 0; i < num_particles; ++i)
    {
      Particles[i].x_pos0 += Particles[i].x_vel0 * dt; //update and adding noise
      Particles[i].y_pos0 += Particles[i].y_vel0 * dt; //update and adding noise

    }



    std::default_random_engine rand_eng;
    // Creating normal distributions

    std::normal_distribution<double> pos_x(0, stdv[0]);
    std::normal_distribution<double> pos_y(0, stdv[1]);
    std::normal_distribution<double> speed_x(0, stdv[2]);
    std::normal_distribution<double> speed_y(0, stdv[3]);

    double sig_x = stdv_meas[0];
    double sig_y = stdv_meas[1];

    for (int i = 0; i < num_particles; ++i)
    {
      Particles[i].weight = 1.0;  // set to 1 for multiplication in the end of the loop

      Particles[i].x_vel0 = meas.velocity.twist.linear.x + speed_x(rand_eng);   //update and adding noise
      Particles[i].y_vel0 = meas.velocity.twist.linear.y + speed_y(rand_eng);   //update and adding noise

      // Calculating differ between predict and meas.
      double dX = (Particles[i].x_pos0 - meas.pose.pose.position.x);
      double dY = (Particles[i].y_pos0 - meas.pose.pose.position.y);

      // Calculating weight using multivariate Gaussian distribution
      double weight = (1 / (2 * M_PI * sig_x * sig_y)) * exp(-(dX * dX / (2 * sig_x * sig_x) + (dY * dY / (2 * sig_y * sig_y))));

      if (weight == 0.0)
      {
        Particles[i].weight = 0.000001;
      }
      else
      {
        Particles[i].weight *= weight;
      }
    }

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


    resample();
    estimate_stamp_ = meas.header.stamp;
    measurement_stamp_ = meas.header.stamp;

  }


  void Particle_Filter::resample(){

  std::default_random_engine gen;
  std::discrete_distribution<size_t> dist_index(weights.begin(), weights.end());

  std::vector<Particle> resampled_particles(Particles.size());

  for (auto i = 0; i < Particles.size(); i++) {
    resampled_particles[i] = Particles[dist_index(gen)];
    // there is no need to clean up resampled particle weight;
    // weight for each particle will be recalculated in the next iteration
  }

  Particles = resampled_particles;

   }



  // Has it been a long time since the last time the filter has been
  // updated with a measurement sample?
  bool Particle_Filter::isStale()
  {
    return (estimate_stamp_ - measurement_stamp_) > ros::Duration(1);
  }

  // Look up amount of time since filter was created
  double Particle_Filter::getAge()
  {
    return (estimate_stamp_ - spawn_stamp_).toSec();
  }

  ros::Time Particle_Filter::getrostime()
  {
    return (measurement_stamp_);
  }

  bool Particle_Filter::timetocurrent(const ros::Time &current_time)
  {
    return (current_time - estimate_stamp_) > ros::Duration(0.1);
  }

  bool Particle_Filter::timetocurrent_output(const ros::Time &current_time)
  {
    return (current_time - estimate_stamp_) > ros::Duration(0.05);
  }

  void Particle_Filter::setId(const std::int32_t &set_id)
  {

    Particle_Filter::id_ = set_id;
    return;
  }

  std::int32_t Particle_Filter::getId()
  {
    std::int32_t getid = Particle_Filter::id_;
    return getid;
  }


  // Create and return a DetectedObject output from filter state

  ds_av_msgs::DataspeedObject Particle_Filter::getEstimate()
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


  int sample = 10;

  double sum_pos_x = 0;
  double sum_pos_y = 0;
  double sum_vel_x = 0;
  double sum_vel_y= 0;


  double estimate_pos_x = 0;
  double estimate_pos_y = 0;
  double estimate_vel_x = 0;
  double estimate_vel_y = 0;

  std::default_random_engine gen;
  std::discrete_distribution<size_t> dist_index(weights.begin(), weights.end());
  
  std::vector<Particle> select_particles(sample);    ////select 10 particles

  for (int i = 0; i < sample; i++) {
    select_particles[i] = Particles[dist_index(gen)];
    // there is no need to clean up resampled particle weight;
    // weight for each particle will be recalculated in the next iteration
  }
/*
  for (int i = 0; i < sample; i++){
    sum_pos_x += select_particles[i].x_pos0;
    sum_pos_y += select_particles[i].y_pos0;

    sum_vel_x += select_particles[i].x_vel0;
    sum_vel_y += select_particles[i].y_vel0;
  }
*/
  for (int i = 0; i < sample; i++){
    sum_pos_x += Particles[i].x_pos0;
    sum_pos_y += Particles[i].y_pos0;

    sum_vel_x += Particles[i].x_vel0;
    sum_vel_y += Particles[i].y_vel0;
  }


   estimate_pos_x = sum_pos_x/sample;
   estimate_pos_y = sum_pos_y/sample;
   estimate_vel_x = sum_vel_x/sample;
   estimate_vel_y = sum_vel_y/sample;


    estimate_output.bounding_box_axes = dimensions_; //fix_axes; //;
                                                     // estimate_output.bounding_box_offset.x = 0.5*dimensions_.x;

    // TODO: Populate output x and y position with filter estimate
    estimate_output.pose.pose.position.x = estimate_pos_x; 
    estimate_output.pose.pose.position.y = estimate_pos_y; 

    // TODO: Populate output x and y velocity with filter estimate
    estimate_output.velocity.twist.linear.x = estimate_vel_x;
    estimate_output.velocity.twist.linear.y = estimate_vel_y;

    return estimate_output;
  }

}

