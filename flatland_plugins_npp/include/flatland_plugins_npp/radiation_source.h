#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <thirdparty/ThreadPool.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <random>
#include <thread>

#ifndef FLATLAND_PLUGINS_NPP_RADIATIONSOURCE_H
#define FLATLAND_PLUGINS_NPP_RADIATIONSOURCE_H

using namespace flatland_server;

namespace flatland_plugins_npp {

/**
 * This class implements the model plugin class and provides laser data
 * for the given configurations
 */
class RadiationSource: public ModelPlugin {
 public:
  std::string topic_;     ///< topic name to publish the laser scan
  Body *body_;            ///<  body the laser frame attaches to
  Pose origin_;           ///< laser frame w.r.t the body
  double rad_val_;        ///< value of source
  //double range_;          ///< laser max range
  double noise_std_dev_;  ///< noise std deviation
  //double max_angle_;      /// < laser max angle
  //double min_angle_;      ///< laser min angle
  //double increment_;      ///< laser angle increment
  double update_rate_;    ///< the rate laser scan will be published
  std::string frame_id_;  ///< laser frame id name
  bool broadcast_tf_;     ///< whether to broadcast laser origin w.r.t body
  uint16_t layers_bits_;  ///< for setting the layers where laser will function
  //ThreadPool pool_;       ///< ThreadPool for managing concurrent scan threads

  /*
   * for setting reflectance layers. if the laser hits those layers,
   * intensity will be high (255)
   */
  //uint16_t reflectance_layers_bits_;

  std::default_random_engine rng_;              ///< random generator
  std::normal_distribution<double> noise_gen_;  ///< gaussian noise generator

  Eigen::Matrix3f m_body_to_source_;       ///< tf from body to laser
  Eigen::Matrix3f m_world_to_body_;       ///< tf  from world to body
  Eigen::Matrix3f m_world_to_source_;      ///< tf from world to laser
  Eigen::MatrixXf m_source_points_;        ///< laser points in the laser' frame
  Eigen::MatrixXf m_world_source_points_;  /// laser point in the world frame
  Eigen::Vector3f v_zero_point_;          ///< point representing (0,0)
  Eigen::Vector3f v_world_source_origin_;  ///< (0,0) in the laser frame
  //sensor_msgs::LaserScan laser_scan_;     ///< for publishing laser scan
  std_msgs::Float32 rad_msg_;

  ros::Publisher source_publisher_;             ///< ros laser topic publisher
  tf::TransformBroadcaster tf_broadcaster_;   ///< broadcast laser frame
  geometry_msgs::TransformStamped source_tf_;  ///< tf from body to laser frame
  flatland_plugins::UpdateTimer update_timer_;                  ///< for controlling update rate

  /**
   * @brief Constructor to start the threadpool with N+1 threads
   */
  //Laser() : pool_(std::thread::hardware_concurrency() + 1) {
    //ROS_INFO_STREAM("Laser plugin loaded with "
                    //<< (std::thread::hardware_concurrency() + 1) << " threads");
  //};

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief Method that contains all of the laser range calculations
   */
  //void ComputeLaserRanges();

  /**
   * @brief helper function to extract the paramters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);
};

/**
 * This class handles the b2RayCastCallback ReportFixture method
 * allowing each thread to access its own callback object
 */
//class LaserCallback : public b2RayCastCallback {
 //public:
  //bool did_hit_ = false;  ///< Box2D ray trace checking if ray hits anything
  //float fraction_ = 0;    ///< Box2D ray trace fraction
  //float intensity_ = 0;   ///< Intensity of raytrace collision
  //Laser *parent_;         ///< The parent Laser plugin

  //*
   //Default constructor to assign parent object
  //LaserCallback(Laser *parent) : parent_(parent){};

  //*
   //@brief Box2D raytrace call back method required for implementing the
   //b2RayCastCallback abstract class
   //@param[in] fixture Fixture the ray hits
   //@param[in] point Point the ray hits the fixture
   //@param[in] normal Vector indicating the normal at the point hit
   //@param[in] fraction Fraction of ray length at hit point
  //float ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                      //const b2Vec2 &normal, float fraction) override;
//};
};

#endif
