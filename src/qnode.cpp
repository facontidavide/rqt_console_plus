/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "../include/rqt_console2/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rqt_console2 {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv, LogsTableModel& tablemodel ) :
  init_argc(argc),
  init_argv(argv),
  model(tablemodel)
{}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"rqt_console2");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.

  // Add your ros communications here.
  _node.reset( new ros::NodeHandle );

  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"rqt_console2");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.

  _node.reset( new ros::NodeHandle );

  this->start();
  return true;
}

void QNode::run()
{
  ros::spin();

  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;

  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool QNode::started()
{
  return ros::isStarted();
}

void QNode::subcribeRosout()
{
  subA = _node->subscribe("rosout", 100, &QNode::callbackRosout, this);
}

void QNode::unsubcribeRosout()
{
  subA.shutdown();
}

void QNode::callbackRosout(const rosgraph_msgs::Log::ConstPtr &msg)
{
  model.appendRow( *msg );
}

#ifdef USE_ROSOUT2
void QNode::subcribeRosout2()
{
  subB = _node->subscribe("rosout2", 100, &QNode::callbackRosout2, this);
}

void QNode::unsubcribeRosout2()
{
  subB.shutdown();
}

void QNode::callbackRosout2(const rosout2_msg::LogMsg::ConstPtr &msg)
{
  model.appendRow( *msg );
}
#endif



}  // namespace rqt_console2
