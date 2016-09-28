/**
 * @file /include/rqt_console2/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rqt_console2_QNODE_HPP_
#define rqt_console2_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QObject>
#include <QStringListModel>
#include "logs_table_model.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rqt_console2 {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QObject
{
  Q_OBJECT
public:
  QNode(int argc, char** argv , LogsTableModel &tablemodel, QObject *parent = 0);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);

  bool started();

  void subcribeRosout();
  void unsubcribeRosout();

#ifdef USE_ROSOUT2
  void subcribeRosout2();
  void unsubcribeRosout2();
#endif

public slots:
  void spin();

Q_SIGNALS:

  void rosShutdown();

private:
  int init_argc;
  char** init_argv;

  LogsTableModel& model;

  ros::Subscriber subA;
  ros::Subscriber subB;
  std::unique_ptr<ros::NodeHandle> _node;


  void callbackRosout(const rosgraph_msgs::Log::ConstPtr& msg);

#ifdef USE_ROSOUT2
  void callbackRosout2(const rosout2_msg::LogMsg::ConstPtr& msg);
#endif
};

}  // namespace rqt_console2

#endif /* rqt_console2_QNODE_HPP_ */
