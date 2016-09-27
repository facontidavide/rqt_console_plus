#include "../include/rqt_console2/logs_table_model.hpp"
#include <QDateTime>
#include <QBrush>
#include <QColor>

LogsTableModel::LogsTableModel(QObject *parent)
  : QAbstractTableModel(parent)
{
}

QVariant LogsTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{

  if (role != Qt::DisplayRole)
    return QVariant();

  if (orientation == Qt::Horizontal){
    switch(section)
    {
    case 0: return "Time"; break;
    case 1: return "Severity";break;
    case 2: return "Node";break;
    case 3: return "Message";break;
    case 4: return "Source";break;
    }
  }
  else{
    return QString("%1").arg(section);
  }

  return QVariant();
}

int LogsTableModel::rowCount(const QModelIndex &parent) const
{
  if (parent.isValid())
    return 0;


  return logs.size();
}

int LogsTableModel::columnCount(const QModelIndex &parent) const
{
  if (parent.isValid())
    return 0;

  return 5;
}

QVariant LogsTableModel::data(const QModelIndex &index, int role) const
{
  if (!index.isValid())
    return QVariant();

  if (index.row() >= logs.size())
    return QVariant();

  const LogItem& log = logs[ index.row() ];

  if (role == Qt::DisplayRole)
  {
    switch( index.column() )
    {
    case 0: return log.time_text;
    case 1: {
      switch( log.level_raw )
      {
      case DEBUG:      return "DEBUG";
      case INFO:       return "INFO";
      case WARNINGS:   return "WARNINGS";
      case ERROR:      return "ERROR";
      }
    } break;
    case 2: return log.node;
    case 3: return log.message;
    case 4: return log.source;
    }
  }
  else if( role== Qt::ForegroundRole){
    switch( logs[ index.row() ].level_raw )
    {
    case DEBUG:    return QBrush( QColor::fromRgb(50,  50 , 50)) ;  // black
    case INFO:     return QBrush( QColor::fromRgb(0,   0  , 255));  // blue
    case WARNINGS: return QBrush( QColor::fromRgb(240, 120, 0));    // orange
    case ERROR:    return QBrush( QColor::fromRgb(255, 0  , 0));    // red
    }
  }
  else{
    return QVariant();
  }
}

const QString &LogsTableModel::message(int index) const
{
  return logs[ index ].message;
}

const QString &LogsTableModel::nodeName(int index) const
{
  return logs[ index ].node;
}

LogsTableModel::Severity LogsTableModel::severity(int index) const
{
  return logs[ index ].level_raw;
}

const QDateTime& LogsTableModel::timestamp(int index) const
{
  return logs[ index ].time_raw;
}

void LogsTableModel::loadRosbag(const rosbag::Bag &bag)
{
  std::vector<std::string> topics;
  topics.push_back(std::string("rosout"));

#ifdef USE_ROSOUT2
  topics.push_back(std::string("rosout2"));
#endif

  rosbag::View bag_view(bag, rosbag::TopicQuery(topics));

  logs.clear();

  ros::Time ros_time;

  for(rosbag::MessageInstance const log: bag_view)
  {
    LogItem item;

    rosgraph_msgs::Log::ConstPtr rout1 = log.instantiate<rosgraph_msgs::Log>();
    if (rout1 != nullptr)
    {
      switch( rout1->level ){
      case rosgraph_msgs::Log::DEBUG : item.level_raw = DEBUG; break;
      case rosgraph_msgs::Log::INFO  : item.level_raw = INFO;break;
      case rosgraph_msgs::Log::WARN  : item.level_raw = WARNINGS;break;
      case rosgraph_msgs::Log::ERROR : item.level_raw = ERROR;break;
      }

      item.node    = rout1->name.c_str();
      item.source  = rout1->function.c_str();
      item.message = rout1->msg.c_str();
      ros_time     = rout1->header.stamp;
    }

#ifdef USE_ROSOUT2
    rosout2_msg::LogMsg::ConstPtr rout2 = log.instantiate<rosout2_msg::LogMsg>();
    if (rout2 != nullptr)
    {
      switch( rout2->level ){
      case rosout2_msg::LogMsg::DEBUG : item.level_raw = DEBUG; break;
      case rosout2_msg::LogMsg::INFO  : item.level_raw = INFO; break;
      case rosout2_msg::LogMsg::WARN  : item.level_raw = WARNINGS; break;
      case rosout2_msg::LogMsg::ERROR : item.level_raw = ERROR; break;
      }

      item.node    = rout2->node_name.c_str();
      item.source  = rout2->logger_name.c_str();
      item.message = rout2->message.c_str();
      ros_time     = rout2->stamp;
    }
#endif

    item.time_raw = QDateTime::fromMSecsSinceEpoch( ros_time.sec*1000 + ros_time.nsec / 1000000 );
    item.time_text = item.time_raw.toString("hh:mm: ss.zzz");

    logs.push_back( item );
  }
  this->beginInsertRows( QModelIndex(), 0, logs.size() -1);
  this->endInsertRows();
}
