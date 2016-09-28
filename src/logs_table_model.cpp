#include "../include/rqt_console_plus/logs_table_model.hpp"
#include <QDateTime>
#include <QBrush>
#include <QColor>
#include <QDebug>

LogsTableModel::LogsTableModel(QObject *parent)
  : QAbstractTableModel(parent),
    _logs(MAX_CAPACITY) // initial capacity
{
  _count = 0;
}

QVariant LogsTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{

  if (role != Qt::DisplayRole)
    return QVariant();

  if (orientation == Qt::Horizontal){
    switch(section)
    {
    case 0: return "#"; break;
    case 1: return "Time"; break;
    case 2: return "Severity";break;
    case 3: return "Node";break;
    case 4: return "Message";break;
    case 5: return "Source";break;
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

  return _logs.size();
}

int LogsTableModel::columnCount(const QModelIndex &parent) const
{
  if (parent.isValid())
    return 0;

  return 6;
}

QVariant LogsTableModel::data(const QModelIndex &index, int role) const
{
  if (!index.isValid())
    return QVariant();

  if (index.row() >= _logs.size())
    return QVariant();

  const LogItem& log = _logs[ index.row() ];

  if (role == Qt::DisplayRole)
  {
    switch( index.column() )
    {
    case 0: return log.count;
    case 1: return log.time_text;
    case 2: {
      switch( log.level_raw )
      {
      case DEBUG:      return "DEBUG";
      case INFO:       return "INFO";
      case WARNINGS:   return "WARNINGS";
      case ERROR:      return "ERROR";
      }
    } break;
    case 3: return log.node;
    case 4: return log.message;
    case 5: return log.source;
    }
  }
  else if( role== Qt::ForegroundRole){
    switch( log.level_raw )
    {
    case DEBUG:    return QBrush( QColor::fromRgb(50,  50 , 50)) ;  // black
    case INFO:     return QBrush( QColor::fromRgb(0,   0  , 255));  // blue
    case WARNINGS: return QBrush( QColor::fromRgb(240, 120, 0));    // orange
    case ERROR:    return QBrush( QColor::fromRgb(255, 0  , 0));    // red
    }
  }
  else if( role == Qt::UserRole){
    switch( index.column() )
    {
    case 0: return log.count;
    case 1: return log.time_raw;
    case 2: return log.level_raw;
    case 3: return log.node;
    case 4: return log.message;
    case 5: return log.source;
    }
  }
  else{
    return QVariant();
  }
}

LogsTableModel::LogItem LogsTableModel::convertRosout( const rosgraph_msgs::Log &log)
{
  _count++;

  LogItem item;
  switch( log.level ){
  case rosgraph_msgs::Log::DEBUG : item.level_raw = DEBUG; break;
  case rosgraph_msgs::Log::INFO  : item.level_raw = INFO;break;
  case rosgraph_msgs::Log::WARN  : item.level_raw = WARNINGS;break;
  case rosgraph_msgs::Log::ERROR : item.level_raw = ERROR;break;
  }

  item.count   = _count;
  item.node    = log.name.c_str();

  item.source  = log.file.c_str();
  item.source  += QString(" ");
  item.source  += log.function.c_str();
  item.source  += QString(":");
  item.source  += QString::number(log.line);

  item.message = log.msg.c_str();

  item.time_raw = QDateTime::fromMSecsSinceEpoch( log.header.stamp.sec*1000 + log.header.stamp.nsec / 1000000 );
  item.time_text = item.time_raw.toString("hh:mm: ss.zzz");
  return item;
}

#ifdef USE_ROSOUT2
LogsTableModel::LogItem LogsTableModel::convertRosout(const rosout2_msg::LogMsg &log)
{
  LogItem item;
  switch( log.level ){
  case rosout2_msg::LogMsg::DEBUG : item.level_raw = DEBUG; break;
  case rosout2_msg::LogMsg::INFO  : item.level_raw = INFO; break;
  case rosout2_msg::LogMsg::WARN  : item.level_raw = WARNINGS; break;
  case rosout2_msg::LogMsg::ERROR : item.level_raw = ERROR; break;
  }

  item.node    = log.node_name.c_str();
  item.source  = log.logger_name.c_str();
  item.message = log.message.c_str();

  item.time_raw = QDateTime::fromMSecsSinceEpoch( log.stamp.sec*1000 + log.stamp.nsec / 1000000 );
  item.time_text = item.time_raw.toString("hh:mm: ss.zzz");
  return item;
}

void LogsTableModel::appendRow(const rosout2_msg::LogMsg& log)
{
  if( _logs.full() && _logs.size() < MAX_CAPACITY)
  {
    _logs.set_capacity( std::min( (size_t)(_logs.size() * 1.5), (size_t)MAX_CAPACITY) );
  }

  LogItem item = convertRosout(log);

  this->beginInsertRows( QModelIndex(), _logs.size(), _logs.size());
  _logs.push_back( item );
  this->endInsertRows();
}


#endif

void LogsTableModel::appendRow(const std::vector<rosgraph_msgs::Log::ConstPtr>& pushed_logs)
{
  size_t old_size = _logs.size();
  size_t new_size = old_size + pushed_logs.size();

  /* if( new_size > _logs.capacity() && _logs.capacity() < MAX_CAPACITY)
  {
    const size_t new_capacity =  std::min( (size_t)(_logs.size() * 1.5), (size_t)MAX_CAPACITY);
    _logs.set_capacity( new_capacity );
  }*/

  int to_add   = pushed_logs.size();
  int to_shift = 0;

  if( new_size > MAX_CAPACITY)
  {
    to_add = (MAX_CAPACITY - old_size);
    to_shift = new_size - MAX_CAPACITY;
    new_size = MAX_CAPACITY;
  }

  const size_t last_row  = new_size - 1;
  const size_t first_row = new_size - pushed_logs.size() ;

  for (int i=0; i < pushed_logs.size(); i++){
    _logs.push_back( convertRosout( *pushed_logs[i]) );
  }

  if(to_shift > 0)
  {
    this->beginRemoveRows( QModelIndex(), 0 , to_shift-1);
    this->endRemoveRows();

    emit dataChanged( index(0,0), index( rowCount()-1, columnCount() -1));
  }

  this->beginInsertRows( QModelIndex(), first_row , last_row);
  this->endInsertRows();

}


const QString &LogsTableModel::message(int index) const
{
  return _logs[ index ].message;
}

const QString &LogsTableModel::nodeName(int index) const
{
  return _logs[ index ].node;
}

LogsTableModel::Severity LogsTableModel::severity(int index) const
{
  return _logs[ index ].level_raw;
}

const QDateTime& LogsTableModel::timestamp(int index) const
{
  return _logs[ index ].time_raw;
}

void LogsTableModel::loadRosbag(const rosbag::Bag &bag)
{
  std::vector<std::string> topics;
  topics.push_back(std::string("rosout"));

#ifdef USE_ROSOUT2
  topics.push_back(std::string("rosout2"));
#endif

  rosbag::View bag_view(bag, rosbag::TopicQuery(topics));

  _logs.clear();

  for(rosbag::MessageInstance const log: bag_view)
  {
    rosgraph_msgs::Log::ConstPtr rout1 = log.instantiate<rosgraph_msgs::Log>();
    if (rout1 != nullptr)
    {
      _logs.push_back( convertRosout( *rout1 ) );
    }

#ifdef USE_ROSOUT2
    rosout2_msg::LogMsg::ConstPtr rout2 = log.instantiate<rosout2_msg::LogMsg>();
    if (rout2 != nullptr)
    {
      _logs.push_back( convertRosout( *rout2 ) );
    }
#endif

  }
  this->beginInsertRows( QModelIndex(), 0, _logs.size() -1);
  this->endInsertRows();
}
