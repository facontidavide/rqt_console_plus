#ifndef LOGSTABLEMODEL_HPP
#define LOGSTABLEMODEL_HPP

#include <QAbstractTableModel>
#include <QString>
#include <QDateTime>
#include <rosgraph_msgs/Log.h>
#include <rosbag/view.h>
#include <boost/circular_buffer.hpp>
#include <boost/flyweight.hpp>

#ifdef USE_ROSOUT2
  #include <rosout2_msg/LogMsg.h>
#endif

class LogsTableModel : public QAbstractTableModel
{
  Q_OBJECT

public:
  explicit LogsTableModel(QObject *parent = 0);

  typedef enum{
    DEBUG = 0,
    INFO = 1,
    WARNINGS = 2,
    ERROR = 3
  }Severity;

  // Header:
  QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;

  // Basic functionality:
  int rowCount(const QModelIndex &parent = QModelIndex()) const override;

  int columnCount(const QModelIndex &parent = QModelIndex()) const override;

  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

  void appendRow(const std::vector<rosgraph_msgs::Log::ConstPtr> &pushed_logs);

#ifdef USE_ROSOUT2
  void appendRow(const rosout2_msg::LogMsg& log);
#endif

  const QString& message(int index) const;

  const QString& nodeName(int index) const;

  Severity severity(int index) const;

  const QDateTime &timestamp(int index) const;

  void loadRosbag(const rosbag::Bag& bag);

private:

  typedef struct{
    size_t count;
    QDateTime time_raw;
    QString  time_text;
    Severity level_raw;
    boost::flyweight<std::string> node;
    QString message;
    boost::flyweight<std::string> source;
  }LogItem;

  boost::circular_buffer<LogItem> _logs;

  size_t _count;


  enum{ MAX_CAPACITY = 20000 }; // max capacity of the circular buffer

  LogItem convertRosout(const rosgraph_msgs::Log &log);

#ifdef USE_ROSOUT2
  LogItem convertRosout(const rosout2_msg::LogMsg &log);
#endif

signals:

  void rowsShifted(int);

};


#endif // LOGSTABLEMODEL_HPP
