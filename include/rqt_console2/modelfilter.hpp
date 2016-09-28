#ifndef MODELFILTER_HPP
#define MODELFILTER_HPP

#include <QSortFilterProxyModel>
#include <QDateTime>
#include <QString>

class ModelFilter : public QSortFilterProxyModel
{
  Q_OBJECT
public:
  explicit ModelFilter(QObject *parent = 0);

  typedef enum{
    CONTAINS_ALL,
    CONTAINS_ONE,
    WINDCARD,
    REGEX
  }FilterMode;

signals:

public slots:

  void setMessageFilterEnabled(bool enabled);
  void setNodeFilterEnabled(bool enabled);
  void setSourceFilterEnabled(bool enabled);
  void setTimeFilterEnabled(bool enabled);

  void messageFilterUpdated(FilterMode mode, const QString& text  );
  void nodeFilterUpdated(FilterMode mode, const QString& text  );
  void sourceFilterUpdated(FilterMode mode, const QString& text  );
  void timeMinUpdated(QDateTime min);
  void timeMaxUpdated(QDateTime max);

  void setSeverityInfoEnabled(bool enabled);
  void setSeverityDebugEnabled(bool enabled);
  void setSeverityErrorEnabled(bool enabled);
  void setSeverityWarningsEnabled(bool enabled);


protected:
  virtual bool filterAcceptsRow(int sourceRow,
          const QModelIndex &sourceParent) const override;

  QDateTime _min;
  QDateTime _max;

  bool _node_filter_enabled;
  bool _source_filter_enabled;
  bool _msg_filter_enabled;
  bool _time_filter_enabled;

  bool _debug_filter_enabled;
  bool _info_filter_enabled;
  bool _error_filter_enabled;
  bool _warn_filter_enabled;

  FilterMode _node_mode;
  FilterMode _msg_mode;
  FilterMode _source_mode;

  QString _node_text;
  QString _msg_text;
  QString _source_text;
};

#endif // MODELFILTER_HPP
