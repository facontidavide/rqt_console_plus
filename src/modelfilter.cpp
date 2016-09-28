#include "../include/rqt_console2/modelfilter.hpp"
#include "../include/rqt_console2/logs_table_model.hpp"


ModelFilter::ModelFilter(QObject *parent) :
  QSortFilterProxyModel (parent)
{
  _msg_filter_enabled    = false;
  _node_filter_enabled   = false;
  _source_filter_enabled = false;
  _time_filter_enabled   = false;

  _info_filter_enabled = false;
  _error_filter_enabled = false;
  _warn_filter_enabled = false;
  _debug_filter_enabled = false;
}

void ModelFilter::setMessageFilterEnabled(bool enabled)
{
  _msg_filter_enabled = enabled;
}

void ModelFilter::setNodeFilterEnabled(bool enabled)
{
  _node_filter_enabled = enabled;
}

void ModelFilter::setSourceFilterEnabled(bool enabled)
{
  _source_filter_enabled = enabled;
}

void ModelFilter::setTimeFilterEnabled(bool enabled)
{
  _time_filter_enabled = enabled;
}

void ModelFilter::messageFilterUpdated(ModelFilter::FilterMode mode, const QString &text)
{
  _msg_mode = mode;
  _msg_text = text;
}

void ModelFilter::nodeFilterUpdated(ModelFilter::FilterMode mode, const QString &text)
{
  _node_mode = mode;
  _node_text = text;
}

void ModelFilter::sourceFilterUpdated(ModelFilter::FilterMode mode, const QString &text)
{
  _source_mode = mode;
  _source_text = text;
}

void ModelFilter::timeMinUpdated(QDateTime min)
{
  _min = min;
}

void ModelFilter::timeMaxUpdated(QDateTime max)
{
  _max = max;
}

void ModelFilter::setSeverityInfoEnabled(bool enabled)
{
  _info_filter_enabled = enabled;
}

void ModelFilter::setSeverityDebugEnabled(bool enabled)
{
  _debug_filter_enabled = enabled;
}

void ModelFilter::setSeverityErrorEnabled(bool enabled)
{
  _error_filter_enabled = enabled;
}

void ModelFilter::setSeverityWarningsEnabled(bool enabled)
{
  _warn_filter_enabled = enabled;
}

bool ModelFilter::filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const
{
  QModelIndex index_time     = sourceModel()->index(sourceRow, 1, sourceParent);
  QModelIndex index_severity = sourceModel()->index(sourceRow, 2, sourceParent);
  QModelIndex index_node     = sourceModel()->index(sourceRow, 3, sourceParent);
  QModelIndex index_message  = sourceModel()->index(sourceRow, 4, sourceParent);
  QModelIndex index_source   = sourceModel()->index(sourceRow, 5, sourceParent);

  int severity = sourceModel()->data( index_severity, Qt::UserRole ).toInt();

  if( !_info_filter_enabled  && severity == LogsTableModel::INFO)     return false;
  if( !_error_filter_enabled && severity == LogsTableModel::ERROR)    return false;
  if( !_warn_filter_enabled  && severity == LogsTableModel::WARNINGS) return false;
  if( !_debug_filter_enabled && severity == LogsTableModel::DEBUG)    return false;

  if( _time_filter_enabled ){
    QDateTime tm = sourceModel()->data( index_time, Qt::UserRole ).toDateTime();
    if( tm < _min || tm > _max ){
      return false;
    }
  }

  return true;
}

