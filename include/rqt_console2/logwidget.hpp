/**
 * @file /include/rqt_console2/main_window.hpp
 *
 * @brief Qt based gui for rqt_console2.
 *
 * @date November 2010
 **/
#ifndef _LOG_WIDGET_H
#define _LOG_WIDGET_H

/*****************************************************************************
** Includes
*****************************************************************************/


#include <QDomDocument>
#include <boost/circular_buffer.hpp>
#include <QWidget>
#include <QValidator>
#include "logs_table_model.hpp"
#include "ui_logwidget.h"
#include "modelfilter.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rqt_console2 {


class LogWidget : public QWidget
{
  Q_OBJECT

public:
  explicit LogWidget(LogsTableModel& tablemodel, QWidget *parent = 0);
  ~LogWidget();


  QDomElement xmlSaveState( QDomDocument &doc);

  void xmlLoadState(QDomNode &state);

private slots:
  void on_lineEditMessageFilter_textEdited(const QString &filter);

  void on_checkBoxMessageFilter_toggled(bool checked);

  void on_checkBoxTimeRange_toggled(bool checked);

  void on_checkBoxLoggerFilter_toggled(bool checked);

  void on_comboBoxMessageFilter_currentIndexChanged(int index);

  void on_comboBoxMessageInclude_currentIndexChanged(int index);

  void on_buttonEnableDebug_toggled(bool checked);

  void on_rowsInserted(const QModelIndex & parent, int first_row, int last_row);

  void on_rowsShifted(int shift);

  void on_rowsRemoved(const QModelIndex & parent, int first, int last_row);


  void on_buttonEnableInfo_toggled(bool checked);

  void on_buttonEnableWarnings_toggled(bool checked);

  void on_buttonEnableError_toggled(bool checked);

  void on_lineEditLoggerFilter_textEdited(const QString &arg1);

  void on_comboBoxLoggerFilter_currentIndexChanged(int index);

  void on_pushButtonTimeReset_pressed();

  void on_timeRangeMax_dateTimeChanged(const QDateTime &dateTime);

  void on_timeRangeMin_dateTimeChanged(const QDateTime &dateTime);

private:
  Ui::LogWidgetDesign ui;
  LogsTableModel& model;

  boost::circular_buffer<bool> message_filter;
  boost::circular_buffer<bool> logger_filter;
  boost::circular_buffer<bool> time_filter;
  boost::circular_buffer<bool> severity_filter;

  void applyFilter(QLineEdit* line_edit, QComboBox* combo,
                   boost::circular_buffer<bool> &filter_vector,
                   std::function<const QString&(int)>,
                   int first_row, int last_row);

  void applyMessageFilter(int first_row, int last_row);

  void applyLoggerFilter(int first_row, int last_row);

  void applyTimeFilter(int first_row, int last_row);

  void applySeverityFilter(int first_row, int last_row);

  void updateRowVisibility(int model_count = -1);

  void updateTimeRange();

  ModelFilter proxy_model;
};

}  // namespace rqt_console2

#endif // rqt_console2_MAIN_WINDOW_H
