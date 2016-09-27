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

#include <QWidget>
#include <QValidator>
#include "logs_table_model.hpp"
#include "ui_logwidget.h"
#include <QDomDocument>

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

  void on_rowsAboutToBeInserted(const QModelIndex & parent, int start, int end);

  void on_buttonEnableInfo_toggled(bool checked);

  void on_buttonEnableWarnings_toggled(bool checked);

  void on_buttonEnableError_toggled(bool checked);

  void on_lineEditLoggerFilter_textEdited(const QString &arg1);

  void on_comboBoxLoggerFilter_currentIndexChanged(int index);

  void on_tableView_RightClick(const QPoint&);

  void on_pushButtonTimeReset_pressed();

  void on_timeRangeMax_dateTimeChanged(const QDateTime &dateTime);

  void on_timeRangeMin_dateTimeChanged(const QDateTime &dateTime);

private:
  Ui::LogWidgetDesign ui;
  LogsTableModel& model;

  std::vector<bool> message_filter;
  std::vector<bool> logger_filter;
  std::vector<bool> time_filter;
  std::vector<bool> severity_filter;

  void applyFilter(QLineEdit* line_edit, QComboBox* combo, std::vector<bool> &filter_vector, std::function<const QString&(int)>, int first_row, int last_row);

  void applyMessageFilter(int first_row, int last_row);

  void applyLoggerFilter(int first_row, int last_row);

  void applyTimeFilter(int first_row, int last_row);

  void applySeverityFilter(int first_row, int last_row);

  void updateRowVisibility();

};

}  // namespace rqt_console2

#endif // rqt_console2_MAIN_WINDOW_H
