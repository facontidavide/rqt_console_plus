/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QDebug>
#include <QMessageBox>
#include <QSettings>
#include <QFileDialog>
#include <QStringList>
#include <QHeaderView>
#include <iostream>
#include "../include/rqt_console_plus/logwidget.hpp"
#include <QDomDocument>

namespace rqt_console_plus {

using namespace Qt;


LogWidget::LogWidget(LogsTableModel& tablemodel, QWidget *parent)
  : QWidget(parent),
    model(tablemodel),
    proxy_model(this)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  proxy_model.setSourceModel(&model);
  ui.tableView->setModel(&proxy_model);
  // ui.tableView->setModel( &model );

  ui.tableView->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  ui.tableView->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  ui.tableView->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
  ui.tableView->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Interactive);
  ui.tableView->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Interactive);
  ui.tableView->horizontalHeader()->setSectionResizeMode(5, QHeaderView::Stretch);

  ui.tableView->verticalHeader()->setVisible(false);

  connect( &model, SIGNAL(rowsInserted(const QModelIndex&, int, int)),
           this, SLOT(on_rowsInserted(const QModelIndex&,int,int))  );

  proxy_model.setSeverityDebugEnabled( ui.buttonEnableDebug->isChecked() );
  proxy_model.setSeverityWarningsEnabled( ui.buttonEnableWarnings->isChecked() );
  proxy_model.setSeverityErrorEnabled( ui.buttonEnableError->isChecked() );
  proxy_model.setSeverityInfoEnabled( ui.buttonEnableInfo->isChecked() );

  proxy_model.setMessageFilterEnabled( ui.checkBoxMessageFilter->isChecked() );
  proxy_model.setNodeFilterEnabled( ui.checkBoxLoggerFilter->isChecked() );
  proxy_model.setTimeFilterEnabled( ui.checkBoxTimeRange->isChecked() );

}

LogWidget::~LogWidget() {}


QDomElement LogWidget::xmlSaveState( QDomDocument &doc)
{
  QDomElement log_el = doc.createElement("LogWidget");

  QCheckBox* checkBoxes[3] =  { ui.checkBoxMessageFilter, ui.checkBoxTimeRange, ui.checkBoxLoggerFilter };
  for (int i=0; i<3; i++)
  {
    QDomElement element = doc.createElement(checkBoxes[i]->objectName());
    element.setAttribute("isChecked", checkBoxes[i]->checkState() == Qt::Checked );
    log_el.appendChild(element);
  }

  QComboBox* combobox[2] =  { ui.comboBoxLoggerFilter, ui.comboBoxMessageFilter };
  for (int i=0; i<2; i++)
  {
    QDomElement element = doc.createElement(combobox[i]->objectName());
    element.setAttribute("currentIndex", combobox[i]->currentIndex() );
    log_el.appendChild(element);
  }

  QLineEdit* lineEdit[2] = { ui.lineEditMessageFilter, ui.lineEditLoggerFilter };
  for (int i=0; i<2; i++)
  {
    QDomElement element = doc.createElement(lineEdit[i]->objectName());
    element.setAttribute("text", lineEdit[i]->text() );
    log_el.appendChild(element);
  }

  QPushButton* button[4] = {  ui.buttonEnableDebug, ui.buttonEnableInfo,  ui.buttonEnableWarnings,   ui.buttonEnableError  };
  for (int i=0; i<4; i++)
  {
    QDomElement element = doc.createElement(button[i]->objectName());
    element.setAttribute("isChecked", button[i]->isChecked() );
    log_el.appendChild(element);
  }

  return log_el;
}

void LogWidget::xmlLoadState(QDomNode &state)
{
  QCheckBox* checkBoxes[3] =  { ui.checkBoxMessageFilter, ui.checkBoxTimeRange, ui.checkBoxLoggerFilter };
  for (int i=0; i<3; i++)
  {
    bool checked = state.firstChildElement( checkBoxes[i]->objectName() ).attribute("isChecked").toInt();
    checkBoxes[i]->setChecked(checked);
  }

  QComboBox* combobox[2] =   { ui.comboBoxLoggerFilter, ui.comboBoxMessageFilter };
  for (int i=0; i<2; i++)
  {
    int index = state.firstChildElement( combobox[i]->objectName() ).attribute("currentIndex").toInt();
    combobox[i]->setCurrentIndex(index);
  }

  QLineEdit* lineEdit[2] = { ui.lineEditMessageFilter, ui.lineEditLoggerFilter };
  for (int i=0; i<2; i++)
  {
    QString text = state.firstChildElement( lineEdit[i]->objectName() ).attribute("text");
    lineEdit[i]->setText(text);
  }

  QPushButton* button[4] = {  ui.buttonEnableDebug, ui.buttonEnableInfo,  ui.buttonEnableWarnings,   ui.buttonEnableError  };
  for (int i=0; i<4; i++)
  {
    bool checked = state.firstChildElement( button[i]->objectName() ).attribute("isChecked").toInt();
    button[i]->setChecked(checked);
  }

}


void LogWidget::on_lineEditMessageFilter_textEdited(const QString &filter)
{
  proxy_model.messageFilterUpdated(
        static_cast<ModelFilter::FilterMode>( ui.comboBoxMessageFilter->currentIndex() ),
        filter );
}

void LogWidget::on_comboBoxMessageFilter_currentIndexChanged(int mode)
{
  proxy_model.messageFilterUpdated(
        static_cast<ModelFilter::FilterMode>( mode ),
         ui.lineEditMessageFilter->text() );
}

void LogWidget::on_lineEditLoggerFilter_textEdited(const QString &filter)
{
  proxy_model.nodeFilterUpdated(
        static_cast<ModelFilter::FilterMode>( ui.comboBoxLoggerFilter->currentIndex() ),
        filter );
}

void LogWidget::on_comboBoxLoggerFilter_currentIndexChanged(int mode)
{
  proxy_model.nodeFilterUpdated(
        static_cast<ModelFilter::FilterMode>( mode ),
         ui.lineEditLoggerFilter->text() );
}

void LogWidget::on_checkBoxMessageFilter_toggled(bool checked)
{
  ui.labelMessageFilter->setEnabled( checked );
  ui.comboBoxMessageFilter->setEnabled( checked );
  ui.lineEditMessageFilter->setEnabled( checked );

  proxy_model.setMessageFilterEnabled( checked );
}

void LogWidget::on_checkBoxTimeRange_toggled(bool checked)
{
  ui.labelTimeRange->setEnabled( checked );
  ui.timeRangeMin->setEnabled( checked );
  ui.timeRangeMax->setEnabled( checked );

  proxy_model.setTimeFilterEnabled( checked );
}

void LogWidget::on_checkBoxLoggerFilter_toggled(bool checked)
{
  ui.labelLoggerFilter->setEnabled( checked );
  ui.comboBoxLoggerFilter->setEnabled( checked );
  ui.lineEditLoggerFilter->setEnabled( checked );

  proxy_model.setNodeFilterEnabled( checked );
}


void LogWidget::updateTimeRange()
{
  ui.timeRangeMin->setMinimumDateTime( model.timestamp(0) );
  ui.timeRangeMax->setMinimumDateTime( model.timestamp(0) );

  const int LAST = model.rowCount() -1;

  ui.timeRangeMin->setMaximumDateTime( model.timestamp(LAST) );
  ui.timeRangeMax->setMaximumDateTime( model.timestamp(LAST) );
}

void LogWidget::on_rowsInserted(const QModelIndex &, int first_row, int last_row)
{
  updateTimeRange();
  ui.tableView->scrollToBottom();
}

void LogWidget::on_buttonEnableDebug_toggled(bool checked)
{
  proxy_model.setSeverityDebugEnabled(checked);
}

void LogWidget::on_buttonEnableInfo_toggled(bool checked)
{
  proxy_model.setSeverityInfoEnabled(checked);
}

void LogWidget::on_buttonEnableWarnings_toggled(bool checked)
{
  proxy_model.setSeverityWarningsEnabled(checked);
}

void LogWidget::on_buttonEnableError_toggled(bool checked)
{
  proxy_model.setSeverityErrorEnabled(checked);
}

void LogWidget::on_pushButtonTimeReset_pressed()
{
  ui.timeRangeMin->setDateTime( ui.timeRangeMin->minimumDateTime() );
  ui.timeRangeMax->setDateTime( ui.timeRangeMin->maximumDateTime() );

  proxy_model.timeMinUpdated( ui.timeRangeMin->minimumDateTime() );
  proxy_model.timeMaxUpdated( ui.timeRangeMin->maximumDateTime() );

}

void LogWidget::on_timeRangeMax_dateTimeChanged(const QDateTime &dateTime)
{
  if( dateTime < ui.timeRangeMin->dateTime())
  {
    ui.timeRangeMax->setDateTime( ui.timeRangeMin->dateTime() );
    return; // will trigger again the slot
  }
  proxy_model.timeMaxUpdated( dateTime );
}

void LogWidget::on_timeRangeMin_dateTimeChanged(const QDateTime &dateTime)
{
  if( dateTime > ui.timeRangeMax->dateTime())
  {
    ui.timeRangeMin->setDateTime( ui.timeRangeMax->dateTime() );
    return; // will trigger again the slot
  }
  proxy_model.timeMinUpdated( dateTime );
}

}  // namespace rqt_console_plus
