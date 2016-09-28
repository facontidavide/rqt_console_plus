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
#include "../include/rqt_console2/logwidget.hpp"
#include <QDomDocument>

namespace rqt_console2 {

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

  connect( &model, SIGNAL(rowsAboutToBeRemoved(const QModelIndex&, int, int)),
           this, SLOT(on_rowsRemoved(const QModelIndex&,int,int))  );

  connect( &model, SIGNAL(rowsShifted(int)),
           this, SLOT(on_rowsShifted(int))  );


  if( model.rowCount() > 0)
  {
    message_filter.resize( model.rowCount(), true );
    logger_filter.resize( model.rowCount(), true );
    severity_filter.resize( model.rowCount(), true );
    time_filter.resize( model.rowCount(), true );
    updateTimeRange();
  }

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

  QComboBox* combobox[2] =  { ui.comboBoxMessageInclude, ui.comboBoxMessageFilter };
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

  QComboBox* combobox[2] =   { ui.comboBoxMessageInclude, ui.comboBoxMessageFilter };
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

  const int N = model.rowCount() -1;

  applyMessageFilter( 0, N );
  applyLoggerFilter( 0, N );
  applyTimeFilter( 0, N );
  applySeverityFilter( 0, N );

  updateRowVisibility();
}

inline bool filterByMessage(const QStringList &filter_words, const QString &message, bool all_of_them)
{
  if( all_of_them) // contains all
  {
    for (int i=0; i< filter_words.size(); i++){
      if( message.contains(filter_words[i], Qt::CaseSensitive) == false )
      {
        return false;
      }
    }
    return true;
  }
  else{ // contains at least one
    for (int i=0; i< filter_words.size(); i++){
      if( message.contains(filter_words[i], Qt::CaseSensitive) == true )
      {
        return true;
      }
    }
    return false;
  }
}

void LogWidget::applyFilter(QLineEdit* line_edit, QComboBox* combo,
                            boost::circular_buffer<bool>& filter_vector,
                            std::function<const QString&(int)> message_to_check,
                            int first_row, int last_row)
{
  const QString& filter = line_edit->text();

  // accept if no filter
  if(filter.count() == 0 )
  {
    for( int row = first_row; row <= last_row; ++row )
    {
      filter_vector[row] = true;
    }
    return;
  }

  QStringList filter_words = filter.split(QRegExp("\\s"), QString::SkipEmptyParts);

  if( combo->currentIndex() == 0) // contains (all)
  {
    for( int row = first_row; row <= last_row; ++row )
    {
      filter_vector[row] = filterByMessage(filter_words, message_to_check(row ), true);
    }
  }
  else if( combo->currentIndex() == 1) // contains (at least one)
  {
    for( int row = first_row; row <= last_row; ++row )
    {
      filter_vector[row] = filterByMessage(filter_words, message_to_check(row ), false) ;
    }
  }
  if( combo->currentIndex() == 2) // wildcards
  {
    QRegExp regexp( filter,  Qt::CaseSensitive, QRegExp::Wildcard );
    QRegExpValidator validator(regexp, 0);

    for( int row = first_row; row <= last_row; ++row )
    {
      QString message = message_to_check(row );
      int pos = 0;
      filter_vector[row] = validator.validate( message, pos ) == QValidator::Acceptable;
    }
  }
  else if( combo->currentIndex() == 3) // regex
  {
    QRegExp regexp( filter,  Qt::CaseSensitive, QRegExp::RegExp2 );
    QRegExpValidator validator(regexp, 0);

    for( int row = first_row; row <= last_row; ++row )
    {
      QString message = message_to_check(row );
      int pos = 0;
      filter_vector[row] = validator.validate( message, pos ) == QValidator::Acceptable;
    }
  }

}

void LogWidget::applyMessageFilter(int first_row, int last_row)
{
  // qDebug() << "applyMessageFilter";

  applyFilter( ui.lineEditMessageFilter, ui.comboBoxMessageFilter, message_filter,
               std::bind(&LogsTableModel::message, &model, std::placeholders::_1),
               first_row, last_row);
}

void LogWidget::applyLoggerFilter(int first_row, int last_row)
{
  //  qDebug() << "applyLoggerFilter";

  applyFilter( ui.lineEditLoggerFilter, ui.comboBoxLoggerFilter, logger_filter,
               std::bind(&LogsTableModel::nodeName, &model, std::placeholders::_1),
               first_row, last_row);
}

void LogWidget::applyTimeFilter(int first_row, int last_row)
{
  //  qDebug() << "applyTimeFilter";

  const auto& min = ui.timeRangeMin->dateTime();
  const auto& max = ui.timeRangeMax->dateTime();

  for (int row=first_row; row<= last_row; row++)
  {
    const auto& t = model.timestamp(row);
    time_filter[row] = ( min <= t && t <= max );
  }
}




void LogWidget::updateRowVisibility(int model_count)
{
  /*if( model_count < 0) model_count = model.rowCount();

  int visible_count = 0;
  for (int row=0; row< model_count; row++)
  {
    bool visible = true;

    if( ui.checkBoxMessageFilter->isChecked() )
    {
      if( ui.comboBoxMessageInclude->currentIndex() == 0 )
        visible &= message_filter[row];
      else
        visible &= !message_filter[row];
    }
    if( ui.checkBoxLoggerFilter->isChecked() ){
      visible &= logger_filter[row];
    }
    if( ui.checkBoxTimeRange->isChecked() ){
      visible &= time_filter[row];
    }

    visible &= severity_filter[row];
    if( visible ) visible_count++;

    ui.tableView->setRowHidden( row, !visible);
  }

  ui.statusbar->showMessage(
        tr("Displaying ") + QString::number(visible_count) + tr(" of ")
        + QString::number(model_count) + tr(" messages") );*/
}

void LogWidget::on_lineEditMessageFilter_textEdited(const QString &filter)
{
  applyMessageFilter( 0, model.rowCount() -1 );
  updateRowVisibility();
}

void LogWidget::on_comboBoxMessageFilter_currentIndexChanged(int )
{
  applyMessageFilter( 0, model.rowCount() -1 );
  updateRowVisibility();
}

void rqt_console2::LogWidget::on_comboBoxMessageInclude_currentIndexChanged(int )
{
  applyMessageFilter( 0, model.rowCount() -1 );
  updateRowVisibility();
}

void LogWidget::on_checkBoxMessageFilter_toggled(bool checked)
{
  ui.comboBoxMessageInclude->setEnabled( checked );
  ui.labelMessageFilter->setEnabled( checked );
  ui.comboBoxMessageFilter->setEnabled( checked );
  ui.lineEditMessageFilter->setEnabled( checked );

  updateRowVisibility();
}

void rqt_console2::LogWidget::on_checkBoxTimeRange_toggled(bool checked)
{
  ui.labelTimeRange->setEnabled( checked );
  ui.timeRangeMin->setEnabled( checked );
  ui.timeRangeMax->setEnabled( checked );

  updateRowVisibility( );
}

void LogWidget::on_checkBoxLoggerFilter_toggled(bool checked)
{
  ui.labelLoggerFilter->setEnabled( checked );
  ui.comboBoxLoggerFilter->setEnabled( checked );
  ui.lineEditLoggerFilter->setEnabled( checked );

  updateRowVisibility( );
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
  const size_t N = model.rowCount();
  message_filter.resize( N , true );
  logger_filter.resize( N , true);
  severity_filter.resize( N , true);
  time_filter.resize( N , true);

  //  qDebug() << N << " " << first_row << "/" <<last_row;

  applyMessageFilter( first_row, last_row );
  applyLoggerFilter( first_row, last_row );
  applyTimeFilter( first_row, last_row );
  applySeverityFilter( first_row, last_row );

  updateTimeRange();

  updateRowVisibility(N);

  ui.tableView->scrollToBottom();

}

void LogWidget::on_rowsShifted(int shift)
{
  on_rowsInserted( QModelIndex(),
                   model.rowCount() -shift,
                   model.rowCount() -1);
}

void LogWidget::on_rowsRemoved(const QModelIndex &parent, int first_row, int last_row)
{
  for (int row=first_row; row<= last_row; row++)
  {
    message_filter.pop_front();
    logger_filter.pop_front();
    severity_filter.pop_front();
    time_filter.pop_front();
  }
}


void LogWidget::on_buttonEnableDebug_toggled(bool checked)
{
  applySeverityFilter( 0, model.rowCount() -1 );
  updateRowVisibility();
  updateRowVisibility();
  proxy_model.setSeverityDebugEnabled(checked);
}

void LogWidget::on_buttonEnableInfo_toggled(bool checked)
{
  applySeverityFilter( 0, model.rowCount() -1 );
  updateRowVisibility();
  proxy_model.setSeverityInfoEnabled(checked);
}

void LogWidget::on_buttonEnableWarnings_toggled(bool checked)
{
  applySeverityFilter( 0, model.rowCount() -1 );
  updateRowVisibility();
  proxy_model.setSeverityWarningsEnabled(checked);
}

void LogWidget::on_buttonEnableError_toggled(bool checked)
{
  applySeverityFilter( 0, model.rowCount() -1 );
  updateRowVisibility();

  proxy_model.setSeverityErrorEnabled(checked);
}

void LogWidget::on_lineEditLoggerFilter_textEdited(const QString &arg1)
{
  applyLoggerFilter( 0, model.rowCount() -1 );
  updateRowVisibility();
}

void LogWidget::on_comboBoxLoggerFilter_currentIndexChanged(int index)
{
  applyLoggerFilter( 0, model.rowCount() -1 );
  updateRowVisibility();

}



}  // namespace rqt_console2




void rqt_console2::LogWidget::on_pushButtonTimeReset_pressed()
{
  ui.timeRangeMin->setDateTime( ui.timeRangeMin->minimumDateTime() );
  ui.timeRangeMax->setDateTime( ui.timeRangeMin->maximumDateTime() );

  applyTimeFilter( 0, model.rowCount() -1 );
  updateRowVisibility();
}

void rqt_console2::LogWidget::on_timeRangeMax_dateTimeChanged(const QDateTime &dateTime)
{
  if( dateTime < ui.timeRangeMin->dateTime())
  {
    ui.timeRangeMax->setDateTime( ui.timeRangeMin->dateTime() );
    return; // will trigger again the slot
  }

  applyTimeFilter( 0, model.rowCount() -1 );
  updateRowVisibility();

  proxy_model.timeMaxUpdated( dateTime );
}

void rqt_console2::LogWidget::on_timeRangeMin_dateTimeChanged(const QDateTime &dateTime)
{
  if( dateTime > ui.timeRangeMax->dateTime())
  {
    ui.timeRangeMin->setDateTime( ui.timeRangeMax->dateTime() );
    return; // will trigger again the slot
  }

  applyTimeFilter( 0, model.rowCount() -1 );
  updateRowVisibility();

  proxy_model.timeMinUpdated( dateTime );
}
