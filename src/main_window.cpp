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


#include <QMessageBox>
#include <QSettings>
#include <QFileDialog>
#include <QStringList>
#include <QTextStream>
#include <QHeaderView>
#include <QToolButton>
#include <QIcon>
#include <iostream>
#include "../include/rqt_console2/main_window.hpp"
#include "../include/rqt_console2/logwidget.hpp"

namespace rqt_console2 {

using namespace Qt;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionRead_rosbag, SIGNAL(triggered(bool)), this, SLOT(loadRosbag()) );

  ui.tab_manager->addTab(new LogWidget(model, this), QString("logging"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing

  QToolButton *newTabButton = new QToolButton(this);
  ui.tab_manager->setCornerWidget(newTabButton, Qt::TopLeftCorner);
  newTabButton->setCursor(Qt::ArrowCursor);
  newTabButton->setAutoRaise(true);
  newTabButton->setText("+");
  QObject::connect(newTabButton, SIGNAL(clicked()), this, SLOT(newTab()));
  newTabButton->setToolTip(tr("Add page"));
}

MainWindow::~MainWindow() {}



void MainWindow::loadRosbag()
{
  QSettings settings( "Martianbots", "rqt_console2");
  QString directory_path = settings.value("MainWindow.loadRosbag.directory", QDir::currentPath() ).toString();

  QString filename = QFileDialog::getOpenFileName(this,
                                                  tr("Open rosbag"),
                                                  directory_path,
                                                  tr("Rosbag Files (*.bag)"));
  if (filename.isEmpty())
    return;

  directory_path = QFileInfo(filename).absolutePath();
  settings.setValue("MainWindow.loadRosbag.directory", directory_path);

  rosbag::Bag bag;
  bag.open( filename.toStdString(), rosbag::bagmode::Read );

  model.loadRosbag( bag );
}


void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}



}  // namespace rqt_console2


void rqt_console2::MainWindow::on_actionSave_Layout_triggered()
{
  QDomDocument doc;
  QDomProcessingInstruction instr =
      doc.createProcessingInstruction("xml", "version='1.0' encoding='UTF-8'");
  doc.appendChild(instr);

  QDomElement root = doc.createElement( "root" );

  for (int tab=0; tab< ui.tab_manager->count(); tab++)
  {
    LogWidget* log_tab = static_cast<LogWidget*>(ui.tab_manager->widget(tab) );
    root.appendChild( log_tab->xmlSaveState(doc) );
  }

  doc.appendChild( root );


  QSettings settings( "Martianbots", "rqt_console2");

  QString directory_path  = settings.value("MainWindow.on_actionSave_Layout_triggered.directory",
                                           QDir::currentPath() ). toString();

  QString filename = QFileDialog::getSaveFileName(this, "Save Layout", directory_path, "*.xml");
  if (filename.isEmpty())
    return;

  if(filename.endsWith(".xml",Qt::CaseInsensitive) == false)
  {
    filename.append(".xml");
  }

  QFile file(filename);
  if (file.open(QIODevice::WriteOnly)) {
    QTextStream stream(&file);
    stream << doc.toString() << endl;
  }

  directory_path = QFileInfo(filename).absolutePath();
  settings.setValue("MainWindow.on_actionSave_Layout_triggered.directory", directory_path);

}

void rqt_console2::MainWindow::on_actionLoadLayout_triggered()
{
  QSettings settings( "Martianbots", "rqt_console2");

  QString directory_path  = settings.value("MainWindow.on_actionLoadLayout_triggered.directory",
                                           QDir::currentPath() ). toString();

  QString filename = QFileDialog::getOpenFileName(this, "Load Layout", directory_path, "*.xml");
  if (filename.isEmpty())
    return;

  QFile file(filename);

  directory_path = QFileInfo(filename).absolutePath();
  settings.setValue("MainWindow.on_actionLoadLayout_triggered.directory", directory_path);


  if (!file.open(QFile::ReadOnly | QFile::Text))
  {
    QMessageBox::warning(this, tr("Layout"),
                         tr("Cannot read file %1:\n%2.")
                         .arg(filename)
                         .arg(file.errorString()));
    return;
  }

  QString errorStr;
  int errorLine, errorColumn;
  QDomDocument doc;

  if (!doc.setContent(&file, true, &errorStr, &errorLine, &errorColumn)) {
    QMessageBox::information(window(), tr("XML Layout"),
                             tr("Parse error at line %1:\n%2")
                             .arg(errorLine)
                             .arg(errorStr));
    return;
  }

  QDomNodeList widget_elements = doc.elementsByTagName("LogWidget" );

  while( ui.tab_manager->count() > widget_elements.count())
  {
    ui.tab_manager->removeTab(0);
  }

  while( ui.tab_manager->count() < widget_elements.count())
  {
    newTab();
  }

  for( int index = 0; index <widget_elements.count(); index++ )
  {
    LogWidget* log_tab = static_cast<LogWidget*>(ui.tab_manager->widget(index) );
    QDomNode element = widget_elements.item( index );
    log_tab->xmlLoadState( element );
  }

}

void rqt_console2::MainWindow::newTab()
{
  ui.tab_manager->insertTab(0, new LogWidget(model, this), QString("logging"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing
}

void rqt_console2::MainWindow::on_tab_manager_tabCloseRequested(int index)
{
  if( ui.tab_manager->count() > 1)
    ui.tab_manager->removeTab(index);
}
