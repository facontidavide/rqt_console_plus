/**
 * @file /include/rqt_console2/main_window.hpp
 *
 * @brief Qt based gui for rqt_console2.
 *
 * @date November 2010
 **/
#ifndef rqt_console2_MAIN_WINDOW_H
#define rqt_console2_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "logs_table_model.hpp"
#include "qnode.hpp"
#include "ui_main_window.h"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rqt_console2 {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:

  void loadRosbag();

private slots:

  void on_actionSave_Layout_triggered();

  void on_actionLoadLayout_triggered();

  void newTab();

  void on_tab_manager_tabCloseRequested(int index);

  void on_actionListen_rosout_toggled(bool toggled);

#ifdef USE_ROSOUT2
  void on_actionListen_rosout2_toggled(bool toggled);
#endif

private:
  Ui::MainWindowDesign ui;
  LogsTableModel model;

  QNode qnode;

};

}  // namespace rqt_console2

#endif // rqt_console2_MAIN_WINDOW_H
