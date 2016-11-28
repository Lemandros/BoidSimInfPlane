#ifndef PLOTMAINWINDOW_H
#define PLOTMAINWINDOW_H

#include <QMainWindow>
#include <QWidget>

#include "qcustomplot.h"
#include "boidsim2d.h"

namespace Ui {
class PlotMainWindow;
}

class PlotMainWindow : public QMainWindow
{
  Q_OBJECT

public:
  int idNr;
  BoidSim2D* sim;
  QString defaultPath;
  QWidget* parent;
  explicit PlotMainWindow(QWidget *parent = 0, QString defaultPath = "", BoidSim2D* sim = 0, int idNr = 0);
  ~PlotMainWindow( );
  QCustomPlot *plot;

protected:
  void closeEvent(QCloseEvent *event);

private:
  Ui::PlotMainWindow *ui;

public slots:
  void UpdateWindow( );
  void on_printPushButton_clicked( );
  void on_plotPlotAllPushButton_clicked( );
  void on_plotLogxCheckBox_toggled(bool checked);
  void on_plotLogyCheckBox_toggled(bool checked);
  void PlotSetLog(bool axis, bool checked, QCustomPlot* plot);
  void on_plotPushButton_clicked( );
  void on_plotMinSpinBox_valueChanged(int arg1);
  void on_plotMaxSpinBox_valueChanged(int arg1);
  void on_plotMaxPushButton_clicked( );
  void PrintFunction(QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin, QString fileName);
  void PlotAll(QCustomPlot* plot, QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin);
  void PlotFunction(QCustomPlot* plot, QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin);

signals:
  void GetsClosed(int idNr);

private slots:
  void on_plotComboBox_currentIndexChanged(int index);
  void on_boidSelectorSpinBox_valueChanged(int arg1);
};

#endif // PLOTMAINWINDOW_H
