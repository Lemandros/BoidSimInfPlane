#ifndef FFT_H
#define FFT_H

#include <QMainWindow>
#include <QString>
#include <QWidget>
#include <QComboBox>
#include <qcustomplot.h>
#include "FFTw/include/fftw3.h"
#include "boidsim2d.h"

namespace Ui {
class FFT;
}

class FFT : public QMainWindow
{
  Q_OBJECT
  vector< vector<double> > inputData;
  vector< vector<double> > outputData;
  vector<int> fouriers;
  vector<QString> titles;
  vector<QPen> plotColours;
  double *in;
  fftw_complex *out;
  uint N; // number of points (end - start + 1)

  int start, end; // start and end points
  fftw_plan plan;

protected:
  void closeEvent(QCloseEvent *event);
public:
  int idNr;
  BoidSim2D* sim;
  QWidget* parent;
  QString defaultPath;
  explicit FFT(QWidget *parent = 0, QString defaultPath = "", BoidSim2D* sim = 0, int idNr = 0);
  ~FFT();

signals:
  void FourierGetsClosed(int idNr);

private slots:
  void PlotFFT( );
  void LoadData(int index);
  void PerformFFT(int type);
  void SetUp( );
  void on_setupPushButton_clicked( );
  void on_fftPushButton_clicked( );
  void on_printPushButton_clicked( );
  void PrintFunction(QComboBox* comboBox, QString fileName);
  void on_logXCheckBox_clicked(bool checked);

  void on_logYCheckBox_clicked(bool checked);
  void PlotSetLog(bool axis, bool checked, QCustomPlot* plot);
private:
  Ui::FFT *ui;
};

#endif // FFT_H
