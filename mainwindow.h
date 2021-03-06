#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "boidsim2d.h"

#include <QMainWindow>
#include <QTimer>
#include <time.h>
#include "qcustomplot.h"
#include "plotmainwindow.h"
#include <vector>
#include <fft.h>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  std::vector <PlotMainWindow*> plotMainWindows;
  std::vector <FFT*> fourierWindows;
  QTimer * myTimer;
  QTimer * simFreqTimer;
  pair <uint, time_t> prevStepTime;

  BoidSim2D * sim;
  vector <pair <uint, double>> informedGroups;
  uint nrOfUninformedBoids;
  uint nrOfTimeStepsPerUpdate;
  bool firstTimeTabSwitch;
  QString defaultPath;
  int screencounter;
  explicit MainWindow(QWidget *parent = 0);
  void resizeEvent();
  ~MainWindow( );

protected:
  void closeEvent();

public slots:
  void ClosePlotWindow(int idNr);
  void CloseFourierWindow(int idNr);
  void initPressed( );
  void runPausePressed(bool checked);
  void loadPressed( );
  void savePressed( );
  void saveDataPressed( );
  void saveAllPressed( );
  void scriptPressed( );
  void createPlot( );
  void createFourier( );

private slots:
  void RunSim( );
  void CalcNrOfUninformedBoids( );

  void on_nrOfInformedGroupsSpinBox_valueChanged(int arg1);

  void on_informedGroupSelectorSpinBox_valueChanged(int arg1);

  void on_informedAngleDoubleSpinBox_valueChanged(double arg1);
  
  void on_nrOfInformedBoidsInGroupSpinBox_valueChanged(int arg1);
  
  void on_nrOfTimeStepsPerUpdateSpinBox_valueChanged(int arg1);

  void updateText( );

  void ShowSim( );

  void ComputeFreq( );
  void Init( );
  void PrintStatusBar( );

  void on_forceConstantSpinBox_valueChanged(double arg1);



  void SaveCheckPoint(QString fileName);
  void SaveData(QString fileName);
  QString FileName( );

  void LoadCheckPoint(QString fileName, bool hists);
  void LoadKeepCurrParams(QString fileName);
  void on_zoomSlider_valueChanged(int value);

  void on_followCenterCheckBox_toggled(bool checked);

  void on_resizeCheckBox_toggled(bool checked);

  void on_drawHullCheckBox_toggled(bool checked);

  void on_drawHullBoidsCheckBox_toggled(bool checked);

  void on_drawGridlinesCheckBox_toggled(bool checked);

  void on_drawPolarizationCheckBox_toggled(bool checked);

  void on_resetViewPusButton_clicked();

  void on_leftArrowPushButton_clicked();

  void on_rightArrowPushButton_clicked();

  void on_upArrowPushButton_clicked();

  void on_downArrowPushButton_clicked();

  void on_saveFramesCheckBox_toggled(bool checked);

  void on_antiAliasingCheckBox_clicked();

  void on_drawDirectionCheckBox_toggled(bool checked);
  void ReDrawOnInterfaceChange( );


  void on_etaSpinBox_valueChanged(double arg1);

  void on_drawBoidsCheckBox_toggled(bool checked);

  void on_updateInfBoidsPushButton_clicked();

  void on_emptyHistsPushButton_clicked();

  void on_vNoughtSpinBox_valueChanged(double arg1);  
  void on_trackComboBox_currentIndexChanged(int index);

  void on_boidTrackerSpinBox_valueChanged(int arg1);

  void on_drawBoidTrailCheckBox_toggled(bool checked);

  void on_nrOfTrailPointsSpinBox_valueChanged(int arg1);

  void on_boidTrailSelectorSpinBox_valueChanged(int arg1);

  void on_numBoidsToTrackSpinBox_valueChanged(int arg1);


  void on_alignRotCheckBox_toggled(bool checked);

  void on_drawDirectionHullCheckBox_toggled(bool checked);

  void WriteSettingsToFile( );
  void LoadSettingsFromFile( );
  void on_nrOfBoidsSpinBox_valueChanged(int arg1);

  void on_initPosComboBox_currentIndexChanged(int index);

  void on_initDirectionComboBox_currentIndexChanged(int index);
  void ClearHists( );
  void ClearVecs( );
  void SaveHists(QString fileName);
  void SaveAll(QString fileName);
  void saveHistsPressed( );
  void PrintHist(QString fileName);
  void on_runHistScriptPushButton_clicked( );
  void on_emptyVecsPushButton_clicked( );
  void ReadHeader(QString fileName, QTreeWidgetItem* item);
  void on_noiseTypeComboBox_currentIndexChanged(int index);

  void on_forceTypeComboBox_currentIndexChanged(int index);


  void on_insertToListPushButton_clicked( );

  void on_fileListTreeWidget_doubleClicked(const QModelIndex &index);

  void on_deleteFromListPushButton_clicked( );

  void on_loadFileListPushButton_clicked( );
  void AddCheckPointToList(QString fileName);

  void on_clearListPushButton_clicked( );

  void on_selectUpItemListPushButton_clicked( );

  void on_selectDownItemListPushButton_clicked( );

  void on_leadershipGroupBox_toggled(bool arg1);

  void on_loadFromListPushButton_clicked( );

  void on_saveFromListPushButton_clicked( );

  void on_saveAllFromListPushButton_clicked();

private:
  Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
