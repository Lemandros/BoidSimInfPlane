#include "plotmainwindow.h"
#include "ui_plotmainwindow.h"
#include <FFTw/include/fftw3.h>
#include <QDebug>
#include <fstream>
#include <cmath>
#include <boidsim2d.h>
#include <QString>
#define M_2PI      6.28318530717958647692


PlotMainWindow::PlotMainWindow(QWidget *parent, QString defaultPath, BoidSim2D *sim, int idNr) :
  QMainWindow(parent),
  ui(new Ui::PlotMainWindow) {
  ui->setupUi(this);
  this->setAttribute(Qt::WA_DeleteOnClose);

  this->idNr = idNr;
  this->sim = sim;
  this->defaultPath = defaultPath;
  this->parent = parent;

  this->setWindowTitle("Plot " + QString::number(idNr));
  this->ui->boidSelectorSpinBox->setVisible(false);
  this->ui->boidSelectorSpinBox->setMaximum(sim->nrOfBoidsToTrack - 1);
  this->ui->cumBeginSpinBox->setVisible(false);
  this->ui->cumBeginSpinBox->setMaximum(sim->t-1);
  on_plotPlotAllPushButton_clicked( );
  // qDebug( ) << "PlotMainWindow aangemaakt!" << idNr;
} // PlotMainWindow

PlotMainWindow::~PlotMainWindow( ) {
  //qDebug( ) << "PlotMainwWindow destroyed";
  delete ui;
} // ~PlotMainWindow

void PlotMainWindow::closeEvent(QCloseEvent *event) {
  qDebug( ) << "PlotMainwWindow" << idNr << "is getting closed!";
  this->ui->plotWidget->clearGraphs( );
  for(uint i = 0; i < fourierWindows.size( ); i++)
    CloseFourierWindow(fourierWindows[i]->idNr);
  emit PlotGetsClosed(idNr);
} // closeEvent

void PlotMainWindow::UpdateWindow( ) {
  ui->cumBeginSpinBox->setMaximum(sim->t-1);
  ui->plotMaxSpinBox->setMaximum(sim->t);
  if(ui->updateCheckBox->isChecked( ))
    on_plotPlotAllPushButton_clicked( );
} // UpdateWindow


void PlotMainWindow::on_printPushButton_clicked( ){
  PrintFunction(ui->plotComboBox, ui->plotMinSpinBox, ui->plotMaxSpinBox, ui->plotLineEdit->text( ));
} // on_printPushButton_clicked

void PlotMainWindow::on_plotPlotAllPushButton_clicked( ){
  PlotAll(ui->plotWidget, ui->plotComboBox, ui->plotMinSpinBox, ui->plotMaxSpinBox);
}

void PlotMainWindow::on_plotLogxCheckBox_toggled(bool checked){
  PlotSetLog(true, checked, ui->plotWidget);
} // on_plotLogxCheckBox_toggled
void PlotMainWindow::on_plotLogyCheckBox_toggled(bool checked){
  PlotSetLog(false, checked, ui->plotWidget);
} // on_plotLogyCheckBox_toggled

void PlotMainWindow::PlotSetLog(bool axis, bool checked, QCustomPlot* plot){
  if(axis){
      if(checked){
        QSharedPointer<QCPAxisTickerLog> logTicker(new QCPAxisTickerLog);
        plot->xAxis->setTicker(logTicker);
        plot->xAxis->setScaleType(QCPAxis::stLogarithmic);
        plot->xAxis->setNumberPrecision(0);
        plot->xAxis->setNumberFormat("eb");
      } // if checked
      else{
        QSharedPointer<QCPAxisTicker> normTicker(new QCPAxisTicker);
        plot->xAxis->setTicker(normTicker);
        plot->xAxis->setScaleType(QCPAxis::stLinear);
        plot->xAxis->setNumberPrecision(8);
        plot->xAxis->setNumberFormat("gb");
      } // else checked
  } // if axis
  else{
    if(checked){
      QSharedPointer<QCPAxisTickerLog> logTicker(new QCPAxisTickerLog);
      plot->yAxis->setTicker(logTicker);
      plot->yAxis->setScaleType(QCPAxis::stLogarithmic);
      plot->yAxis->setNumberPrecision(0);
      plot->yAxis->setNumberFormat("eb");
    } // if checked
    else{
      QSharedPointer<QCPAxisTicker> normTicker(new QCPAxisTicker);
      plot->yAxis->setTicker(normTicker);
      plot->yAxis->setScaleType(QCPAxis::stLinear);
      plot->yAxis->setNumberPrecision(8);
      plot->yAxis->setNumberFormat("gb");
    } // else checked
  }// else axis
  plot->replot( );
} // PlotSetLog

void PlotMainWindow::on_plotPushButton_clicked( ){
  PlotFunction(ui->plotWidget, ui->plotComboBox, ui->plotMinSpinBox, ui->plotMaxSpinBox);
}//on_plotPushButton_clicked

void PlotMainWindow::on_plotMinSpinBox_valueChanged(int arg1){
  ui->plotMaxSpinBox->setMinimum(arg1 + 1);

  if(ui->plotMinSpinBox->hasFocus( ))
      on_plotPushButton_clicked( );
}//on_plotminSpinBox_valueChanged

void PlotMainWindow::on_plotMaxSpinBox_valueChanged(int arg1){
  ui->plotMinSpinBox->setMaximum(max(arg1 - 2,0));

  ui->plotMaxSpinBox->setMaximum(sim->t);

  if(ui->plotMaxSpinBox->hasFocus( ))
    on_plotPushButton_clicked( );
}//on_plotmaxSpinBox_valueChanged

void PlotMainWindow::on_plotMaxPushButton_clicked( ){
  if(ui->plotComboBox->currentIndex( ) > 9 && ui->plotComboBox->currentIndex( ) < 16)
    ui->plotMaxSpinBox->setValue(sim->t-ui->cumBeginSpinBox->value( ));
  else
    ui->plotMaxSpinBox->setValue(sim->t);
} // on_plotmaxPushButton_clicked

// prints raw data of an observable to plaintext file
void PlotMainWindow::PrintFunction(QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin, QString fileName){
    // determine path and filename
    ofstream file((defaultPath + comboBox->currentText( ) + "_" + fileName + ".dat").toStdString( ));

    // check what kind of data to output to file and do so
    if(comboBox->currentIndex( ) < 10){
      for(uint i = minSpin->value( ); i < maxSpin->value( ); i++){
        switch(comboBox->currentIndex( )){
          case 0:
            file << i << "\t" << sim->convHullAreaVec[i] << endl;
            break;
          case 1:
            file << i << "\t" << sim->curvatureVec[i] << endl;
            break;
          case 2:
            file << i << "\t" << sim->curvatureBulkVec[i] << endl;
            break;
          case 3:
            file << i << "\t" << sim->curvatureHullVec[i] << endl;
            break;
          case 4:
            file << i << "\t" << sim->p[i].Length( ) << endl;
            break;
          case 5:
            file << i << "\t" << sim->p[i].x << "\t" << sim->p[i].y << endl;
            break;
          case 6:
            file << i << "\t" << sim->pBulk[i].Length( ) << endl;
            break;
          case 7:
            file << i << "\t" << sim->pBulk[i].x << "\t" << sim->pBulk[i].y << endl;
            break;
          case 8:
            file << i << "\t" << sim->pHull[i].Length( ) << endl;
            break;
          case 9:
            file << i << "\t" << sim->pHull[i].x << "\t" << sim->pHull[i].y << endl;
           break;
          case 10:
            file << i << "\t" << sim->anglesHistory[ui->boidSelectorSpinBox->value( )][i].x
                      << "\t" << sim->anglesHistory[ui->boidSelectorSpinBox->value( )][i].y << endl;
        } // switch
      } // for
    } // if 10
    else if (comboBox->currentIndex( ) == 11)
      for (uint i = 0; i < this->sim->numBins; i++)
        file << double(i) / sim->numBins * M_2PI - M_PI << "\t" << this->sim->angHist[i] << endl;

    else if(comboBox->currentIndex( ) == 12)
      for(uint i = 0; i < sim->numBins; i++)
        file << double(i) / sim->numBins * M_2PI - M_PI << "\t" << sim->componentHist[i].first / sim->angHist[i] << "\t"
             << double(i) / sim->numBins * M_2PI - M_PI << "\t" << sim->componentHist[i].second / sim->angHist[i] << endl;

    else if (comboBox->currentIndex( ) == 13)
      for (uint i = 0; i < sim->dAngleHist.size( ); i++)
        file << double(i)/sim->dAngleHist.size( ) * M_2PI - M_PI << "\t" <<  sim->dAngleHist[i].second / sim->dAngleHist[i].first << endl;
    file.close( );
} // PrintFunction

void PlotMainWindow::PlotAll(QCustomPlot* plot, QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin){
  int tempMin = minSpin->value( );
  int tempMax = maxSpin->value( );
  minSpin->setValue(0);
  maxSpin->setValue(sim->t-1);
  PlotFunction(plot, comboBox, minSpin, maxSpin);
  minSpin->setValue(tempMin);
  maxSpin->setValue(tempMax);
} // PlotAll

// plot the raw data in one of the 4 plots on the plot tab
void PlotMainWindow::PlotFunction(QCustomPlot* plot, QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin){
  int min = minSpin->value( );
  int max = maxSpin->value( );
  int cumBegin = ui->cumBeginSpinBox->value( ) - 1;
  plot->clearGraphs( );

  if(comboBox->currentIndex( ) < 18){
    plot->addGraph( );
    if(comboBox->currentIndex( ) == 9 || comboBox->currentIndex( ) == 5 || comboBox->currentIndex( ) == 7){ // components of Pol
      plot->addGraph( );
      plot->graph(0)->setPen(QPen(Qt::red)); // colour x-component red
    }
    plot->xAxis->setTicker(QSharedPointer<QCPAxisTicker>(new QCPAxisTicker)); // set x axis to normal tickers

    if(comboBox->currentIndex( ) == 17){
      plot->graph( )->setLineStyle(QCPGraph::lsNone);
      plot->graph( )->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,QPen(Qt::NoPen),QBrush(Qt::NoBrush),1.0));
    } // if

    for(int i = min; i < max; i++){
      switch(comboBox->currentIndex( )){
      case 0:
        plot->graph( )->addData(i, sim->convHullAreaVec[i]);
        break;
      case 1:
        plot->graph( )->addData(i, abs(sim->curvatureVec[i]));
        break;
      case 2:
        plot->graph( )->addData(i, abs(sim->curvatureBulkVec[i]));
        break;
      case 3:
        plot->graph( )->addData(i, abs(sim->curvatureHullVec[i]));
        break;
      case 4:
        plot->graph( )->addData(i, sim->p[i].Length( ));
        break;
      case 5:
        plot->graph(0)->addData(i, sim->p[i].x);
        plot->graph(1)->addData(i, sim->p[i].y);
        break;
      case 6:
        plot->graph( )->addData(i, sim->pBulk[i].Length( ));
        break;
      case 7:
        plot->graph(0)->addData(i, sim->pBulk[i].x);
        plot->graph(1)->addData(i, sim->pBulk[i].y);
        break;
      case 8:
        plot->graph( )->addData(i, sim->pHull[i].Length( ));
        break;
      case 9:
        plot->graph(0)->addData(i, sim->pHull[i].x);
        plot->graph(1)->addData(i, sim->pHull[i].y);
        break;
      case 10:
        plot->graph( )->addData(i, sim->polCumAvg[i] / (double (i-cumBegin)) );
        break;
      case 11:
        plot->graph( )->addData(i, sim->polBulkCumAvg[i] / (double (i-cumBegin)) );
        break;
      case 12:
        plot->graph( )->addData(i, sim->polHullCumAvg[i] / (double (i-cumBegin)) );
        break;
      case 13:
        plot->graph( )->addData(i, abs(sim->curvCumAvg[i] / (double (i-cumBegin)) ) );
        break;
      case 14:
        plot->graph( )->addData(i, abs(sim->curvBulkCumAvg[i] / (double (i-cumBegin)) ));
        break;
      case 15:
        plot->graph( )->addData(i, abs(sim->curvHullCumAvg[i] / (double (i-cumBegin)) ));
        break;
      case 16:
        plot->graph( )->addData(i,sim->numBoidsOnHull[i]);
        break;
      case 17:        //  plot->graph( )->addData(i, sim->anglesHistory[ui->boidSelectorSpinBox->value( )][i]);
        plot->graph( )->addData(sim->anglesHistory[ui->boidSelectorSpinBox->value( )][i].x, sim->anglesHistory[ui->boidSelectorSpinBox->value( )][i].y);
        break;
      } // switch
    } // for
    plot->rescaleAxes( );
    if(comboBox->currentIndex( ) == 16)
      plot->yAxis->setRangeLower(0.0);
  } // if
  else if (comboBox->currentIndex( ) == 18){ // angular histogram of boids on the hull
    plot->addGraph( );
    for (uint i = 0; i < this->sim->numBins; i++)
      plot->graph( )->addData(double(i) / sim->numBins * M_2PI - M_PI, this->sim->angHist[i]);
    plot->rescaleAxes( );
    plot->xAxis->setTicker(QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi));
    plot->yAxis->setRangeLower(0);
  } // else if
  else if(comboBox->currentIndex( ) == 19){ // component histogram of boids on the hull
    plot->addGraph( );
    plot->addGraph( );
    plot->graph(0)->setPen(QPen(Qt::red));
    plot->xAxis->setTicker(QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi));
    for(uint i = 0; i < sim->numBins; i++){
      plot->graph(0)->addData(double(i) / sim->numBins * M_2PI - M_PI, sim->componentHist[i].first / double(sim->angHist[i]));
      plot->graph(1)->addData(double(i) / sim->numBins * M_2PI - M_PI, sim->componentHist[i].second / double(sim->angHist[i]));
    } // for
    plot->rescaleAxes( );
    plot->yAxis->setRangeLower(-1.0);
  } //  else if
  else if (comboBox->currentIndex( ) == 20) { // histogram of change in angle on the hull
    plot->addGraph( );
    plot->xAxis->setTicker(QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi));
    for (uint i = 0; i < sim->dAngleHist.size( ); i++) {
      plot->graph( )->addData(double(i)/sim->dAngleHist.size( ) * M_2PI - M_PI, sim->dAngleHist[i].second / double(sim->dAngleHist[i].first));
    } // for
    plot->rescaleAxes( );
  } // else if
  else if (comboBox->currentIndex( ) == 21) { // histogram of distance from geometric center
    plot->addGraph( );
    plot->xAxis->setTicker(QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi));
    for (uint i = 0; i < sim->radiusHist.size( ); i++) {
      plot->graph( )->addData(double(i)/sim->radiusHist.size( ) * M_2PI - M_PI, sim->radiusHist[i] / double(sim->angHist[i]));
    } // for
    plot->yAxis->setRangeLower(-1.0);
    plot->rescaleAxes( );
  } // else if

  plot->replot( );

  if(comboBox->currentIndex( ) < 17){
    double y1Min = 0.0;
    double y1Max = 0.0;
    double y2Min = 0.0;
    double y2Max = 0.0;
    switch(comboBox->currentIndex( )){
    case 0:
      y1Min = sim->convHullAreaVec[min];
      y1Max = sim->convHullAreaVec[max];
      break;
    case 1:
      y1Min = abs(sim->curvatureVec[min]);
      y1Max = abs(sim->curvatureVec[max]);
      break;
    case 2:
      y1Min = abs(sim->curvatureBulkVec[min]);
      y1Max = abs(sim->curvatureBulkVec[max]);
      break;
    case 3:
      y1Min = abs(sim->curvatureHullVec[min]);
      y1Max = abs(sim->curvatureHullVec[max]);
      break;
    case 4:
      y1Min = sim->p[min].Length( );
      y1Max = sim->p[max].Length( );
      break;
    case 5:
      y1Min = sim->p[min].x;
      y1Max = sim->p[max].x;
      y2Min = sim->p[min].y;
      y2Max = sim->p[max].y;
      break;
    case 6:
      y1Min = sim->pBulk[min].Length( );
      y1Max = sim->pBulk[max].Length( );
      break;
    case 7:
      y1Min = sim->pBulk[min].x;
      y1Max = sim->pBulk[max].x;
      y2Min = sim->pBulk[min].y;
      y2Max = sim->pBulk[max].y;
      break;
    case 8:
      y1Min = sim->pHull[min].Length( );
      y1Max = sim->pHull[max].Length( );
      break;
    case 9:
      y1Min = sim->pHull[min].x;
      y1Max = sim->pHull[max].x;
      y2Min = sim->pHull[min].y;
      y2Max = sim->pHull[max].y;
      break;
    case 10:
      y1Min = sim->polCumAvg[min] / double(min + 1);
      y1Max = sim->polCumAvg[max] / double(max + 1);
      break;
    case 11:
      y1Min = sim->polBulkCumAvg[min] / double(min + 1);
      y1Max = sim->polBulkCumAvg[max] / double(max + 1);
      break;
    case 12:
      y1Min = sim->polHullCumAvg[min] / double(min + 1);
      y1Max = sim->polHullCumAvg[max] / double(max + 1);
      break;
    case 13:
      y1Min = sim->curvCumAvg[min] / double(min + 1);
      y1Max = sim->curvCumAvg[max] / double(max + 1);
      break;
    case 14:
      y1Min = sim->curvBulkCumAvg[min] / double(min + 1);
      y1Max = sim->curvBulkCumAvg[max] / double(max + 1);
      break;
    case 15:
      y1Min = sim->curvBulkCumAvg[min] / double(min + 1);
      y1Max = sim->curvBulkCumAvg[max] / double(max + 1);
      break;
    case 16:
      y1Min = sim->numBoidsOnHull[min];
      y1Max = sim->numBoidsOnHull[max];
      break;
    } // switch
    ui->statusbar->showMessage("y1 = " + QString::number(y1Min) + " Y1 = " + QString::number(y1Max) + " y2 = " + QString::number(y2Min) + " Y2 = " + QString::number(y2Max));
  }
} // plotFunction

void PlotMainWindow::on_plotComboBox_currentIndexChanged(int index){
  if(index == 17)
    ui->boidSelectorSpinBox->setVisible(true);
  else
    ui->boidSelectorSpinBox->setVisible(false);
  if(index > 17){
    ui->plotMaxPushButton->setVisible(false);
    ui->plotMaxSpinBox->setVisible(false);
    ui->plotMinSpinBox->setVisible(false);
    ui->plotPlotAllPushButton->setVisible(false);
    ui->fftPushButton->setVisible(false);
  }
  else{
    ui->fftPushButton->setVisible(true);
    ui->plotMaxPushButton->setVisible(true);
    ui->plotMaxSpinBox->setVisible(true);
    ui->plotMinSpinBox->setVisible(true);
    ui->plotPlotAllPushButton->setVisible(true);
  }
  if(index > 9 && index < 16)
    ui->cumBeginSpinBox->setVisible(true);
  else{
    ui->cumBeginSpinBox->setVisible(false);
    ui->cumBeginSpinBox->setValue(0);
  }
} // on_plotComboBox_currentIndexChanged

void PlotMainWindow::on_boidSelectorSpinBox_valueChanged(int arg1){
    on_plotPlotAllPushButton_clicked( );
} // on_boidSelectorSpinBox_valuechanged


void PlotMainWindow::createFourier( ){
  int fftIdNr = (fourierWindows.size( ) == 0) ? 0 : fourierWindows.back( )->idNr + 1;
  FFT * fourierWindow;
  int type = 0;
  if(ui->plotComboBox->currentIndex( ) == 4 || ui->plotComboBox->currentIndex( )  == 5) type = 0;
  if(ui->plotComboBox->currentIndex( ) == 6 || ui->plotComboBox->currentIndex( )  == 7) type = 1;
  if(ui->plotComboBox->currentIndex( ) == 8 || ui->plotComboBox->currentIndex( )  == 9) type = 2;
  if(ui->plotComboBox->currentIndex( )  == 10) type = 3;
  int boidNr = (type == 3) ? ui->boidSelectorSpinBox->value( ) : -1;
  fourierWindow = new FFT(defaultPath, this, sim, idNr, fftIdNr, ui->plotMinSpinBox->value( ),
                          ui->plotMaxSpinBox->value( ), type, boidNr);
  fourierWindow->show( );

  QMainWindow::connect(fourierWindow, SIGNAL(FourierGetsClosed(int)), SLOT(CloseFourierWindow(int)));

  this->fourierWindows.push_back(fourierWindow);
} // createFourier

void PlotMainWindow::CloseFourierWindow(int idNr) {
  qDebug( ) << "CloseFourierWindow! idNr =" << idNr;
  for (uint i = 0; i < fourierWindows.size( ); i++)
    if (fourierWindows[i]->idNr == idNr) { // we got him
      delete fourierWindows[i];
      fourierWindows[i] = NULL; // just in case
      fourierWindows.erase(fourierWindows.begin( ) + i);
      return;
    } // if
} // ClosePlotWindow


void PlotMainWindow::on_fftPushButton_clicked( ){
    createFourier( );
} // on_fftPushButton_clicked

void PlotMainWindow::on_cumBeginSpinBox_valueChanged(int arg1){
  sim->CalcCumAvg(arg1);
  ui->plotMaxSpinBox->setMaximum(sim->t-arg1);
} // on_cumBeginSpinBox_valueChanged
