#include "plotmainwindow.h"
#include "ui_plotmainwindow.h"

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
  on_plotPlotAllPushButton_clicked( );
  // qDebug( ) << "PlotMainWindow aangemaakt!" << idNr;
} // PlotMainWindow

PlotMainWindow::~PlotMainWindow( ) {
  // qDebug( ) << "PlotMainwWindow destroyed";
  delete ui;
} // ~PlotMainWindow

void PlotMainWindow::closeEvent(QCloseEvent *event) {
  // qDebug( ) << "PlotMainwWindow" << idNr << "is getting closed!";
  this->ui->plotWidget->clearGraphs( );
  emit GetsClosed(idNr);
} // closeEvent

void PlotMainWindow::UpdateWindow( ) {
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
  ui->plotMaxSpinBox->setValue(sim->t);
} // on_plotmaxPushButton_clicked

// prints raw data of an observable to plaintext file
void PlotMainWindow::PrintFunction(QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin, QString fileName){
    // determine path and filename
    ofstream file((defaultPath + comboBox->currentText( ) + "_" + fileName + ".dat").toStdString( ));

    // check what kind of data to output to file and do so
    if(comboBox->currentIndex( ) < 10){
      for(int i = minSpin->value( ); i < maxSpin->value( ); i++){
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
      for (int i = 0; i < this->sim->numBins; i++)
        file << double(i) / sim->numBins * M_2PI - M_PI << "\t" << this->sim->angHist[i] << endl;

    else if(comboBox->currentIndex( ) == 12)
      for(int i = 0; i < sim->numBins; i++)
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
  maxSpin->setValue(sim->t);
  PlotFunction(plot, comboBox, minSpin, maxSpin);
  minSpin->setValue(tempMin);
  maxSpin->setValue(tempMax);
} // PlotAll

// plot the raw data in one of the 4 plots on the plot tab
void PlotMainWindow::PlotFunction(QCustomPlot* plot, QComboBox* comboBox, QSpinBox* minSpin, QSpinBox* maxSpin){
  plot->clearGraphs( );

  if(comboBox->currentIndex( ) < 11){
    plot->addGraph( );
    if(comboBox->currentIndex( ) == 9 || comboBox->currentIndex( ) == 5 || comboBox->currentIndex( ) == 7){
      plot->addGraph( );
      plot->graph(0)->setPen(QPen(Qt::red));
    }
    plot->xAxis->setTicker(QSharedPointer<QCPAxisTicker>(new QCPAxisTicker));
    plot->graph()->setLineStyle(QCPGraph::lsNone);
    plot->graph( )->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc,QPen(Qt::NoPen),QBrush(Qt::NoBrush),1.0));
    for(int i = minSpin->value( ); i < maxSpin->value( ); i++){
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
        //  plot->graph( )->addData(i, sim->anglesHistory[ui->boidSelectorSpinBox->value( )][i]);
        plot->graph( )->addData(sim->anglesHistory[ui->boidSelectorSpinBox->value( )][i].x, sim->anglesHistory[ui->boidSelectorSpinBox->value( )][i].y);
        break;
      } // switch
    } // for
    plot->rescaleAxes( );
  } // if
  else if (comboBox->currentIndex( ) == 11){ // angular histogram of boids on the hull
    plot->addGraph( );
    for (int i = 0; i < this->sim->numBins; i++)
      plot->graph( )->addData(double(i) / sim->numBins * M_2PI - M_PI, this->sim->angHist[i]);
    plot->rescaleAxes( );
    plot->xAxis->setTicker(QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi));
    plot->yAxis->setRangeLower(0);
  } // else if
  else if(comboBox->currentIndex( ) == 12){ // component histogram of boids on the hull
    plot->addGraph( );
    plot->addGraph( );
    plot->graph(0)->setPen(QPen(Qt::red));
    plot->xAxis->setTicker(QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi));
    for(int i = 0; i < sim->numBins; i++){
      plot->graph(0)->addData(double(i) / sim->numBins * M_2PI - M_PI, sim->componentHist[i].first / sim->angHist[i]);
      plot->graph(1)->addData(double(i) / sim->numBins * M_2PI - M_PI, sim->componentHist[i].second / sim->angHist[i]);
    } // for
    plot->rescaleAxes( );
    plot->yAxis->setRangeLower(-1.0);
  } //  else if
  else if (comboBox->currentIndex( ) == 13) { // histogram of change in angle on the hull
    plot->addGraph( );
    plot->xAxis->setTicker(QSharedPointer<QCPAxisTickerPi>(new QCPAxisTickerPi));
    for (uint i = 0; i < sim->dAngleHist.size( ); i++) {
      plot->graph( )->addData(double(i)/sim->dAngleHist.size( ) * M_2PI - M_PI, sim->dAngleHist[i].second / sim->dAngleHist[i].first);
    } // for
    plot->rescaleAxes( );
  } // else if

  plot->replot( );
} // plotFunction

void PlotMainWindow::on_plotComboBox_currentIndexChanged(int index){
  if(index == 10)
    ui->boidSelectorSpinBox->setVisible(true);
  else
    ui->boidSelectorSpinBox->setVisible(false);
} // on_plotComboBox_currentIndexChanged

void PlotMainWindow::on_boidSelectorSpinBox_valueChanged(int arg1){
    on_plotPlotAllPushButton_clicked( );
} // on_boidSelectorSpinBox_valuechanged
