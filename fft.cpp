#include "fft.h"
#include "ui_fft.h"
#include <boidsim2d.h>
#include <FFTw/include/fftw3.h>
#include <qcustomplot.h>
FFT::FFT(QWidget *parent, QString defaultPath, BoidSim2D* sim, int idNr) :
  QMainWindow(parent),
  ui(new Ui::FFT)
{
  ui->setupUi(this);
  this->setAttribute(Qt::WA_DeleteOnClose);

  this->idNr = idNr;
  this->sim = sim;
  this->defaultPath = defaultPath;
  this->parent = parent;
  oldStart = -1;
  oldEnd = -1;
  this->setWindowTitle("FFT " + QString::number(idNr));
  this->ui->boidSelectorSpinBox->setVisible(false);
  this->ui->radiusCheckBox->setVisible(false);
  this->ui->boidSelectorSpinBox->setMaximum(sim->nrOfBoidsToTrack - 1);
  titles.push_back("X-Coordinate");
  titles.push_back("Y-Coordinate");
  titles.push_back("T-Coordinate");
  titles.push_back("R-Coordinate");
  ui->plotMaxSpinBox->setMaximum(sim->t);
  ui->plotMinSpinBox->setMaximum(sim->t-1);
  ui->plotMaxSpinBox->setValue(sim->t);
  ui->plotMinSpinBox->setValue(max(0, int(sim->t) - 10000));
  plotColours.clear( );
  plotColours.push_back(QPen(QColor(255,0,0)));
  plotColours.push_back(QPen(QColor(0,255,0)));
  plotColours.push_back(QPen(QColor(0,0,255)));
  plotColours.push_back(QPen(QColor(0,0,0)));
  in = (double*) fftw_malloc(sizeof(double) * 1);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *1);
  plan = fftw_plan_dft_r2c_1d(1,in,out,FFTW_ESTIMATE);
} // constructor

FFT::FFT(QString defaultPath, QWidget *parent, BoidSim2D* sim, int idNr, int plotId, int tMin, int tMax, int type, int boidNr) :
  QMainWindow(parent),
  ui(new Ui::FFT)
{
  ui->setupUi(this);
  this->setAttribute(Qt::WA_DeleteOnClose);
  oldStart = -1;
  oldEnd = -1;
  this->idNr = idNr;
  this->sim = sim;
  this->defaultPath = defaultPath;
  this->parent = parent;
  ui->xCoordCheckBox->setChecked(true);
  ui->yCoordCheckBox->setChecked(true);
  ui->thetaCheckBox->setChecked(true);
  if(type == 3)
    ui->radiusCheckBox->setChecked(true);
  ui->plotMinSpinBox->setValue(tMin);
  ui->plotMaxSpinBox->setMaximum(tMax);
  ui->plotMaxSpinBox->setValue(tMax);

  this->plotId = plotId;
  this->idNr = idNr;

  this->setWindowTitle("FFT " + QString::number(idNr) + " from Plot " + QString::number(plotId));
  this->ui->boidSelectorSpinBox->setVisible(false);
  this->ui->radiusCheckBox->setVisible(false);
  this->ui->boidSelectorSpinBox->setMaximum(sim->nrOfBoidsToTrack - 1);
  titles.push_back("X-Coordinate");
  titles.push_back("Y-Coordinate");
  titles.push_back("T-Coordinate");
  titles.push_back("R-Coordinate");
  ui->fftComboBox->setCurrentIndex(type);
  if(type == 3)
    ui->boidSelectorSpinBox->setValue(boidNr);
  plotColours.clear( );
  plotColours.push_back(QPen(QColor(255,0,0)));
  plotColours.push_back(QPen(QColor(0,255,0)));
  plotColours.push_back(QPen(QColor(0,0,255)));
  plotColours.push_back(QPen(QColor(0,0,0)));
  in = (double*) fftw_malloc(sizeof(double) * 1);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * 1);
  plan = fftw_plan_dft_r2c_1d(1, in, out, FFTW_ESTIMATE);

  on_setupPushButton_clicked( );
  on_fftPushButton_clicked( );
} // constructor


FFT::~FFT(){
  delete ui;
} // destructor

void FFT::closeEvent(QCloseEvent *event) {
  //qDebug( ) << "fourierWindow" << idNr << "is getting closed!";
  this->ui->plotWidget->clearGraphs( );

  //delete [] in;
  //delete [] out;

  inputData.clear( );
  outputData.clear( );
  fftw_destroy_plan(plan);
  fftw_free(in);
  fftw_free(out);
  fftw_cleanup( );
  emit FourierGetsClosed(idNr);
} // closeEvent

void FFT::SetUp( ){

  ui->plotWidget->clearGraphs( );

  start = ui->plotMinSpinBox->value( );
  end = ui->plotMaxSpinBox->value( );
  N = end - start + 1;
  ui->fftPlotMaxSpinBox->setMaximum(N/2 + 1);
  ui->fftPlotMaxSpinBox->setValue(N/2 + 1);
  ui->fftPlotMaxSpinBox->setMinimum(0);
  fouriers.clear( );
  fftw_free(in);
  fftw_free(out);
  in = (double*) fftw_malloc(sizeof(double) * N);
  out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) *N);

  if(ui->xCoordCheckBox->isChecked( ))
    fouriers.push_back(0);
  if(ui->yCoordCheckBox->isChecked( ))
    fouriers.push_back(1);
  if(ui->thetaCheckBox->isChecked( ))
    fouriers.push_back(2);
  if(ui->radiusCheckBox->isChecked( ))
    fouriers.push_back(3);
  outputData.clear( );
  outputData = vector <vector <double> > (4);
  plan = fftw_plan_dft_r2c_1d(N, in, out, FFTW_MEASURE);
} // SetUp

void FFT::PerformFFT(int index){
  for(uint i = 0; i < N; i++)
    in[i] = inputData[index][i];
  fftw_execute(plan);
  for(uint i = 0; i < N; i++){
    outputData[index].push_back(sqrt(out[i][0] * out[i][0] + out[i][1] * out[i][1]));
  } // for
} // PerformFFT

void FFT::PlotFFT( ){
  ui->plotWidget->clearGraphs( );
  uint min = ui->fftPlotMinSpinBox->value( );
  uint max = ui->fftPlotMaxSpinBox->value( );
  for(uint i = 0; i < fouriers.size( ); i++){
    ui->plotWidget->addGraph( );
    ui->plotWidget->graph(i)->setPen(plotColours[fouriers[i]]);
    ui->plotWidget->graph(i)->setName(titles[fouriers[i]]);

    for(uint j = min; j < max; j++)
      ui->plotWidget->graph(i)->addData(double(j), outputData[fouriers[i]][j]);
  } // for i
  ui->plotWidget->rescaleAxes( );
  ui->plotWidget->legend->setVisible(true);
  ui->plotWidget->replot( );
} // Plot

void FFT::LoadData(int index){
  inputData.clear( );
  inputData = vector <vector <double> > (4);
  BoidSim2D::Vector2D P;
  int boid = ui->boidSelectorSpinBox->value( );
  for(uint i = 0; i < N; i++){
    uint t = start + i;
    switch(index){
      case 0:
        P = sim->p[t];
        break;
      case 1:
        P = sim->pBulk[t];
        break;
      case 2:
        P = sim->pHull[t];
        break;
      case 3:
        P = sim->anglesHistory[boid][t].rotated(-sim->p[t].Theta( ));
        break;
    } // switch index
    inputData[0].push_back(P.x);
    inputData[1].push_back(P.y);
    inputData[2].push_back(P.Theta( ));
    inputData[3].push_back(P.Length( ));
  } // for
} // LoadData

void FFT::on_setupPushButton_clicked( ){
  SetUp( );
  LoadData(ui->fftComboBox->currentIndex( ));
} // on_setupPushButton_clicked

void FFT::on_fftPushButton_clicked( ){
  if(ui->plotMaxSpinBox->value( ) != oldEnd || ui->plotMinSpinBox->value( ) != oldStart){
    oldEnd = ui->plotMaxSpinBox->value( );
    oldStart = ui->plotMinSpinBox->value( );
    SetUp( );
    LoadData(ui->fftComboBox->currentIndex( ));
  }

  for(uint i = 0; i < fouriers.size( ); i++)
    PerformFFT(fouriers[i]);
  PlotFFT( );
} // on_fftPushButton_clicked

void FFT::on_printPushButton_clicked( ){
    PrintFunction(ui->fftComboBox, ui->fftLineEdit->text( ));
} // on_printPushButton_clicked

void FFT::PrintFunction(QComboBox* comboBox, QString fileName){
  // determine path and filename
  ofstream file((defaultPath + comboBox->currentText( ) + "_" + fileName + ".dat").toStdString( ));

  for(uint i = 0; i < N; i++)
    switch(fouriers.size( )){
      case 1:
        file << i << "\t" << outputData[fouriers[0]][i] << endl;
        break;
      case 2:
        file << i << "\t" << outputData[fouriers[0]][i]
             << "\t" << outputData[fouriers[1]][i] << endl;
        break;
      case 3:
        file << i << "\t" << outputData[fouriers[0]][i]
             << "\t" << outputData[fouriers[1]][i]
             << "\t" << outputData[fouriers[2]][i] << endl;
        break;
      case 4:
        file << i << "\t" << outputData[fouriers[0]][i]
             << "\t" << outputData[fouriers[1]][i]
             << "\t" << outputData[fouriers[2]][i]
             << "\t" << outputData[fouriers[3]][i] << endl;
        break;
    } // switch
} // PrintFunction

void FFT::on_logXCheckBox_clicked(bool checked){
    PlotSetLog(true, checked, ui->plotWidget);
} // on_logXCheckBox_clicked

void FFT::on_logYCheckBox_clicked(bool checked){
    PlotSetLog(false, checked, ui->plotWidget);
} // on_logYCheckBox_clicked

void FFT::PlotSetLog(bool axis, bool checked, QCustomPlot* plot){
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

void FFT::on_fftComboBox_currentIndexChanged(int index){
  if(index == 3){
    ui->boidSelectorSpinBox->setVisible(true);
    ui->radiusCheckBox->setVisible(true);
  }else{
    ui->boidSelectorSpinBox->setVisible(false);
    ui->radiusCheckBox->setVisible(false);
  }

} // on_fftComboBox_currentIndexChanged

void FFT::on_plotMaxSpinBox_valueChanged(int arg1){
  ui->plotMaxSpinBox->setMaximum(sim->t);
  ui->plotMinSpinBox->setMaximum(arg1 - 1);
} // on_plotMaxSpinBox_valueChanged

void FFT::on_plotMinSpinBox_valueChanged(int arg1){
  ui->plotMaxSpinBox->setMinimum(arg1 + 1);
} // on_plotMinSpinBox_valueChanged

void FFT::on_plotMaxPushButton_clicked( ){
  ui->plotMaxSpinBox->setMaximum(sim->t);
  ui->plotMaxSpinBox->setValue(sim->t);
} // on_plotMaxPushButton_clicked

void FFT::on_fftPlotMinSpinBox_valueChanged(int arg1){
  ui->fftPlotMaxSpinBox->setMinimum(arg1 + 1);
  PlotFFT( );
} // on_fftPlotMinSpinBox_valueChanged

void FFT::on_fftPlotMaxSpinBox_valueChanged(int arg1){
  ui->fftPlotMinSpinBox->setMaximum(arg1 - 1);
  PlotFFT( );
} // on_fftPlotMaxSpinBox_valueChanged

void FFT::on_fftMaxPushButton_clicked( ){
  ui->fftPlotMaxSpinBox->setValue(N/2+1);
  PlotFFT( );
} // on_fftMaxPushButton_clicked
