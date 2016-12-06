#include "fft.h"
#include "ui_fft.h"
#include <boidsim2d.h>
#include <FFTw/include/fftw3.h>
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
} // constructor

FFT::~FFT()
{
  delete ui;

} // destructor

void FFT::closeEvent() {
  // qDebug( ) << "fourierWindow" << idNr << "is getting closed!";
  this->ui->plotWidget->clearGraphs( );

  //delete [] in;
  //delete [] out;

  inputData.clear( );
  outputData.clear( );
  fftw_destroy_plan(plan);
  fftw_free(in);
  fftw_free(out);
  fftw_cleanup( );
  emit GetsClosed(idNr);
} // closeEvent

void FFT::SetUp( ){

  ui->plotWidget->clearGraphs( );

  start = ui->plotMinSpinBox->value( );
  end = ui->plotMaxSpinBox->value( );
  N = end - start + 1;

  fouriers.clear( );
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
  for(uint i = 0; i < fouriers.size( ); i++){
    ui->plotWidget->addGraph( );
    ui->plotWidget->graph(0)->setPen(plotColours[fouriers[i]]);
    ui->plotWidget->graph(i)->setName(titles[fouriers[i]]);

    for(uint j = 0; j < N; j++)
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
        P = sim->anglesHistory[index][t].rotated(-sim->p[t].Theta( ));
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
