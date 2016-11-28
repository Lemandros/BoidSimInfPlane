#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "plotclass.h"
#include <QDebug>
#include <QImage>
#include <QString>
#include <utility>
#include <cmath>
#include <math.h>
#include <QFileDialog>
#include "qcustomplot.h"
#include <fstream>
#include <sstream>
#include <cstring>
#include <istream>
#include <iostream>
#include <qdesktopservices.h>

#define M_2PI      6.28318530717958647692

using namespace std;



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    bool OS;
#if defined(Q_OS_LINUX)
    OS = true;
#elif defined(Q_OS_WIN)
    OS = false;
#endif

  //this->setAttribute(Qt::WA_DeleteOnClose);

  plotVec.clear( );
  // set to false for windows!
  if(OS) defaultPath = "/net/zilcken/data1/InfPlane/";
  else defaultPath = "F:\\Boids\\InfPlane\\";
  this->screencounter = 0;
  nrOfTimeStepsPerUpdate = 1;
  ui->setupUi(this);
  ui->informedAngleDoubleSpinBox->setEnabled(false);
  ui->informedGroupSelectorSpinBox->setEnabled(false);
  ui->nrOfInformedBoidsInGroupSpinBox->setEnabled(false);
  ui->updateInfBoidsPushButton->setEnabled(false);
  ui->groupInfo->setEnabled(false);
  ui->boidTrackerSpinBox->setEnabled(false);
  ui->nrOfInformedBoidsInGroupSpinBox->setMaximum(ui->nrOfBoidsSpinBox->value( ));
  qDebug( ) << "Init...";
  Init( );
  this->myTimer = new QTimer(this);
  this->myTimer->setSingleShot(true);
  connect(myTimer, SIGNAL(timeout( )), SLOT(RunSim( )));

  this->simFreqTimer = new QTimer(this);
  this->simFreqTimer->setSingleShot(this);
  connect(simFreqTimer, SIGNAL(timeout( )), SLOT(ComputeFreq( )));
  simFreqTimer->start(1000);
  qDebug( ) << "MainWindow constructor done, starting timer...";
  QTimer::singleShot(50, this, SLOT(ShowSim( )));

} // MainWindow


// should be called when MainWindow is closed
void MainWindow::closeEvent(QCloseEvent *event) {
  // close all plot windows
  qDebug( ) << "MainWindow closeEvent, now closing" << plotMainWindows.size( ) << "plots";
  for (uint i = 0; i < plotMainWindows.size( ); i++) {
    plotMainWindows[i]->close( );
    delete plotMainWindows[i];
    plotMainWindows[i] = NULL; // just in case
  } // for
} // closeEvent

void MainWindow::ClosePlotWindow(int idNr) {
  qDebug( ) << "ClosePlotWindow! idNr =" << idNr;
  for (uint i = 0; i < plotMainWindows.size( ); i++)
    if (plotMainWindows[i]->idNr == idNr) { // we got him
      delete plotMainWindows[i];
      plotMainWindows[i] = NULL; // just in case
      plotMainWindows.erase(plotMainWindows.begin( ) + i);
      return;
    } // if
} // ClosePlotWindow


MainWindow::~MainWindow( ) {
  delete ui;
}

void MainWindow::initPressed( ){
  delete this->sim;
  sim = NULL;
  Init( );
} // ~MainWindow
void MainWindow::runPausePressed(bool checked){
  if(checked)
    RunSim( );
} // runPausePressed

void MainWindow::noisePressed(bool checked){
  sim->noiseType = checked;
} // noisePressed

void MainWindow::loadPressed( ){
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open boid config"), defaultPath, tr("BoidInf Files (*.boidinf)"));
  if(!fileName.isEmpty( ))
    LoadCheckPoint(fileName);
} // loadPressed

void MainWindow::savePressed( ){
  QString fileName = QFileDialog::getSaveFileName(this, tr("Save boid config"), defaultPath,tr("BoidInf Files (*.boidInf"));
  if(!fileName.isEmpty( ))
    SaveCheckPoint(fileName);
} // savePressed



void MainWindow::Init( ){
  plotVec.clear( );
  this->sim = new BoidSim2D(ui->seedSpinBox->value( ),
                            ui->rhoSpinBox->value( ),
                            ui->nrOfBoidsSpinBox->value( ),
                            ui->initPosComboBox->currentIndex( ));
  sim->drawCross = ui->drawCrossCheckBox->isChecked( );
  sim->drawGridlines = ui->drawGridlinesCheckBox->isChecked( );
  sim->drawBoids = ui->drawBoidsCheckBox->isChecked( );
  sim->drawHull = ui->drawHullCheckBox->isChecked( );
  sim->drawHullDirections = ui->drawDirectionHullCheckBox->isChecked( );
  sim->drawHullBoids = ui->drawHullBoidsCheckBox->isChecked( );
  sim->drawPolarization = ui->drawPolarizationCheckBox->isChecked( );
  sim->drawBoidTrail = ui->drawBoidTrailCheckBox->isChecked( );
  sim->nrOfBoidsToTrack = ui->numBoidsToTrackSpinBox->value( );
  sim->nrOfTrailPoints = ui->nrOfTrailPointsSpinBox->value( );
  sim->drawBoidTrail = ui->drawBoidTrailCheckBox->isChecked( );
  sim->rotateToxAxis = ui->alignRotCheckBox->isChecked( );
  if(ui->followCenterCheckBox->isChecked( )){
    if(ui->trackComboBox->currentIndex() == 0)
      sim->trackCOM = -1;
    else sim->trackCOM = ui->boidTrackerSpinBox->value( );
  }
  else
    sim->trackCOM = -2;
//  sim->trackCOM = ui->followCenterCheckBox->isChecked( );
  sim->trackSize = ui->resizeCheckBox->isChecked( );
  sim->zoomScale = (double) ui->zoomSlider->value( ) / 100.0;
  //sim->noiseType = ui->noisePushButton->isChecked( );
  sim->noiseType = ui->actionNoise->isChecked( );
  sim->eta = ui->etaSpinBox->value( );
  sim->nrOfTimeStepsToRun = ui->nrOfTimeStepsSpinBox->value( );
  sim->vNought = ui->vNoughtSpinBox->value( );
  sim->nrOfInformedGroups = ui->nrOfInformedGroupsSpinBox->value( );
  nrOfUninformedBoids = ui->nrOfBoidsSpinBox->value( );
  sim->forceConstant = ui->forceConstantSpinBox->value( );
  sim->InitBoids(ui->initPosComboBox->currentIndex( ), ui->initDirectionComboBox->currentIndex( ));
  sim->FillBoxes( );
  sim->drawDirection = ui->drawDirectionCheckBox->isChecked( );
  sim->groups = informedGroups;
  sim->indices.resize(ui->nrOfBoidsSpinBox->value( ));
  iota(sim->indices.begin( ),sim->indices.end( ),0);
  sim->SetInformedBoids(informedGroups);
  sim->FindConvexHull( );
  sim->CalcP( );
  sim->FindArea( );
  PrintStatusBar( );
  qDebug( ) << "Done with init. Showing sim...";
  ShowSim( );
}

void MainWindow::RunSim( ) {
  for(uint T = 0;T<nrOfTimeStepsPerUpdate;T++){
    sim->NextStep( );
  }
  ShowSim( );

  PrintStatusBar( );

  // update plot windows:
  for (uint i = 0; i < plotMainWindows.size( ); i++) {
    if (plotMainWindows[i] != NULL)
      plotMainWindows[i]->UpdateWindow( );
  } // for

  if (sim->t < (uint) ui->nrOfTimeStepsSpinBox->value( ) && ui->actionRunSim->isChecked( )){//ui->runTogglePushButton->isChecked( )) {
    myTimer->start(1);
  } // if
} // RunSim

void MainWindow::PrintStatusBar( ){
  BoidSim2D::Vector2D P = sim->p.back( );
  double v = P.Length( );
  QString statusBarText;
  statusBarText = "v = " + QString::number(v,'g',6) + ", vX = " + QString::number(P.x,'g', 6) + ", vY = " + QString::number(P.y,'g',6);
  statusBarText += ", A = " + QString::number(sim->convHullArea,'g',6);
  statusBarText += ", nH = " + QString::number((double) sim->nrOfBoidsOnConvHull/ (double) sim->nrOfBoids,'g',6);
  ui->statusBar->showMessage(statusBarText);


}// PrintStatusBar

// converts sim to QImage and shows it on screen
void MainWindow::ShowSim( ) {
  uint len = min(ui->imgLabel->width( ), ui->imgLabel->height( )) - 10;
  QImage img = sim->ToQImage(len, ui->antiAliasingCheckBox->isChecked( ));
  ui->imgLabel->setPixmap(QPixmap::fromImage(img));

  ui->simStepLabel->setText("Simulation step: " + QString::number(sim->t));
  if(ui->saveFramesCheckBox->isChecked( )){
    QString fileName = defaultPath + "frame_" + QString::number(screencounter).rightJustified(5, '0') + ".png";
    img.save(fileName);
    screencounter++;
  } // if saveFrames

} // ShowSim

// updates variable nrOfUninformedBoids somehow...
void MainWindow::CalcNrOfUninformedBoids( ){
  nrOfUninformedBoids = ui->nrOfBoidsSpinBox->value( );
  uint nrOfInformedBoids = 0;
  for(uint i = 0;i < informedGroups.size( );i++){
    nrOfInformedBoids += informedGroups[i].first;
  }
  nrOfUninformedBoids = ui->nrOfBoidsSpinBox->value( ) - nrOfInformedBoids;
} // CalcNrOfUninformedBoids


void MainWindow::updateText( ) {
  QString text = "";
  for (uint i = 0; i < informedGroups.size( ); i++){
    text.append("Group: ");
    text.append(QString::number(i+1));
    text.append(" Boids: ");
    text.append(QString::number(informedGroups[i].first));
    text.append(" Direction: ");
    text.append(QString::number(informedGroups[i].second));
    text.append("\n");
  } // for
  text.append("Uninformed boids: ");
  text.append(QString::number(nrOfUninformedBoids));
  ui->groupInfo->setText(text);
} // updateText

// updates the spinboxes if number of informed groups changes
void MainWindow::on_nrOfInformedGroupsSpinBox_valueChanged(int arg1) {
  if (arg1 == 0) {
    ui->informedAngleDoubleSpinBox->setEnabled(false);
    ui->informedGroupSelectorSpinBox->setEnabled(false);
    ui->nrOfInformedBoidsInGroupSpinBox->setEnabled(false);
    ui->updateInfBoidsPushButton->setEnabled(false);
    ui->groupInfo->setEnabled(false);
    ui->informedGroupSelectorSpinBox->setValue(1);
    ui->nrOfInformedBoidsInGroupSpinBox->setValue(0);
    ui->informedAngleDoubleSpinBox->setValue(0.0);

  } // if
  else {
    ui->informedAngleDoubleSpinBox->setEnabled(true);
    ui->informedGroupSelectorSpinBox->setEnabled(true);
    ui->nrOfInformedBoidsInGroupSpinBox->setEnabled(true);
    ui->updateInfBoidsPushButton->setEnabled(true);
    ui->groupInfo->setEnabled(true);
  } // else

  // if nr of informed groups decreases, remove elements from groups vector
  // and change value of the group selector if it is larger than current nr
  // of groups.

  if (arg1 < (int) informedGroups.size( )) {
    // Update nrOfUninformedBoids
    informedGroups.erase(informedGroups.begin( ) + arg1, informedGroups.end( ));
    CalcNrOfUninformedBoids( );
    if (ui->informedGroupSelectorSpinBox->value( ) > arg1 && arg1 != 0) {
      ui->informedGroupSelectorSpinBox->setValue(arg1);
      ui->nrOfInformedBoidsInGroupSpinBox->setValue(informedGroups[arg1 - 1].first);
      ui->informedAngleDoubleSpinBox->setValue(informedGroups[arg1 - 1].second);
    } // if
  } // if
  else {
    // add new groups of size 0 and angle 0
    while((int) informedGroups.size( ) < arg1)
      informedGroups.push_back(pair <uint, double> (0, 0.0));
  } // else

  // set group number selector max to number of groups
  if (arg1 != 0)
    ui->informedGroupSelectorSpinBox->setMaximum(arg1);

  updateText( );
} // on_nrOfInformedGroupsSpinBox_valueChanged

// updates angle of selected informed group
void MainWindow::on_informedAngleDoubleSpinBox_valueChanged(double arg1) {
  informedGroups[ui->informedGroupSelectorSpinBox->value( ) - 1].second = arg1;
  updateText( );
} // on_informedAngleDoubleSpinBox_valueChanged

// updates values of spinboxes for appropriate group
void MainWindow::on_informedGroupSelectorSpinBox_valueChanged(int arg1) {
  arg1--;
  ui->nrOfInformedBoidsInGroupSpinBox->setValue(informedGroups[arg1].first);
  ui->informedAngleDoubleSpinBox->setValue(informedGroups[arg1].second);
  updateText( );
} // on_informedGroupSelectorSpinBox_valueChanged

 // updates number of boids in a given informed group
void MainWindow::on_nrOfInformedBoidsInGroupSpinBox_valueChanged(int arg1) {
  informedGroups[ui->informedGroupSelectorSpinBox->value( ) - 1].first = arg1;
  CalcNrOfUninformedBoids( );
  updateText( );
} // on_nrOfInformedBoidsInGroupSpinBox_valueChanged

void MainWindow::on_nrOfTimeStepsPerUpdateSpinBox_valueChanged(int arg1) {
  nrOfTimeStepsPerUpdate = arg1;
  updateText( );
} // on_nrOfTimeStepsPerUpdateSpinBox_valueChanged

void MainWindow::ComputeFreq( ) {
  time_t t = clock( );
  double dt = double(t - prevStepTime.second) / CLOCKS_PER_SEC;
  double dStep = sim->t - prevStepTime.first;
  double freq = int(10 * dStep / dt) / 10.0;

  ui->freqLabel->setText("Steps per second: " + QString::number(freq));
  prevStepTime = pair <uint, time_t> (sim->t, clock( ));
  simFreqTimer->start(1000);
} // ComputeFreq

void MainWindow::on_forceConstantSpinBox_valueChanged(double arg1) {
  sim->forceConstant = arg1;
} // on_forceConstantSpinBox_valueChanged



void MainWindow::resizeEvent(QResizeEvent *event){
  ReDrawOnInterfaceChange( );
} // resizeEvent

void MainWindow::SaveCheckPoint(QString fileName){
  ofstream file(fileName.toStdString( ));
  file.precision(10);
  file.setf(ios::fixed, ios::floatfield);
  file << "# Boids\tRho\tEta\tScalar\tvNought\tSeed\tTimesteps\tGroups\tInitPos\tInitDir\tGamma\tangularPDF" << endl;

  file  << "# " << sim->nrOfBoids << "\t" << sim->rho << "\t" << sim->eta << "\t"
        << sim->noiseType << "\t\t" << sim->vNought << "\t" << sim->seed << "\t\t"
        << sim->t << "\t\t" << informedGroups.size( ) << "\t"
        << ui->initPosComboBox->currentIndex( ) << "\t" << ui->initDirectionComboBox->currentIndex( ) << "\t"
        << ui->forceConstantSpinBox->value( ) << "\t" << sim->angHist.size( )<< endl << "# ";

  for(uint i = 0; i < informedGroups.size( ); i++)
    file << informedGroups[i].first << "\t" << informedGroups[i].second << "\t";
  file << endl;

  for(uint i = 0; i < sim->boids.size( ); i++)
    file << sim->boids[i].r.x <<  "\t" << sim->boids[i].r.y << "\t"
         << sim->boids[i].v.x << "\t" << sim->boids[i].v.y << "\t" << sim->boids[i].prefAngle << endl;
  file << endl;

  for(uint i = 0; i < sim->t; i++)
    file << sim->p[i].x << "\t" << sim->p[i].y << "\t" << sim->convHullAreaVec[i] << "\t"
         << sim->pBulk[i].x << "\t" << sim->pBulk[i].y << "\t"
         << sim->pHull[i].x << "\t" << sim->pHull[i].y << endl ;

  for(uint i = 0; i < sim->angHist.size( ); i++)
      file << sim->angHist[i] << "\t" << sim->componentHist[i].first << "\t" << sim->componentHist[i].second << "\t"
           << sim->dAngleHist[i].first << "\t" << sim->dAngleHist[i].second << endl ;

  file << endl;
  file.close( );
} // saveCheckPoint

void MainWindow::LoadCheckPoint(QString fileName){
  ifstream file(fileName.toStdString( ));

  string header;
  getline(file, header);

  string info;
  getline(file, info);
  istringstream lineStream1(info);
  string hash;
  lineStream1 >> hash;
  uint nrOfBoids;
  lineStream1 >> nrOfBoids;
  ui->nrOfBoidsSpinBox->setValue(nrOfBoids);
  qDebug( ) << "N" << nrOfBoids;
  double rho;
  lineStream1 >> rho;
  ui->rhoSpinBox->setValue(rho);
  qDebug( ) << "Rho" << rho;
  double eta;
  lineStream1 >> eta;
  ui->etaSpinBox->setValue(eta);
  qDebug( ) << "Eta" << eta;
  bool noiseType;
  lineStream1 >> noiseType;
  ui->actionNoise->setChecked(noiseType);
  //ui->noisePushButton->setChecked(noiseType);
  double vNought;
  lineStream1 >> vNought;
  ui->vNoughtSpinBox->setValue(vNought);
  int seed;
  lineStream1 >> seed;
  ui->seedSpinBox->setValue(seed);
  uint timesteps;
  lineStream1 >> timesteps;
  sim->t = timesteps;
  uint groups;
  lineStream1 >> groups;
  ui->nrOfInformedGroupsSpinBox->setValue(groups);
  int posInfo;
  lineStream1 >> posInfo;
  ui->initPosComboBox->setCurrentIndex(posInfo);
  int dirInfo;
  lineStream1 >> dirInfo;
  ui->initDirectionComboBox->setCurrentIndex(dirInfo);
  double forceConst;
  lineStream1 >> forceConst;
  ui->forceConstantSpinBox->setValue(forceConst);
  uint nrBins;
  lineStream1 >> nrBins;

  if(nrBins != 0){
    sim->angHist.clear( );
    sim->componentHist.clear( );
    sim->dAngleHist.clear( );
  }

  string groupInfo;
  getline(file, groupInfo);
  istringstream lineStream2(groupInfo);
  lineStream2 >> hash;
  informedGroups.clear( );
  for(uint i = 0; i < groups; i++){
    ui->informedGroupSelectorSpinBox->setValue(i+1);
    uint nrOfBoidsInGroup;
    lineStream2 >> nrOfBoidsInGroup;
    ui->nrOfInformedBoidsInGroupSpinBox->setValue(nrOfBoidsInGroup);
    double angle;
    lineStream2 >> angle;
    ui->informedAngleDoubleSpinBox->setValue(angle);
  }
  ui->nrOfInformedGroupsSpinBox->setValue(groups);

  sim->boids.clear( );
  sim->avgPosGeom.Zero( );
  for(uint i = 0; i < nrOfBoids; i++){
    string boidInfo;
    getline(file, boidInfo);
    istringstream lineStream3(boidInfo);
    double rX, rY, vX, vY, prefAngle;
    int trackNr = - 10;
    lineStream3 >> rX;
    lineStream3 >> rY;
    lineStream3 >> vX;
    lineStream3 >> vY;
    lineStream3 >> prefAngle;
    lineStream3 >> trackNr;
    sim->boids.push_back(BoidSim2D::Boid (BoidSim2D::Vector2D(rX, rY), BoidSim2D::Vector2D(vX, vY), prefAngle, trackNr));
    sim->avgPosGeom += BoidSim2D::Vector2D(rX, rY);
  }
    sim->avgPosGeom /= nrOfBoids;

  string tempstring;
  getline(file, tempstring);
  sim->p.clear( );
  for(uint i = 0; i < timesteps; i++){
    string polInfo;
    getline(file, polInfo);
    istringstream lineStream4(polInfo);
    double x, y, area;
    lineStream4 >> x;
    lineStream4 >> y;
    lineStream4 >> area;
    sim->p.push_back(BoidSim2D::Vector2D(x, y));
    if(area != 0)
      sim->convHullAreaVec.push_back(area);
  } // for

  for(uint i = 0; i < nrBins; i++){
    string angleInfo;
    getline(file, angleInfo);
    istringstream lineStream4(angleInfo);
    int dAngleCount;
    double counts, parallelCount, transverseCount, dAngleSum;
    lineStream4 >> counts;
    lineStream4 >> parallelCount;
    lineStream4 >> transverseCount;
    lineStream4 >> dAngleCount;
    lineStream4 >> dAngleSum;
    sim->angHist.push_back(counts);
    sim->componentHist.push_back(pair<double, double> (parallelCount, transverseCount));
    sim->dAngleHist.push_back(pair<uint, double> (dAngleCount, dAngleSum));
  }
  file.close( );

  sim->FindGeometry( );
  sim->FindConvexHull( );
  sim->CalcP( );
  ShowSim( );
  qDebug( ) << "Checkpoint loaded!";
} // loadCheckPoint

void MainWindow::LoadKeepCurrParams(QString fileName){
  double vNought = ui->vNoughtSpinBox->value( );
  double rho = ui->rhoSpinBox->value( );
  double eta = ui->etaSpinBox->value( );
  double gamma = ui->forceConstantSpinBox->value( );
  vector <pair <uint, double>> informedGroupsTemp = informedGroups;
  LoadCheckPoint(fileName);
  ui->vNoughtSpinBox->setValue(vNought);
  ui->rhoSpinBox->setValue(rho);
  ui->etaSpinBox->setValue(eta);
  ui->forceConstantSpinBox->setValue(gamma);
  informedGroups.clear( );
  ui->nrOfInformedGroupsSpinBox->setValue(0);
  for(uint i = 0; i < informedGroupsTemp.size( ); i++){
      ui->informedGroupSelectorSpinBox->setValue(i+1);
      ui->nrOfInformedBoidsInGroupSpinBox->setValue(informedGroupsTemp[i].first);
      ui->informedAngleDoubleSpinBox->setValue(informedGroupsTemp[i].second);
  }
} // LoadKeepCurrParams

void MainWindow::on_zoomSlider_valueChanged(int value){
    sim->zoomScale = (double) value / 100.0;
    ReDrawOnInterfaceChange( );
} // on_zoomSlider_valueChanged


void MainWindow::on_followCenterCheckBox_toggled(bool checked){
  if(checked){
    if(ui->trackComboBox->currentIndex() == 0)
      sim->trackCOM = -1;
    else sim->trackCOM = ui->boidTrackerSpinBox->value( );
    ui->boidTrackerSpinBox->setEnabled(true);
    ui->trackComboBox->setEnabled(true);
  }
  else{
    sim->trackCOM = -2;
    ui->boidTrackerSpinBox->setEnabled(false);
    ui->trackComboBox->setEnabled(false);
  }
  ReDrawOnInterfaceChange( );
} // on_followCenterCheckBox_toggled

void MainWindow::on_resizeCheckBox_toggled(bool checked){
    sim->trackSize = checked;
} // on_resizeCheckBox_toggled

void MainWindow::on_drawHullCheckBox_toggled(bool checked){
    sim->drawHull = checked;
    ReDrawOnInterfaceChange( );
} // on_drawHullCheckbox_toggled

void MainWindow::on_drawHullBoidsCheckBox_toggled(bool checked){
    sim->drawHullBoids = checked;
    ReDrawOnInterfaceChange( );
} // on_drawHullBoidsCheckBox_toggled

void MainWindow::on_drawGridlinesCheckBox_toggled(bool checked){
    sim->drawGridlines = checked;
    ReDrawOnInterfaceChange( );
} // on_drawGridlinesCheckBox_toggled

void MainWindow::on_drawPolarizationCheckBox_toggled(bool checked){
    sim->drawPolarization = checked;
    ReDrawOnInterfaceChange( );
} // on_drawPolarizationCheckBox_toggled

void MainWindow::on_resetViewPusButton_clicked( ){
    sim->xDrawOffset = 0;
    sim->yDrawOffset = 0;
    sim->FindGeometry( );
    if(ui->zoomSlider->value( ) != 100) ui->zoomSlider->setValue(100);
    else ReDrawOnInterfaceChange( );
} // on_resetViewPusButton_clicked

void MainWindow::on_leftArrowPushButton_clicked( ){
    sim->xDrawOffset++;
    ReDrawOnInterfaceChange( );
} // on_leftArrowPushButton_clicked

void MainWindow::on_rightArrowPushButton_clicked( ){
    sim->xDrawOffset--;
    ReDrawOnInterfaceChange( );
} // on_rightArrowPushButton_clicked

void MainWindow::on_upArrowPushButton_clicked( ){
    sim->yDrawOffset++;
    ReDrawOnInterfaceChange( );
} // on_upArrowPushButton_clicked

void MainWindow::on_downArrowPushButton_clicked( ){
    sim->yDrawOffset--;
    ReDrawOnInterfaceChange( );
} // on_downArrowPushButton_clicked

void MainWindow::on_saveFramesCheckBox_toggled(bool checked){
   ui->framePrefixLineEdit->setEnabled(checked);
} // on_saveFramesCheckBox_toggled

void MainWindow::on_antiAliasingCheckBox_clicked( ){
    ReDrawOnInterfaceChange( );
} // on_antiAliasingCheckBox_clicked

QString MainWindow::FileName( ){
  ostringstream datFileName;

  datFileName.str("");
  datFileName.clear( );
  datFileName.precision(4);
  datFileName.setf(ios::fixed, ios::floatfield);
  datFileName << defaultPath.toStdString( ) << "BoidPlots_";

  switch(ui->initPosComboBox->currentIndex( )){
    case 0:  datFileName << "PRa";  break;
    case 1:  datFileName << "PDi";  break;
    case 2:  datFileName << "PSq";  break;
    case 3:  datFileName << "PRi";  break;
  } // switch position init

  switch(ui->initDirectionComboBox->currentIndex( )){
    case 0:  datFileName << "DRa";  break;
    case 1:  datFileName << "DRo";  break;
    case 2:  datFileName << "DRI";  break;
    case 3:  datFileName << "DRO";  break;
  } // switch direction init

  datFileName << "F" << ui->forceConstantSpinBox->value( ); // force
  datFileName << "H" << ui->etaSpinBox->value( ); // noise
  //datFileName << "v" << ui->vNoughtSpinBox->value( ); // speed

  if(ui->nrOfInformedGroupsSpinBox->value( ) > 0){
    datFileName << "NG" << ui->nrOfInformedGroupsSpinBox->value( );
    for(int d = 0; d < ui->nrOfInformedGroupsSpinBox->value( ); d++){
        datFileName << "Group" << d << "NB" << informedGroups[d].first << "A" << informedGroups[d].second;
    } // for
  } // if
  return QString::fromStdString(datFileName.str( ));
} // FileName

void MainWindow::ReDrawOnInterfaceChange( ){
  if(ui->saveFramesCheckBox->isChecked( )){
    ui->saveFramesCheckBox->setChecked(false);
    ShowSim( );
    ui->saveFramesCheckBox->setChecked(true);
  }else
    ShowSim( );
} // ReDrawOnInterfaceChange

void MainWindow::on_drawDirectionCheckBox_toggled(bool checked){
  sim->drawDirection = checked;
  ReDrawOnInterfaceChange( );
} // on_drawDirectionCheckBox_toggled

void MainWindow::SaveData(QString datFileName){
    ofstream datFile(datFileName.toStdString( ));
    datFile.precision(8);
    datFile.setf(ios::fixed, ios::floatfield);

    datFile << "#Init pos:";

    switch(ui->initPosComboBox->currentIndex( )){
      case 0: datFile << "random, ";break;
      case 1: datFile << "disk, ";break;
      case 2: datFile << "square perimeter, ";break;
      case 3: datFile << "circle, ";break;
    } // switch position init

    switch(ui->initDirectionComboBox->currentIndex( )){
      case 0: datFile << "random, ";break;
      case 1: datFile << "rotating, ";break;
      case 2: datFile << "radially inward, ";break;
      case 3: datFile << "radially outward, ";break;
    }// switch direction init

    datFile << "rho: " << sim->rho << " N: " << sim->nrOfBoids << " Eta: " << sim->eta
            << " v0: " << sim->vNought << " gamma: " << sim->forceConstant << " seed: "
            << sim->seed;

    for(uint k = 0; k < informedGroups.size( ); k++){
      if(k == 0)
        datFile << endl << "#\t";

      datFile << k << "\t" << informedGroups[k].first << "\t" << informedGroups[k].second << "\t";
    } // for

    datFile << endl;

    for(uint k = 0; k < sim->p.size( ); k++)
        datFile << sim->p[k].Length() << "\t" << sim->p[k].x << "\t" << sim->p[k].y << "\t"
                << sim->convHullAreaVec[k] << "\t" << sim->pBulk[k].Length( ) << "\t" << sim->pBulk[k].x << "\t" << sim->pBulk[k].y << "\t"
                << sim->pHull[k].Length( ) << "\t" << sim->pHull[k].x << "\t" << sim->pHull[k].y << endl;

    datFile.close( );
}

void MainWindow::on_etaSpinBox_valueChanged(double arg1){
  sim->eta = arg1;
} // on_etaSpinBox_valueChanged

void MainWindow::on_drawBoidsCheckBox_toggled(bool checked){
  sim->drawBoids = checked;
  ReDrawOnInterfaceChange( );
} // on_drawBoidsCheckBox_toggled
void MainWindow::on_drawCrossCheckBox_toggled(bool checked){
  sim->drawCross = checked;
  ReDrawOnInterfaceChange( );
} // on_drawCrossCheckBox_toggled

void MainWindow::on_updateInfBoidsPushButton_clicked( ){
  sim->SetInformedBoids(informedGroups);
} // on_updateInfBoidsPushButton_clicked

void MainWindow::on_emptyHistsPushButton_clicked( ){
  for(int i = 0; i < sim->numBins; i++){
    sim->dAngleHist[i].first = 0;
    sim->dAngleHist[i].second = 0.0;
    sim->angHist[i] = 0;
    sim->componentHist[i].first = 0.0;
    sim->componentHist[i].second = 0.0;
  } // for
} // on_emptyHistsPushButton_clicked

void MainWindow::on_vNoughtSpinBox_valueChanged(double arg1){
  sim->vNought = arg1;
} // on_vNoughtSpinBox_valueChanged

void MainWindow::createPlot( ){

  int idNr = (plotMainWindows.size( ) == 0) ? 0 : plotMainWindows.back( )->idNr + 1;
  PlotMainWindow * plotMainWindow;
  plotMainWindow = new PlotMainWindow(this, defaultPath, sim, idNr);
  plotMainWindow->show( );

  QMainWindow::connect(plotMainWindow, SIGNAL(GetsClosed(int)), SLOT(ClosePlotWindow(int)));

  this->plotMainWindows.push_back(plotMainWindow);
    //plotMainWindow->show( );

} // createPlot

void MainWindow::on_trackComboBox_currentIndexChanged(int index){
  if(index == 1){
    sim->trackCOM = ui->boidTrackerSpinBox->value( );
    ui->boidTrackerSpinBox->setEnabled(true);
  }
  else{
    sim->trackCOM = -1;
    ui->boidTrackerSpinBox->setEnabled(false);
  }
} // on_trackComboBox_currentIndexChanged

void MainWindow::on_boidTrackerSpinBox_valueChanged(int arg1){
  sim->trackCOM = arg1;
  ReDrawOnInterfaceChange( );
} // on_boidTrackerSpinBox_valueChanged

void MainWindow::on_drawBoidTrailCheckBox_toggled(bool checked){
  sim->drawBoidTrail = checked;
  ui->boidTrailSelectorSpinBox->setEnabled(checked);
  ui->nrOfTrailPointsSpinBox->setEnabled(checked);
  if(checked)
    sim->trackBoidTrail = ui->boidTrailSelectorSpinBox->value( );
  else
    sim->trackBoidTrail = -10;
  ReDrawOnInterfaceChange( );
} // on_drawBoidTrailCheckBox_toggled

void MainWindow::on_nrOfTrailPointsSpinBox_valueChanged(int arg1){
  sim->nrOfTrailPoints = arg1;
  ReDrawOnInterfaceChange( );
} // on_nrOfTrailPointsSpinBox_valueChanged

void MainWindow::on_boidTrailSelectorSpinBox_valueChanged(int arg1){
  ReDrawOnInterfaceChange( );
  sim->trackBoidTrail = arg1;
} // on_boidTrailSelectorSpinBox_valueChanged

void MainWindow::on_numBoidsToTrackSpinBox_valueChanged(int arg1){
  ui->boidTrailSelectorSpinBox->setMaximum(arg1-1);
} // on_numBoidsToTrackSpinBox_valueChanged

void MainWindow::on_alignRotCheckBox_toggled(bool checked){
  sim->rotateToxAxis = checked;
  ReDrawOnInterfaceChange( );
} // on_alignRotCheckBox_toggled

void MainWindow::scriptPressed( ){
  int timestepstorun = 100000;
  int numIncA = 1;
  int numIncB = 41;
  int stepsPerUpdate = 100;

  ui->etaSpinBox->setValue(0.1);
  ui->initPosComboBox->setCurrentIndex(1);
  ui->nrOfBoidsSpinBox->setValue(2000);
QString oldFileName = "/net/zilcken/data1/InfPlane/VarForceEta0.10/Rotating/BoidPlots_PDiDRaF10.0000H0.1000.boidInf";
oldFileName = "/net/zilcken/data1/InfPlane/BoidPlots_PDiDRaF10.0000H0.1000.boidInf";
  for(int i = 0; i < numIncA; i++){
    //ui->etaSpinBox->setValue(0.1 + i * 0.05);
    //ui->informedAngleDoubleSpinBox->setValue((double) i / 100.0);
    //ui->forceConstantSpinBox->setValue(1.0 + double(i) * 4.0);
    for(int j = 1; j < numIncB; j++){
      ui->forceConstantSpinBox->setValue(10.0 - double (j) * 0.25);
      //ui->forceConstantSpinBox->setValue(10.0 * pow(10, 2.0 * double(j)/21));
      //ui->etaSpinBox->setValue( + double(j) * 0.025);
      qDebug( ) << "Start init...";
      Init( );
      qDebug( ) << "Done with init";

      qDebug( ) << "Loading checkpoint...";
      LoadKeepCurrParams(oldFileName);

      sim->p.clear( );
      sim->convHullAreaVec.clear( );
      sim->t = 0;
      ShowSim( );
      for(int k = 0; k < timestepstorun;k++){
        sim->NextStep( );

        if(k%stepsPerUpdate == 0){
          if (sim->convHullArea >= 10000.0) {
            k = 0;
            Init( );
            qDebug( ) << "Diverging! Restarting";
          }
          PrintStatusBar( );
          ShowSim( );
          qDebug( ) << i << j << sim->t << sim->eta << sim->rho << sim->forceConstant << sim->convHullArea;
        } // if
      } // for k
      qDebug( ) << "Done with time loop, i" << i << "j" << j;
      QString datFileName = FileName( );
      oldFileName = datFileName + ".boidInf";
      SaveData(datFileName + ".dat");
      SaveCheckPoint(datFileName + ".boidInf");
    } // for j
    qDebug( ) << "Done with inner loop, i" << i;
  } // for i
} // scriptPressed

void MainWindow::on_drawDirectionHullCheckBox_toggled(bool checked){
  sim->drawHullDirections = checked;
  ReDrawOnInterfaceChange( );
} // on_drawDirectionHullCheckBox_toggled
