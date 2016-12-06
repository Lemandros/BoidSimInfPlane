#include "boidsim2d.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <QDebug>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <omp.h>
#include <fstream>
#include <sstream>
#include <cstring>
#include <istream>
#include <iostream>
#include <qmath.h>
#include <float.h>
#define M_2PI      6.28318530717958647692

// This function creates the visualisation
QImage BoidSim2D::ToQImage(uint size, bool antiAliasing) {
  QImage img(size, size, QImage::Format_RGB888);
  QPainter qPainter(&img);


  qPainter.fillRect(QRect(0, 0, size, size), QColor(255, 255, 255));
  if(antiAliasing) qPainter.setRenderHint(QPainter::Antialiasing);

  double W = max(fabs(avgPosGeom.x-minXd), fabs(avgPosGeom.x-maxXd));
  double H = max(fabs(avgPosGeom.y-minYd), fabs(avgPosGeom.y-maxYd));

  double Lmax = 2*max(W, H);
  double drawSize;
  if(trackSize) drawSize = Lmax * 2.0 * zoomScale;
  else drawSize = drawSizeConst * zoomScale * 40.0;
  drawSizeConst = drawSize;

  Vector2D pixOffset = Vector2D(0.5 + xDrawOffset / 40.0, 0.5 + yDrawOffset / 40.0) * drawSize;
  Vector2D offset = pixOffset - COM;

  double scale = (double) size / drawSize;

  QPen boidPen;
  // draw gridlines (3 scales) and scale bars for the lower 2 scales
  if(drawGridlines){
    
    double boxOrder = log10(drawSize);
    double diff = boxOrder - floor(boxOrder);
    double xLeftPos = xDrawOffset - offset.x;
    double yLeftPos = yDrawOffset - offset.y;

    for(int k = -1; k < 2; k++){

      int boxO = floor(boxOrder) + k;

      double d = pow(10,boxO);

      int xLineStart = floor(xLeftPos / d);
      int yLineStart = floor(yLeftPos / d);

      bool xDraw = false;
      bool yDraw = false;

      boidPen.setColor(QColor(0, 0, 0, 255 * min(2.0 - diff + k, 1.0)));
      boidPen.setWidth(1.5 - diff);

      qPainter.setPen(boidPen);

      if(drawSize / d > 200) continue;

      for(int i = min(xLineStart, yLineStart); i < max(xLineStart, yLineStart) + drawSize/d + 5; i++){
        if(i == xLineStart) xDraw = true;
        if(i == yLineStart) yDraw = true;
        if(i == xLineStart + drawSize/d + 5) xDraw = false;
        if(i == yLineStart + drawSize/d + 5) yDraw = false;
        if(xDraw){
          double lineXPos = i * d;
          lineXPos -= xLeftPos;
          lineXPos *= scale;
          if (lineXPos >= 0 && lineXPos < size) qPainter.drawLine(QPointF(lineXPos, 0), QPointF(lineXPos, size));
        } // if
        if(yDraw){
          double lineYPos = i * d;
          lineYPos -= yLeftPos;
          lineYPos *= scale;
          if (lineYPos >= 0 && lineYPos < size) qPainter.drawLine(QPointF(0, lineYPos), QPointF(size, lineYPos));
        } // if
      } // for
      if(k !=1){
        boidPen.setWidth(5);
        qPainter.setPen(boidPen);
        qPainter.drawLine(QPointF(10, 20 * (k + 2)), QPointF((d) * scale + 10, 20 * (k + 2)));
        qPainter.drawText(QPointF(10, 20 * (k + 2) + 15),QString::number(d));
      }
    } // for
  } // if drawGridlines

  boidPen.setColor(QColor(20, 255, 20));
  boidPen.setWidth(1);
  qPainter.setPen(boidPen);
  if(rotateToxAxis){
    pixOffset = Vector2D(xDrawOffset / 40.0, yDrawOffset / 40.0) * drawSize;
    offset = pixOffset - COM;
    qPainter.save( );
    qPainter.translate(0.5 * size, 0.5 * size);
    qPainter.rotate(-P.Theta( ) / M_PI * 180.0);
  }

  //qPainter.setBrush(Qt::red);
//  // DRAW CONVEX HULL TRAIL
//  uint hoeveelShow = 50;
//  uint hullStartNum = (convexHullList.size( ) > hoeveelShow)? convexHullList.size( ) - hoeveelShow : 0;

//  for (uint hullNr = hullStartNum; hullNr < convexHullList.size( ); hullNr++) {
//    boidPen.setWidthF(3.0 * (double(hullNr+1)-hullStartNum) / (hoeveelShow));
//    boidPen.setColor(QColor(120, 55, 120, 255 * (double(hullNr+1)-hullStartNum) / (hoeveelShow)));

//    if (hullNr == convexHullList.size( ) - 1) { // current hull
//      boidPen.setColor(QColor(20, 255, 20));
//      boidPen.setWidth(3);
//    } // if

//    qPainter.setPen(boidPen);
//    QPainterPath qPainterPath;
//    for (uint i = 0; i < convexHullList[hullNr].size( ); i++) {
//        Vector2D hPos = convexHullList[hullNr][i];
//        double x = (hPos.x + offset.x) * scale;
//        double y = (hPos.y + offset.y) * scale;

//        if (hullNr == convexHullList.size( ) - 1)
//          qPainter.drawEllipse(x, y, 9, 9);
//        if (i > 0)
//            qPainterPath.lineTo(x, y);
//        else
//            qPainterPath.moveTo(x, y);
//    } // for
//    qPainterPath.lineTo((convexHullList[hullNr][0].x + offset.x) * scale,
//                        (convexHullList[hullNr][0].y + offset.y) * scale);
//    qPainter.drawPath(qPainterPath);
//  } // for

  //draw boids (and directions if true)
  for (Boid & b : boids){
    if(drawBoids){
      if(b.prefAngle == 0.0)
        boidPen.setColor(QColor(0, 0, 255, 200));
      else if(b.prefAngle < 0.0)
        boidPen.setColor(QColor(255, 0, 0, 200));
      else
        boidPen.setColor(QColor(0, 255, 0, 200));
      qPainter.setPen(boidPen);
      qPainter.drawEllipse((b.r.x + offset.x)*scale,
                           (b.r.y + offset.y)*scale, 2, 2);
    } // if drawboids
    if(b.trackAngles == trackBoidTrail && drawBoidTrail){

      uint nrOfTrails;
      if(nrOfTrailPoints >= t)
        nrOfTrails = t;
      else nrOfTrails = nrOfTrailPoints;

      boidPen.setColor(QColor(0, 0, 0, 200));
      qPainter.setPen(boidPen);

      QPainterPath qPainterPath;
      qPainterPath.moveTo((b.r.x + offset.x) * scale, (b.r.y + offset.y) * scale);
          qPainter.drawText((b.r.x + offset.x) * scale,(b.r.y + offset.y) * scale,QString::number(b.trackAngles));

      for(uint i = 1; i < nrOfTrails; i++){
        Vector2D vec = (anglesHistory[trackBoidTrail][t-i] - COM + pixOffset + avgPosGeom) * scale;

        if(rotateToxAxis)
          vec.Rotate(P.Theta( ) - p[t-i].Theta( ));

        qPainterPath.lineTo(vec.x, vec.y);
      } // for
      qPainter.drawPath(qPainterPath);
    }
    if(drawDirection || drawHullDirections){
      if(drawDirection){
          boidPen.setColor(QColor(0, 0, 255, 200));
          qPainter.setPen(boidPen);
          qPainter.drawLine((b.r.x + offset.x) * scale, (b.r.y + offset.y) * scale,
                            (b.r.x + offset.x) * scale + 10.0 * b.v.x, (b.r.y + offset.y) * scale + 10.0 * b.v.y);
      } // if drawDirection
      if(drawHullDirections){
        if(b.prevAngle != 10.0){
          boidPen.setColor(QColor(0, 255, 0, 200));
          qPainter.setPen(boidPen);
          qPainter.drawLine((b.r.x + offset.x) * scale, (b.r.y + offset.y) * scale,
                            (b.r.x + offset.x ) * scale + 10.0 * b.v.x, (b.r.y + offset.y) * scale + 10.0 * b.v.y);
        } // if hull
      } // if drawHullDirections
    } // if draw directions
  } // for boids

  // draw lines parallel and perpendicular to the polarisation through the center of the hull
  if(drawCross){
    QPoint origin((pixOffset - COM + avgPosGeom).x * scale, (pixOffset - COM + avgPosGeom).y * scale);
    boidPen.setWidth(5);
    boidPen.setColor(QColor(0,0,0,127));
    qPainter.setPen(boidPen);

    // in direction of polarization:
    qPainter.drawLine(origin, QPoint((P.normalised( )*2.0 - avgPosGeom).x * scale,(P.normalised( )*2.0 - avgPosGeom).y * scale));
    qPainter.drawLine(origin, QPoint((P.normalised( )*2.0 - avgPosGeom).x * scale,(P.normalised( )*-2.0 - avgPosGeom).y * scale));

    // perpendicular
    qPainter.drawLine(origin, QPoint(( P.normalised( ) - avgPosGeom).y * scale,(P.normalised( ) * - 1.0 - avgPosGeom).x * scale));
    qPainter.drawLine(origin, QPoint(( P.normalised( ) * - 1.0 - avgPosGeom).y * scale,(P.normalised( ) - avgPosGeom).x * scale));

    // again in direction of polarization, but better
    boidPen.setColor(QColor(255,0,0,255));
    qPainter.setPen(boidPen);

    Boid & A = boids[parBoidNrA];
    qPainter.drawLine(origin, QPoint((offset + A.r).x * scale,(offset + A.r).y * scale));

    Boid & B = boids[parBoidNrB];
    qPainter.drawLine(origin, QPoint((offset + B.r).x * scale,(offset + B.r).y * scale));

//    double S = closestParAngleMin / (closestParAngleMin - closestParAnglePlus);

//    Vector2D snij = B.r * S + (A.r * (1.0 - S));

//    boidPen.setWidth(7.5);
//    boidPen.setColor(QColor(50,50,255,255));
//    qPainter.setPen(boidPen);

//    qPainter.drawLine(origin, QPoint((offset + snij + COM).x * scale,(offset + snij + COM).y * scale));

    // compute super line:
    double theta = P.Theta( );
    double _t = -1.0 * tan(theta) * A.r.x + A.r.y;
    _t /= tan(theta) * (B.r.x - A.r.x) - B.r.y + A.r.y;
    double R = B.r.y * _t + A.r.y * (1.0 - _t);
    R /= sin(theta);

    boidPen.setWidth(2.5);
    boidPen.setColor(QColor(50,50,255,255));
    qPainter.setPen(boidPen);

    qPainter.drawLine(origin, QPoint((offset + P.normalised( )*R + COM).x * scale,(offset + P.normalised( )*R + COM).y * scale));

    R = B.r.x * _t + A.r.x * (1.0 - _t);
    R /= cos(theta);

    boidPen.setWidth(2.5);
    boidPen.setColor(QColor(150,150,255,255));
    qPainter.setPen(boidPen);

    qPainter.drawLine(origin, QPoint((offset + P.normalised( )*R + COM).x * scale,(offset + P.normalised( )*R + COM).y * scale));


    boidPen.setWidth(1);qPainter.setBrush(QBrush(QColor(0,0,0,127)));;
  } // if drawCross

  //  //draw boids on convex hull and edges of convex hull
  if(drawHull || drawHullBoids){
    boidPen.setColor(QColor(0, 255, 0, 200));
    qPainter.setPen(boidPen);
    QPainterPath qPainterPath;
    for (uint i = 0; i < this->convexHull.size( ); i++) {
      Boid &b = boids[convexHull[i]];
      double x = (b.r.x + offset.x) * scale;
      double y = (b.r.y + offset.y) * scale;

      if(drawHullBoids) qPainter.drawEllipse(x-2, y-2, 9, 9);
      if(drawHull){
        if (i > 0)
          qPainterPath.lineTo(x, y);
        else
          qPainterPath.moveTo(x, y);
      } // if
    } // for

    if(drawHull){
      qPainterPath.lineTo((boids[convexHull[0]].r.x + offset.x) * scale,
          (boids[convexHull[0]].r.y + offset.y) * scale);
      qPainter.drawPath(qPainterPath);
      // fill convex hull
      QBrush brush(QColor(0, 255, 0, 120));
      qPainter.fillPath(qPainterPath, brush);
    } // if
  } // if

  if(rotateToxAxis)
    qPainter.restore( );

  // draw polarisation vector and direction (useful in disordered regime as the vector is small)
  if(drawPolarization){

    boidPen.setColor(QColor(50, 50, 255, 255));
    boidPen.setWidth(4);
    qPainter.setPen(Qt::NoPen);
    qPainter.setBrush(QBrush(QColor(0,0,0,127)));;
    qPainter.drawEllipse(QPointF(size/8, 7*size/8), size/10, size/10);


    boidPen.setWidth(4);
    boidPen.setColor(QColor(0, 0, 0, 255));
    qPainter.setPen(boidPen);

    Vector2D avgDing = p.back( );
    avgDing = avgDing * (size / 10);
    qPainter.drawLine(QPointF(size/8, 7*size/8), QPointF(size/8 + avgDing.x, 7*size/8 + avgDing.y));
    qPainter.drawEllipse(QPointF(size/8 + size/10 * avgDing.normalised( ).x, 7*size/8 + size/10 * avgDing.normalised( ).y),2,2);

  } // if drawPolarization
  qPainter.setBrush(QBrush(QColor(0,0,0,0)));;
  boidPen.setColor(QColor(0, 200, 0));
  qPainter.setPen(boidPen);
  qPainter.drawRect(0, 0, size-1, size-1);
  return img;
} // ToQImage

/**
INIT/CHECKPOINT PREPARATION FUNCTIONS
**/

void BoidSim2D::CalcP( ) {
  //Calculates the polarisation vector for loaded checkpoints
  Vector2D P, Pbulk, Phull, temp;
  P.Zero( );
  Pbulk.Zero( );
  Phull.Zero( );
  temp.Zero( );
  int hullCount = 0;
  int bulkCount = 0;

  for (Boid b : boids){
    P += b.v;
    if(b.convHullNormal.x == 0.0 && b.convHullNormal.y == 0){
      bulkCount++;
      Pbulk += b.v;
    }else{
      hullCount++;
      Phull += b.v;
    }
  }

  P /= nrOfBoids;
  Pbulk /= bulkCount;
  Phull /= hullCount;

  p.push_back(P);
  pHull.push_back(Phull);
  pBulk.push_back(Pbulk);

} // P

void BoidSim2D::InitBoids(int posIndex, int dirIndex ) {
  // Initialises the position and velocity vectors of boids
  boids.clear( );
  boids = vector <Boid> (nrOfBoids);
  avgPosGeom.Zero( );
  P.Zero( );
  for (uint i = 0; i < boids.size( ); i++) {
    Boid &b = boids[i];
    b.curAngle = 10;
    b.prefAngle = 10;
    b.prevAngle = 10;

    switch(posIndex){
      case 0: b.r.RandBox(this, rng); // square
              break;
      case 1: b.r.RandDisk(this, rng); // disk
              break;
      case 2: b.r.RandSquarePerim(this, rng); // perimeter of square
              break;
      case 3: b.r.RandCircle(this, rng); // on a circle
              break;
    }//switch posIndex
    avgPosGeom += b.r;


    if(i < nrOfBoidsToTrack){
      b.trackAngles = i;
      //anglesHistory.push_back(vector<double> (0));
      anglesHistory.push_back(vector<Vector2D> {b.r});
    }
    else
      b.trackAngles = -10;


    switch(dirIndex){
      case 0: b.v.RandNorm(this, rng);
              break;
      case 1: b.OrientRotating( );
              break;
      case 2: b.OrientInward( );
              break;
      case 3: b.OrientOutward( );
              break;
    }
    P += b.v;


    if(b.r.x < (double)minX){
        minX = b.r.x;
        minXd = b.r.x;
    }
    else if(b.r.x > (double)maxX){
        maxX = ceil(b.r.x) + 1;
        maxXd = b.r.x;
    }
    if(b.r.y < (double) minY){
        minY = b.r.y;
        minYd = b.r.y;
    }
    else if(b.r.y > (double) maxY){
        maxY = ceil(b.r.y) + 1;
        maxYd = b.r.y;
    }

    b.prefAngle = 0;
  } // for
  P /= nrOfBoids;
  nBoxX = maxX - minX + 2;
  nBoxY = maxY - minY + 2;
  avgPosGeom /= nrOfBoids;
  t++;
} // InitBoids

// set informed boids from the UI or a checkpoint
void BoidSim2D::SetInformedBoids(vector<pair<uint, double> > groups) {
  // clear old angles
  for(Boid &b : boids)
    b.prefAngle = 0.0;

  shuffle(begin(indices),end(indices),*rng);
  int counter = 0;
  for (uint i = 0; i < groups.size( ); i++)
    for (uint j = 0; j < groups[i].first; j++){
      int index = indices[counter];
      boids[index].prefAngle = groups[i].second;
      counter++;
    }// for j
} // SetInformedBoids

// find extremal values to create proper amount of boxes
void BoidSim2D::FindGeometry( ){
  // set extremal values to somewhere inside the hull
  minXd = avgPosGeom.x;
  minX = minXd;
  minYd = avgPosGeom.y;
  minY = minYd;
  maxXd = avgPosGeom.x;
  maxX = maxXd;
  maxYd = avgPosGeom.y;
  maxY = maxYd;

  for(Boid &b : boids){
      if(b.r.x < (double) minX) minX = b.r.x;
      else if(b.r.x > (double) maxX) maxX = ceil(b.r.x) + 1;
      if(b.r.y < (double) minY) minY = b.r.y;
      else if(b.r.y > (double) maxY) maxY = ceil(b.r.y) + 1;
      if(b.r.x < (double)minXd) minXd = b.r.x;
      else if(b.r.x > (double)maxXd) maxXd = b.r.x;
      if(b.r.y < (double)minYd) minYd = b.r.y;
      else if(b.r.y > (double)maxYd) maxYd = b.r.y;

  } // for boids

    nBoxX = maxX - minX + 2;
    nBoxY = maxY - minY + 2;
} // FindGeometry

/**
VICSEK MODEL FUNCTIONS
**/
// put boids in appropriate boxes
void BoidSim2D::FillBoxes( ) {
  uint nX, nY;
  boxes.clear( );

  boxes = vector <vector <uvec> > (nBoxX, vector <uvec> (nBoxY));
  for (uint n = 0; n < boids.size( ); n++) {
    //nX = boids[n].r.x;
    //nY = boids[n].r.y;
    nX = boids[n].r.x - minX + 1;
    nY = boids[n].r.y - minY + 1;
    boxes[nX][nY].push_back(n);
  } // for

} // FillBoxes

// go through neighbouring boxes to find if boids will interact
void BoidSim2D::Interact( ){
    //interact bottom and top rows of boxes
  #pragma omp parallel for
    for(uint nX = 0; nX < nBoxX - 1; nX++){
      InteractBoxes(boxes[nX][0]);
      InteractBoxes(boxes[nX][0],boxes[nX+1][0]);
      InteractBoxes(boxes[nX][0],boxes[nX][1]);
      InteractBoxes(boxes[nX][0],boxes[nX+1][1]);

      InteractBoxes(boxes[nX][nBoxY - 1]);
      InteractBoxes(boxes[nX][nBoxY - 1],boxes[nX+1][nBoxY - 1]);
      InteractBoxes(boxes[nX][nBoxY - 1],boxes[nX+1][nBoxY - 2]);
    }
    //interact right column of boxes
    #pragma omp parallel for
    for(uint nY = 0; nY < nBoxY - 1; nY++){
      InteractBoxes(boxes[nBoxX - 1][nY]);
      InteractBoxes(boxes[nBoxX - 1][nY],boxes[nBoxX - 1][nY + 1]);
    }
    //interact rest
    #pragma omp parallel for
    for (uint nX = 0; nX < nBoxX - 1; nX++){
      for (uint nY = 1; nY < nBoxY - 1; nY++) {
        InteractBoxes(boxes[nX][nY]);

        InteractBoxes(boxes[nX][nY], boxes[nX+1][nY]);
        InteractBoxes(boxes[nX][nY], boxes[nX][nY+1]);
        InteractBoxes(boxes[nX][nY], boxes[nX+1][nY+1]);
        InteractBoxes(boxes[nX][nY], boxes[nX+1][nY-1]);
      } // for
    }
} // Interact

// computes next positions and velocities of boids
void BoidSim2D::MoveBoids( ){

    Vector2D Pold = p.back( );
    double PoldTheta = Pold.Theta( );
    int hullCount = 0;
    int bulkCount = 0;

    // temp vars for MP reduction
    double polX = 0.0;
    double polY = 0.0;
    double polBulkX = 0.0;
    double polBulkY = 0.0;
    double polHullX = 0.0;
    double polHullY = 0.0;
    int mX = avgPosGeom.x; // int minX for reduction
    double mXd = avgPosGeom.x; // double minXd for reduction
    int Mx = mX; // int maxX for reduction
    double MxD = mXd; // double maxXd for reduction
    int mY = avgPosGeom.y; // int minY for reduction
    double mYd = avgPosGeom.y; // double minYd for reduction
    int My = mY; // int maxY for reduction
    double MyD = mYd; // int maxYd for reduction
    //double theta = 0.0;
    //double polTheta = P.Theta( );
    #pragma omp parallel for reduction(+:hullCount, bulkCount, polX, polY, polBulkX, polBulkY, polHullX, polHullY), reduction(max:Mx, MxD, My, MyD), reduction(min:mX, mXd, mY, mYd)
    for (uint i = 0; i < boids.size( ); i++){
      Boid & b = boids[i];
      if(interactFast){
        if(noiseType) // scalar
            b.v = Vector2D(PoldTheta + eta * rand2Pi(*rng));
        else // vectorial
            b.v = (Vector2D(this, rng) * eta * nrOfBoids + Pold * nrOfBoids).Normalised( );
      }
      else{
        if(noiseType) // scalar
          b.v = Vector2D(b.avgV.Theta( ) + eta * rand2Pi(*rng));
        else // vectorial
          b.v = (Vector2D(this, rng) * eta * b.numNeighbours + b.avgV).Normalised( );
      }


      //update informed boid orientation and convex hull boids
      b.v = Vector2D(b.v.Theta( ) + b.prefAngle) + b.convHullNormal * forceConstant;

      b.v.Normalise( );

      if(b.trackAngles != -10){
        //theta = (b.r - avgPosGeom).Theta( ) - polTheta; // ang of boid in [-2PI, 2PI]
        //theta += (theta > M_PI) ? - M_2PI : ((theta < - M_PI) ? M_2PI : 0);
        //anglesHistory[b.trackAngles].push_back(theta);
        anglesHistory[b.trackAngles].push_back(Vector2D(b.r - avgPosGeom));
      } // if trackAngles

      b.Move(vNought);
      b.avgV.Zero( );
      b.numNeighbours = 0;
      if(b.trackAngles == trackCOM) COM = b.r;

      if(b.r.x < (double)mX)
        mX = b.r.x;
      else if(b.r.x > (double)Mx)
        Mx = ceil(b.r.x) + 1;
      if(b.r.y < (double) mY)
        mY = b.r.y;
      else if(b.r.y > (double) My)
        My = ceil(b.r.y) + 1;

      if(b.r.x < mXd)
        mXd = b.r.x;
      else if(b.r.x > MxD)
        MxD = b.r.x;
      if(b.r.y < mYd)
        mYd = b.r.y;
      else if(b.r.y > MyD)
        MyD = b.r.y;

      if(b.convHullNormal.x == 0.0 && b.convHullNormal.y == 0.0){ // b is not on hull
        bulkCount++;
        polBulkX += b.v.x;
        polBulkY += b.v.y;
        b.curAngle = 10;
      }else{
        hullCount++;
        polHullX += b.v.x;
        polHullY += b.v.y;
      }

      polX += b.v.x;
      polY += b.v.y;
      b.prevAngle = b.curAngle;
      b.convHullNormal.Zero( );
    }

    minX = mX;
    maxX = Mx;
    minY = mY;
    maxY = My;
    minXd = mXd;
    minYd = mYd;
    maxXd = MxD;
    maxYd = MyD;
    nBoxX = maxX - minX + 2;
    nBoxY = maxY - minY + 2;

    P = Vector2D(polX, polY);
    Pbulk = Vector2D(polBulkX, polBulkY);
    Phull = Vector2D(polHullX, polHullY);
    P /= nrOfBoids;
    Phull /= hullCount;
    Pbulk /= bulkCount;

    Vector2D temp = p.back( );
    Vector2D temp2 = P;
    p.push_back(P);
    temp2.Normalise( );
    temp.Normalise( );
    curvatureVec.push_back(temp.CrossProduct(temp2));

    temp = pHull.back( );
    temp2 = Phull;
    pHull.push_back(Phull);
    temp.Normalise( );
    temp2.Normalise( );
    curvatureHullVec.push_back(temp.CrossProduct(temp2));

    temp = pBulk.back( );
    temp2 = Pbulk;
    pBulk.push_back(Pbulk);
    temp.Normalise( );
    temp2.Normalise( );
    curvatureBulkVec.push_back(temp.CrossProduct(temp2));

} // MoveBoids

// perform the simulation one timestep
void BoidSim2D::NextStep( ) {

  FillBoxes( );

  if(!interactFast)
    Interact( );

  MoveBoids( );

  FindConvexHull( );

  t++;
} // NextStep

// interacts boids from 2 different boxes
void BoidSim2D::InteractBoxes(BoidSim2D::uvec & box1, BoidSim2D::uvec & box2) {
  for (uint b1 = 0; b1 < box1.size( ); b1++){
    Boid & boid1 = boids[box1[b1]];
    for (uint b2 = 0; b2 < box2.size( ); b2++){
      Boid & boid2 = boids[box2[b2]];
      if ( (boid1.r - boid2.r).length( ) < 1.0){
        boid1.numNeighbours++;
        boid2.numNeighbours++;
        boid1.avgV += boid2.v;
        boid2.avgV += boid1.v;
      }
    }
  }//ExchangeInfo(b1, box1, b2, box2);
} // InteractBoxes

// interacts boids within the same box
void BoidSim2D::InteractBoxes(BoidSim2D::uvec & box1) {
  for (uint b1 = 0; b1 < box1.size( ); b1++){
    Boid & boid1 = boids[box1[b1]];
    for (uint b2 = b1; b2 < box1.size( ); b2++){
      Boid & boid2 = boids[box1[b2]];
      if ( (boid1.r - boid2.r).length( ) < 1.0){
        boid1.numNeighbours++;
        boid2.numNeighbours++;
        boid1.avgV += boid2.v;
        boid2.avgV += boid1.v;
      }
    }  // ExchangeInfo(b1, box1, b2, box1);
  }
} // InteractBoxes


/**
 CONVEX HULL FUNCTIONS
**/

void BoidSim2D::FindConvexHull( ){
  avgPosGeomOld = avgPosGeom;
  int k = 0;
  convexHull.clear( );
  convexHull = vector<uint>(2*boids.size( ));

  //sort points lexicographically
  sort(boids.begin( ), boids.end( ));
  //minX = boids.front( ).r.x-1;
  //maxX = boids.back( ).r.x+1;
  //build lower hull
  for(uint i = 0; i < boids.size( ); ++i){
    BoidSim2D::Vector2D r = boids[i].r;
    while(k >= 2 && r.CrossProduct(boids[convexHull[k-2]].r,boids[convexHull[k-1]].r) <= 0) k--;
    convexHull[k++] = i;
  }

  //build upper hull
  for(int i = boids.size( ) - 2, t = k + 1; i >= 0; i--){
    BoidSim2D::Vector2D r = boids[i].r;
    while(k >= t && r.CrossProduct(boids[convexHull[k-2]].r,boids[convexHull[k-1]].r) <= 0) k--;
    convexHull[k++] = i;
  }

  convexHull.resize(k - 1);


  convHullArea = 0.0;
  avgPosGeom.Zero( );
  convHullLength = 0.0;
  maxSep = 0.0;
  closestParAnglePlus = 10000;
  closestParAngleMin = -10000;
  closestPerpAngle = 10000;
  double polAngle = -P.Theta( );
  Vector2D pNorm = P.normalised( );
  for(uint i = 0; i < convexHull.size( ); i++){
    Boid &b = boids[convexHull[i]];

    // find normal vector for boid b
    CalcHullNormal(b, i);
    // find angle
    FindHullAngle(b, pNorm, polAngle, i);
    UpdateDAngleHist(b, i);


    // check if all distances between boids on hull are smaller than 1
    // (if so, the entire interaction algorithm can be skipped and
    // the average neighbour orientation can be extracted from the
    // polarization)
    for(uint j = i + 1; j < convexHull.size( ); j++){
      double len = (boids[convexHull[i]].r - boids[convexHull[j]].r).Length( );
      if(len > maxSep)
        maxSep = len;
    }
  }  // for

  if(maxSep > 1.0) interactFast = false;
  else interactFast = true;

  avgPosGeom /= convHullLength;
  avgPosGeomHistory.push_back(avgPosGeom);
  if(trackCOM == -1) COM = avgPosGeom;
  convHullArea /= 2.0;
  convHullAreaVec.push_back(convHullArea);

  nrOfBoidsOnConvHull = k-1;
  numBoidsOnHull.push_back(nrOfBoidsOnConvHull);

  // update convexHullList:
//  vector <Vector2D> curHull;
//  for(uint i = 0; i < convexHull.size( ); i++){
//    Boid &b = boids[convexHull[i]];Vector2D t1, t2;

//    curHull.push_back(b.r);
//  } // for
//  if (this->t % 25 == 0 || t <= 2)
//    convexHullList.push_back(curHull);

} //FindConvexHull

void BoidSim2D::CalcHullNormal(Boid &b, uint i){
    Vector2D t1, t2;

    int bPrev = (i > 0) ? convexHull[i-1] : convexHull.back( );
    int bNext = (i < convexHull.size( ) - 1)? convexHull[i+1] : convexHull.front( );

    t1 = b.r - boids[bPrev].r; // vec van cur boid naar vorige op hull
    t2 = boids[bNext].r - b.r; // vec naar volgende op hull

    double t2Length = t2.Length( );
    convHullLength += t2Length;

    b.convHullNormal =  (t2 - t1) * (2.0 / (t1.Length( ) + t2Length ) );

    avgPosGeom += (b.r + boids[bNext].r) * (t2Length / 2.0);

    convHullArea += b.r.x * boids[bNext].r.y - b.r.y * boids[bNext].r.x;
} // CalcHullNormalForce

void BoidSim2D::FindArea( ){
  convHullArea = 0.0;
  uint max = convexHull.size( ) - 1;
  for(uint i = 0; i < max;i++)
    convHullArea += boids[convexHull[i]].r.x * boids[convexHull[i+1]].r.y
                  - boids[convexHull[i]].r.y * boids[convexHull[i+1]].r.x;

  convHullArea += boids[convexHull[max]].r.x * boids[convexHull[0]].r.y
                  - boids[convexHull[max]].r.y * boids[convexHull[0]].r.x;

  convHullArea /= 2.0;
  convHullAreaVec.push_back(convHullArea);
}//FindArea

void BoidSim2D::FindHullAngle(Boid &b, Vector2D pNorm, double polAngle, uint boidIndex){
  // find angle between polarisation vector and position on hull wrt the geometric center
//  double theta = (b.r - avgPosGeomOld).Theta( ) - polAngle; // ang of boid in [-2PI, 2PI]
//  theta += (theta > M_PI) ? - M_2PI : ((theta < - M_PI) ? M_2PI : 0);
//  theta += M_PI;
  double theta = (b.r - avgPosGeomOld).Rotated(polAngle).Theta( ) + M_PI;
  /*
    if (theta >= 0 && theta < closestParAnglePlus) { // found new boid closest to polarization direction
    closestParAnglePlus = theta;
    parBoidNrA = convexHull[boidIndex];
    parBoidNrB = (boidIndex > 0)? convexHull[boidIndex-1] : convexHull.back( );

    Boid & kk = boids[parBoidNrB];
    double thetaB = (kk.r - avgPosGeomOld).Theta( ) - polAngle; // ang of boid in [-2PI, 2PI]
    thetaB += (thetaB > M_PI) ? - M_2PI : ((thetaB < - M_PI) ? M_2PI : 0);
    thetaB += M_PI;
    closestParAngleMin = thetaB;
  } // if
  */
//  if (theta <= 0 && theta > closestParAngleMin) { // found new boid closest to polarization direction
//    closestParAngleMin = theta;
//    parBoidNrB = convexHull[boidIndex];
//  } // if

  // update curAngle and prevAngle of current boid

  b.curAngle = theta;

  // find bind number for angular distribution of boids on the hull and add it
  uint nr = (1.0 - theta / M_2PI) * angHist.size( ); // een getal in [0, numBins-1];
  if (nr == angHist.size( ))
    nr = 0;
  angHist[nr]++;
  radiusHist[nr] += (avgPosGeomOld - b.r).Length( );


  // find boid velocity components parallel and transverse to polarisation and add to distribution
  double parallelComp = b.v.DotProduct(pNorm);
  componentHist[nr].first += parallelComp; // cos(relAngle);

  componentHist[nr].second += sqrt(1.0 - parallelComp * parallelComp); // sin(relAngle);

} // FindHullAngles

// updates dAngleHist (histogram of dAngle for all angles)
// using the current state of vector {boids}
void BoidSim2D::UpdateDAngleHist(Boid &b, uint i) {
  if (b.curAngle != 10 && b.prevAngle != 10) { // b is on hull now as well as during the previous step
    double dAngle = b.curAngle - b.prevAngle; // [-2pi, 2pi] // difference between current and previous angle in hull

    dAngle += (dAngle > M_PI) ? - M_2PI : ((dAngle < - M_PI) ? M_2PI : 0);

    uint angleIndex = dAngleHist.size( ) * (1.0 - (b.prevAngle) / M_2PI); // bin number in histogram

    if (angleIndex == dAngleHist.size( ))
      angleIndex = 0;

    dAngleHist[angleIndex].first++; // number of boids in bin
    dAngleHist[angleIndex].second += dAngle; // total dAngle for this bin
  } // if
} // UpdateDAngleHist

/**
VECTOR FUNCTIONS
**/

BoidSim2D::Vector2D::Vector2D(Vector2D * v) {
  this->x = v->x;
  this->y = v->y;
} // Vector2D

BoidSim2D::Vector2D::Vector2D(double x, double y) {
  this->x = x;
  this->y = y;
} // Vector2D

BoidSim2D::Vector2D::Vector2D(double nT) {
  this->x = cos(nT);
  this->y = sin(nT);
} // Vector2D

BoidSim2D::Vector2D::Vector2D(BoidSim2D* parent, mt19937* rng) {
  double nT = parent->rand2Pi(*rng);
  this->x = cos(nT);
  this->y = sin(nT);
} // Vector2D

BoidSim2D::Vector2D::Vector2D( ) {
  this->x = 0;
  this->y = 0;
} // Vector2D

void BoidSim2D::Vector2D::RandBox(BoidSim2D* parent, mt19937* rng){
  this->x = parent->randR(*rng);
  this->y = parent->randR(*rng);
} // RandBox

void BoidSim2D::Vector2D::RandDisk(BoidSim2D* parent, mt19937* rng){
  double r = sqrt(parent->randR(*rng) + parent->L / 2.0);
  double t = parent->rand2Pi(*rng);
  this->x = r * cos(t);
  this->y = r * sin(t);
} // RandDisk

void BoidSim2D::Vector2D::RandCircle(BoidSim2D* parent, mt19937* rng){
  double t = parent->rand2Pi(*rng);
  this->x = parent->L * cos(t) ;
  this->y = parent->L * sin(t);
} // RandDisk

double BoidSim2D::Vector2D::CrossProduct(Vector2D &v){
  return this->x * v.y - this->y * v.x;
}

void BoidSim2D::Vector2D::RandSquarePerim(BoidSim2D* parent, mt19937* rng){
  double pos = parent->randR(*rng);
  uniform_int_distribution<int> side(0,3);
  switch(side(*rng)){
    case 0: this->x = -parent->L/2.0; this->y = pos; break;
    case 1: this->y = -parent->L/2.0; this->x = pos; break;
    case 2: this->y = parent->L/2.0; this->x = pos; break;
    case 3: this->x = parent->L/2.0; this->y = pos; break;
  }
}  // RandSquarePerim

void BoidSim2D::Vector2D::Rotate(double theta){
  double c = cos(theta);
  double s = sin(theta);
  double xtemp = this->x;
  double ytemp = this->y;
  this->x = xtemp * c - ytemp * s;
  this->y = xtemp * s + ytemp * c;
} // Rotate

BoidSim2D::Vector2D BoidSim2D::Vector2D::Rotated(double theta){
  this->Rotate(theta);
  return this;
} // Rotated

BoidSim2D::Vector2D BoidSim2D::Vector2D::rotated(double theta){
  Vector2D temp = this;
  return temp.Rotated(theta);
} // Rotated

void BoidSim2D::Vector2D::RandNorm(BoidSim2D* parent, mt19937* rng){
  double nT = parent->rand2Pi(*rng);
  this->x = cos(nT);
  this->y = sin(nT);
} // RandVec

BoidSim2D::Vector2D BoidSim2D::Vector2D::Normalised( ){
  this->Normalise( );
  return *this;
} // Normalised

BoidSim2D::Vector2D BoidSim2D::Vector2D::normalised( ){
  Vector2D temp = this;
  return temp.Normalised( );
} // normalised

void BoidSim2D::Vector2D::Normalise( ){
  double len = this->Length( );
  this->x /= len;
  this->y /= len;
}
double BoidSim2D::Vector2D::CrossProduct(Vector2D &O, Vector2D &v){
  return (this->y - O.y) * (v.x - O.x) - (this->x - O.x) * (v.y - O.y);
} // crossProduct

double BoidSim2D::Vector2D::DotProduct(Vector2D &v){
  return this->x * v.x + this->y * v.y;
} // dotProduct

double BoidSim2D::CrossProduct(Vector2D &O, Vector2D &a, Vector2D &b){
  return (a.x-O.x)*(b.y-O.y) - (a.y-O.y)*(b.x-O.x);
} // crossProduct

/**
CONSTRUCTORS
**/

BoidSim2D::BoidSim2D(int seed, double rho, uint nrOfBoids, int posIndex) {
  this->trackCOM = -1;
  nrOfTrailPoints = 10;
  drawBoidTrail = false;
  this->nrOfBoids = nrOfBoids;
  this->t = 0;
  nrOfBoidsToTrack = 100;
  if(seed == 0)
    this->generator.seed(unsigned(time(0)));
  else this->generator.seed(seed);
  this->rho = rho;
  this->rng = &generator;
  switch(posIndex){
    case 0: this->L = sqrt(nrOfBoids / rho);
            break;//box
    case 1: this->L = sqrt(nrOfBoids / (M_PI * rho));
            break;//disk
    case 2: this->L = nrOfBoids / (4 * rho);
            break;//square perimeter
    case 3: this->L = nrOfBoids / (2 * M_PI * rho);
            break;//circle
  }//switch posIndex

  this->nBoxX = ceil(L);
  this->nBoxY = ceil(L);
  this->minX = 0;
  this->maxX = 0;
  this->minY = 0;
  this->maxY = 0;
  this->xDrawOffset = 0;
  this->yDrawOffset = 0;

  this->randR = uniform_real_distribution<double>(-L / 2.0, L / 2.0);
  this->rand2Pi = uniform_real_distribution<double>(-M_PI, M_PI);

  numBins = 1001;
  angHist = vector <int>(numBins, 0);
  componentHist = vector<pair<double, double> > (numBins);
  dAngleHist = vector < pair <uint, double> > (numBins);
  radiusHist = vector<double> (numBins);
} // BoidSim2D

BoidSim2D::Boid::Boid(Vector2D r, Vector2D v, double prefAngle, int trackNr){
  this->r = r;
  this->v = v;
  this->prefAngle = prefAngle;
  this->numNeighbours = 0;
  this->avgV = Vector2D( );
  this->trackAngles = trackNr;
} // Boid prefAngle constructor

BoidSim2D::Boid::Boid( ){
  this->r = Vector2D( );
  this->v = Vector2D( );
  this->prefAngle = 0.0;
  this->numNeighbours = 0;
  this->avgV = Vector2D( );
} // Boid zero constructor
