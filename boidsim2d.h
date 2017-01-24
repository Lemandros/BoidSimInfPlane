#ifndef BOIDSIM2D_H
#define BOIDSIM2D_H
#include <FFTw/include/fftw3.h>
#include <vector>
#include <QImage>
#include <utility>
#include <cmath>
#include <math.h>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>
#include <istream>
#include <iostream>
#include <random>
using namespace std;

class BoidSim2D
{
public:
  typedef unsigned int uint;
  typedef vector <uint> uvec;

  class Vector2D
  {
  public:
    double x, y;

    double Length( ) {
      return sqrt(x*x + y*y);
    } // Length
    double length( ) {
      return x*x + y*y;
    }
    double Theta( ) {
      return atan2(y,x);
    } // theta
    double theta( ) {
      return atan2(y,x) * 180.0 / M_PI;
    } // theta
    void Zero( ){
      this->x = 0.0;
      this->y = 0.0;
    }

    Vector2D operator+(const Vector2D & v) {
      Vector2D ans(this);
      ans.x += v.x;
      ans.y += v.y;
      return ans;
    } // operator+

    Vector2D operator-(const Vector2D & v) {
      Vector2D ans(this);
      ans.x -= v.x;
      ans.y -= v.y;
      return ans;
    } // operator+

    Vector2D operator/(double a) {
      Vector2D ans(this);
      ans.x /= a;
      ans.y /= a;
      return ans;
    } // operator/

    Vector2D operator*(double a) {
      Vector2D ans(this);
      ans.x *= a;
      ans.y *= a;
      return ans;
    } // operator*

    Vector2D operator*(uint a) {
      Vector2D ans(this);
      ans.x *= a;
      ans.y *= a;
      return ans;
    } // operator*

    Vector2D& operator+=(const Vector2D & v) {
      this->x += v.x;
      this->y += v.y;
      return *this;
    } // operator+=

    Vector2D& operator-=(const Vector2D & v) {
      this->x -= v.x;
      this->y -= v.y;
      return *this;
    } // operator+=

    Vector2D& operator=(const Vector2D & v) {
      this->x = v.x;
      this->y = v.y;
      return *this;
    } // operator=

    Vector2D& operator/=(double a) {
      this->x /= a;
      this->y /= a;
      return *this;
    } // operator/=

    Vector2D& operator*=(double a) {
      this->x *= a;
      this->y *= a;
      return *this;
    } // operator*=

    void RandBox(BoidSim2D* parent, mt19937* rng);
    void RandDisk(BoidSim2D* parent, mt19937* rng);
    void RandCircle(BoidSim2D* parent, mt19937* rng);
    void RandSquarePerim(BoidSim2D* parent, mt19937* rng);
    void RandNorm(BoidSim2D* parent, mt19937* rng);
    void Normalise( );
    void Rotate(double theta);
    Vector2D Rotated(double theta);
    Vector2D rotated(double theta);
    Vector2D Normalised( );
    Vector2D normalised( );
    Vector2D(BoidSim2D* parent, mt19937* rng);
    Vector2D(Vector2D * v);
    Vector2D(double nT);
    Vector2D(double x, double y);
    Vector2D( );
    double CrossProduct(Vector2D &o, Vector2D &v);
    double CrossProduct(Vector2D &v);
    double DotProduct(Vector2D &v);
  }; // Vector2D


  struct Boid {
    Vector2D r, v, avgV, convHullNormal;
    double prefAngle;
    uint numNeighbours;

    int trackAngles;

    void Move(double vNought){
      r.x = r.x + v.x * vNought;
      r.y = r.y + v.y * vNought;
      numNeighbours = 0;
    }
    bool operator<(Boid &b){
      return r.x < b.r.x || (r.x == b.r.x && r.y < b.r.y);
    }
    void OrientRotating( ){
      v.x = -r.y;
      v.y = r.x;
      v.Normalise( );
    }
    void OrientOutward( ){
      v = Vector2D(r).Normalised( );
    }
    void OrientInward( ){
      v = Vector2D(r).Normalised( ) * - 1.0;
    }


    double curAngle, prevAngle; // angles in convex hull


    Boid( );
    Boid(Vector2D r, Vector2D v, double prefAngle, int trackNr);
  }; // Boid


  // PARAMS:
  bool noiseType; // false = vectorial, true = scalar
  bool drawBoids;
  bool drawHull;
  bool drawHullBoids;
  bool drawGridlines;
  bool drawPolarization;
  bool drawDirection;
  bool drawHullDirections;
  bool drawCross;
  bool drawBoidTrail;
  bool rotateToxAxis;
  int trackBoidTrail;
  int trackCOM; // -2: don't track anymore, -1: track center of hull, 0-nrOfTrackedBoids-1: center on a boid
  bool trackSize;

  int xDrawOffset;
  int yDrawOffset;
  double zoomScale;
  Vector2D COM;
  bool drawSizeConst;
  uint nrOfBoids;
  uint nrOfTimeStepsToRun;
  uint nrOfInformedBoids;
  uint nrOfInformedGroups;
  uint nrOfBoidsOnConvHull;
  uint t;
  uint nrOfTrailPoints;
  uint forceType;
  int minX, maxX, minY, maxY;
  double minXd, maxXd, minYd, maxYd;
  uint nBox, nBoxX, nBoxY;
  uint nrOfBoidsToTrack;
  // for drawing cross:
  uint parBoidNrA, parBoidNrB;                        // boids on hull closest to polarization vector direction
  uint perpBoidNrA, perpBoidNrB;                      // boids on hull closest to vector perpendicular to polarization vector direction
  double closestParAnglePlus, closestParAngleMin, closestPerpAngle; // angle closest to cross directions
  vector<pair <uint, double> > groups;
  vector<double> polCumAvg;
  vector<double> polBulkCumAvg;
  vector<double> polHullCumAvg;
  vector<double> curvCumAvg;
  vector<double> curvBulkCumAvg;
  vector<double> curvHullCumAvg;
  vector<double> areaCumAvg;
  double rho; // density
  double eta; // random error range
  double vNought; // velocity of boids
  double L; // linear size of world
  double forceConstant;
  double convHullArea;
  double maxSep; // largest value of (boid[i].r-boid[j].r).Length( ) for all i != j
  double convHullLength;
  bool interactFast;
  vector<double> convHullAreaVec; // vector of surface areas
  vector<uint> numBoidsOnHull; // vector of amount of boids on the hull
  vector<uint> indices; // vector of indices to boids for setting informed boids
  vector<vector <Vector2D> > anglesHistory;
  vector <vector <uvec> > boxes; // boxes containing boid indices
  vector<uint> convexHull; // list of boid indices on the hull
  vector <Boid> boids; // the boids
  vector <pair <uint, double> > informedBoids;// (numBoids, prefAngle) per informed group
  vector<Vector2D> p; // total polarisation history
  vector<Vector2D> pHull; // hull polarisation history
  vector<Vector2D> pBulk; // bull polarisation history
  Vector2D P; // current total polarisation
  Vector2D Pold;
  Vector2D Phull; // current hull polarisation
  Vector2D Pbulk; // current bulk polarisation
  Vector2D avgPos, avgPosGeom, avgPosGeomOld;
  vector<Vector2D> avgPosGeomHistory;
  vector<double> curvatureVec; // total curvature history
  vector<double> curvatureHullVec; // hull curvature history
  vector<double> curvatureBulkVec; // bulk curvature history


//  vector <vector <Vector2D> > convexHullList;
  vector <int> angHist; // histogram over time of boids on the hull
  vector <pair<double, double> > componentHist; // histogram of velocity components on the hull
  vector <pair<uint, double>> dAngleHist; // hoeveel boids op dAngle's en som van dAngle's
  vector <double> radiusHist;
  uint numBins;


  mt19937 generator;
  mt19937* rng;
  int seed;
  uniform_real_distribution<double> randR;
  uniform_real_distribution<double> rand2Pi;
  uniform_int_distribution<uint> randBoid;


  // FUNCTIONS:

  QImage ToQImage(uint size = 500, bool antiAliasing = false);

  void FindGeometry( );
  void CalcP( );
  void CalcCumAvg(uint begin);
  void InitTrackedBoids( );
  void InitBoids(int posIndex, int dirIndex);
  void SetInformedBoids(vector<pair <uint, double> > groups);
  void FillBoxes( );
  void NextStep( );
  void Interact( );
  void InteractBoxes(uvec & box1, uvec & box2);
  void InteractBoxes(uvec & box1);
  void MoveBoids( );
  void FindConvexHull( );
  void FindArea( );
  void FindHullAngle(Boid &b, Vector2D pNorm, double polAngle, uint boidIndex);
  void UpdateDAngleHist(Boid &b, uint i);
  void CalcHullNormal(Boid &b, uint i);
  Vector2D NormalNearestNeighbour(Vector2D t1, Vector2D t2, double t2Length);

  BoidSim2D(int seed, double rho, uint nrOfBoids, int posIndex);
  double CrossProduct(Vector2D &O, Vector2D &a, Vector2D &b);
};

#endif // BOIDSIM2D_H

