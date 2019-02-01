//  node.cpp
//  Node of a path (implementation).

#include <cmath>

#include "node.h"
#include "util.h"


Node::Node() : mX(0), mY(0), mTheta(0), mCostSoFar(0), mCostToCome(0), mIsEvaluated(false), mPred(nullptr), mPos(0)
{

}

Node::Node(const state& st) : mX(st.x), mY(st.y), mTheta(st.theta), mCostSoFar(0), mCostToCome(0), mIsEvaluated(false), mPred(nullptr), mPos(0)
{

}


Node::Node(double x, double y, double theta) : mX(x), mY(y), mTheta(theta), mCostSoFar(0), mCostToCome(0), mIsEvaluated(false), mPred(nullptr), mPos(0)
{

}


Node::Node(double x, double y, double theta, double costSoFar, double costToCome, const Node* pred)
: mX(x), mY(y), mTheta(theta), mCostSoFar(costSoFar), mCostToCome(costToCome), mIsEvaluated(false), mPred(pred), mPos(0)
{

}


Node* Node::findSuccessor(int i, double resolution, int nbAnglesToDrive, double V, double W)
{
  double t = 0.0;
  double finalT = (1.5 * resolution) /  V;
  int nSteps = 100;

  double dt = finalT / nSteps;

  state currentPos = { this->x(), this->y(), this->theta() };
  state previousPos = currentPos;

  double travelledDistance = 0;
  double deltaX = 0, deltaY = 0;
  double deltaDist;

  while (t < finalT)
  {
    currentPos.x = currentPos.x + dt*V*cos(currentPos.theta);
    currentPos.y = currentPos.y + dt*V*sin(currentPos.theta);
    currentPos.theta = currentPos.theta + dt*W;

    deltaX = currentPos.x - previousPos.x;
    deltaY = currentPos.y - previousPos.y;
    deltaDist = sqrt( deltaX * deltaX + deltaY * deltaY );
    travelledDistance = travelledDistance + deltaDist;

    previousPos = currentPos;
    t = t + dt;
  }

  double weightGain = 0.05;
  double h = (1 - (-1)) / (nbAnglesToDrive - 1);
  double x = -1 + i*h;
  travelledDistance = travelledDistance * (weightGain*( x*x*x*x + 1 ) + 1);

  double newCostSoFar = this->costSoFar() + travelledDistance;

  Node* newNode = new Node(previousPos.x, previousPos.y, foldOverTwoPi(previousPos.theta));
  newNode->setCostSoFar(newCostSoFar);

  return newNode;
}
