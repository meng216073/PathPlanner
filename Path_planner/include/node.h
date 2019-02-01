//  node.h
//  Node of a path (definition).

#ifndef NODE_H
#define NODE_H

typedef struct {
  double x;
  double y;
  double theta;
  double z;
} state;

class Node {

private:
  // Attributes
  double mX;
  double mY;
  double mTheta;
  double mZ;
  double mCostSoFar; // g
  double mCostToCome; // h
  bool mIsEvaluated;
  const Node* mPred;
  int mPos;

public:
  // Constructor
  Node();
  Node(const state& st);
  Node(double x, double y, double theta);
  Node(double x, double y, double theta, double z);
  Node(double x, double y, double theta, double costSoFar, double costToCome, const Node* pred);

  // Getters
  double x() const { return mX; }
  double y() const { return mY; }
  double theta() const { return mTheta; }
  double z() const { return mZ; }
  double costSoFar() const { return mCostSoFar; }
  double costToCome() const { return mCostToCome; }
  double totalCost() const { return mCostSoFar + mCostToCome; }
  bool isEvaluated() const { return mIsEvaluated; }
  const Node* pred() const { return mPred; }
  int pos() const { return mPos; }

  // Setters
  void setX(double x) { mX = x; }
  void setY(double y) { mX = y; }
  void setTheta(double theta) { mX = theta; }
  void setCostSoFar(double costSoFar) { mCostSoFar = costSoFar; }
  void setCostToCome(double costToCome) { mCostToCome = costToCome; }
  void evaluate() { mIsEvaluated = true; }
  void setPred(const Node* pred) { mPred = pred; }
  void setPos(int pos) { mPos = pos; }

  // Calculate the successor node
  Node* findSuccessor(int i, double resolution, int nbAnglesToDrive, double V, double W);
};

#endif