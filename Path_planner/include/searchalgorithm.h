
//  searchalgorithm.h
//  Base class definition for algorithm families.

#ifndef SEARCHALGORITHM_H
#define SEARCHALGORITHM_H

#include "searchgrid.h"

class SearchAlgorithm {

public:
  // Perform the path search
  virtual const Node* launchSearch(const SearchGrid& searchGrid) = 0;
  virtual ~SearchAlgorithm() {};
};

#endif
