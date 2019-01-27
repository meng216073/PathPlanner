
//  algorithmfactory.cpp
//  Factory used to instanciate search algorithms (implementation).

#include "algorithmfactory.h"
#include "astarhybrid.h"

using namespace std;

// HybridAlgorithmFactory
HybridAlgorithmFactory::~HybridAlgorithmFactory()
{
  for(map< string, HybridAlgorithm* >::iterator it = f_map.begin(); it != f_map.end(); it++)
  {
    delete it->second;
  }

  f_map.clear();
}

HybridAlgorithmFactory::HybridAlgorithmFactory()
{
  HybridAlgorithmFactory::registerAlgo("AStarHybrid", AStarHybrid::create());
}

void HybridAlgorithmFactory::registerAlgo(const string& name, HybridAlgorithm* algo)
{
  if(f_map.find(name) == f_map.end())
  {
    f_map[name] = algo;
  }
}

HybridAlgorithm* HybridAlgorithmFactory::getSearchAlgorithm(const string& name) const
{
  map< string, HybridAlgorithm* >::const_iterator it = f_map.find(name);

  if(it != f_map.end())
  {
    return it->second;
  }

  return nullptr;
}


// BasicAlgorithmFactory
BasicAlgorithmFactory::~BasicAlgorithmFactory()
{
  for(map< string, BasicAlgorithm* >::iterator it = f_map.begin(); it != f_map.end(); it++)
  {
    delete it->second;
  }

  f_map.clear();
}

BasicAlgorithmFactory::BasicAlgorithmFactory()
{

}

void BasicAlgorithmFactory::registerAlgo(const string& name, BasicAlgorithm* algo)
{
  if(f_map.find(name) == f_map.end())
  {
    f_map[name] = algo;
  }
}

BasicAlgorithm* BasicAlgorithmFactory::getSearchAlgorithm(const string& name) const
{
  map< string, BasicAlgorithm* >::const_iterator it = f_map.find(name);

  if(it != f_map.end())
  {
    return it->second;
  }

  return nullptr;
}

