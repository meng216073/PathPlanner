
//  algorithmfactory.h
//  Factory used to instanciate search algorithms (definition).

#ifndef ALGORITHMFACTORY_H
#define ALGORITHMFACTORY_H

#include <string>
#include <map>
#include "hybridalgorithm.h"

class HybridAlgorithmFactory {

private:
  // Attributes
  std::map< std::string, HybridAlgorithm* > f_map;

  HybridAlgorithmFactory();

public:
  // Singleton => copy forbidden
  HybridAlgorithmFactory(const HybridAlgorithmFactory&) = delete;
  HybridAlgorithmFactory(HybridAlgorithmFactory&&) = delete;
  HybridAlgorithmFactory& operator=(const HybridAlgorithmFactory&) = delete;
  HybridAlgorithmFactory& operator=(HybridAlgorithmFactory&&) = delete;

  ~HybridAlgorithmFactory();

  // Get single instance
  static HybridAlgorithmFactory& getFactory()
  {
    static HybridAlgorithmFactory instance;
    return instance;
  }

  // Register algorithm types (A*, A* Hybrid...)
  void registerAlgo(const std::string& name, HybridAlgorithm* algo);

  // Return the requested search algorithm
  HybridAlgorithm* getSearchAlgorithm(const std::string& name) const;
};

class BasicAlgorithmFactory {

private:
  // Attributes
  std::map< std::string, BasicAlgorithm* > f_map;

  BasicAlgorithmFactory();

public:
  // Singleton => copy forbidden
  BasicAlgorithmFactory(const BasicAlgorithmFactory&) = delete;
  BasicAlgorithmFactory(BasicAlgorithmFactory&&) = delete;
  BasicAlgorithmFactory& operator=(const BasicAlgorithmFactory&) = delete;
  BasicAlgorithmFactory& operator=(BasicAlgorithmFactory&&) = delete;

  ~BasicAlgorithmFactory();

  // Get single instance
  static BasicAlgorithmFactory& getFactory()
  {
    static BasicAlgorithmFactory instance;
    return instance;
  }

  // Register algorithm types (A*, A* Hybrid...)
  void registerAlgo(const std::string& name, BasicAlgorithm* algo);

  // Return the requested search algorithm
  BasicAlgorithm* getSearchAlgorithm(const std::string& name) const;
};

#endif