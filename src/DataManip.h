#ifndef DA_PROJECT2_DATAMANIP_H
#define DA_PROJECT2_DATAMANIP_H

#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>

class DataManip {

    Graph graph_;
    vector<int> bestPath;
    double bestCost=numeric_limits<int>::max();


public:
    DataManip();

    std::vector<int> getBestPath() const;


    int getBestCost() const ;

    void readTourism(string filename);
    void readEdges(string filename);
    void readNodes(string filename);


    //Backtracking
    void RecursiveBackTracking(vector<int>& path,double currCost, int currPos);
    bool Solution(const std::vector<int>& path);
    bool Bound(double currCost);


    //TriangularApprox
    double TriangularApprox(vector<int>&path);
    double CalculateTourCost(vector<int>&path);



    //Other heuristics
    double NearestNeighborApprox(vector<int> &path);
    Graph getGraph();
};



#endif //DA_PROJECT2_DATAMANIP_H
