#ifndef DA_PROJECT2_DATAMANIP_H
#define DA_PROJECT2_DATAMANIP_H

#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <unordered_set>

class DataManip {

    Graph graph_;
    vector<int> bestPath;
    double bestCost=numeric_limits<int>::max();


public:
    DataManip();
    void clearData();

    std::vector<int> getBestPath() const;


    int getBestCost() const ;

    void readTourism(string filename);
    void readEdges(string filename);
    void readNodes(string filename);
    void readToy(string filename);
    void readEdgesLarge(string filename);
    //Backtracking
    void RecursiveBackTracking(vector<int>& path,double currCost, int currPos);
    bool Solution(const std::vector<int>& path);
    bool Bound(double currCost);


    //TriangularApprox
    double TriangularApprox(vector<int>&path);
    double CalculateTourCost(vector<int>&path);



    //Other heuristics
    double NearestNeighborApprox(vector< int> &path);
    double simulatedAnnealing(vector<int>& path, double initialTemperature = 5000.0, double coolingRate = 0.95);


    void resetGraph(std::unordered_set<int> &unvisitedNodes);
    int findNearestNeighbor(int currentNode, double &minDistance);
    int findNearestNeighborNotConnected(int currentNode, double &minDistance);
    double NearestNeighborApproxNotConnected(std::vector<int> &route,int startNode);


    Graph getGraph();
};



#endif //DA_PROJECT2_DATAMANIP_H
