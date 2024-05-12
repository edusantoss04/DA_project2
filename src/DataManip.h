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
    int bestCost=numeric_limits<int>::max();


public:
    DataManip();

    std::vector<int> getBestPath() const;


    int getBestCost() const ;

    void readTourism(string filename);
    void readEdges(string filename);
    void readNodes(string filename);
    void RecursiveBackTracking(vector<int>& path,int currCost, int currPos);
    bool Solution(const std::vector<int>& path);
    bool Bound(int currCost);
    /**
     * @brief Get the graph representing connections between cities, stations, and reservoirs.
     *
     * @return Graph representing connections.
     */
    Graph getGraph();
};


#endif //DA_PROJECT2_DATAMANIP_H
