#ifndef DA_PROJECT2_DATAMANIP_H
#define DA_PROJECT2_DATAMANIP_H

#include "Graph.h"
#include <iostream>
#include <fstream>
#include <sstream>

class DataManip {

    Graph graph_;

public:
    DataManip();

    void readEdges(string filename);
    void readNodes(string filename);

    /**
     * @brief Get the graph representing connections between cities, stations, and reservoirs.
     *
     * @return Graph representing connections.
     */
    Graph getGraph();
};


#endif //DA_PROJECT2_DATAMANIP_H
