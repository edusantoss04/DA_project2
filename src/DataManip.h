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

    /**
    * @brief Constructor for DataManip class.
    *
    * This constructor initializes a DataManip object.
    */
    DataManip();

    /**
    * @brief Clears the current data, resetting the best path, best cost, and graph.
    *
    * @note Time Complexity: O(1)
     */
    void clearData();

/**
 * @brief Retrieves the best path computed by the algorithm.
 *
 * This method returns the best path as a vector of integers.
 * The best path is assumed to be computed and stored in the class instance.
 *
 * @return std::vector<int> The best path as a vector of integers.
 *
 * @note The time complexity of this method is O(1), as it simply returns the stored vector.
 */
    std::vector<int> getBestPath() const;

/**
 * @brief Retrieves the best cost computed by the algorithm.
 *
 * This method returns the best cost as an integer.
 * The best cost is assumed to be computed and stored in the class instance.
 *
 * @return int The best cost computed by the algorithm.
 *
 * @note The time complexity of this method is O(1), as it simply returns the stored integer.
 */
    int getBestCost() const ;

/**
 * @brief Reads tourism data from a file and populates the graph.
 */
    void readTourism(string filename);

/**
 * @brief Reads edges from a file and populates the graph.
 */
    void readEdges(string filename);

/**
 * @brief Reads node information from a file and adds them to the graph.
 */
    void readNodes(string filename);

    /**
 * @brief Reads toy dataset from a file and populates the graph.
 */
    void readToy(string filename);

    /**
 * @brief Reads large edges data from a file and populates the graph.
 */
    void readEdgesLarge(string filename);

    /**
 * @brief Performs recursive backtracking to find the best path.
 *
 * This method performs recursive backtracking to explore all possible paths in the graph
 * starting from the current position. It updates the best path and cost if a better solution
 * is found during the exploration process.
 *
 * @param path A vector representing the current path being explored.
 * @param currCost The current cost of the path being explored.
 * @param currPos The current position (vertex) in the graph.
 */
    void RecursiveBackTracking(vector<int>& path,double currCost, int currPos);
    bool Solution(const std::vector<int>& path);

    /**
 * @brief Checks if the current cost is less than the best cost.
 *
 * This method checks if the current cost is less than the best cost obtained so far.
 * If the current cost is less than the best cost, it indicates that the current path
 * being explored is promising and can potentially lead to a better solution.
 *
 * @param currCost The current cost to be compared with the best cost.
 *
 * @return bool True if the current cost is less than the best cost, indicating a promising path.
 *              False otherwise.
 *
 */
    bool Bound(double currCost);

    /**
 * @brief Performs the Triangular Approximation Algorithm to find an approximate solution for the TSP.
 *
 * This method implements the Triangular Approximation Algorithm to find an approximate solution
 * for the Traveling Salesman Problem (TSP). It consists of the following steps:
 * 1. Executes Prim's algorithm to calculate the Minimum Spanning Tree (MST) of the graph.
 * 2. Performs a pre-order traversal of the MST to obtain a minimum path.
 * 3. Constructs a tour from the minimum path by visiting each vertex once.
 * 4. Calculates the cost of the tour.
 *
 * @param path Reference to a vector to store the approximate tour found by the algorithm.
 *
 * @return double The cost of the approximate tour.
 *
 * @note The time complexity of this method depends on the size and structure of the graph,
 * as it involves executing Prim's algorithm, performing a pre-order traversal,
 * constructing a tour, and calculating its cost. The overall complexity is typically
 * dominated by the time complexity of Prim's algorithm and the pre-order traversal,
 * which are both O(V^2) for dense graphs and O(V + E) for sparse graphs,
 * where V is the number of vertices and E is the number of edges in the graph.
 */
    double TriangularApprox(vector<int>&path);

/**
 * @brief Calculates the cost of a tour represented by a sequence of vertices.
 *
 * This method calculates the total cost of a tour represented by a sequence of vertices.
 * It computes the sum of distances between consecutive vertices in the tour.
 * If the distance between two vertices is not directly available in the graph,
 * it calculates the distance using the Haversine formula based on their latitude and longitude.
 *
 * @param path A vector representing the sequence of vertices in the tour.
 *
 * @return double The total cost of the tour.
 *
 * @note The time complexity of this method depends on the size and structure of the graph,
 * as it involves traversing the vertices in the given path and calculating distances between them.
 * If the distance between two vertices is not directly available in the graph,
 * calculating the distance using the Haversine formula contributes to the overall time complexity.
 */
    double CalculateTourCost(vector<int>&path);

    /**
 * @brief Performs the Nearest Neighbor Approximation Algorithm to find an approximate solution for the TSP.
 *
 * This method implements the Nearest Neighbor Approximation Algorithm to find an approximate solution
 * for the Traveling Salesman Problem (TSP). It starts from an initial vertex (usually vertex 0) and iteratively
 * selects the nearest unvisited neighbor until all vertices are visited. It returns to the start vertex to complete
 * the tour. The approximate tour and its total cost are stored in the provided route vector.
 *
 * @param route Reference to a vector to store the approximate tour found by the algorithm.
 *
 * @return double The total cost of the approximate tour.
 *
 * @note The time complexity of this method depends on the size and structure of the graph.
 * In the worst case, it iterates over all vertices in the graph and their adjacent vertices,
 * resulting in O(V^2) complexity, where V is the number of vertices in the graph.
 */
    double NearestNeighborApprox(vector< int> &path);

    /**
 * @brief Performs the Simulated Annealing algorithm to find an approximate solution for the TSP.
 *
 * This method implements the Simulated Annealing algorithm to find an approximate solution
 * for the Traveling Salesman Problem (TSP). It starts from an initial solution and iteratively
 * explores neighboring solutions, accepting worse solutions with a certain probability based on
 * the current temperature and the difference in cost between the current and neighbor solutions.
 * The algorithm gradually decreases the temperature over iterations according to a cooling rate
 * until a termination condition is met. The approximate tour and its total cost are stored in the
 * provided path vector.
 *
 * @param path Reference to a vector to store the approximate tour found by the algorithm.
 * @param initialTemperature The initial temperature parameter for the simulated annealing process.
 * @param coolingRate The cooling rate parameter for the simulated annealing process.
 *
 * @return double The total cost of the approximate tour.
 *
 * @note The time complexity of this method depends on the number of iterations and the complexity
 * of the neighbor solution generation process. Typically, the algorithm performs a large number of
 * iterations, resulting in a high time complexity. The overall time complexity is often non-deterministic
 * due to the probabilistic nature of the acceptance of worse solutions.
 */
    double simulatedAnnealing(vector<int>& path, double initialTemperature = 5000.0, double coolingRate = 0.95);

/**
 * @brief Resets the visited status of vertices in the graph and populates the set of unvisited nodes.
 *
 * This method resets the visited status of all vertices in the graph to false and populates
 * the set of unvisited nodes with the IDs of all vertices in the graph.
 *
 * @param unvisitedNodes Reference to an unordered set to store the IDs of unvisited nodes.
 *
 * @note The time complexity of this method is O(V), where V is the number of vertices in the graph,
 * as it iterates over all vertices in the graph and performs constant-time operations to reset
 * their visited status and insert their IDs into the set of unvisited nodes.
 */
    void resetGraph(std::unordered_set<int> &unvisitedNodes);

    /**
 * @brief Finds the nearest unvisited neighbor of a given vertex.
 *
 * This method finds the nearest unvisited neighbor of a given vertex in the graph.
 * It first checks for directly connected neighbors and then searches for the nearest unvisited node
 * in the entire graph if no directly connected neighbor is found or if the minimum distance is greater than 9000.
 *
 * @param currentNode The ID of the current node for which the nearest neighbor is to be found.
 * @param minDistance Reference to a double variable to store the minimum distance to the nearest neighbor.
 *
 * @return int The ID of the nearest unvisited neighbor vertex.
 *
 * @note The time complexity of this method depends on the size and structure of the graph.
 * In the worst case, if no directly connected neighbor is found, it iterates over all vertices
 * in the graph to find the nearest unvisited node, resulting in O(V) complexity,
 * where V is the number of vertices in the graph.
 */
    int findNearestNeighbor(int currentNode, double &minDistance);

    /**
 * @brief Finds the nearest unvisited neighbor directly connected to a given vertex.
 *
 * This method finds the nearest unvisited neighbor directly connected to a given vertex in the graph.
 * If no directly connected neighbor is found, it returns -1, indicating that no such neighbor exists.
 *
 * @param currentNode The ID of the current node for which the nearest neighbor is to be found.
 * @param minDistance Reference to a double variable to store the minimum distance to the nearest neighbor.
 *
 * @return int The ID of the nearest unvisited neighbor vertex directly connected to the current node,
 *             or -1 if no such neighbor exists.
 *
 * @note The time complexity of this method depends on the size and structure of the graph.
 * It iterates over the adjacent vertices of the current node to find the nearest unvisited neighbor,
 * resulting in O(E) complexity, where E is the number of edges incident to the current node.
 */
    int findNearestNeighborNotConnected(int currentNode, double &minDistance);

    /**
 * @brief Performs the Nearest Neighbor Approximation Algorithm for a graph with disconnected components.
 *
 * This method implements the Nearest Neighbor Approximation Algorithm to find an approximate solution
 * for the Traveling Salesman Problem (TSP) on a graph with disconnected components. It starts from a specified
 * start node and iteratively selects the nearest unvisited neighbor until all vertices are visited. It returns
 * to the start node to complete the tour. The approximate tour and its total cost are stored in the provided
 * route vector.
 *
 * @param route Reference to a vector to store the approximate tour found by the algorithm.
 * @param startNode The ID of the start node from which the algorithm begins the tour.
 *
 * @return double The total cost of the approximate tour.
 *
 * @note This method assumes that the graph may have disconnected components. It iterates over all unvisited
 * nodes to find the nearest neighbor, resulting in O(V^2) complexity in the worst case, where V is the number
 * of vertices in the graph.
 */
    double NearestNeighborApproxNotConnected(std::vector<int> &route,int startNode);

/**
 * @brief Retrieves the graph associated with the DataManip object.
 *
 * This method returns a copy of the graph stored within the DataManip object.
 * The graph represents the underlying structure on which various graph algorithms
 * are performed within the DataManip class.
 *
 * @return Graph The graph associated with the DataManip object.
 */
    Graph getGraph();
};



#endif //DA_PROJECT2_DATAMANIP_H
