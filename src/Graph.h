/**
 * @file Graph.h
 * @brief Definition of Graph class and its associated classes.
 */
#ifndef DA_PROJECT2_GRAPH_H
#define DA_PROJECT2_GRAPH_H

#include <map>
#include <string>
#include <vector>
#include "MutablePriorityQueue.h"


using namespace std;
class Edge;
class Graph;
class Vertex;
#include <limits>
#include <set>
#include <unordered_map>

// Definição de INF
const int INF = std::numeric_limits<int>::max();




class Graph {
    map<int, Vertex* > vertexSet;
public:

    /**
 * @brief Finds a vertex in the graph by its ID.
 * @param id The ID of the vertex to find.
 * @return Vertex* A pointer to the vertex if found, nullptr otherwise.
 */
    Vertex* findVertex(const int& id) const;

/**
 * @brief Adds a vertex to the graph with the specified ID and coordinates.
 * @param id The ID of the vertex to be added.
 * @param longitude The longitude coordinate of the vertex.
 * @param latitude The latitude coordinate of the vertex.
 * @return bool True if the vertex is successfully added, false if a vertex with the same ID already exists.
 */
    bool addVertex(int id, double longitude, double latitude);

/**
 * @brief Adds a bidirectional edge between two vertices with the given weight.
 * @param sourc The ID of the source vertex.
 * @param dest The ID of the destination vertex.
 * @param w The weight of the edge.
 * @return bool True if the edge is successfully added, false otherwise.
 */
    bool addBidirectionalEdge( int &sourc, int &dest, double w);

    /**
 * @brief Retrieves the number of vertices in the graph.
 * @return int The number of vertices in the graph.
 */
    int getNumVertex() const;

    /**
 * @brief Retrieves the set of vertices in the graph.
 * @return std::map<int, Vertex*> A map containing the vertices in the graph.
 */
    map<int, Vertex*> getVertexSet();

    /**
 * @brief Calculates the distance between two vertices in the graph.
 * @param v A pointer to the first vertex.
 * @param w A pointer to the second vertex.
 * @return double The distance between the two vertices.
 */
    double getDistance(Vertex* v, Vertex* w);

    /**
 * @brief Finds the Minimum Spanning Tree (MST) of the graph using Prim's algorithm.
 *
 * This method applies Prim's algorithm to find the Minimum Spanning Tree (MST) of the graph.
 * It starts from an arbitrary vertex and repeatedly adds the nearest vertex that is not already
 * in the tree until all vertices are included. The method returns a set of edges representing
 * the MST.
 *
 * @return std::set<Edge*> A set containing the edges of the Minimum Spanning Tree (MST).
 *
 * @note The time complexity of this method depends on the number of vertices and edges in the graph.
 * It involves updating priority queues and iterating over adjacent vertices, resulting in a time
 * complexity of O((V + E) * log(V)), where V is the number of vertices and E is the number of edges
 * in the graph.
 */
    std::set<Edge*> MSTprims();

/**
 * @brief Performs a pre-order traversal starting from a specified vertex in the graph.
 *
 * This method performs a pre-order traversal starting from the vertex with the given ID in the graph.
 * It recursively visits each vertex in the graph in pre-order fashion, adding the IDs of visited vertices
 * to the provided vector.
 *
 * @param id The ID of the vertex from which the pre-order traversal begins.
 * @param visitedNodes Reference to a vector to store the IDs of visited vertices in pre-order.
 *
 * @note The time complexity of this method depends on the number of vertices and edges in the graph.
 * In the worst case, if the graph is densely connected, the time complexity can be O(V^2), where V is
 * the number of vertices in the graph. However, for sparse graphs, the time complexity may be lower.
 */
    void preOrderVisit(int id, std::vector<int> &visitedNodes);
};


class Edge{
    Vertex* dest;
    Vertex* orig;
    double distance;
    Edge *reverse = nullptr;
public:

    /**
 * @brief Constructs an Edge object with the given origin, destination, and distance.
 * @param o A pointer to the origin vertex of the edge.
 * @param d A pointer to the destination vertex of the edge.
 * @param distance The distance or weight of the edge.
 */
    Edge(Vertex* o, Vertex* d , double distance);

    /**
 * @brief Get the destination vertex of the edge.
 * @return Pointer to the destination vertex.
 */
    Vertex* getDest() const;

    /**
 * @brief Get the origin vertex of the edge.
 * @return Pointer to the origin vertex.
 */
    Vertex* getOrig() const;

    /**
 * @brief Retrieves the distance attribute of the edge.
 * @return double The distance of the edge.
 */
    double getDistance() const;

/**
 * @brief Sets the reverse edge of the current edge.
 * @param reverse A pointer to the reverse edge.
 */
    void setReverse(Edge *reverse);
};
struct CompareEdge {
    bool operator()(const Edge* a, const Edge* b) const {
        return a->getDistance() < b->getDistance();
    }
};

class Vertex{

    unordered_map<int ,Edge*> adj;
    set<Edge*,CompareEdge> adjSet;
    unordered_map<int ,Edge*> incoming;
    vector<Vertex*> children;
    double longitude;
    double latitude;
    int id;
    bool visited = false;
    Vertex *path = nullptr;
    double dist = 0;

public:
    int queueIndex = 0;

    /**
 * @brief Constructs a Vertex object with the given ID, longitude, and latitude.
 * @param id The unique identifier of the vertex.
 * @param longitude The longitude coordinate of the vertex.
 * @param latitude The latitude coordinate of the vertex.
 */
    Vertex(int id, double longitude, double latitude);

    /**
 * @brief Less than comparison operator for vertices based on distance.
 * @param vertex The vertex to compare with.
 * @return bool True if the distance of the current vertex is less than the distance of the specified vertex, false otherwise.
 */
    bool operator<(Vertex & vertex) const;

    /**
 * @brief Get the identifier of the vertex.
 * @return Identifier of the vertex.
 */
    int getId();

    /**
 * @brief Retrieves the adjacency list of the vertex.
 * @return std::unordered_map<int, Edge*>& A reference to the adjacency list of the vertex.
 */
    unordered_map< int ,Edge*> &getAdj();

    /**
 * @brief Retrieves the set of adjacent edges of the vertex.
 * @return std::set<Edge*, CompareEdge>& A reference to the set of adjacent edges of the vertex.
 */
    set<Edge*,CompareEdge> &getAdjSet();

/**
 * @brief Retrieves the map of incoming edges to the vertex.
 * @return std::unordered_map<int, Edge*>& A reference to the map of incoming edges to the vertex.
 */
    unordered_map<int ,Edge*> &getIncoming();

    /**
 * @brief Check if the vertex has been visited.
 * @return True if visited, false otherwise.
 */
    bool isVisited();

/**
 * @brief Retrieves the distance attribute of the vertex.
 * @return double The distance of the vertex from a source vertex.
 */
    double getDist();

/**
 * @brief Retrieves the path attribute of the vertex.
 * @return Vertex* A pointer to the predecessor vertex in the path.
 */
    Vertex* getPath();

/**
 * @brief Retrieves the longitude coordinate of the vertex.
 * @return double The longitude coordinate of the vertex.
 */
    double getLongitude();

    /**
 * @brief Retrieves the latitude coordinate of the vertex.
 * @return double The latitude coordinate of the vertex.
 */
    double getLatitude();

    /**
 * @brief Retrieves the list of child vertices of the current vertex.
 * @return std::vector<Vertex*> A vector containing pointers to the child vertices of the current vertex.
 */
    vector<Vertex *> getChildren();

    /**
 * @brief Set the visited status of the vertex.
 * @param visited Visited status to be set.
 */
    void setVisited(bool visited);

/**
 * @brief Sets the distance attribute of the vertex.
 * @param dist The distance value to set for the vertex.
 */
    void setDist(double dist);

/**
 * @brief Sets the path attribute of the vertex.
 * @param path A pointer to the predecessor vertex in the path.
 */
    void setPath(Vertex *path);

    /**
* @brief Set the identifier of the vertex.
* @param id Identifier to be set.
*/
    void setId(int id);

/**
 * @brief Adds an edge from the current vertex to a destination vertex with a specified distance.
 * @param dest A pointer to the destination vertex of the new edge.
 * @param distance The distance of the new edge.
 * @return Edge* A pointer to the newly created edge.
 */
    Edge *addEdge(Vertex* dest, double distance);

/**
 * @brief Adds a child vertex to the current vertex.
 * @param v A pointer to the child vertex to be added.
 */
    void addChild(Vertex* v);

/**
 * @brief Clears the list of child vertices of the current vertex.
 *
 * This method clears the list of child vertices associated with the current vertex.
 * It removes all child vertices from the list, effectively emptying it.
 */
    void clearChildren();

};

#endif //DA_PROJECT2_GRAPH_H
