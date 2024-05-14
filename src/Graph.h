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

// Definição de INF
const int INF = std::numeric_limits<int>::max();

/**
 * @class Vertex
 * @brief Represents a vertex in the graph.
 */
class Vertex{

    vector<Edge*> adj;
    vector<Edge*> incoming;
    double longitude;
    double latitude;
    unsigned int id;
    bool visited;
    bool processing;
    unsigned indegree;
    Vertex *path = nullptr;
    double dist = 0;

public:
    int queueIndex = 0;
    /**
     * @brief Constructor for Vertex class.
     * @param id Identifier of the vertex.
     * @param id Id associated with the vertex.
     */
    Vertex(unsigned int id, double longitude, double latitude);

    /**
     * @brief Get the outgoing edges from the vertex.
     * @return Reference to the vector of outgoing edges.
     */
    vector<Edge*> &getAdj();

    /**
     * @brief Get the incoming edges to the vertex.
     * @return Reference to the vector of incoming edges.
     */
    vector<Edge*> &getIncoming();

    /**
     * @brief Get the identifier of the vertex.
     * @return Identifier of the vertex.
     */
    unsigned int getId();

    /**
     * @brief Get the path associated with the vertex.
     * @return Pointer to the path.
     */

    /**
     * @brief Get the longitude of the vertex.
     * @return Longitude of the vertex.
     */
    double getLongitude();

    /**
     * @brief Get the latitude of the vertex.
     * @return Latitude of the vertex.
     */
    double getLatitude();

    double getDist();

    Vertex* getPath();

    /**
     * @brief Check if the vertex has been visited.
     * @return True if visited, false otherwise.
     */
    bool isVisited();

    /**
    * @brief Set the identifier of the vertex.
    * @param id Identifier to be set.
    */
    void setId(unsigned int id);


    /**
     * @brief Set the visited status of the vertex.
     * @param visited Visited status to be set.
     */
    void setVisited(bool visited);

    /**
     * @brief Get the outgoing edges from the vertex.
     * @return Reference to the vector of outgoing edges.
     */
    void setPath(Vertex *path);

    void setDist(double dist);

    /**
    * @brief Add an outgoing edge from the vertex to another vertex.
    * @param dest Pointer to the destination vertex.
    * @param capacity Capacity of the edge.
    */
    Edge *addEdge(Vertex* dest, double distance);

    /**
     * @brief Remove all outgoing edges from the vertex.
     */
    void removeOutgoingEdges();

    /**
    * @brief Remove an edge with a given id.
    * @param id id of the destination vertex of the edge to be removed.
    * @return True if an edge was removed, false otherwise.
    */
    bool removeEdge(unsigned int id);

    /**
     * @brief Delete an edge from the vertex.
     * @param edge Pointer to the edge to be deleted.
     */
    void deleteEdge(Edge *edge);

    bool operator<(Vertex & vertex) const;


};

/**
 * @class Graph
 * @brief Represents a graph.
 */
class Graph {
    map< unsigned int, Vertex* > vertexSet;
public:
    /**
     * @brief Find a vertex with a given id.
     * @param id Id of the vertex to be found.
     * @return Pointer to the vertex if found, nullptr otherwise.
     */
    Vertex* findVertex(const unsigned int& id) const;

    /**
     * @brief Add a vertex to the graph.
     * @param id Identifier of the vertex.
     * @param id Id associated with the vertex.
     * @return True if the vertex was added successfully, false otherwise.
     */
    bool addVertex(unsigned int id, double longitude, double latitude);

    /**
     * @brief Add an edge between vertices with given ids.
     * @param sourceId Id of the source vertex.
     * @param destId Id of the destination vertex.
     * @param capacity Capacity of the edge.
     * @return True if the edge was added successfully, false otherwise.
     */
    bool addEdge(const unsigned int &sourceId,const unsigned int &destId, double distance);

    /**
     * @brief Get the set of vertices in the graph.
     * @return Map containing the vertices.
     */
    map<unsigned int, Vertex*> getVertexSet();

    /**
     * @brief Remove a vertex with a given id from the graph.
     * @param id Id of the vertex to be removed.
     * @return True if the vertex was removed successfully, false otherwise.
     */
    bool removeVertex(unsigned int id);

    /**
     * @brief Find an edge between vertices with given origin and destination ids.
     * @param idOrigin Id of the origin vertex.
     * @param idDest Id of the destination vertex.
     * @return Pointer to the edge if found, nullptr otherwise.
     */
    Edge* findEdge(int idOrigin,int idDest);

    map< unsigned int, Vertex* > MSTprims();

    void preOrderVisit(unsigned int id, std::vector<Vertex*> &visitedNodes);

    vector<Vertex*> findOddDegree();

    set<Edge*> perfectMatching(const std::vector<Vertex*>& oddVertices);
};

/**
 * @class Edge
 * @brief Represents an edge in the graph.
 */
class Edge{
    Vertex* dest;
    Vertex* orig;
    double distance;

public:

    /**
     * @brief Constructor for Edge class.
     * @param o Pointer to the origin vertex.
     * @param d Pointer to the destination vertex.
     * @param capacity Capacity of the edge.
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
     * @brief Get the capacity of the edge.
     * @return Capacity of the edge.
     */
    double getDistance() const;

    /**
     * @brief Set the capacity of the edge.
     * @param capacity_ Capacity to be set.
     */
    void setCapacity(double distance);

};

#endif //DA_PROJECT2_GRAPH_H
