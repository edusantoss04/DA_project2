/**
 * @file Graph.h
 * @brief Definition of Graph class and its associated classes.
 */
#ifndef DA_PROJECT1_GRAPH_H
#define DA_PROJECT1_GRAPH_H

#include <map>
#include <string>
#include <vector>

using namespace std;
class Edge;
class Graph;
class Vertex;

/**
 * @class Vertex
 * @brief Represents a vertex in the graph.
 */
class Vertex{

    vector<Edge*> adj;
    vector<Edge*> incoming;
    unsigned int id;
    string code;
    bool visited;
    bool processing;
    unsigned indegree;
    Edge* path;


public:

    /**
     * @brief Constructor for Vertex class.
     * @param id Identifier of the vertex.
     * @param code Code associated with the vertex.
     */
    Vertex(unsigned int id,string code);

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
     * @brief Get the code associated with the vertex.
     * @return Code associated with the vertex.
     */
    string getCode();

    /**
     * @brief Get the path associated with the vertex.
     * @return Pointer to the path.
     */
    Edge* getPath();

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
     * @brief Set the code associated with the vertex.
     * @param code Code to be set.
     */
    void setCode(string code);

    /**
     * @brief Set the visited status of the vertex.
     * @param visited Visited status to be set.
     */
    void setVisited(bool visited);

    /**
     * @brief Get the outgoing edges from the vertex.
     * @return Reference to the vector of outgoing edges.
     */
    void setPath(Edge* edj);

    /**
    * @brief Add an outgoing edge from the vertex to another vertex.
    * @param dest Pointer to the destination vertex.
    * @param capacity Capacity of the edge.
    */
    void addEdge(Vertex* dest, unsigned int capacity);

    /**
     * @brief Remove all outgoing edges from the vertex.
     */
    void removeOutgoingEdges();

    /**
    * @brief Remove an edge with a given code.
    * @param code Code of the destination vertex of the edge to be removed.
    * @return True if an edge was removed, false otherwise.
    */
    bool removeEdge(string code);

    /**
     * @brief Delete an edge from the vertex.
     * @param edge Pointer to the edge to be deleted.
     */
    void deleteEdge(Edge *edge);

    /**
     * @brief Get the current flow through the outgoing edges of the vertex.
     * @return Current flow through the outgoing edges.
     */
    unsigned int getCurrentFlow();
};

/**
 * @class Graph
 * @brief Represents a graph.
 */
class Graph {
    map< string , Vertex* > vertexSet;
public:
    /**
     * @brief Find a vertex with a given code.
     * @param code Code of the vertex to be found.
     * @return Pointer to the vertex if found, nullptr otherwise.
     */
    Vertex* findVertex(const string& code) const;

    /**
     * @brief Add a vertex to the graph.
     * @param id Identifier of the vertex.
     * @param code Code associated with the vertex.
     * @return True if the vertex was added successfully, false otherwise.
     */
    bool addVertex(unsigned int id,string code);

    /**
     * @brief Add an edge between vertices with given codes.
     * @param sourceCode Code of the source vertex.
     * @param destCode Code of the destination vertex.
     * @param capacity Capacity of the edge.
     * @return True if the edge was added successfully, false otherwise.
     */
    bool addEdge(const string &sourceCode,const string &destCode, unsigned int capacity);

    /**
     * @brief Get the set of vertices in the graph.
     * @return Map containing the vertices.
     */
    map<string, Vertex*> getVertexSet();

    /**
     * @brief Remove a vertex with a given code from the graph.
     * @param code Code of the vertex to be removed.
     * @return True if the vertex was removed successfully, false otherwise.
     */
    bool removeVertex(string code);

    /**
     * @brief Find an edge between vertices with given origin and destination codes.
     * @param codeOrigin Code of the origin vertex.
     * @param codeDest Code of the destination vertex.
     * @return Pointer to the edge if found, nullptr otherwise.
     */
    Edge* findEdge(string codeOrigin,string codeDest);

};

/**
 * @class Edge
 * @brief Represents an edge in the graph.
 */
class Edge{
    Vertex* dest;
    Vertex* orig;
    unsigned int capacity;
    unsigned int flow;

public:

    /**
     * @brief Constructor for Edge class.
     * @param o Pointer to the origin vertex.
     * @param d Pointer to the destination vertex.
     * @param capacity Capacity of the edge.
     */
    Edge(Vertex* o, Vertex* d , unsigned int capacity);

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
    unsigned int getCapacity() const;

    /**
     * @brief Get the flow through the edge.
     * @return Flow through the edge.
     */
    unsigned int getFlow();

    /**
    * @brief Set the flow through the edge.
    * @param flow_ Flow to be set.
    */
    void setFlow(unsigned int flow_);

    /**
     * @brief Set the capacity of the edge.
     * @param capacity_ Capacity to be set.
     */
    void setCapacity(unsigned int capacity_);

};

#endif //DA_PROJECT1_GRAPH_H
