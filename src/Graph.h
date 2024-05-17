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

    Vertex* findVertex(const int& id) const;
    bool addVertex(int id, double longitude, double latitude);
    bool addBidirectionalEdge( int &sourc, int &dest, double w);
    int getNumVertex() const;
    //bool addEdge(const unsigned int &sourceId,const unsigned int &destId, double distance);

    map<int, Vertex*> getVertexSet();

    double getDistance(Vertex* v, Vertex* w);

    //Edge* findEdge(int idOrigin,int idDest);

    std::set<Edge*> MSTprims();
    void preOrderVisit(int id, std::vector<int> &visitedNodes);

    vector<Vertex*> findOddDegree();
    set<Edge*> perfectMatching(const std::vector<Vertex*>& oddVertices);
};


class Edge{
    Vertex* dest;
    Vertex* orig;
    double distance;
    Edge *reverse = nullptr;
public:

    Edge(Vertex* o, Vertex* d , double distance);
    Vertex* getDest() const;
    Vertex* getOrig() const;
    double getDistance() const;


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

    Vertex(int id, double longitude, double latitude);
    bool operator<(Vertex & vertex) const;

    int getId();
    unordered_map< int ,Edge*> &getAdj();
    set<Edge*,CompareEdge> &getAdjSet();
    unordered_map<int ,Edge*> &getIncoming();
    bool isVisited();
    double getDist();
    Vertex* getPath();
    double getLongitude();
    double getLatitude();
    vector<Vertex *> getChildren();

    void setVisited(bool visited);
    void setDist(double dist);
    void setPath(Vertex *path);
    void setId(int id);
    Edge *addEdge(Vertex* dest, double distance);
    void addChild(Vertex* v);
    void clearChildren();

};

#endif //DA_PROJECT2_GRAPH_H
