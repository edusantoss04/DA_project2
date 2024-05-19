#include <iostream>
#include <cmath>
#include "Graph.h"

double calculateDist(double lat1, double lon1, double lat2, double lon2) {

    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;

    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    return rad * c; //in km
}

//............................VERTEX..................................................

Edge * Vertex::addEdge(Vertex *dest, double distance) {
    auto newEdge = new Edge(this, dest, distance);
    adj.insert({dest->getId(),newEdge});
    dest->incoming.insert({this->id,newEdge});
    adjSet.insert(newEdge);
    return newEdge;
}

//constructor
Vertex::Vertex(int id, double longitude, double latitude) {
    this->id = id;
    this->longitude = longitude;
    this->latitude = latitude;
}

// getters
int Vertex::getId() {
    return id;
}

Vertex* Vertex::getPath() {
    return path;
}


bool Vertex::isVisited() {
    return visited;
}

unordered_map<int ,Edge*> &Vertex::getAdj() {
    return adj;
}


set<Edge*,CompareEdge> &Vertex::getAdjSet(){
    return adjSet;
}

unordered_map< int ,Edge*> &Vertex::getIncoming() {
    return incoming;
}

double Vertex::getLatitude() {
    return latitude;
}

double Vertex::getLongitude() {
    return longitude;
}

double Vertex::getDist() {
    return dist;
}


vector<Vertex *> Vertex::getChildren(){
    return this->children;
}

//setters
void Vertex::setId(int id) {
    this->id = id;
}

void Vertex::setVisited(bool visited) {
    this->visited= visited;
}

void Vertex::setPath(Vertex *path){
    this->path = path;
}

void Vertex::setDist(double dist) {
    this->dist= dist;
}

void Vertex::addChild(Vertex* v){
    children.push_back(v);
}

void Vertex::clearChildren(){
    children.clear();
}


bool Vertex::operator<(Vertex & vertex) const {
    return this->dist < vertex.dist;
}


//....................GRAPH................................................

Vertex *Graph::findVertex(const int &id) const {
    auto it = vertexSet.find(id);

    if(it!= vertexSet.end()){
        return it->second;
    }
    return nullptr;
}

bool Graph::addVertex( int id, double longitude, double latitude) {
    if(findVertex(id)!= nullptr){
        return false;
    }
    vertexSet.insert({id,new Vertex(id, longitude, latitude)});
    return true;
}

bool Graph::addBidirectionalEdge( int &sourc, int &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    auto e1 = v1->addEdge(v2, w);
    auto e2 = v2->addEdge(v1, w);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

double Graph ::getDistance(Vertex* v, Vertex* w){

    for (auto e: v->getAdj()){
        if (e.second->getDest()->getId() == w->getId()){
           return e.second->getDistance();
        }
    }

    return calculateDist(v->getLatitude(),v->getLongitude(),w->getLatitude(),w->getLongitude());
}

int Graph::getNumVertex() const {
    return vertexSet.size();
}

map< int, Vertex *> Graph::getVertexSet() {
    return vertexSet;
}

std::set<Edge*> Graph::MSTprims() {
    std::set<Edge*> mstEdges;

    if (vertexSet.empty()) {
        return mstEdges;
    }

    for (auto v : vertexSet) {
        v.second->setPath(nullptr);
        v.second->setVisited(false);
        v.second->setDist(std::numeric_limits<double>::infinity());
    }

    Vertex* s = vertexSet.begin()->second;
    s->setDist(0);

    MutablePriorityQueue<Vertex> q;
    q.insert(s);

    while (!q.empty()) {
        auto v = q.extractMin();
        v->setVisited(true);

        for (auto &adj : v->getAdj()) {
            Vertex* w = adj.second->getDest();
            if (!w->isVisited()) {
                double weight = adj.second->getDistance();
                if (weight == 0) {
                    weight = calculateDist(v->getLatitude(), v->getLongitude(), w->getLatitude(), w->getLongitude());
                }

                double oldDist = w->getDist();
                if (weight < oldDist) {
                    w->setDist(weight);
                    w->setPath(v);
                    if (oldDist == std::numeric_limits<double>::infinity()) {
                        q.insert(w);
                    } else {
                        q.decreaseKey(w);
                    }
                }

                if (w->getDist() == weight) {
                    mstEdges.insert(adj.second);
                }
            }
        }
    }

    return mstEdges;
}
void Graph :: preOrderVisit( int id, std::vector<int> &visitedNodes) {
    auto v= findVertex(id);
    if (v == nullptr) {
        return;
    }

    visitedNodes.push_back(v->getId());

    for (auto w: vertexSet) {
        if (w.second->getPath() == v) {
            preOrderVisit(w.first, visitedNodes);
        }
    }
}

//............................EDGE........................................
Edge::Edge(Vertex *o, Vertex *d, double distance) {
    this->orig = o;
    this->dest= d;
    this->distance= distance;
}

Vertex *Edge::getDest() const {
    return dest;
}

Vertex *Edge::getOrig() const{
    return orig;
}

double Edge::getDistance() const {
    return distance;
}

void Edge::setReverse(Edge *reverse) {
    this->reverse = reverse;
}
