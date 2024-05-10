#include <iostream>
#include "Graph.h"

//Vertex

void Vertex::addEdge(Vertex *dest, double distance) {
    auto newEdge = new Edge(this, dest,distance);
    adj.push_back(newEdge);
    dest->incoming.push_back(newEdge);
}

//constructor
Vertex::Vertex(unsigned int id, double longitude, double latitude) {
    this->id = id;
    this->longitude = longitude;
    this->latitude = latitude;
}

// getters
unsigned int Vertex::getId() {
    return id;
}

Edge* Vertex::getPath() {
    return path;
}


bool Vertex::isVisited() {
    return visited;
}

//setters
void Vertex::setId(unsigned int id) {
    this->id = id;
}

void Vertex::setVisited(bool visited) {
    this->visited= visited;
}

void Vertex::setPath(Edge* edj) {
    this->path = edj;
}

vector<Edge*> &Vertex::getAdj() {     //*
    return adj;
}

vector<Edge *> &Vertex::getIncoming() {
    return incoming;
}

void Vertex::removeOutgoingEdges() {
    auto it = adj.begin();
    while(it!= adj.end()){
        Edge *edge= *it;
        it = adj.erase(it);
        deleteEdge(edge);
    }
}

bool Vertex::removeEdge(unsigned int id) {
    bool removedEdge=false;
    auto it = adj.begin();
    while(it!=adj.end()){
        Edge *edge= *it;
        Vertex *dest = edge->getDest();
        if(dest->getId()== id){
            it=adj.erase(it);
            deleteEdge(edge);
            removedEdge = true;
        }
        else{
            it++;
        }
    }
    return removedEdge;
}

void Vertex::deleteEdge(Edge *edge) {
    Vertex *dest = edge->getDest();
    auto it = dest->incoming.begin();
    while(it != dest->incoming.end()){
        if((*it)->getOrig()->getId()==id){
            it=dest->incoming.erase(it);
        }
        else{
            it++;
        }
    }
    delete edge;
}

double Vertex::getLatitude() {
    return latitude;
}

double Vertex::getLongitude() {
    return longitude;
}

Vertex *Graph::findVertex(const unsigned int &id) const {
    auto it = vertexSet.find(id);

    if(it!= vertexSet.end()){
        return it->second;
    }
    return nullptr;
}

bool Graph::addVertex(unsigned int id, double longitude, double latitude) {
    if(findVertex(id)!= nullptr){
        return false;
    }
    vertexSet.insert({id,new Vertex(id, longitude, latitude)});
    return true;
}

bool Graph::addEdge(const unsigned int &sourceId, const unsigned int &destId, double distance) {
    auto v1 = findVertex(sourceId);
    auto v2 = findVertex(destId);
    if(v1 == nullptr|| v2 == nullptr){
        return false;
    }
    v1->addEdge(v2,distance);
    return true;
}

map<unsigned int, Vertex *> Graph::getVertexSet() {
    return vertexSet;
}


bool Graph::removeVertex(unsigned int id) {
    for(auto it = vertexSet.begin();it!= vertexSet.end();it++){
        if((*it).second->getId()== id){
            auto v = *it;
            v.second->removeOutgoingEdges();
            for(auto u: vertexSet){
                u.second->removeEdge(v.second->getId());
            }
            vertexSet.erase(it);
            delete v.second;
            return true;
        }
    }
    return false;
}

//Edge
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

void Edge::setCapacity(double distance_) {
    distance = distance_;
}

Edge* Graph::findEdge(unsigned int IdOrigin,unsigned int IdDest){

    for (auto v : vertexSet){
        for(auto e : v.second->getAdj()){
            if(e->getOrig()->getId() == IdOrigin && e->getDest()->getId() == IdDest){
                return e;
            }
        }
    }
    return nullptr;
}