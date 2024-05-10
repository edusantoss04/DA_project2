#include <iostream>
#include "Graph.h"

//Vertex

void Vertex::addEdge(Vertex *dest, unsigned int capacity) {
    auto newEdge = new Edge(this, dest,capacity);
    adj.push_back(newEdge);
    dest->incoming.push_back(newEdge);
}

//constructor
Vertex::Vertex(unsigned int id, string code) {
    this->id = id;
    this->code = code;
}

// getters
unsigned int Vertex::getId() {
    return id;
}

string Vertex::getCode() {
    return code;
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

void Vertex::setCode(string code) {
    this->code = code;
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

bool Vertex::removeEdge(std::string code) {
    bool removedEdge=false;
    auto it = adj.begin();
    while(it!=adj.end()){
        Edge *edge= *it;
        Vertex *dest = edge->getDest();
        if(dest->getCode()== code){
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
        if((*it)->getOrig()->getCode()==code){
            it=dest->incoming.erase(it);
        }
        else{
            it++;
        }
    }
    delete edge;
}

unsigned int Vertex::getCurrentFlow() {
    unsigned int sum=0;
    for(auto edge: this->adj){
        sum+ edge->getFlow();
    }
    return sum;
}

Vertex *Graph::findVertex(const string &code) const {
    auto it = vertexSet.find(code);

    if(it!= vertexSet.end()){
        return it->second;
    }
    return nullptr;
}

bool Graph::addVertex(unsigned int id, string code) {
    if(findVertex(code)!= nullptr){
        return false;
    }
    vertexSet.insert({code,new Vertex(id,code)});
    return true;
}

bool Graph::addEdge(const string &sourceCode, const string &destCode, unsigned int capacity) {
    auto v1 = findVertex(sourceCode);
    auto v2 = findVertex(destCode);
    if(v1 == nullptr|| v2 == nullptr){
        return false;
    }
    v1->addEdge(v2,capacity);
    return true;
}

map<string, Vertex *> Graph::getVertexSet() {
    return vertexSet;
}


bool Graph::removeVertex(string code) {
    for(auto it = vertexSet.begin();it!= vertexSet.end();it++){
        if((*it).second->getCode()== code){
            auto v = *it;
            v.second->removeOutgoingEdges();
            for(auto u: vertexSet){
                u.second->removeEdge(v.second->getCode());
            }
            vertexSet.erase(it);
            delete v.second;
            return true;
        }
    }
    return false;
}

//Edge
Edge::Edge(Vertex *o, Vertex *d, unsigned int capacity) {
    this->orig = o;
    this->dest= d;
    this->capacity= capacity;
}

Vertex *Edge::getDest() const {
    return dest;
}

Vertex *Edge::getOrig() const{
    return orig;
}

unsigned int Edge::getCapacity() const {
    return capacity;
}

unsigned int Edge::getFlow() {
    return flow;
}

void Edge::setFlow(unsigned int flow_) {
    flow = flow_;
}

void Edge::setCapacity(unsigned int capacity_) {
    capacity = capacity_;
}

Edge* Graph::findEdge(string codeOrigin,string codeDest){

    for (auto v : vertexSet){
        for(auto e : v.second->getAdj()){
            if(e->getOrig()->getCode() == codeOrigin && e->getDest()->getCode() == codeDest){
                return e;
            }
        }
    }
    return nullptr;
}