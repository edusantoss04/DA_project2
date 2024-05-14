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
//Vertex

Edge * Vertex::addEdge(Vertex *dest, double distance) {
    auto newEdge = new Edge(this, dest, distance);
    adj.push_back(newEdge);
    dest->incoming.push_back(newEdge);
    return newEdge;
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

Vertex* Vertex::getPath() {
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

void Vertex::setPath(Vertex *path){
    this->path = path;
}

void Vertex::setDist(double dist) {
    this->dist= dist;
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

double Vertex::getDist() {
    return dist;
}

bool Vertex::operator<(Vertex & vertex) const {
    return this->dist < vertex.dist;
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

map< unsigned int, Vertex* > Graph::MSTprims() {
    // Check if the graph is empty
    if (vertexSet.empty()) {
        return vertexSet; // Return an empty set if the graph is empty
    }
// Initialize the vertices in the graph
    for(auto v : vertexSet) {
        v.second->setPath(nullptr); // Set path to null
        v.second->setVisited(false); // Mark as not visited
        v.second->setDist(INF);
    }
// Select the first vertex as the starting point
    Vertex* s = vertexSet.begin()->second;
    s->setDist(0); // Set distance of the starting vertex to 0

// Priority queue to store vertices based on their distances
    MutablePriorityQueue<Vertex> q;
    q.insert(s);

// Main loop for the Prim's algorithm
    while(!q.empty() ) {
        double weight = 0;
        auto v = q.extractMin();

        v->setVisited(true);

        for(auto &w : vertexSet) { // vai iterar por todos os vértices porque também temos de considerar aqueles que não têm uma edge
            if (!w.second->isVisited()) { //para todos os vértices que ainda não foram visitados

                for (auto e: v->getAdj()){
                    if (e->getDest() == w.second){
                        weight = e->getDistance(); // se ele existir vai ser esse o peso
                        break;
                    }
                }
                if (weight == 0) weight = calculateDist(v->getLatitude(), v->getLongitude(), w.second->getLatitude(), w.second->getLongitude());
                // se não existir é a distância

                double oldDist = w.second->getDist();
                if(weight < oldDist) {
                    w.second->setDist(weight);
                    w.second->setPath(v);
                    if (oldDist == INF) {
                        q.insert(w.second);
                    }
                    else {
                        q.decreaseKey(w.second);
                    }
                }
            }
        }
    }
// Return the set of vertices after the Prim's algorithm completes
    return vertexSet;

}

void Graph :: preOrderVisit(unsigned int id, std::vector<Vertex*> &visitedNodes) {
    auto v= findVertex(id);
    if (v == nullptr) {
        return;
    }

    visitedNodes.push_back(v);

    for (auto w: vertexSet) {
        if (w.second->getPath() == v) {
            preOrderVisit(w.second->getId(), visitedNodes);
        }
    }
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

Edge* Graph::findEdge(int IdOrigin,int IdDest){

    for (auto v : vertexSet){
        for(auto e : v.second->getAdj()){
            if(e->getOrig()->getId() == IdOrigin && e->getDest()->getId() == IdDest){
                return e;
            }
        }
    }
    return nullptr;
}
/*
vector<Vertex*> Graph::findOddDegree() {
    std::vector<Vertex*> oddVertices;
    for (auto vertex: vertexSet) {
        if (vertex.second->getIncoming().size() % 2 != 0) {
            oddVertices.push_back(vertex.second);
        }
    }
    return oddVertices;
}
std::set<Edge*> Graph::perfectMatching(const std::vector<Vertex*>& oddVertices) {
    std::set<Edge*> matching;

    // Implementação simples: emparelha cada vértice ímpar com o mais próximo
    for (size_t i = 0; i < oddVertices.size(); ++i) {
        Vertex* v1 = oddVertices[i];
        Vertex* closestVertex = nullptr;
        double minDistance = std::numeric_limits<double>::max();

        for (size_t j = 0; j < oddVertices.size(); ++j) {
            if (i != j) {
                Vertex* v2 = oddVertices[j];
                double distance = calculateDist(v1->getLatitude(), v1->getLongitude(),
                                                    v2->getLatitude(), v2->getLongitude());
                if (distance < minDistance) {
                    minDistance = distance;
                    closestVertex = v2;
                }
            }
        }

        // Cria uma aresta entre os dois vértices para o emparelhamento
        v1->addEdge(closestVertex, minDistance);
        Edge* edge = nullptr;
        for (Edge* e : v1->getAdj()) {
            if (e->getDest() == closestVertex) {
                edge = e;
                break;
            }
        }

// Verifica se a aresta foi encontrada e a adiciona ao conjunto de emparelhamento
        if (edge != nullptr) {
            matching.insert(edge);
        }
    }

    return matching;
}
 */