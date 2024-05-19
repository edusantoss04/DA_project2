#include <iomanip>
#include "DataManip.h"
#include <vector>
#include <limits>
#include <cmath>
#include <unordered_set>
#include <stack>
#include <random>
#include <chrono>
#include <algorithm>

using namespace std;
DataManip::DataManip() {}

void DataManip::clearData() {
    bestPath.clear();
    bestCost = numeric_limits<int>::max();
    graph_ = Graph();
}

std::vector<int> DataManip::getBestPath() const {
    return bestPath;
}

int DataManip::getBestCost() const {
    return bestCost;
}
void DataManip::readTourism(string filename) {

    ifstream in(filename);
    int destino, origem;
    string labelo,labeld;
    double distancia;
    string line;

    getline(in, line);

    if (in.is_open()) {

        while(getline(in, line)){

            istringstream iss(line);
            iss >> origem;

            iss.ignore();
            iss >> destino;

            iss.ignore();
            iss >> distancia;

            iss.ignore();
            iss >> labelo;
            iss.ignore();
            iss>>labeld;



            graph_.addVertex(origem,0,0);
            graph_.addVertex(destino,0,0);
            graph_.addBidirectionalEdge(origem, destino, distancia);

        }

    } else
        cout << "Could not open the file\n";
}
void DataManip::readToy(std::string filename) {
    ifstream in(filename);

    int destino, origem;
    double distancia;
    string line;


    if (in.is_open()) {

        while(getline(in, line)){
            if (line == "origem,destino,distancia"||line == "origem,destino,haversine_distance") {
                std::getline(in, line);
            }

            istringstream iss(line);
            iss >> origem;

            iss.ignore();
            iss >> destino;

            iss.ignore();
            iss >> distancia;

            graph_.addVertex(origem,0,0);
            graph_.addVertex(destino,0,0);
            graph_.addBidirectionalEdge(origem, destino, distancia);

        }

    } else
        cout << "Could not open the file\n";
}

void DataManip::readEdges(string filename) {

    ifstream in(filename);
     int destino, origem;
    double distancia;
    string line;

    if (in.is_open()) {

        while(getline(in, line)){
            if (line == "origem,destino,distancia"||line == "origem,destino,haversine_distance") {
                std::getline(in, line);
            }

            istringstream iss(line);
            iss >> origem;

            iss.ignore();
            iss >> destino;

            iss.ignore();
            iss >> distancia;
            graph_.addVertex(origem,0,0);
            graph_.addVertex(destino,0,0);
            graph_.addBidirectionalEdge(origem, destino, distancia);

        }

    } else
        cout << "Could not open the file\n";
}
void DataManip::readEdgesLarge(string filename) {

    ifstream in(filename);
     int destino, origem;
    double distancia;
    string line;

    if (in.is_open()) {
        std::getline(in, line);
        while(getline(in, line)){

            istringstream iss(line);
            iss >> origem;

            iss.ignore();
            iss >> destino;

            iss.ignore();
            iss >> distancia;
            graph_.addBidirectionalEdge(origem, destino, distancia);

        }

    } else
        cout << "Could not open the file\n";
}
void DataManip::readNodes(string filename) {

    ifstream in(filename);
    int id;
    double longitude, latitude;
    string line;

    getline(in, line);

    if (in.is_open()) {

        while(getline(in, line)){

            istringstream iss(line);

            iss >> id;
            iss.ignore();
            iss >> fixed >> setprecision(15) >> longitude;
            iss.ignore();
            iss >> fixed >> setprecision(15) >> latitude;


            graph_.addVertex(id, longitude, latitude);

        }

    } else
        cout << "Could not open the file\n";
}
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {

    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = (lat1) * M_PI / 180.0;
    lat2 = (lat2) * M_PI / 180.0;

    double a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
    double rad = 6371;
    double c = 2 * asin(sqrt(a));
    return rad * c; //in km
}
bool DataManip::Solution(const vector<int>& path) {
    if (path.size() != graph_.getVertexSet().size()) {
        return false;
    }

    int finalVertex = path.back();
    int startVertex = path[0];
    for (auto pair : graph_.findVertex(finalVertex)->getAdj()) {
        Edge* edge = pair.second;
        if (edge->getDest()->getId() == startVertex) {
            return true;
        }
    }

    return false;
}

bool DataManip::Bound(double currCost) {
    return currCost < bestCost;
}

void DataManip::RecursiveBackTracking(vector<int>& path, double currCost, int currPos) {

    for (auto pair : graph_.findVertex(currPos)->getAdj()) {
        Edge *edge = pair.second;
        int nextVertex = edge->getDest()->getId();
        if (edge && !edge->getDest()->isVisited()) {

            if (Bound(currCost + edge->getDistance())) {

                path.push_back(nextVertex);

                edge->getDest()->setVisited(true);

                RecursiveBackTracking(path, currCost + edge->getDistance(), nextVertex);

                edge->getDest()->setVisited(false);
                path.pop_back();
            }
        }

    }
    if (Solution(path)) {
        auto vertex1 = graph_.findVertex(path.back());
        auto vertex2 = graph_.findVertex(path[0]);
        double newCost = graph_.getDistance(vertex1,vertex2);
        if (currCost + newCost < bestCost) {
            bestCost = currCost + newCost;
            bestPath = path;

        }
        return;
    }
}


double DataManip::TriangularApprox(vector<int> &path) {
    //1. Executar o Prims para calcular o MST
    graph_.MSTprims();

    //2. Executar uma Visita pré ordem
    vector<int> minPath;
    graph_.preOrderVisit(0,minPath);

    // 3. Construir o tour H a partir do caminho pré-ordem
    vector<int> tour;
    unordered_set<int> visited;
    visited.insert(minPath[0]);

    tour.push_back(minPath[0]);

    for (int i = 1; i < minPath.size(); ++i) {
        if (visited.find(minPath[i]) == visited.end()) {
            tour.push_back(minPath[i]);
            visited.insert(minPath[i]);
        }
    }

    path = tour;

    // 4. Calcular o custo do tour
    double cost = CalculateTourCost(path);

    return cost;
}

double DataManip::CalculateTourCost(vector<int> &path) {
    double tourCost = 0.0;
    if(path.empty())return 0.0;

    Vertex* previousVertex = graph_.findVertex(path[0]);
    for (size_t i = 1; i < path.size(); ++i) {

        Vertex* currentVertex = graph_.findVertex(path[i]);


        double distance =  graph_.getDistance(previousVertex,currentVertex);
        if (distance != 0) {
            tourCost += distance;
        } else {
            tourCost += calculateDistance(previousVertex->getLatitude(), previousVertex->getLongitude(),
                                          currentVertex->getLatitude(), currentVertex->getLongitude());
        }

        previousVertex = currentVertex;
    }

    Vertex* lastVertex = graph_.findVertex(path.back());
    Vertex* firstVertex = graph_.findVertex(path.front());

    double distance1 =  graph_.getDistance(lastVertex,firstVertex);

    if (distance1 != 0) {
        tourCost += distance1;
    } else {
        tourCost += calculateDistance(lastVertex->getLatitude(), lastVertex->getLongitude(),
                                      firstVertex->getLatitude(), firstVertex->getLongitude());
    }

    return tourCost;
}



void DataManip::resetGraph(std::unordered_set<int> &unvisitedNodes) {
    for (const auto &v : graph_.getVertexSet()) {
        v.second->setVisited(false);
        unvisitedNodes.insert(v.second->getId());
    }
}

int DataManip::findNearestNeighbor(int currentNode, double &minDistance) {
    int nearestNeighbor = -1;
    minDistance = std::numeric_limits<double>::max();
    Vertex *currentVertex = graph_.findVertex(currentNode);

    for (const auto &pair : currentVertex->getAdj()) {
        Edge* edge = pair.second;
        Vertex *vertex = edge->getDest();
        double distance = edge->getDistance();

        if (!vertex->isVisited() && distance < minDistance) {
            nearestNeighbor = vertex->getId();
            minDistance = distance;
        }
    }

    if ((nearestNeighbor == -1) || (minDistance > 9000) ) {
        for (const auto &v : graph_.getVertexSet()) {
            Vertex *vertex = v.second;
            if (!vertex->isVisited()) {
                double distance = graph_.getDistance(currentVertex, vertex);
                if (distance < minDistance) {
                    nearestNeighbor = vertex->getId();
                    minDistance = distance;
                }
            }
        }
    }

    return nearestNeighbor;
}

int DataManip::findNearestNeighborNotConnected(int currentNode, double &minDistance) {
    int nearestNeighbor = -1;
    minDistance = std::numeric_limits<double>::max();
    Vertex *currentVertex = graph_.findVertex(currentNode);

    for (const auto &pair : currentVertex->getAdj()) {
        Edge* edge = pair.second;
        Vertex *vertex = edge->getDest();
        double distance = edge->getDistance();

        if (!vertex->isVisited() && distance < minDistance) {
            nearestNeighbor = vertex->getId();
            minDistance = distance;
        }
    }

    if (nearestNeighbor == -1) {
       return -1;
    }

    return nearestNeighbor;
}

double DataManip::NearestNeighborApprox(std::vector<int> &route) {
    std::unordered_set<int> unvisitedNodes;
    double totalCost = 0.0;

    resetGraph(unvisitedNodes);

    int currentNode = 0;
    route.push_back(currentNode);
    graph_.findVertex(currentNode)->setVisited(true);
    unvisitedNodes.erase(currentNode);

    while (!unvisitedNodes.empty()) {
        double minDistance;
        int nextNode = findNearestNeighbor(currentNode, minDistance);

        if (nextNode == -1) {
            break;
        }

        totalCost += minDistance;
        graph_.findVertex(nextNode)->setVisited(true);
        route.push_back(nextNode);
        unvisitedNodes.erase(nextNode);
        currentNode = nextNode;
    }

    route.push_back(0);
    for (const auto &pair : graph_.findVertex(currentNode)->getAdj()) {
        Edge *edge = pair.second;
        if (edge->getDest()->getId() == 0) {
            totalCost += edge->getDistance();
            break;
        }
    }

    return totalCost;
}

double DataManip::NearestNeighborApproxNotConnected(std::vector<int> &route,int startNode) {
    std::unordered_set<int> unvisitedNodes;
    double totalCost = 0.0;

    resetGraph(unvisitedNodes);

    int currentNode = startNode;
    route.push_back(currentNode);
    graph_.findVertex(currentNode)->setVisited(true);
    unvisitedNodes.erase(currentNode);

    while (!unvisitedNodes.empty()) {
        double minDistance;
        int nextNode = findNearestNeighborNotConnected(currentNode, minDistance);

        if (nextNode == -1) {
            cout << "No possible path starting at node "<< startNode << endl ;
            return 0;
        }

        totalCost += minDistance;
        graph_.findVertex(nextNode)->setVisited(true);
        route.push_back(nextNode);
        unvisitedNodes.erase(nextNode);
        currentNode = nextNode;
    }

    route.push_back(startNode);
    for (const auto &pair : graph_.findVertex(currentNode)->getAdj()) {
        Edge *edge = pair.second;
        if (edge->getDest()->getId() == startNode) {
            totalCost += edge->getDistance();
            break;
        }
    }

    return totalCost;
}

double DataManip::simulatedAnnealing(std::vector<int>& path, double initialTemperature, double coolingRate) {
    int numVertices = graph_.getVertexSet().size();
    int maxIterations =  100;

    std::vector<int> currentSolution = path;
    bestPath = currentSolution;
    double currentCost = CalculateTourCost(currentSolution);
    bestCost = currentCost;

    const double minTemperature = 1e-6;
    double temperature = initialTemperature;

    std::mt19937 randomGenerator(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> vertexDist(1, numVertices - 1);
    std::uniform_real_distribution<double> probabilityDist(0.0, 1.0);

    while (temperature > minTemperature) {
        for (int iteration = 0; iteration < maxIterations / (1 + log(1 + temperature)); ++iteration) {
            std::vector<int> neighborSolution = currentSolution;
            int swapIndex1 = vertexDist(randomGenerator);
            int swapIndex2 = vertexDist(randomGenerator);
            while (swapIndex1 == swapIndex2) {
                swapIndex2 = vertexDist(randomGenerator);
            }

            std::swap(neighborSolution[swapIndex1], neighborSolution[swapIndex2]);

            double neighborCost = CalculateTourCost(neighborSolution);

            double acceptanceProbability = exp((currentCost - neighborCost) / temperature);

            if (neighborCost < currentCost || probabilityDist(randomGenerator) < acceptanceProbability) {
                currentSolution = neighborSolution;
                currentCost = neighborCost;
            }

            if (currentCost < bestCost) {
                bestPath = currentSolution;
                bestCost = currentCost;
            }
        }

        temperature *= coolingRate;
    }

    std::cout << "Best cost: " << bestCost << std::endl;

    path = bestPath;
    return bestCost;
}

Graph DataManip::getGraph() {
    return graph_;
}





