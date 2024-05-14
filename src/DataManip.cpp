#include <iomanip>
#include "DataManip.h"
#include <vector>
#include <limits>
#include <cmath>
#include <unordered_set>
#include <stack>

using namespace std;
DataManip::DataManip() {}

std::vector<int> DataManip::getBestPath() const {
    return bestPath;
}

int DataManip::getBestCost() const {
    return bestCost;
}
void DataManip::readTourism(string filename) {

    ifstream in(filename);
    unsigned int destino, origem;
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


            if(graph_.findVertex(origem)== nullptr){
                graph_.addVertex(origem,0,0);
            }
            if(graph_.findVertex(destino)== nullptr){
                graph_.addVertex(destino,0,0);
            }
            graph_.addEdge(origem, destino, distancia);
            graph_.addEdge(destino, origem, distancia);

        }

    } else
        cout << "Could not open the file\n";
}
void DataManip::readEdges(string filename) {

    ifstream in(filename);
    unsigned int destino, origem;
    double distancia;
    string line;


    // Verifica se a primeira linha é "origem,destino,distancia"


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


            if(graph_.findVertex(origem)== nullptr){
                graph_.addVertex(origem,0,0);
            }
            if(graph_.findVertex(destino)== nullptr){
                graph_.addVertex(destino,0,0);
            }
            graph_.addEdge(origem, destino, distancia);
            graph_.addEdge(destino, origem, distancia);

        }

    } else
        cout << "Could not open the file\n";
}

void DataManip::readNodes(string filename) {

    ifstream in(filename);
    unsigned int id;
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
    int startVertex = 0;
    for (Edge* edge : graph_.findVertex(finalVertex)->getAdj()) {
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

    for (Edge* edge : graph_.findVertex(currPos)->getAdj()) {
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

        if (currCost + graph_.findEdge(path.back(),path[0])->getDistance() < bestCost) {
            bestCost = currCost + graph_.findEdge(path.back(),path[0])->getDistance();
            bestPath = path;

        }
        return;
    }
}


double DataManip::TriangularApprox(vector<int> &path) {
    //1. Executar o Prims para calcular o MST
    graph_.MSTprims();

    //2. Executar uma Visita pré ordem
    vector<Vertex*> minPath;
    graph_.preOrderVisit(0,minPath);

    // 3. Construir o tour H a partir do caminho pré-ordem
    vector<int> tour;
    unordered_set<int> visited;

    visited.insert(minPath[0]->getId());

    tour.push_back(minPath[0]->getId());

    for (int i = 1; i < minPath.size(); ++i) {
        if (visited.find(minPath[i]->getId()) == visited.end()) {
            tour.push_back(minPath[i]->getId());
            visited.insert(minPath[i]->getId());
        }
    }

    path = tour;

    // 4. Calcular o custo do tour
    double cost = CalculateTourCost(path);

    return cost;
}

double DataManip::CalculateTourCost(vector<int> &path) {
    double tourCost = 0.0;


    Vertex* previousVertex = graph_.findVertex(path[0]);
    for (size_t i = 1; i < path.size(); ++i) {

        Vertex* currentVertex = graph_.findVertex(path[i]);

        Edge* edge = graph_.findEdge(previousVertex->getId(), currentVertex->getId());

        if (edge != nullptr) {
            tourCost += edge->getDistance();
        } else {
            tourCost += calculateDistance(previousVertex->getLatitude(), previousVertex->getLongitude(),
                                          currentVertex->getLatitude(), currentVertex->getLongitude());
        }

        previousVertex = currentVertex;
    }

    Vertex* lastVertex = graph_.findVertex(path.back());
    Vertex* firstVertex = graph_.findVertex(path.front());

    Edge* edge = graph_.findEdge(lastVertex->getId(), firstVertex->getId());

    if (edge != nullptr) {
        tourCost += edge->getDistance();
    } else {
        tourCost += calculateDistance(lastVertex->getLatitude(), lastVertex->getLongitude(),
                                      firstVertex->getLatitude(), firstVertex->getLongitude());
    }

    return tourCost;
}

/*
double DataManip::NearestNeighborApprox(vector<int> &path) {
    // Inicia a partir do primeiro vértice como o atual
    Vertex* currentVertex = graph_.findVertex(0);

    // Inicializa o caminho com o primeiro vértice
    vector<int> tour;
    tour.push_back(currentVertex->getId());

    // Conjunto para manter controle dos vértices visitados
    unordered_set<int> visited;
    visited.insert(currentVertex->getId());

    // Repete até que todos os vértices tenham sido visitados
    while (visited.size() < graph_.getVertexSet().size()) {
        double minDistance = numeric_limits<double>::max();
        Vertex* nextVertex = nullptr;

        // Encontra o vizinho mais próximo não visitado
        for (Edge* edge : currentVertex->getAdj()) {
            Vertex* neighbor = edge->getDest();
            if (visited.find(neighbor->getId()) == visited.end() && edge->getDistance() < minDistance) {
                minDistance = edge->getDistance();
                nextVertex = neighbor;
            }
        }

        // Se não houver vizinho alcançável, calcule a distância para o vértice mais próximo não visitado
        if (nextVertex == nullptr) {
            for (auto v : graph_.getVertexSet()) {
                if (visited.find(v.second->getId()) == visited.end()) {
                    double distance = calculateDistance(currentVertex->getLatitude(), currentVertex->getLongitude(),
                                                        v.second->getLatitude(), v.second->getLongitude());
                    if (distance < minDistance) {
                        minDistance = distance;
                        nextVertex = v.second;
                    }
                }
            }
        }

        // Adiciona o próximo vértice ao caminho e marca como visitado
        if (nextVertex != nullptr) {
            tour.push_back(nextVertex->getId());
            visited.insert(nextVertex->getId());
            currentVertex = nextVertex;
        }
    }

    // Adiciona a distância do último vértice de volta ao primeiro vértice para fechar o ciclo
    Vertex* lastVertex = graph_.findVertex(tour.back());
    Vertex* firstVertex = graph_.findVertex(tour.front());
    Edge* edge = graph_.findEdge(lastVertex->getId(), firstVertex->getId());
    if (edge != nullptr) {
        tour.push_back(firstVertex->getId());
    }

    // Calcula o custo total do tour
    double cost = CalculateTourCost(tour);

    // Atualiza o vetor de caminho de saída
    path = tour;

    return cost;
}
*/
Graph DataManip::getGraph() {
    return graph_;
}
