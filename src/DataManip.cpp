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

    // Verifica se a primeira linha é "origem,destino,distancia"

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
    int startVertex = 0;
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


double DataManip::NearestNeighborApprox(vector<int> &path) {
    unordered_set<int> not_visited;
    double cost = 0;

    // reset graph
    for (auto v: graph_.getVertexSet()) {
        v.second->setVisited(false);
        not_visited.insert(v.second->getId());
    }

   int current_stop = 0;
    path.push_back(current_stop);
    graph_.findVertex(current_stop)->setVisited(true);
    not_visited.erase(current_stop);

    while (!not_visited.empty()) {
        bool found = false;
        int next_stop = -1;
        double min_distance = std::numeric_limits<double>::max();
        Vertex *currentVertex = graph_.findVertex(current_stop);

        for (auto pair: currentVertex->getAdj()) {
            Edge* edge = pair.second;
            Vertex *vertex = edge->getDest();
            double distance = edge->getDistance();


            if (distance == 0) {
                distance = calculateDistance(currentVertex->getLatitude(), currentVertex->getLongitude(), vertex->getLatitude(), vertex->getLongitude());
            }

            if (!vertex->isVisited() && distance < min_distance) {
                next_stop = vertex->getId();
                min_distance = distance;
                found = true;
            }
        }

        if (found) {
            cost += min_distance;
            graph_.findVertex(next_stop)->setVisited(true);
            path.push_back(next_stop);
            not_visited.erase(next_stop);
            current_stop = next_stop;
        } else {
            break;
        }
    }
    // Return to the start vertex (0)
    path.push_back(0);
    for (auto pair: graph_.findVertex(current_stop)->getAdj()) {
        Edge *edge = pair.second;
        if (edge->getDest()->getId() == 0) {
            cost += edge->getDistance();
            break;
        }
    }

    return cost;
}

double DataManip::christofides(std::vector<int> &path) {
    // Obter a MST usando o algoritmo de Prim
    set<Edge*> mst = graph_.MSTprims();

    // Encontrar vértices de grau ímpar
    std::vector<Vertex*> odds = graph_.findOddDegree();

    // Obter o emparelhamento perfeito dos vértices de grau ímpar
    std::set<Edge*> matching = graph_.perfectMatching(odds);

    // Combinar MST e o matching para criar um multigrafo
    set<Edge*> combine_graph;
    for (auto e: mst){
        if (combine_graph.find(e)==combine_graph.end()){ // se ainda nao estiver lá
            combine_graph.insert(e);
            combine_graph.insert(e->getDest()->addEdge(e->getOrig(),e->getDistance()));
            //combine_graph.insert(new Edge(e->getDest(), e->getOrig(),e->getWeight()));
        }
    }
    bool reverse=false;
    for (Edge* edge : matching) {
        if (combine_graph.find(edge) == combine_graph.end()) {
            combine_graph.insert(edge);
            combine_graph.insert(edge->getDest()->addEdge(edge->getOrig(), edge->getDistance()));
        }
    }

    // Encontrar o caminho de Euler no multigrafo
    std::vector<Vertex*> euler_path;
    Vertex* start = graph_.getVertexSet().begin()->second; // Assumindo que o grafo não está vazio e escolhendo um vértice inicial
    std::stack<Vertex*> s;
    s.push(start);

    while (!combine_graph.empty()) {
        Vertex* current = s.top();
        Edge* next_edge = nullptr;

        // Encontrar a próxima aresta no multigrafo combinada
        for (auto pair : current->getAdj()) {
            auto e = pair.second;
            if (combine_graph.find(e) != combine_graph.end()) {
                next_edge = e;
                break;
            }
        }

        if (next_edge != nullptr) {
            s.push(next_edge->getDest());
            combine_graph.erase(next_edge);
            for (auto it = combine_graph.begin(); it != combine_graph.end(); ++it) {
                if ((*it)->getDest() == next_edge->getOrig() && (*it)->getOrig() == next_edge->getDest()) {
                    combine_graph.erase(it);
                    break;
                }
            }
        } else {
            euler_path.push_back(current);
            s.pop();
        }
    }
    euler_path.push_back(start); // Completar o ciclo de Euler

    // Converter o caminho de Euler em um ciclo Hamiltoniano
    std::set<Vertex*> visited;
    for (Vertex* vertex : euler_path) {
        if (visited.find(vertex) == visited.end()) {
            path.push_back(vertex->getId());
            visited.insert(vertex);
        }
    }
    path.push_back(start->getId()); // Fechar o ciclo Hamiltoniano

    // Calcular o custo do ciclo Hamiltoniano
    double min_cost = CalculateTourCost( path);

    return min_cost;
}



Graph DataManip::getGraph() {
    return graph_;
}





/*
double DataManip::simulatedAnnealing(vector<int>& path, double initialTemperature,
                                     double coolingRate){

    int maxIterations = 5*graph_.getVertexSet().size() * graph_.getVertexSet().size();


    // Inicializa o caminho atual e o melhor caminho
    vector<int> currentPath = path;
    bestPath= currentPath;

    // Inicializa o custo atual e o melhor custo
    double currentCost = CalculateTourCost(currentPath);
    bestCost= currentCost;

    // Inicializa a temperatura mínima
    const double minTemperature = 1e-6;
    double temperature = initialTemperature;

    // Inicializa o gerador de números aleatórios com uma semente baseada no relógio do sistema
    std::mt19937 gen(chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> randDist(1, graph_.getVertexSet().size() - 1); // Modificado para incluir o índice 0
    std::uniform_real_distribution<double> probDist(0.0, 1.0);
    // Itera até que a temperatura atinja o mínimo
    while(temperature > minTemperature){
        for(int i = 0; i < maxIterations; i++){
            // Gera um vizinho aleatório
            vector<int> neighborPath = currentPath;
            int index1 = randDist(gen);
            int index2 = randDist(gen);
            while(index1 == index2){
                index2 = randDist(gen);
            }

            // Troca os elementos nos índices index1 e index2
            swap(neighborPath[index1], neighborPath[index2]);

            // Calcula o custo do novo caminho
            double newCost = CalculateTourCost(neighborPath);

            // Calcula a probabilidade de aceitação
            double acceptanceProbability = exp((currentCost - newCost) / temperature);

            // Aceita o vizinho se for melhor ou com uma certa probabilidade se for pior
            if(newCost < currentCost || probDist(gen) < acceptanceProbability){
                currentPath = neighborPath;
                currentCost = newCost;
            }

            // Atualiza o melhor caminho
            if(currentCost < bestCost){
                bestPath = currentPath;
                bestCost = currentCost;
            }
        }

        // Resfria a temperatura
        temperature *= coolingRate;
    }
    cout << "Melhor custo: " << bestCost << endl;

    // Atualiza o caminho de entrada com o melhor caminho encontrado
    path = bestPath;
    return bestCost;
}
*/