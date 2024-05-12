#include <iomanip>
#include "DataManip.h"
#include <vector>
#include <limits>
using namespace std;
DataManip::DataManip() {}

std::vector<int> DataManip::getBestPath() const {
    return bestPath;
}

int DataManip::getBestCost() const {
    return bestCost;
}

void DataManip::readEdges(string filename) {

    ifstream in(filename);
    unsigned int destino, origem;
    double distancia;
    string line;


    // Verifica se a primeira linha é "origem,destino,distancia"


    if (in.is_open()) {

        while(getline(in, line)){
            if (line == "origem,destino,distancia") {
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

bool DataManip::Solution(const vector<int>& path) {
    if (path.size() != graph_.getVertexSet().size()) {
        return false; // Se o caminho não passou por todos os vértices, não é uma solução
    }

    // Verifica se o último vértice se conecta de volta ao primeiro vértice
    int finalVertex = path.back();
    int startVertex = 0;
    for (Edge* edge : graph_.findVertex(finalVertex)->getAdj()) {
        if (edge->getDest()->getId() == startVertex) {
            return true; // Se houver uma aresta de volta para o primeiro vértice, é uma solução
        }
    }

    return false; // Se nenhum vértice estiver conectado de volta ao primeiro vértice, não é uma solução
}


bool DataManip::Bound(const vector<int>& path, int currCost) {
    return currCost < bestCost;
}

void DataManip::RecursiveBackTracking(vector<int>& path, int currCost, int currPos) {

    // Para cada cidade possível não visitada
    for (Edge* edge : graph_.findVertex(currPos)->getAdj()) {
        int nextVertex = edge->getDest()->getId();
        if (edge && !edge->getDest()->isVisited()) {
            // Se a inclusão da cidade não ultrapassar o melhor custo encontrado até agora
            if (Bound(path, currCost + edge->getDistance())) {
                // Adiciona a cidade ao caminho
                path.push_back(nextVertex);
                // Marca a cidade como visitada após adicioná-la ao caminho
                edge->getDest()->setVisited(true);
                // Continua a busca recursivamente
                RecursiveBackTracking(path, currCost + edge->getDistance(), nextVertex);
                // Desmarca a cidade e remove do caminho para explorar outras possibilidades
                edge->getDest()->setVisited(false);
                path.pop_back();
            }
        }

    }
    if (Solution(path)) {

        // Atualiza o melhor custo e o melhor caminho
        if (currCost + graph_.findEdge(path.back(),path[0])->getDistance() < bestCost) {
            bestCost = currCost + graph_.findEdge(path.back(),path[0])->getDistance();
            bestPath = path;

        }
        return;
    }
}
Graph DataManip::getGraph() {
    return graph_;
}
