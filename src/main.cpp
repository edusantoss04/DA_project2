#include <iostream>
#include <iomanip>
#include <chrono>
#include "DataManip.h"
#include "Menu.h"

using namespace std;

int main(){

    auto inicio = std::chrono::high_resolution_clock::now();

    // Seu código


    DataManip data;

    //Menu menu = Menu(data);
    //menu.MainMenu();

    data.readTourism("../Toy-Graphs/tourism.csv");
    //data.readEdges("../Toy-Graphs/stadiums.csv");
    //data.readEdges("../Extra_Fully_Connected_Graphs/edges_25.csv");
    vector<int> path;
    path.push_back(0);
    int currCost = 0;
    data.getGraph().findVertex(0)->setVisited(true);
    data.RecursiveBackTracking(path,currCost,0);

    std::vector<int> bestPath = data.getBestPath();
    int bestCost = data.getBestCost();

    std::cout << "Melhor caminho encontrado: ";
    for (int vertex : bestPath) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    std::cout << "Melhor custo encontrado: " << bestCost << std::endl;

    auto resultado = std::chrono::high_resolution_clock::now() - inicio;
    long long milisseconds = std::chrono::duration_cast<std::chrono::milliseconds >(resultado).count();

    cout << "Tempo de execução: " << milisseconds;


    //cout << data.getGraph().getVertexSet().size();
    /*
    for (auto v: data.getGraph().getVertexSet()){
        cout << v.first << endl;
        for(auto e : v.second->getAdj()){
            cout << "com destino " << e->getDest()->getId() << " a uma distancia de "  << e->getDistance() << endl;
        }
    }
     */


    return 0;
}