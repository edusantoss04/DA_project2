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

    //data.readTourism("../Toy-Graphs/tourism.csv");
    //data.readEdges("../Toy-Graphs/stadiums.csv");
    //data.readEdges("../Extra_Fully_Connected_Graphs/edges_25.csv");
    data.readEdges("../Extra_Fully_Connected_Graphs/edges_900.csv");

    //....................................TESTAR LEITURAS .................................
    /*
    for (auto v: data.getGraph().getVertexSet()){
        cout << v.first << endl;
        for(auto e : v.second->getAdj()){
            cout << "com destino " << e->getDest()->getId() << " a uma distancia de "  << e->getDistance() << endl;
        }
    }
     */


    //.....................................1.TESTAR BACKTRACKING ...........................
    /*
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

    */
    //cout << data.getGraph().getVertexSet().size();




    //.....................................2.TESTAR TriangularApprox .............................

    auto begin = std::chrono::high_resolution_clock::now();
    vector<int> path;
    double minCost = data.TriangularApprox(path);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

    cout << '\n' << "The minimum cost to travel between all points is " << minCost << endl;

    cout << "You should take the following path: " << endl;

    for (int i = 0; i < data.getGraph().getVertexSet().size(); i++) {
        cout << " " << path[i] << " ->";
    }
    cout << " " << path[0] << " (again eheheh ;)" << endl << endl;

    cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;

    return 0;
}