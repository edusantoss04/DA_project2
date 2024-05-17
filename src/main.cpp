#include <iostream>
#include <iomanip>
#include <chrono>
#include "DataManip.h"
#include "Menu.h"

using namespace std;

int main(){

    DataManip data;

    //auto begin = std::chrono::high_resolution_clock::now();

    //Menu menu = Menu(data);
    //menu.MainMenu();
    //....................................Escolher ficheiro para ler .................................

    //...........TOY GRAPHS.........
    //data.readTourism("../Toy-Graphs/tourism.csv");
    //data.readToy("../Toy-Graphs/stadiums.csv");
   // data.readToy("../Toy-Graphs/shipping.csv");

    //..............Extra_Fully............
    //data.readNodes("../Extra_Fully_Connected_Graphs/nodes.csv");
    //data.readEdges("../Extra_Fully_Connected_Graphs/edges_25.csv");
    //data.readEdges("../Extra_Fully_Connected_Graphs/edges_900.csv");



    //................Real-World..........
    data.readNodes("../Real-world Graphs/graph1/nodes.csv");
    data.readEdgesLarge("../Real-world Graphs/graph1/edges.csv");

    //data.readNodes("../Real-world Graphs/graph2/nodes.csv");
    //data.readEdgesLarge("../Real-world Graphs/graph2/edges.csv");

    //data.readNodes("../Real-world Graphs/graph3/nodes.csv");
    //data.readEdgesLarge("../Real-world Graphs/graph3/edges.csv");



    //....................................TEMPO LEITURAS .................................
    /*auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;*/


    //....................................TESTAR LEITURAS .................................
    /*
    for (auto v: data.getGraph().getVertexSet()){
        if(v.first > 3){
            break;
        }
        cout << v.first << endl;
        for(auto e : v.second->getAdj()){
            cout << "com destino " << e.second->getDest()->getId() << " a uma distancia de "  << e.second->getDistance() << endl;
        }
    }
    */


    //.....................................1.TESTAR BACKTRACKING ...........................
/*
    vector<int> path;
    path.push_back(0);
    int currCost = 0;
    data.getGraph().findVertex(0)->setVisited(true);
    auto begin = std::chrono::high_resolution_clock::now();
    data.RecursiveBackTracking(path,currCost,0);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    std::vector<int> bestPath = data.getBestPath();
    int bestCost = data.getBestCost();

    std::cout << "Melhor caminho encontrado: ";
    for (int vertex : bestPath) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    std::cout << "Melhor custo encontrado: " << bestCost << std::endl;

    auto resultado = end-begin;
    cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;


    //cout << data.getGraph().getVertexSet().size();


*/
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
    cout << " " << path[0] << endl << endl;

    cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;



    //............3.TESTAR HEURISTICAS...............
    /*
    auto begin   = std::chrono::high_resolution_clock::now();
    vector<int> path;
    double minCost = data.NearestNeighborApproxNotConnected(path,12);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);



    if(minCost!=0) {
        cout << '\n' << "The minimum cost to travel between all points is " << minCost << endl;
        cout << "You should take the following path: " << endl;

        for (int i = 0; i < data.getGraph().getVertexSet().size(); i++) {
            cout << " " << path[i] << " ->";
        }
        cout << " " << path[0] << endl << endl;
    }
    cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;
    */
    return 0;
}