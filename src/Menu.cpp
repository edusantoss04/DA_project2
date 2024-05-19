#include "Menu.h"
#include <iostream>
#include <chrono>

using namespace std;

Menu::Menu(DataManip data) {
    data_ = data;
}

void Menu::MainMenu() {
    char inputTypeO;
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│           Algorithm Menu           │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - Backtracking                  │" << endl
         << "│  2 - TriangularApprox              │" << endl
         << "│  3 - Other Heuristics              │" << endl
         << "│  4 - TSP in Real World             │" << endl
         << "│                                    │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    int flag = 1;
    int node;

    while (flag) {
        cout << "Choose an option: ";
        cin >> inputTypeO;

        switch (inputTypeO) {
            case ('1'):

                ChooseMenu();
                DisplayBacktracking();
                data_.clearData();
                back();

                flag = 0;
                MainMenu();
                break;

            case ('2'):

                ChooseMenu();
                DisplayTriangularApprox();
                data_.clearData();
                back();

                flag = 0;
                MainMenu();
                break;

            case ('3'):

                HeuristicsMenu();

                flag = 0;
                MainMenu();
                break;

            case ('4'):

                ChooseMenu();
                cout << endl << "Choose start node: " << endl;
                cin >> node;
                DisplayTSPRW(node);
                data_.clearData();
                back();

                flag = 0;
                MainMenu();
                break;
            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}


void Menu::ChooseMenu() {
    char option;
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│        Choose the data set         │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - Toy-Graphs                    │" << endl
         << "│  2 - Extra_Fully_Connected_Graphs  │" << endl
         << "│  3 - Real-world Graphs             │" << endl
         << "│                                    │" << endl
         << "│  b - Back                          │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    int flag = 1;
    while(flag) {
        cout << "Choose an option: ";
        cin >> option;

        switch (option) {
            case ('1'):

                ToyMenu();

                flag = 0;
                break;
            case ('2'):

                ExtraMenu();

                flag = 0;
                break;
            case ('3'):

                RealMenu();

                flag = 0;
                break;

            case ('b'):
                return MainMenu();

            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}


void Menu::HeuristicsMenu() {
    char option;
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│          Other Heuristics          │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - Simulated Annealing           │" << endl
         << "│  2 - Nearest neighbor approximation│" << endl
         << "│                                    │" << endl
         << "│  b - Back                          │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    int flag = 1;
    while(flag) {
        cout << "Choose an option: ";
        cin >> option;

        switch (option) {
            case ('1'):

                ChooseMenu();
                DisplaySimulatedAnnealing();
                data_.clearData();
                back();

                flag = 0;
                break;
            case ('2'):

                ChooseMenu();
                DisplayNearestNeighborApprox();
                data_.clearData();
                back();

                flag = 0;
                break;

            case ('b'):
                return;

            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}


void Menu::ToyMenu() {
    char inputTypeO;
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│              Toy Menu              │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - shipping                      │" << endl
         << "│  2 - stadiums                      │" << endl
         << "│  3 - tourism                       │" << endl
         << "│                                    │" << endl
         << "│  b - Back                          │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    int flag = 1;

    while (flag) {
        cout << "Choose an option: ";
        cin >> inputTypeO;

        switch (inputTypeO) {
            case ('1'):

                data_.readEdges("../Toy-Graphs/shipping.csv");

                flag = 0;
                break;

            case ('2'):

                data_.readEdges("../Toy-Graphs/stadiums.csv");

                flag = 0;
                break;

            case ('3'):

                data_.readTourism("../Toy-Graphs/tourism.csv");

                flag = 0;
                break;

            case ('b'):
                return ChooseMenu();
            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}


void Menu::ExtraMenu() {
    char inputTypeO;
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│             Extra Menu             │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - edges                         │" << endl
         << "│  2 - nodes                         │" << endl
         << "│                                    │" << endl
         << "│  b - Back                          │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    int flag = 1;

    while (flag) {
        cout << "Choose an option: ";
        cin >> inputTypeO;

        switch (inputTypeO) {
            case ('1'):

                ChooseEdges();

                flag = 0;
                break;

            case ('2'):

                data_.readNodes("../Extra_Fully_Connected_Graphs/nodes.csv");

                flag = 0;
                break;

            case ('b'):
                return ChooseMenu();
            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}


void Menu::RealMenu() {
    char inputTypeO;
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│              Real Menu             │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - graph 1                       │" << endl
         << "│  2 - graph 2                       │" << endl
         << "│  3 - graph 3                       │" << endl
         << "│                                    │" << endl
         << "│  b - Back                          │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    int flag = 1;

    while (flag) {
        cout << "Choose an option: ";
        cin >> inputTypeO;

        switch (inputTypeO) {
            case ('1'):

                data_.readNodes("../Real-world Graphs/graph1/nodes.csv");
                data_.readEdgesLarge("../Real-world Graphs/graph1/edges.csv");

                flag = 0;
                break;

            case ('2'):

                data_.readNodes("../Real-world Graphs/graph2/nodes.csv");
                data_.readEdgesLarge("../Real-world Graphs/graph2/edges.csv");

                flag = 0;
                break;

            case ('3'):

                data_.readNodes("../Real-world Graphs/graph3/nodes.csv");
                data_.readEdgesLarge("../Real-world Graphs/graph3/edges.csv");

                flag = 0;
                break;

            case ('b'):
                return ChooseMenu();
            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}


bool isValidInput(int inputTypeO) {
    static const std::set<int> validValues = {25, 50, 75, 100, 200, 300, 400, 500, 600, 700, 800, 900};
    return validValues.find(inputTypeO) != validValues.end();
}

void Menu::ChooseEdges() {
    int inputTypeO;
    int flag = 1;

    while (flag) {
        cout << "\n\n";
        cout << "Type the number of nodes that you want to read? ";
        cin >> inputTypeO;

        if (cin.fail()) {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << "Not a valid value." << endl;
            break;
        }

        if (isValidInput(inputTypeO)) {
            string numberStr = to_string(inputTypeO);
            string path = "../Extra_Fully_Connected_Graphs/edges_" + numberStr + ".csv";
            data_.readEdges(path);
            flag = 0;
        } else {
           cout << "Not a valid value." << endl;
        }
    }
}



void Menu::back() const {
    char input;
    cout << endl << endl;
    cout << "b - Back" << endl
         << "e - Exit" << endl;

    while (true){
        cout << "Choose option: ";
        cin >> input;
        switch (input) {
            case ('b'):
                return;
            case ('e'):
                return exitProgram();
            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}

void Menu::exitProgram() {
    cout << endl << "Exiting program..." << endl;
    exit(0);
}



void Menu::DisplayBacktracking(){

    vector<int> path;
    path.push_back(0);
    int currCost = 0;
    data_.getGraph().findVertex(0)->setVisited(true);
    auto begin = std::chrono::high_resolution_clock::now();
    data_.RecursiveBackTracking(path,currCost,0);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    std::vector<int> bestPath = data_.getBestPath();
    int bestCost = data_.getBestCost();

    std::cout << "Best path found: ";
    for (int vertex : bestPath) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    std::cout << "Best cost found: " << bestCost << std::endl;

    cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;
}

void Menu::DisplayTriangularApprox() {

    auto begin = std::chrono::high_resolution_clock::now();
    vector<int> path;
    double minCost = data_.TriangularApprox(path);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

    cout << '\n' << "Best cost found: " << minCost << endl;

    cout << "Best path found: " << endl;

    for (int i = 0; i < data_.getGraph().getVertexSet().size(); i++) {
        cout << " " << path[i] << " ->";
    }
    cout << " " << path[0] << endl << endl;

    cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;
}

void Menu::DisplayNearestNeighborApprox() {

    auto begin   = std::chrono::high_resolution_clock::now();
    vector<int> path;
    double minCost = data_.NearestNeighborApprox(path);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

    cout << '\n' << "Best cost found: " << minCost << endl;

    cout << "Best path found: " << endl;

    for (int i = 0; i < data_.getGraph().getVertexSet().size(); i++) {
        cout << " " << path[i]<< " ->";
    }
    cout << " " << path[0] << endl << endl;

    cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;
}

void Menu::DisplaySimulatedAnnealing() {

    auto begin   = std::chrono::high_resolution_clock::now();
    vector<int> path;
    double minCost = data_.NearestNeighborApprox(path);
    minCost = data_.simulatedAnnealing(path);
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);



    if(minCost!=0) {
        cout << '\n' << "Best cost found: " << minCost << endl;
        cout << "Best path found: " << endl;

        for (int i = 0; i < data_.getGraph().getVertexSet().size(); i++) {
            cout << " " << path[i] << " ->";
        }
        cout << " " << path[0] << endl << endl;
    }
    cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;
}

void Menu::DisplayTSPRW(int node) {
    if(data_.getGraph().getNumVertex() < 20) {
        vector<int> path;
        path.push_back(node);
        int currCost = 0;
        data_.getGraph().findVertex(node)->setVisited(true);
        auto begin = std::chrono::high_resolution_clock::now();
        data_.RecursiveBackTracking(path,currCost,node);
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
        std::vector<int> bestPath = data_.getBestPath();
        int bestCost = data_.getBestCost();

        std::cout << "Best path found: ";
        for (int vertex : bestPath) {
            std::cout << vertex << " ";
        }
        cout << path[0] << endl << endl;
        std::cout << "Best cost found: " << bestCost << std::endl;

        auto resultado = end-begin;
        cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;
    }

    else {
        vector<int> path;
        auto begin = std::chrono::high_resolution_clock::now();
        double minCost = data_.NearestNeighborApproxNotConnected(path,node);
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);



        if(minCost!=0) {
            cout << '\n' << "Best cost found: " << minCost << endl;
            cout << "Best path found: " << endl;

            for (int i = 0; i < data_.getGraph().getVertexSet().size(); i++) {
                cout << " " << path[i] << " ->";
            }
            cout << " " << path[0] << endl << endl;
        }
        cout << "Execution time: " << elapsed.count() * 1e-9 << " seconds." << endl;
    }
}