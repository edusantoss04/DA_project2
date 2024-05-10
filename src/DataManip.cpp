#include <iomanip>
#include "DataManip.h"

DataManip::DataManip() {}

void DataManip::readEdges(string filename) {

    ifstream in(filename);
    unsigned int destino, origem;
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

Graph DataManip::getGraph() {
    return graph_;
}
