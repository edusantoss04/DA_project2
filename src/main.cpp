#include <iostream>
#include <iomanip>
#include "DataManip.h"
#include "Menu.h"

using namespace std;

int main(){

    DataManip data;

    Menu menu = Menu(data);
    menu.MainMenu();

    /*data.readNodes("../Real-world Graphs/graph1/nodes.csv");

    for (auto v: data.getGraph().getVertexSet()){
        cout << v.first << " " << fixed << setprecision(15) << v.second->getLongitude() << " " << v.second->getLatitude() << endl;
    }*/

    return 0;
}