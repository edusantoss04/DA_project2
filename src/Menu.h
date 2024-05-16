#ifndef DA_PROJECT1_MENU_H
#define DA_PROJECT1_MENU_H

#include "DataManip.h"

class Menu {

private:
    DataManip data_;


public:
    Menu(DataManip data);

    void MainMenu();

    void ChooseMenu();
    void ToyMenu();
    void ExtraMenu();
    void RealMenu();

    void DisplayBacktracking();
    void DisplayTriangularApprox();
    //void DisplayHeuristics();

    void ChooseEdges();

    /*void maxWater();
    void deficitPerCity();
    void balanceFlow();
    void networkFailures();
    void removedReservoir();
    void removedStation();
    void removedPipe();

    vector<string> createVecR();
    vector<string> createVecS();
    vector<pair<string, string>> createVecPipe();*/

    void back() const;
    static void exitProgram() ;



};


#endif //DA_PROJECT1_MENU_H
