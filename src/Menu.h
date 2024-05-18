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
    void HeuristicsMenu();

    void DisplayNearestNeighborApprox();
    void DisplaySimulatedAnnealing();


    void ChooseEdges();

    void back() const;
    static void exitProgram() ;



};


#endif //DA_PROJECT1_MENU_H
