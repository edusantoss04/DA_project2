#include "Menu.h"
#include <iostream>
using namespace std;

Menu::Menu(DataManip data) {
    data_ = data;
}

void Menu::MainMenu() {
    char option;
    cout << "Loading program...";
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│        Choose the data set         │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - Toy-Graphs                    │" << endl
         << "│  2 - Extra_Fully_Connected_Graphs  │" << endl
         << "│  3 - Real-world Graphs             │" << endl
         << "│                                    │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    while(true) {
        cout << "Choose an option: ";
        cin >> option;

        switch (option) {
            case ('1'):
                ToyMenu();
                return MainMenu();
            case ('2'):
                ExtraMenu();
                return MainMenu();
            case ('3'):
                RealMenu();
                return MainMenu();
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
         << "│  b - back                          │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    int flag = 1;
    string inputOrigin;

    while (flag) {
        cout << "Choose an option: ";
        cin >> inputTypeO;

        switch (inputTypeO) {
            case ('1'):

                //chamar reads

                flag = 0;
                break;

            case ('2'):

                //chamar reads

                flag = 0;
                break;

            case ('3'):

                //chamar reads

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

void Menu::RealMenu() {
    char inputTypeO;
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│              Toy Menu              │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - graph 1                       │" << endl
         << "│  2 - graph 2                       │" << endl
         << "│  3 - graph 3                       │" << endl
         << "│                                    │" << endl
         << "│  b - back                          │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    int flag = 1;
    string inputOrigin;

    while (flag) {
        cout << "Choose an option: ";
        cin >> inputTypeO;

        switch (inputTypeO) {
            case ('1'):

                //chamar reads

                flag = 0;
                break;

            case ('2'):

                //chamar reads

                flag = 0;
                break;

            case ('3'):

                //chamar reads

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

void Menu::ExtraMenu() {          //ver como vai ficar
    char inputTypeO;
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│              Toy Menu              │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - shipping                      │" << endl
         << "│  2 - stadiums                      │" << endl
         << "│  3 - tourism                       │" << endl
         << "│                                    │" << endl
         << "│  b - back                          │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";

    int flag = 1;
    string inputOrigin;

    while (flag) {
        cout << "Choose an option: ";
        cin >> inputTypeO;

        switch (inputTypeO) {
            case ('1'):

                //chamar reads

                flag = 0;
                break;

            case ('2'):

                //chamar reads

                flag = 0;
                break;

            case ('3'):

                //chamar reads

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

/*
void Menu::networkFailures() {
    char option;
    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│        Failures on Network         │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - Reservoirs Out of Comission   │" << endl
         << "│  2 - Pumping Stations Removed      │" << endl
         << "│  3 - Pipeline Rupture              │" << endl
         << "│  b - Go Back                       │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";


    while(true) {
        cout << "Choose an option: ";
        cin >> option;

        switch(option) {
            case ('1'):
                removedReservoir();
                back();
                return networkFailures();
            case ('2'):
                removedStation();
                back();
                return networkFailures();
            case ('3'):
                removedPipe();
                back();
                return networkFailures();
            case ('b'):
                return;
            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}

void Menu::removedReservoir() {
    char option;
    vector<string> a;

    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│     Reservoirs Out of Comission    │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - See Affected Cities           │" << endl
         << "│  b - Go Back                       │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";


    while(true) {
        cout << "Choose an option: ";
        cin >> option;

        switch(option) {
            case ('1'):
                a = createVecR();

                if(a.empty()){
                    cout << "No values selected." << endl;
                    return;
                }

                data_.reservoirOutOfCommission(a);

                back();
                return removedReservoir();

            case ('b'):
                return;
            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}




void Menu::removedStation() {
    char option;
    vector<string> sC;

    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│      Pumping Stations Removed      │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - See Affected Cities           │" << endl
         << "│  b - Go Back                       │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";


    while(true) {
        cout << "Choose an option: ";
        cin >> option;

        switch(option) {
            case ('1'):

                sC = createVecS();

                if(sC.empty()){
                    cout << "No values selected." << endl;
                    return;
                }


                data_.stationRemoved(sC);

                back();
                return removedStation();

            case ('b'):
                return;
            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}




void Menu::removedPipe() {
    char option;

    vector<pair<string, string>> pC;

    cout << endl << endl;
    cout << "┌────────────────────────────────────┐" << endl
         << "│          Pipeline Rupture          │" << endl
         << "├────────────────────────────────────┤" << endl
         << "│  1 - See Affected Cities           │" << endl
         << "│  b - Go Back                       │" << endl
         << "│  e - Exit                          │" << endl
         << "└────────────────────────────────────┘" << endl
         << endl
         << "What would you like to do next? ";


    while(true) {
        cout << "Choose an option: ";
        cin >> option;

        switch(option) {
            case ('1'):

                pC = createVecPipe();

                if(pC.empty()){
                    cout << "No valid values selected." << endl;
                    return;
                }

                data_.pipelineRemoved(pC);

                back();
                return removedPipe();

            case ('b'):
                return;
            case ('e'):
                return exitProgram();

            default:
                cout << endl << "Not a valid option!" << endl;
        }
    }
}




vector<string> Menu::createVecR() {
    bool flag = true;
    vector<string> v = {};

    cout << "Type Reservoir Codes/Names to remove and hit Enter and 'd' when done.\n\n";

    while(flag){
        string inp;
        string x;

        getline(cin, inp);

        x = data_.verifyReservoirCode(inp);

        if (x == "d") flag = false;
        else {
            auto it = data_.getReservoirs().find(x);

            if (it != data_.getReservoirs().end()) {
                v.push_back(x);
            } else {
                if(inp != ""){
                    cout << "Not a valid Reservoir." << endl;
                }
            }
        }
    }

    if(!v.empty()) cout << "List done." << endl << endl;

    return v;
}



vector<string> Menu::createVecS() {
    bool flag = true;
    vector<string> v = {};

    cout << "Type Station Codes to remove and hit Enter and 'd' when done.\n\n";

    while(flag){
        string inp = "";
        cin >> inp;
        if ( inp == "d") flag = false;
        else{
            auto it = data_.getStations().find(inp);

            if (it != data_.getStations().end())
                v.push_back(inp);

            else cout << "Not a valid Station."<< endl;
        }
    }

    if(!v.empty()) cout << "List done." << endl << endl;

    return v;
}



vector<pair<string, string>> Menu::createVecPipe() {
    bool flag = true;
    vector<pair<string, string>> v = {};
    cout << "Type Origin code and Destiny Code to remove and hit Enter and 'd' when done.\n\n";
    cout << "Insert a pair (One Code per line): " << endl;

    while(flag){
        string pipe1 = "";
        string pipe2 = "";

        cin >> pipe1;

        if ( pipe1 == "d") break;
        cin >> pipe2;
        if (pipe2 == "d") break;

        if((pipe1 != "d") && (pipe2 != "d")){

            if((data_.getGraph().findVertex(pipe1) != nullptr) && (data_.getGraph().findVertex(pipe2) != nullptr)){
                v.push_back({pipe1, pipe2});
                cout << "Insert another pair (One Code per line) or type 'd' if done:" << endl;
            }

            else cout << "Invalid values."<< endl;
        }
    }

    if(!v.empty()) cout << "List done." << endl << endl;

    return v;
}*/




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