#include <iostream>
#include "DataManip.h"

using namespace std;

int main(){

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
         << endl;

    DataManip data;

    cout << "Choose an option: ";
    cin >> option;


    switch (option) {
        case '1':
            //Toy-Graphs
            string file_name = "";
            cout << "Write the filename: ";
            file_name += ".csv";
            cin >> file_name;

            //chamar função

            break;

        case '2':
            //Extra_Fully_Connected_Graphs

            break;

        case '3':
            //Real-world Graphs
            file_name = "";
            char number;
            cout << "Which graph would you like to choose (1, 2, 3)? ";
            cin >> number;
            file_name ="graph" + string(1, number) + ".csv";

            //chamar função

            break;

        case 'e':
            cout << endl << endl << "Exiting program..." << endl;
            return 0;

        default:
            cout << endl << "Not a valid option!" << endl;
    }

    return 0;
}