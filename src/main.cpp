#include <iostream>
#include "DataManip.h"
#include "Menu.h"

using namespace std;

int main(){

    DataManip data;

    Menu menu = Menu(data);
    menu.MainMenu();

    return 0;
}