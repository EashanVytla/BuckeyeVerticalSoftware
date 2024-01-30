#include "hex.h"
#include <iostream>
#include <stdint.h>
#include <message.h>
#include <string>


using namespace std;

int main() {

    cout << Message::encode( Message::autoFocus() ) << endl;

    return 0;

}