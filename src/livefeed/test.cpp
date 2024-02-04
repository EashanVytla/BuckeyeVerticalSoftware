#include "hex.h"
#include <iostream>
#include <stdint.h>
#include <messages.h>
#include <string>
#include <commands.h>
#include <SIYI.h>

using namespace std;

int main() {

    cout << Message::encode( Message::autoFocus() ).data << endl;

    string encodedMessage = "556601010000000401bc57";
    // string encodedMessage =  Message::encode({ "0001", Command::AUTO_FOCUS});

    cout << "data: " << encodedMessage << endl;
    cout << "t: " << Message::decode(encodedMessage).data << endl;
    cout << "t: " << Message::decode(encodedMessage).seq << endl;

    // cout << "x: " << stoi("FFFFF", 0, 16) << endl;

    return 0;

}