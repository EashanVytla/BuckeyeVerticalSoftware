#include "hex.h"
#include <iostream>
#include <stdint.h>
#include <messages.h>
#include <string>
#include <commands.h>
#include <SIYI.h>
#include <safequeue.h>

using namespace std;

int main() {

    SafeQueue<int> queue;

    queue.enqueue(5);

    cout << queue.dequeue() << endl;

    cout << Message::encode( Message::autoFocus() ).data << endl;

    string encodedMessage = "556601010000000401bc57";
    // string encodedMessage =  Message::encode({ "0001", Command::AUTO_FOCUS});

    cout << "data: " << encodedMessage << endl;
    cout << "t: " << Message::decode(encodedMessage).data << endl;
    cout << "t: " << Message::decode(encodedMessage).seq << endl;


    vector<char> vvv;

    vvv.push_back(static_cast<char>(255));
    vvv.push_back(static_cast<char>(2));
    vvv.push_back(static_cast<char>(3));

    cout << Hex::toHex(vvv, 3) << endl;
    cout << Hex::asInt("ff") << endl;

    SIYI siyi;

    siyi.connect();

    // cout << "x: " << stoi("FFFFF", 0, 16) << endl;

    siyi.disconnect();
    siyi.disconnect();

    siyi.connect();

    // siyi.connect();


    return 0;

}

// template class SafeQueue<int>;