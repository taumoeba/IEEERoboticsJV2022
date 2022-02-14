#include <iostream>
#include <chrono>
#include <thread>

using namespace std;
using namespace std::this_thread;
using namespace std::chrono_literals;
using std::chrono::system_clock;

bool armcode(bool beads){
    if(beads){
        cout << "lower grabber\n";
        sleep_for(1s);
        cout << "extend lead screw\n";
        sleep_for(1s);
        cout << "let go of beads\n";
        sleep_for(1s);
        cout << "retract lead screw\n";
        sleep_for(1s);
        cout << "lower grabber\n";
        return false;
    }
    else{
        cout << "extend arm\n";
        sleep_for(1s);
        cout << "raise grabber\n";
        sleep_for(1s);
        cout << "extend lead screw\n";
        sleep_for(1s);
        cout << "grab beads\n";
        sleep_for(1s);
        cout << "retract lead screw\n";
        sleep_for(1s);
        cout << "lower grabber\n";
        return true;
    }
}

int main(){
    bool beads = false;
    beads = armcode(beads);
    sleep_for(2s);
    if(beads)
        cout << "GOT BEADS!\n";
    else
        cout << "NO BEADS!\n";
    sleep_for(2s);
    beads = armcode(beads);
    sleep_for(2s);
    if(beads)
        cout << "GOT BEADS!\n";
    else
        cout << "NO BEADS!\n";
    sleep_for(2s);
}