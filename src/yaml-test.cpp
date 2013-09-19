#include <iostream>
#include <algorithm> // copy
#include <iterator> // ostream_iterator

#include "objectconfig.h"

using namespace std;

int main() {

    auto objects = ObjectConfig("markers.yml").objects();

    for (auto o : objects) {
        std::cout << "Found object: " << o.name << std::endl;

        for (auto marker:o.markers) {
            cout << "\tMarker " << marker.id << endl;
            cout << "\t\tsize: " << marker.size << endl;
            cout << "\t\ttranslation: [";
            copy(marker.translation.begin(), marker.translation.end(), ostream_iterator<float>(cout, ", "));
            cout << "]" << endl;
            cout << "\t\trotation: [";
            copy(marker.rotation.begin(), marker.rotation.end(), ostream_iterator<float>(cout, ", "));
            cout << "]" << endl;
            cout << "\t\tkeep: " << marker.keep << endl;
        }
    }


    return 0;
}
