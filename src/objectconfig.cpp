#include <fstream>
#include <iostream>
#include <algorithm> // copy
#include <iterator> // ostream_iterator

#include "objectconfig.h"

using namespace std;
using namespace cv;
using namespace YAML;

ObjectConfig::ObjectConfig(const string& configuration) {

    ifstream fin(configuration);
    Parser parser(fin);
    Node doc;
    parser.GetNextDocument(doc);
    parse(doc);

}

ObjectConfig::ObjectConfig(const Node& configuration) {
    parse(configuration);
}

void operator >> (const YAML::Node& node, vector<float>& vector) {
    node[0] >> vector[0];
    node[1] >> vector[1];
    node[2] >> vector[2];
}

void operator >> (const YAML::Node& node, MarkerConfig& conf) {

    node["marker"] >> conf.id;
    node["size"] >> conf.size;
    if(const YAML::Node *pVec = node.FindValue("translation")) *pVec >> conf.translation;
    if(const YAML::Node *pVec = node.FindValue("rotation")) *pVec >> conf.rotation;
    if(const YAML::Node *pBool = node.FindValue("keep")) *pBool >> conf.keep;

}

vector<Object> ObjectConfig::objects() const {
    return _objects;
}

const Object* ObjectConfig::usedBy(int markerId) const {

    for (auto& o : _objects) {
        for (auto& m : o.markers) {
            // here, we return a pointer to an element of
            // a vector<Object> which is dangerous: if 
            // elements are added to the vector, it may
            // resize and the pointer would become invalid.
            // In our case it's however ok since _objects
            // is only modified at construction.
            if (m.id == markerId) return &o;
        }
    }
    return nullptr;

}
void ObjectConfig::parse(const YAML::Node& configuration){

    for(auto it=configuration.begin();
             it!=configuration.end();
             ++it) {

        Object object;

        it.first() >> object.name;

        for(auto marker=it.second().begin();marker!=it.second().end();++marker) {
            MarkerConfig config;

            *marker >> config;

            computeCorners(config);

            object.markers.push_back(config);
        }

        _objects.push_back(object);
    }

}

void ObjectConfig::computeCorners(MarkerConfig& marker) const {

    marker.corners.push_back(Point3f(0.,0.,0.));
    marker.corners.push_back(Point3f(marker.size,0.,0.));
    marker.corners.push_back(Point3f(marker.size,marker.size,0.));
    marker.corners.push_back(Point3f(0.,marker.size,0.));

}

/*
// Usage example
int main() {

    auto objects = ObjectConfig("markers.yml").objects();

    for (auto o : objects) {
        std::cout << "Found object: " << o.name << std::endl;

        for (auto marker:o.markers) {
            cout << "\tMarker " << marker.id << endl;
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
*/
