#ifndef OBJECTCONFIG_H
#define OBJECTCONFIG_H

#include <vector>
#include <memory> // for shared_ptr
#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp> // for Point3f

struct MarkerConfig {
    int id;
    float size = 0.;
    std::vector<float> translation = {0., 0., 0.};
    std::vector<float> rotation = {0., 0., 0.};
    bool keep = false;

    // this vector stores the 3D location of the 
    // 4 corners of the marker *in the parent 
    // object frame*.
    std::vector<cv::Point3f> corners;
};

struct Object {
    std::string name;
    std::vector<MarkerConfig> markers;
};

/**
 * ObjectConfig reads and stores markers configurations.
 *
 * A marker configuration allows to associate a marker to
 * an object name (ie, alias) with possibly a 6D transformation.
 * It also allows to associate several markers to the same object,
 * for robust detection.
 */
class ObjectConfig {

public:
    ObjectConfig(const std::string& configuration);
    ObjectConfig(const YAML::Node& configuration);

    std::vector<Object> objects() const;

    const Object* usedBy(int markerId) const;

private:
    void parse(const YAML::Node& configuration);
    void computeCorners(MarkerConfig& marker) const;
    std::vector<Object> _objects;

};

#endif
