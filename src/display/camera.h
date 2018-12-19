//
// Created by lijie on 9/9/18.
//

#ifndef ARGUS_DISTRIBUTION_CAMERA_H
#define ARGUS_DISTRIBUTION_CAMERA_H

#include "display_helper.h"
#include <iostream>
#include <cmath>
#include <bvh.hpp>
#include <glm/glm.hpp>

namespace display {

class Camera {
private:
    // double distance;
    double fov;
    double near, far;
    int winWidth, winHeight;
    glm::vec3 worldUp;

public:
    glm::vec3 target;
//     double latitude, longitude;
    glm::vec3 pos;
    Camera(int width, int height);
    void resize(int width, int height);

    glm::mat4 getViewMatrix();
    glm::mat4 getProjectionMatrix();
    void modifyFOV(float offset);
    void moveCamera(float xOffset, float yOffset);
    void rotateCamera(float horizontal, float vertical);
};

} // display

#endif //ARGUS_DISTRIBUTION_CAMERA_H
