//
// Created by lijie on 9/9/18.
//

#include "camera.h"
#include <glm/gtc/matrix_transform.hpp>

namespace display {


    Camera::Camera(int width, int height) : winWidth(width), winHeight(height), pos(2, 2, 3), target(0, 0, 0), worldUp(0, 0, 1), fov(20), near(0.05), far(100)  {
    }

    void Camera::resize(int width, int height) {
        winWidth = width;
        winHeight = height;
    }

    glm::mat4 Camera::getViewMatrix() {
        return glm::lookAt(pos, target, worldUp);
    }

    glm::mat4 Camera::getProjectionMatrix() {
        return glm::perspective(glm::radians(fov), (double)winWidth/winHeight, near, far);
    }

    void Camera::modifyFOV(float offset) {
        fov += offset;
        if (fov <= 1.) {
            fov = 1.;
        } else if (fov >= 45.) {
            fov = 45.;
        }
    }

    void Camera::moveCamera(float xOffset, float yOffset) {
        glm::vec3 viewDir = target - pos;
        glm::vec3 right = glm::normalize(glm::cross(viewDir, worldUp));
        glm::vec3 up = glm::normalize(glm::cross(right, viewDir));
        glm::vec3 dx = right*xOffset;
        glm::vec3 dy = up*yOffset;
        pos += (dx + dy);
        target += (dx + dy);
    }

    void Camera::rotateCamera(float horizontal, float vertical) {

        glm::vec3 viewDir = target - pos;
        glm::vec4 relativePos(-viewDir.x, -viewDir.y, -viewDir.z, 1);
        glm::mat4 rotateMatrix(1.);
        float lat = glm::dot(glm::normalize(viewDir), worldUp);
        if (!((lat > .9 && vertical > 0)||(lat < -.9 && vertical < 0))) {
            glm::vec3 right = glm::normalize(glm::cross(viewDir, worldUp));
            rotateMatrix = glm::rotate(rotateMatrix, vertical, right);
        }
        rotateMatrix = glm::rotate(rotateMatrix, horizontal, worldUp);
        relativePos = rotateMatrix*relativePos;


        pos = glm::vec3(relativePos.x, relativePos.y, relativePos.z) + target;

    }

} // display

