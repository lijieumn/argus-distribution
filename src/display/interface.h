//
// Created by lijie on 9/9/18.
//

#ifndef ARGUS_DISTRIBUTION_INTERFACE_H
#define ARGUS_DISTRIBUTION_INTERFACE_H

#include <memory>
#include "camera.h"
#include "shader.h"
#include "timestep.h"

namespace display {

#define WIDTH 1080
#define HEIGHT 720

class Interface
{
private:
    static std::shared_ptr<Timestep> mTimestep;
    // std::unique_ptr<GLFWwindow, GLFWWinDeleter> window;
    GLFWwindow* window;
    Shader shader;
    void display();
public:
    Interface(std::shared_ptr<Timestep> timestep, int w = WIDTH, int h = HEIGHT);
    ~Interface();
    void run();

//    Callback functions
    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void errorCallback(int error, const char* description);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void cursorPositionCallback(GLFWwindow* window, double x, double y);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
};

} // display

#endif //ARGUS_DISTRIBUTION_INTERFACE_H
