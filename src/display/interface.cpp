//
// Created by lijie on 9/9/18.
//

#include "interface.h"
#include <sstream>
#include <glm/gtc/type_ptr.hpp>

namespace display {

struct MouseInfo {
    bool leftDragging, middleDragging;
    float xdrag, ydrag;
    double xprev, yprev;
};

std::shared_ptr<Timestep> Interface::mTimestep;
static bool play = false;
static bool showMesh = false;
static MouseInfo mouseInfo;
static std::unique_ptr<Camera> camera;

Interface::Interface(std::shared_ptr<Timestep> timestep, int w, int h) {
    mTimestep = timestep;
    camera = std::make_unique<Camera>(w, h);
    glfwSetErrorCallback(Interface::errorCallback);
    if (!glfwInit())
        exit(1);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    window = glfwCreateWindow(w, h, "Argus-distribution", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, Interface::framebufferSizeCallback);
    glfwSetKeyCallback(window, Interface::keyCallback);
    glfwSetMouseButtonCallback(window, Interface::mouseButtonCallback);
    glfwSetCursorPosCallback(window, Interface::cursorPositionCallback);
    glfwSetScrollCallback(window, Interface::scrollCallback);

    glewExperimental = GL_TRUE;
    glewInit();
    std::string shaderPrefix = "./shader/shader.";
    shader.init((shaderPrefix + "vert").c_str(), (shaderPrefix + "frag").c_str());
    shader.enable();

    mTimestep->getClothDrawable().prepare();
    mTimestep->getObstacleDrawable().prepare();
    display();
}

Interface::~Interface() {
    // glfwDestroyWindow(window);
}

void Interface::display() {
    glClearColor(1,1,1, 0);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
//    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);

    shader.setUniformMatrix4fv("model", glm::value_ptr(glm::mat4(1.0)));
    shader.setUniformMatrix4fv("view", glm::value_ptr(camera->getViewMatrix()));
    shader.setUniformMatrix4fv("projection", glm::value_ptr(camera->getProjectionMatrix()));
    shader.setUniform3f("eyePos", camera->pos[0], camera->pos[1], camera->pos[2]);
    shader.setUniform1i("Tex1", 0);

    shader.setUniform1i("useTex", !mTimestep->getClothDrawable().texFile.empty());
    mTimestep->getClothDrawable().bind();
    mTimestep->getClothDrawable().draw(showMesh);
    shader.setUniform1i("useTex", !mTimestep->getObstacleDrawable().texFile.empty());
    mTimestep->getObstacleDrawable().bind();
    mTimestep->getObstacleDrawable().draw(showMesh);

    glfwSwapBuffers(window);

}

void Interface::run() {


    while (!glfwWindowShouldClose(window)) {

        if (play) {
            mTimestep->step();
            int width = 0, height = 0;
            glfwGetWindowSize(window, &width, &height);
            mTimestep->saveScreenshot(width, height);
        }
        display();
        glfwPollEvents();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    
}

void Interface::framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void Interface::errorCallback(int error, const char* description) {
    std::cerr << "Error: " << description << std::endl;
}

void Interface::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {

    if (action == GLFW_RELEASE) {
        switch (key) {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, true);
                break;
            case ' ':
                play = !play;
                break;
            case GLFW_KEY_S:
                play = !play;
                mTimestep->step();
                play = !play;
                break;
            case GLFW_KEY_M:
                showMesh = !showMesh;
                break;
            default:
                break;
        }
    }
}

void Interface::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            glfwGetCursorPos(window, &mouseInfo.xprev, &mouseInfo.yprev);
            mouseInfo.leftDragging = true;
        } else if (action == GLFW_RELEASE) {
            mouseInfo.leftDragging = false;
        }
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
        if (action == GLFW_PRESS) {
            glfwGetCursorPos(window, &mouseInfo.xprev, &mouseInfo.yprev);
            mouseInfo.middleDragging = true;
        } else if (action == GLFW_RELEASE) {
            mouseInfo.middleDragging = false;
        }
    }
}

void Interface::cursorPositionCallback(GLFWwindow* window, double x, double y) {
    if (mouseInfo.leftDragging) {
        camera->rotateCamera(-(x - mouseInfo.xprev)*0.2*M_PI/180, -(y - mouseInfo.yprev)*0.2*M_PI/180);
    } else if (mouseInfo.middleDragging) {
        float speed = 5e-3;
        camera->moveCamera(-speed*(x - mouseInfo.xprev), speed*(y - mouseInfo.yprev));
    }
    mouseInfo.xprev = x;
    mouseInfo.yprev = y;
}


void Interface::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    camera->modifyFOV(-yoffset);
}

} //display
