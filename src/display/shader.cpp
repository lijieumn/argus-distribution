//
// Created by jie on 10/24/18.
//

#include "shader.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace display {

    Shader::Shader() : ID(0) {

    }

    void Shader::checkShaderErrors(GLenum type, GLint shaderID, std::string message) {
        GLint  status;
        glGetShaderiv(shaderID, type, &status);
        if (status == GL_FALSE) {
            GLchar log[512];
            glGetShaderInfoLog(shaderID, 512, NULL, log);
            throw std::runtime_error(message + "\n" + std::string(log));
        }

    }

    void Shader::checkProgramErrors(GLenum type, std::string message) {
        GLint status;
        glGetProgramiv(ID, type, &status);
        if (status == GL_FALSE) {
            GLchar log[512];
            glGetProgramInfoLog(ID, 512, NULL, log);
            throw std::runtime_error(message + "\n" + std::string(log));
        }
    }

    void Shader::compile(GLenum type, const GLchar* shaderContent, GLint* shaderID) {
        *shaderID = glCreateShader(type);
        glShaderSource(*shaderID, 1, &shaderContent, NULL);
        glCompileShader(*shaderID);
        checkShaderErrors(GL_COMPILE_STATUS, *shaderID, "ERROR: Vertex shader complication failed");
    }

    void Shader::init(const char *vertexFile, const char *fragFile) {
        std::ifstream vertexIn(vertexFile, std::ios::in | std::ios::binary);
        std::ifstream fragIn(fragFile, std::ios::in | std::ios::binary);
        std::stringstream vertexStream, fragStream;
        vertexStream << vertexIn.rdbuf();
        fragStream << fragIn.rdbuf();
        vertexIn.close();
        fragIn.close();

        std::string vertexContent = vertexStream.str();
        std::string fragContent = fragStream.str();

        const char* vertexChar = vertexContent.c_str();
        GLint vertexShader;
        compile(GL_VERTEX_SHADER, vertexChar, &vertexShader);

        const char* fragChar = fragContent.c_str();
        GLint fragShader;
        compile(GL_FRAGMENT_SHADER, fragChar, &fragShader);

        ID = glCreateProgram();
        glAttachShader(ID, vertexShader);
        glAttachShader(ID, fragShader);
        glLinkProgram(ID);

        checkProgramErrors(GL_LINK_STATUS, "ERROR: shader program linking failed");
        glDeleteShader(vertexShader);
        glDeleteShader(fragShader);

    }

    void Shader::enable() {
        if (ID == 0) {
            throw std::runtime_error("ERROR: shader program has not been initialized!");
        }
        glUseProgram(ID);
    }

    void Shader::setUniform1i(const GLchar *name, GLint value) {
        glUniform1i(glGetUniformLocation(ID, name), value);
    }

    void Shader::setUniformMatrix4fv(const GLchar *name, const GLfloat *value) {
        glUniformMatrix4fv(glGetUniformLocation(ID, name), 1, GL_FALSE, value);
    }

    void Shader::setUniform3f(const GLchar *name, GLfloat v0, GLfloat v1, GLfloat v2) {
        glUniform3f(glGetUniformLocation(ID, name), v0, v1, v2);
    }
}