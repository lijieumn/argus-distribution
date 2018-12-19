//
// Created by jie on 10/24/18.
//

#ifndef ARGUS_DISTRIBUTION_SHADER_H
#define ARGUS_DISTRIBUTION_SHADER_H

#include "display_helper.h"
#include <string>

namespace display {

    class Shader {
    public:

        Shader();

        void init(const char *vertexFile, const char *fragFile);

        void enable();

        void setUniform1i(const GLchar *name, GLint value);

        void setUniformMatrix4fv(const GLchar *name, const GLfloat *value);

        void setUniform3f(const GLchar *name, GLfloat v0, GLfloat v1, GLfloat v2);

    private:
        unsigned int ID;
        void compile(GLenum type, const GLchar* shaderContent, GLint* shaderID);
        void checkShaderErrors(GLenum type, GLint shaderID, std::string message);
        void checkProgramErrors(GLenum type, std::string message);

    };
}
#endif //ARGUS_DISTRIBUTION_SHADER_H
