//
// Created by lijie on 9/13/18.
//

#ifndef ARGUS_DISTRIBUTION_DRAWABLE_H
#define ARGUS_DISTRIBUTION_DRAWABLE_H

#include "display_helper.h"
#include <memory>
#include <string>

namespace display {

class Drawable {
private:
    enum DataName {
        VERTEX, COORDINATE, COLOR, NORMAL, ELEMENT, numData
    };
    GLuint  vaoHandle;
    GLuint vboHandles[numData];
    GLuint texture;
public:

    std::unique_ptr<float[]> vertices;
    std::unique_ptr<float[]> texCoords;
    std::unique_ptr<float[]> colors;
    std::unique_ptr<float[]> normals;
    std::unique_ptr<int[]> elements;
    int vSize, fSize;
    std::string texFile;

    void prepare();
    void bind();
    void draw(bool showMesh = false);
};

} // display

#endif //ARGUS_DISTRIBUTION_DRAWABLE_H
