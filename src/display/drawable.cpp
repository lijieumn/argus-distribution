//
// Created by lijie on 9/13/18.
//

#include "drawable.h"
#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>


namespace display {

const float frameColor[3] = {0., 0., 0.};

void Drawable::prepare() {
     glGenBuffers(numData, vboHandles);
     glGenVertexArrays(1, &vaoHandle);
     glBindVertexArray(vaoHandle);

     glEnableVertexAttribArray(0);
     glBindBuffer(GL_ARRAY_BUFFER, vboHandles[VERTEX]);
     glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLubyte *)NULL);

     glEnableVertexAttribArray(1);
     glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COORDINATE]);
     glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (GLubyte *)NULL);

     glEnableVertexAttribArray(2);
     glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COLOR]);
     glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (GLubyte *)NULL);

    // glEnableVertexAttribArray(3);
    // glBindBuffer(GL_ARRAY_BUFFER, vboHandles[ALPHA]);
    // glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 0, (GLubyte *)NULL);

     glEnableVertexAttribArray(3);
     glBindBuffer(GL_ARRAY_BUFFER, vboHandles[NORMAL]);
     glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, (GLubyte *)NULL);

     glActiveTexture(GL_TEXTURE0);
     glGenTextures(1, &texture);

     glBindTexture(GL_TEXTURE_2D, texture);
     int width, height, nrChannels;
     unsigned char *data;
     if (texFile.empty()) {
     } else {
         data = stbi_load(texFile.c_str(), &width, &height, &nrChannels, 0);
         glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
                      GL_RGB, GL_UNSIGNED_BYTE, data);
         glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
         glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
         stbi_image_free(data);
     }
}

void Drawable::bind() {
    glBindVertexArray(vaoHandle);

    glBindBuffer(GL_ARRAY_BUFFER, vboHandles[VERTEX]);
    glBufferData(GL_ARRAY_BUFFER, vSize*3*sizeof(float), vertices.get(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COORDINATE]);
    glBufferData(GL_ARRAY_BUFFER, vSize*2*sizeof(float), texCoords.get(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COLOR]);
    glBufferData(GL_ARRAY_BUFFER, vSize*3*sizeof(float), colors.get(), GL_STATIC_DRAW);
//
//    glBindBuffer(GL_ARRAY_BUFFER, vboHandles[ALPHA]);
//    glBufferData(GL_ARRAY_BUFFER, vSize*sizeof(float), &alpha[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, vboHandles[NORMAL]);
    glBufferData(GL_ARRAY_BUFFER, vSize*3*sizeof(float), normals.get(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboHandles[ELEMENT]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, (3*fSize)*sizeof(int), elements.get(), GL_STATIC_DRAW);
}

void Drawable::draw(bool showMesh) {

    if (showMesh) {
        // Prevent z-fighting with mesh edges. Reverted at end of this function.
        glPushAttrib(GL_POLYGON_BIT);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1., 1.);
    }

    glBindVertexArray(vaoHandle);

    glBindTexture(GL_TEXTURE_2D, texture);
//    glDrawArrays(GL_POINTS, vSize, pSize);
//    glDrawArrays(GL_POINTS, 0, vSize);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDrawElements(GL_TRIANGLES, fSize*3, GL_UNSIGNED_INT, (void*)0);

    if (showMesh) {
        float frameColors[vSize*3];
        for (int v = 0; v < vSize; v++) {
            for (int i = 0; i < 3; i++) {
                frameColors[v*3 + i] = frameColor[i];
            }
        }

        glBindBuffer(GL_ARRAY_BUFFER, vboHandles[COLOR]);
        glBufferData(GL_ARRAY_BUFFER, vSize*3*sizeof(float), frameColors, GL_STATIC_DRAW);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, fSize*3, GL_UNSIGNED_INT, (void*)0);
    }
    
//    glDrawElements(GL_LINES, lSize*2, GL_UNSIGNED_INT, (void*)(fSize*3*sizeof(int)));

    if (showMesh) {
        // Revert z-fighting prevention enabled at beginning of this function.
        glPopAttrib();
    }
}


} // display
