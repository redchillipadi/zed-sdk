
///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/**************************************************************************************************
 ** This sample demonstrates how to grab images and disparity map with the ZED SDK                **
 ** The GPU buffer is ingested directly into OpenGL texture for avoiding  GPU->CPU readback time  **
 ** For the Left image, a GLSL shader is used for RGBA-->BGRA transformation , just as an example **
 ***************************************************************************************************/

#include <stdio.h>
#include <string.h>
#include <ctime>

#include <sl/Camera.hpp>
#include <thread>

#include <GL/glew.h>
#include "GL/freeglut.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

using namespace sl;
using namespace std;

///Defines
#define DROPPED_FRAME_COUNT //uncomment this line to activate frame dropped counter

//declare some resources (GL texture ID, GL shader ID...)
GLuint imageTex;
GLuint depthTex;
cudaGraphicsResource* pcuImageRes;
cudaGraphicsResource* pcuDepthRes;
Camera zed;
RuntimeParameters runtime_parameters;
InitParameters init_parameters;
Mat gpuLeftImage ;
Mat gpuDepthImage;


int w;
int h;

GLuint shaderF;
GLuint program;

bool quit;

#ifdef DROPPED_FRAME_COUNT
int save_dropped_frame = 0;
int grab_counter = 0;
#endif

//Here is the very simple fragment shader for flipping Red and Blue
std::string strFragmentShad = ("uniform sampler2D texImage;\n"
        " void main() {\n"
        " vec4 color = texture2D(texImage, gl_TexCoord[0].st);\n"
        " gl_FragColor = vec4(color.b, color.g, color.r, color.a);\n}");


//glut main loop : grab --> extract GPU Mat --> send to OpenGL and quad.

void draw() {
    // used for jetson only, the calling thread will be executed on core n2.
    // does nothing for Linux/windows.
    sl::Camera::sticktoCPUCore(2);

    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;
    int res = zed.grab(runtime_parameters);

    if (res == 0) {

        //get frame dropped.
#ifdef DROPPED_FRAME_COUNT
        grab_counter++;
        if (zed.getFrameDroppedCount() > save_dropped_frame) {
            save_dropped_frame = zed.getFrameDroppedCount();
            std::cout << save_dropped_frame << " dropped frames detected (ratio = " << 100.f * save_dropped_frame / (float) (save_dropped_frame + grab_counter) << ")" << std::endl;

        }
#endif

        // Map GPU Ressource for Image
        // With Gl texture, we have to use the cudaGraphicsSubResourceGetMappedArray cuda functions. It will link the gl texture with a cuArray.
        // Then, we just have to copy our GPU Buffer to the CudaArray (D2D copy).
        // Between , you can do some other GPU stuffs.
        if (zed.retrieveImage(gpuLeftImage,VIEW_LEFT,MEM_GPU)==SUCCESS)
        {
            cudaArray_t ArrIm;
            cudaGraphicsMapResources(1, &pcuImageRes, 0);
            cudaGraphicsSubResourceGetMappedArray(&ArrIm, pcuImageRes, 0, 0);
            cudaMemcpy2DToArray(ArrIm, 0, 0, gpuLeftImage.getPtr<sl::uchar1>(MEM_GPU), gpuLeftImage.getStepBytes(MEM_GPU), gpuLeftImage.getWidth() * sizeof(sl::uchar4), gpuLeftImage.getHeight(), cudaMemcpyDeviceToDevice);
            cudaGraphicsUnmapResources(1, &pcuImageRes, 0);
        }

        //map GPU Ressource for Depth !! Image of depth == 8U 4channels
        if (zed.retrieveImage(gpuDepthImage,VIEW_DEPTH,MEM_GPU)==SUCCESS)
        {
            cudaArray_t ArrDe;
            cudaGraphicsMapResources(1, &pcuDepthRes, 0);
            cudaGraphicsSubResourceGetMappedArray(&ArrDe, pcuDepthRes, 0, 0);
            cudaMemcpy2DToArray(ArrDe, 0, 0, gpuDepthImage.getPtr<sl::uchar1>(MEM_GPU), gpuDepthImage.getStepBytes(MEM_GPU), gpuLeftImage.getWidth() * sizeof(sl::uchar4), gpuLeftImage.getHeight(), cudaMemcpyDeviceToDevice);
            cudaGraphicsUnmapResources(1, &pcuDepthRes, 0);
        }



        //OpenGL Part
        glDrawBuffer(GL_BACK); //write to both BACK_LEFT & BACK_RIGHT
        glLoadIdentity();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        //Draw Image Texture in Left Part of Side by Side
        glBindTexture(GL_TEXTURE_2D, imageTex);
        //flip R and B with GLSL Shader
        glUseProgram(program);

        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 1.0);
        glVertex2f(-1.0, -1.0);
        glTexCoord2f(1.0, 1.0);
        glVertex2f(0.0, -1.0);
        glTexCoord2f(1.0, 0.0);
        glVertex2f(0.0, 1.0);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(-1.0, 1.0);
        glEnd();

        glUseProgram(0);

        //Draw Depth Texture in Right Part of Side by Side
        glBindTexture(GL_TEXTURE_2D, depthTex);

        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 1.0);
        glVertex2f(0.0, -1.0);
        glTexCoord2f(1.0, 1.0);
        glVertex2f(1.0, -1.0);
        glTexCoord2f(1.0, 0.0);
        glVertex2f(1.0, 1.0);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.0, 1.0);
        glEnd();

        //swap.
        glutSwapBuffers();
    }

    glutPostRedisplay();

    if (quit) {
        gpuLeftImage.free();
        gpuDepthImage.free();
        zed.close();
        glDeleteShader(shaderF);
        glDeleteProgram(program);
        glBindTexture(GL_TEXTURE_2D, 0);
        glutDestroyWindow(1);
    }
}

void close() {
    gpuLeftImage.free();
    gpuDepthImage.free();
    zed.close();
    glDeleteShader(shaderF);
    glDeleteProgram(program);
    glBindTexture(GL_TEXTURE_2D, 0);
}

int main(int argc, char **argv) {

    if (argc > 2) {
        std::cout << "Only the path of a SVO can be passed in arg" << std::endl;
        return -1;
    }
    //init glut
    glutInit(&argc, argv);

    /*Setting up  The Display  */
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    //Configure Window Postion
    glutInitWindowPosition(50, 25);

    //Configure Window Size
    glutInitWindowSize(1280, 480);

    //Create Window
    glutCreateWindow("ZED DISPARITY Viewer");

    //glutFullScreen();

    //init GLEW Library
    glewInit();


    //Setup our ZED Camera (construct and Init)
    if (argc == 1) { // Use in Live Mode
       init_parameters.camera_resolution = sl::RESOLUTION_HD720;
       init_parameters.camera_fps = 30.f;
    } else // Use in SVO playback mode
        init_parameters.svo_input_filename = sl::String(argv[1]);

    init_parameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
    sl::ERROR_CODE err = zed.open(init_parameters);
    // ERRCODE display

    if (err != SUCCESS) {
        std::cout << "ZED Opening Error: " << errorCode2str(err) << std::endl;
        zed.close();
        return -1;
    }

    quit = false;

    //get Image Size
    w = zed.getResolution().width;
    h = zed.getResolution().height;

    cudaError_t err1, err2;

    //Create and Register OpenGL Texture for Image (RGBA -- 4channels)
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &imageTex);
    glBindTexture(GL_TEXTURE_2D, imageTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
    err1 = cudaGraphicsGLRegisterImage(&pcuImageRes, imageTex, GL_TEXTURE_2D, cudaGraphicsMapFlagsNone);

    //Create and Register a OpenGL texture for Depth (RGBA- 4 Channels)
    glGenTextures(1, &depthTex);
    glGenTextures(1, &depthTex);
    glBindTexture(GL_TEXTURE_2D, depthTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
    err2 = cudaGraphicsGLRegisterImage(&pcuDepthRes, depthTex, GL_TEXTURE_2D, cudaGraphicsMapFlagsNone);

    if (err1 != 0 || err2 != 0) return -1;

    //Create GLSL fragment Shader for future processing (and here flip R/B)
    GLuint shaderF = glCreateShader(GL_FRAGMENT_SHADER); //fragment shader
    const char* pszConstString = strFragmentShad.c_str();
    glShaderSource(shaderF, 1, (const char**) &pszConstString, NULL);

    // compile the shader source code and check
    glCompileShader(shaderF);
    GLint compile_status = GL_FALSE;
    glGetShaderiv(shaderF, GL_COMPILE_STATUS, &compile_status);
    if (compile_status != GL_TRUE) return -2;

    //create the progamm for both V and F Shader
    program = glCreateProgram();
    glAttachShader(program, shaderF);
    //and link + Check
    glLinkProgram(program);
    GLint link_status = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &link_status);
    if (link_status != GL_TRUE) return -2;

    glUniform1i(glGetUniformLocation(program, "texImage"), 0);

    //Set Draw Loop
    glutDisplayFunc(draw);
    glutCloseFunc(close);
    glutMainLoop();

    return 0;
}



