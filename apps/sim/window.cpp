#include <cstdlib>
#include <iostream>

#include "window.h"

Window::Window(const char * title, int width, int height) {
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    this->window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!this->window)
    {
        std::cerr << "Could not create window" << std::endl;
        glfwTerminate();
        std::exit(EXIT_FAILURE);
    }
    
    glfwMakeContextCurrent(this->window);
    glfwSwapInterval(1);
}

Window::~Window(void) {
    glfwDestroyWindow(this->window);
    glfwTerminate();
}

bool
Window::good(void) {
    return !glfwWindowShouldClose(this->window);
}

void
Window::update(void) {  
    glfwSwapBuffers(this->window);
    glfwPollEvents();
}

//glfwSetWindowSizeCallback(window, window_size_callback);
//glfwSetWindowUserPointer(window)

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

