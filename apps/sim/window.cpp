#include <cstdlib>
#include <iostream>

#include "window.h"

static void key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

Window::Window(const char * title, int width, int height) {
    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    this->window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!this->window)
    {
        std::cerr << "Could not create window" << std::endl;
        glfwTerminate();
        std::exit(EXIT_FAILURE);
    }

    glfwSetKeyCallback(this->window, key_callback);

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

