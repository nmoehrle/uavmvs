#include <cstdlib>
#include <iostream>

#include <GLFW/glfw3.h>

#include "window.h"

static void key_callback(GLFWwindow* window, int key, int /*scancode*/, int action, int /*mods*/) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
}

Window::Window(const char * title, int width, int height) {
    if (!glfwInit()) {
        throw std::runtime_error("Could not initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    if (title[0] == 0) {
        glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
    }

    this->window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!this->window)
    {
        throw std::runtime_error("Could not create GLFW window");
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

