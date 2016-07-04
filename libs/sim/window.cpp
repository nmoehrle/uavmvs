#include <cstdlib>
#include <iostream>

#include "window.h"

void key_callback_wrapper(GLFWwindow* glfw_window,
    int key, int /*scancode*/, int action, int /*mods*/)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(glfw_window, GL_TRUE);
    }

    Window * window = (Window*) glfwGetWindowUserPointer(glfw_window);
    if (window->key_callback != nullptr) {
        window->key_callback(key);
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

    glfwSetWindowUserPointer(this->window, this);
    glfwSetKeyCallback(this->window, key_callback_wrapper);

    glfwMakeContextCurrent(this->window);
    glfwSwapInterval(1);
}

Window::~Window(void) {
    glfwDestroyWindow(this->window);
    glfwTerminate();
}


void
Window::set_key_callback(KeyCallback key_callback) {
    this->key_callback = key_callback;
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

