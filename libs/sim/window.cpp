#include <cstdlib>
#include <iostream>

#include "window.h"

void
Window::key_callback_wrapper(GLFWwindow* glfw_window,
    int key, int /*scancode*/, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(glfw_window, GL_TRUE);
    }

    Window * window = (Window*) glfwGetWindowUserPointer(glfw_window);
    std::unordered_map<int, std::function<void (int, int)> >::iterator it;
    it = window->key_callbacks.find(key);
    if (it != window->key_callbacks.end()) {
        it->second(action, mods);
    }
}

void
Window::mouse_button_callback_wrapper(GLFWwindow* glfw_window,
    int button, int action, int /*mods*/)
{
    Window * window = (Window*) glfwGetWindowUserPointer(glfw_window);
    std::unordered_map<int, std::function<void (int)> >::iterator it;
    it = window->mouse_button_callbacks.find(button);
    if (it != window->mouse_button_callbacks.end()) {
        it->second(action);
    }
}

void
Window::cursor_position_callback_wrapper(GLFWwindow* glfw_window,
    double xpos, double ypos)
{
    Window * window = (Window*) glfwGetWindowUserPointer(glfw_window);
    if (window->cursor_position_callback != nullptr) {
        (*window->cursor_position_callback)(xpos, ypos);
    }
}

void
Window::scroll_callback_wrapper(GLFWwindow* glfw_window,
    double xoffset, double yoffset)
{
    Window * window = (Window*) glfwGetWindowUserPointer(glfw_window);
    if (window->scroll_callback != nullptr) {
        (*window->scroll_callback)(xoffset, yoffset);
    }
}

Window::Window(const char * title, int width, int height) {
    if (!glfwInit()) {
        throw std::runtime_error("Could not initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_SRGB_CAPABLE, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    if (title[0] == 0) {
        glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
    }

    this->window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!this->window) {
        throw std::runtime_error("Could not create GLFW window");
    }

    glfwSetWindowUserPointer(this->window, this);
    glfwSetKeyCallback(this->window, key_callback_wrapper);

    glfwSetMouseButtonCallback(window, mouse_button_callback_wrapper);
    glfwSetCursorPosCallback(window, cursor_position_callback_wrapper);
    glfwSetScrollCallback(window, scroll_callback_wrapper);

    glfwMakeContextCurrent(this->window);
    glfwSwapInterval(1);
}

Window::~Window(void) {
    glfwDestroyWindow(this->window);
    glfwTerminate();
}

void
Window::register_key_callback(int key, std::function<void(int, int)> const & func) {
    key_callbacks.insert(std::make_pair(key, func));
}

void
Window::register_mouse_button_callback(int button, std::function<void(int)> const & func) {
    mouse_button_callbacks.insert(std::make_pair(button, func));
}

void
Window::register_cursor_position_callback(std::function<void(double, double)> const & func) {
    cursor_position_callback = std::unique_ptr<std::function<void(double, double)> >(
        new std::function<void(double, double)>(func)
    );
}

void
Window::register_scroll_callback(std::function<void(double, double)> const & func) {
    scroll_callback = std::unique_ptr<std::function<void(double, double)> >(
        new std::function<void(double, double)>(func)
    );
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
