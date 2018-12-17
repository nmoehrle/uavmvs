/*
 * Copyright (C) 2016-2018, Nils Moehrle
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the LICENSE.txt file for details.
 */

#ifndef WINDOW_HEADER
#define WINDOW_HEADER

#define GL_GLEXT_PROTOTYPES
#include <GLFW/glfw3.h>

#include <memory>
#include <functional>
#include <unordered_map>

class Window {
private:
    GLFWwindow* window;

    std::unordered_map<int, std::function<void(int, int)> > key_callbacks;
    std::unordered_map<int, std::function<void(int)> > mouse_button_callbacks;
    std::unique_ptr<std::function<void(double, double)> > cursor_position_callback;
    std::unique_ptr<std::function<void(double, double)> > scroll_callback;
    static void key_callback_wrapper(GLFWwindow* glfw_window,
        int key, int scancode, int action, int mods);
    static void mouse_button_callback_wrapper(GLFWwindow* glfw_window,
        int button, int action, int mods);
    static void cursor_position_callback_wrapper(GLFWwindow* window,
        double xpos, double ypos);
    static void scroll_callback_wrapper(GLFWwindow* window,
        double xoffset, double yoffset);
public:
    Window(const char * title, int width, int height);
    ~Window(void);

    void register_key_callback(int key, std::function<void(int, int)> const & func);
    void register_mouse_button_callback(int button, std::function<void(int)> const & func);
    void register_cursor_position_callback(std::function<void(double, double)> const & func);
    void register_scroll_callback(std::function<void(double, double)> const & func);
    bool good(void);
    void update(void);
};

#endif // WINDOW_HEADER
