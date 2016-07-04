#ifndef WINDOW_HEADER
#define WINDOW_HEADER

#define GL_GLEXT_PROTOTYPES
#include <GLFW/glfw3.h>

class Window {
public:
    typedef void (*KeyCallback)(int);
private:
    GLFWwindow* window;

    KeyCallback key_callback = nullptr;
    friend void key_callback_wrapper(GLFWwindow* glfw_window,
        int key, int scancode, int action, int mods);
public:
    Window(const char * title, int width, int height);
    ~Window(void);

    void set_key_callback(KeyCallback key_callback);
    bool good(void);
    void update(void);
};

#endif // WINDOW_HEADER
