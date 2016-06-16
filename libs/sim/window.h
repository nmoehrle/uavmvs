#ifndef WINDOW_HEADER
#define WINDOW_HEADER

#define GL_GLEXT_PROTOTYPES
#include <GLFW/glfw3.h>

class Window {
private:
    GLFWwindow* window;

public:
    Window(const char * title, int width, int height);
    ~Window(void);

    bool good(void);
    void update(void);
};

#endif // WINDOW_HEADER
