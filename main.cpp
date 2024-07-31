#include <opencv2/opencv.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

// Globals
std::atomic<bool> should_exit(false);

// SLAMSystem class (placeholder for actual SLAM implementation)
class SLAMSystem {
public:
    void processFrame(const cv::Mat& frame) {
        // Implement SLAM algorithm here
    }
    void updateMap() {
        // Update internal map representation
    }
};

class Renderer3D {
public:
    Renderer3D() : window(nullptr) {}

    bool initialize(int width, int height, int x, int y) {
        window = glfwCreateWindow(width, height, "3D View", NULL, NULL);
        if (!window) {
            return false;
        }
        glfwSetWindowPos(window, x, y);
        glfwMakeContextCurrent(window);
        if (glewInit() != GLEW_OK) return false;
        return true;
    }

    void render(const SLAMSystem& slam) {
        glfwMakeContextCurrent(window);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Implement 3D rendering here

        glfwSwapBuffers(window);
    }

    GLFWwindow* getWindow() { return window; }

private:
    GLFWwindow* window;
};

class Renderer2D {
public:
    Renderer2D() : window(nullptr) {}

    bool initialize(int width, int height, int x, int y) {
        window = glfwCreateWindow(width, height, "2D View", NULL, NULL);
        if (!window) {
            return false;
        }
        glfwSetWindowPos(window, x, y);
        glfwMakeContextCurrent(window);
        if (glewInit() != GLEW_OK) return false;
        return true;
    }

    void render(const SLAMSystem& slam) {
        glfwMakeContextCurrent(window);
        glClear(GL_COLOR_BUFFER_BIT);

        // Implement 2D rendering here

        glfwSwapBuffers(window);
    }

    GLFWwindow* getWindow() { return window; }

private:
    GLFWwindow* window;
};

struct FrameData {
    cv::Mat frame;
    double timestamp;
};

class FrameQueue {
public:
    void push(const FrameData& data) {
        std::unique_lock<std::mutex> lock(mutex);
        // Wait until the queue is not empty
        queue.push(data);
        condition.notify_one();
    }

    bool pop(FrameData& data) {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this] { return !queue.empty(); });
        data = queue.front();
        queue.pop();
        return true;
    }

private:
    std::queue<FrameData> queue;
    std::mutex mutex;
    std::condition_variable condition;
};

// Function to process video frames in a separate thread
void processVideoThread(FrameQueue& frameQueue, const std::string& videoPath, int& videoWidth, int& videoHeight) {
    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cerr << "Error opening video file" << std::endl;
        should_exit = true;
        return;
    }

    videoWidth = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    videoHeight = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

    double fps = cap.get(cv::CAP_PROP_FPS);
    double frameTime = 1000.0 / fps;

    cv::Mat frame;
    double timestamp = 0;
    while (!should_exit && cap.read(frame)) {
        frameQueue.push({frame.clone(), timestamp});
        timestamp += frameTime;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(frameTime)));
    }
}


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_video_file>" << std::endl;
        return -1;
    }

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Get primary monitor resolution
    GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primaryMonitor);
    int monitorWidth = mode->width;
    int monitorHeight = mode->height;

    // Calculate window sizes
    int window3DWidth = monitorWidth / 2;
    int window3DHeight = monitorHeight / 2;
    int window2DWidth = monitorWidth / 2;
    int window2DHeight = monitorHeight / 2;

    int videoWidth = 0, videoHeight = 0;
    FrameQueue frameQueue;
    SLAMSystem slam;
    Renderer3D renderer3d;
    Renderer2D renderer2d;

    // Initialize renderers
    if (!renderer3d.initialize(window3DWidth, window3DHeight, 0, 0) ||
        !renderer2d.initialize(window2DWidth, window2DHeight, 0, window3DHeight)) {
        std::cerr << "Failed to initialize renderers" << std::endl;
        return -1;
    }

    // Start video processing thread
    std::thread videoThread(processVideoThread, std::ref(frameQueue), argv[1], std::ref(videoWidth), std::ref(videoHeight));

    // Wait for video dimensions to be set by the video thread
    while (videoWidth == 0 || videoHeight == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Calculate video window size while maintaining aspect ratio
    float videoAspectRatio = static_cast<float>(videoWidth) / videoHeight;
    int maxVideoWidth = monitorWidth / 2;
    int maxVideoHeight = monitorHeight;

    int actualVideoWidth, actualVideoHeight;
    if (videoAspectRatio > static_cast<float>(maxVideoWidth) / maxVideoHeight) {
        actualVideoWidth = maxVideoWidth;
        actualVideoHeight = static_cast<int>(maxVideoWidth / videoAspectRatio);
    } else {
        actualVideoHeight = maxVideoHeight;
        actualVideoWidth = static_cast<int>(maxVideoHeight * videoAspectRatio);
    }

    // Calculate video window position (centered in the right half of the screen)
    int videoX = monitorWidth / 2;
    int videoY = (monitorHeight - actualVideoHeight) / 2;

    // Create and position the video window
    cv::namedWindow("Video", cv::WINDOW_NORMAL);
    cv::moveWindow("Video", videoX, videoY);
    cv::resizeWindow("Video", actualVideoWidth, actualVideoHeight);

    while (!should_exit) {
        // Check for ESC key press in both 3D and 2D windows
        if (glfwGetKey(renderer3d.getWindow(), GLFW_KEY_ESCAPE) == GLFW_PRESS ||
            glfwGetKey(renderer2d.getWindow(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            should_exit = true;
            break;
            }

        // Check if either window is closed
        if (glfwWindowShouldClose(renderer3d.getWindow()) || glfwWindowShouldClose(renderer2d.getWindow())) {
            should_exit = true;
            break;
        }

        FrameData frameData;
        if (frameQueue.pop(frameData)) {
            slam.processFrame(frameData.frame);
            slam.updateMap();

            renderer3d.render(slam);
            renderer2d.render(slam);

            cv::imshow("Video", frameData.frame);

            // Check for ESC key press in OpenCV window
            if (cv::waitKey(1) == 27) {
                should_exit = true;
                break;
            }
        }

        glfwPollEvents();
    }

    // Signal the video thread to stop
    should_exit = true;

    // Cleanup
    glfwTerminate();
    videoThread.join();

    // Ensure OpenCV windows are closed
    cv::destroyAllWindows();

    return 0;
}
