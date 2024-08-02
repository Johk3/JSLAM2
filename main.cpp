#include <opencv2/opencv.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <opencv2/features2d.hpp>
#include <Eigen/Dense>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#define M_PI 3.14159265358979323846


// Globals
std::atomic<bool> should_exit(false);

// SLAMSystem class (placeholder for actual SLAM implementation)
class SLAMSystem {
public:
    SLAMSystem(const cv::Mat& K) : K_(K), orb_(cv::ORB::create(2000)) {
        initializeOptimizer();
    }

    void processFrame(const cv::Mat& frame) {
        if (prev_frame_.empty()) {
            prev_frame_ = frame.clone();
            detectAndComputeFeatures(prev_frame_, prev_keypoints_, prev_descriptors_);
            current_keypoints_ = prev_keypoints_; // Store current keypoints
            return;
        }

        std::vector<cv::KeyPoint> curr_keypoints;
        cv::Mat curr_descriptors;
        detectAndComputeFeatures(frame, curr_keypoints, curr_descriptors);

        std::vector<cv::DMatch> matches;
        matchFeatures(prev_descriptors_, curr_descriptors, matches);

        Eigen::Matrix4d transform = estimateMotion(prev_keypoints_, curr_keypoints, matches);

        updateMap(transform, curr_keypoints, matches);

        prev_frame_ = frame.clone();
        prev_keypoints_ = curr_keypoints;
        prev_descriptors_ = curr_descriptors;
        current_keypoints_ = curr_keypoints; // Store current keypoints
    }

    const std::vector<cv::KeyPoint>& getCurrentKeypoints() const {
        return current_keypoints_;
    }

    const std::vector<Eigen::Vector3d>& getMapPoints() const {
        return map_points_;
    }

    const std::vector<Eigen::Matrix4d>& getCameraPoses() const {
        return camera_poses_;
    }

private:
    cv::Mat K_;
    cv::Ptr<cv::ORB> orb_;
    cv::Mat prev_frame_;
    std::vector<cv::KeyPoint> prev_keypoints_;
    cv::Mat prev_descriptors_;
    std::vector<Eigen::Vector3d> map_points_;
    std::vector<Eigen::Matrix4d> camera_poses_;
    g2o::SparseOptimizer optimizer_;
    std::vector<cv::KeyPoint> current_keypoints_;

    void initializeOptimizer() {
        auto linearSolver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
        auto blockSolver = std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));
        auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
        optimizer_.setAlgorithm(algorithm);
    }

    void detectAndComputeFeatures(const cv::Mat& frame, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
        orb_->detectAndCompute(frame, cv::noArray(), keypoints, descriptors);
    }

    void matchFeatures(const cv::Mat& desc1, const cv::Mat& desc2, std::vector<cv::DMatch>& matches) {
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(desc1, desc2, knn_matches, 2);

        const float ratio_thresh = 0.7f;
        for (const auto& knn_match : knn_matches) {
            if (knn_match[0].distance < ratio_thresh * knn_match[1].distance) {
                matches.push_back(knn_match[0]);
            }
        }
    }

    Eigen::Matrix4d estimateMotion(const std::vector<cv::KeyPoint>& kp1, const std::vector<cv::KeyPoint>& kp2, const std::vector<cv::DMatch>& matches) {
        std::vector<cv::Point2f> points1, points2;
        for (const auto& match : matches) {
            points1.push_back(kp1[match.queryIdx].pt);
            points2.push_back(kp2[match.trainIdx].pt);
        }

        cv::Mat E, R, t, mask;
        E = cv::findEssentialMat(points1, points2, K_, cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, points1, points2, K_, R, t, mask);

        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R.ptr<double>());
        transform.block<3, 1>(0, 3) = Eigen::Map<Eigen::Matrix<double, 3, 1>>(t.ptr<double>());

        return transform;
    }

    void updateMap(const Eigen::Matrix4d& transform, const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::DMatch>& matches) {
        if (camera_poses_.empty()) {
            camera_poses_.push_back(Eigen::Matrix4d::Identity());
        }
        camera_poses_.push_back(camera_poses_.back() * transform);

        Eigen::Matrix4d current_pose = camera_poses_.back();
        Eigen::Matrix3d R1 = camera_poses_[camera_poses_.size() - 2].block<3, 3>(0, 0);
        Eigen::Vector3d t1 = camera_poses_[camera_poses_.size() - 2].block<3, 1>(0, 3);
        Eigen::Matrix3d R2 = current_pose.block<3, 3>(0, 0);
        Eigen::Vector3d t2 = current_pose.block<3, 1>(0, 3);

        for (const auto& match : matches) {
            cv::Point2f p1 = prev_keypoints_[match.queryIdx].pt;
            cv::Point2f p2 = keypoints[match.trainIdx].pt;

            Eigen::Vector3d p3d = triangulatePoint(R1, t1, R2, t2, p1, p2);
            map_points_.push_back(p3d);

            // Add vertex and edge to the optimizer
            g2o::VertexPointXYZ* point = new g2o::VertexPointXYZ();
            point->setEstimate(p3d);
            point->setId(optimizer_.vertices().size());
            optimizer_.addVertex(point);

            g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
            edge->setVertex(0, point);
            edge->setMeasurement(Eigen::Vector2d(p2.x, p2.y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber());
            optimizer_.addEdge(edge);
        }

        // Perform local optimization
        optimizer_.initializeOptimization();
        optimizer_.optimize(5);
    }

    Eigen::Vector3d triangulatePoint(const Eigen::Matrix3d& R1, const Eigen::Vector3d& t1,
                                     const Eigen::Matrix3d& R2, const Eigen::Vector3d& t2,
                                     const cv::Point2f& p1, const cv::Point2f& p2) {
        Eigen::Matrix<double, 3, 4> P1, P2;
        P1.block<3, 3>(0, 0) = R1;
        P1.block<3, 1>(0, 3) = t1;
        P2.block<3, 3>(0, 0) = R2;
        P2.block<3, 1>(0, 3) = t2;

        Eigen::Matrix4d A;
        A.row(0) = p1.x * P1.row(2) - P1.row(0);
        A.row(1) = p1.y * P1.row(2) - P1.row(1);
        A.row(2) = p2.x * P2.row(2) - P2.row(0);
        A.row(3) = p2.y * P2.row(2) - P2.row(1);

        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
        Eigen::Vector4d point_homogeneous = svd.matrixV().col(3);
        return point_homogeneous.head<3>() / point_homogeneous(3);
    }
};

class Renderer3D {
public:
    Renderer3D() : window(nullptr), cameraPosition(0, 0, -5), cameraFront(0, 0, 1), cameraUp(0, 1, 0),
                   yaw(-90.0f), pitch(0.0f), lastX(0), lastY(0), firstMouse(true),
                   moveSpeed(0.05f), mouseSensitivity(0.1f) {}

    bool initialize(int width, int height, int x, int y) {
        window = glfwCreateWindow(width, height, "3D View", NULL, NULL);
        if (!window) {
            return false;
        }
        glfwSetWindowPos(window, x, y);
        glfwMakeContextCurrent(window);
        if (glewInit() != GLEW_OK) return false;

        glEnable(GL_DEPTH_TEST);
        glPointSize(3.0f);

        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwSetWindowUserPointer(window, this);
        glfwSetCursorPosCallback(window, mouse_callback_static);
        glfwSetScrollCallback(window, scroll_callback_static);
        return true;
    }

    void render(const std::vector<Eigen::Vector3d>& mapPoints, const std::vector<Eigen::Matrix4d>& cameraPoses, bool isPaused, int currentPoseIndex) {
        if (cameraPoses.empty()) return;

        glfwMakeContextCurrent(window);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(45.0, (double)width / height, 0.1, 100.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        if (!isPaused) {
            // Update camera position and orientation based on the latest pose
            Eigen::Vector3d latestPosition = cameraPoses.back().block<3, 1>(0, 3);
            Eigen::Vector3d latestOrientation = cameraPoses.back().block<3, 3>(0, 0) * Eigen::Vector3d(0, 0, 1);

            cameraPosition = latestPosition - latestOrientation.normalized() * 2.0;
            cameraFront = latestOrientation.normalized();
            cameraUp = Eigen::Vector3d(0, 1, 0);
        } else {
            // Use the current camera position and orientation
            cameraFront.x() = cos(yaw * M_PI / 180.0) * cos(pitch * M_PI / 180.0);
            cameraFront.y() = sin(pitch * M_PI / 180.0);
            cameraFront.z() = sin(yaw * M_PI / 180.0) * cos(pitch * M_PI / 180.0);
            cameraFront.normalize();
        }

        gluLookAt(cameraPosition.x(), cameraPosition.y(), cameraPosition.z(),
                  cameraPosition.x() + cameraFront.x(), cameraPosition.y() + cameraFront.y(), cameraPosition.z() + cameraFront.z(),
                  cameraUp.x(), cameraUp.y(), cameraUp.z());

        // Draw map points
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        for (const auto& point : mapPoints) {
            glVertex3d(point.x(), point.y(), point.z());
        }
        glEnd();

        // Draw camera trajectory
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINE_STRIP);
        for (const auto& pose : cameraPoses) {
            Eigen::Vector3d position = pose.block<3, 1>(0, 3);
            glVertex3d(position.x(), position.y(), position.z());
        }
        glEnd();

        // Draw current position
        glColor3f(0.0f, 0.0f, 1.0f);
        glPointSize(10.0f);
        glBegin(GL_POINTS);
        Eigen::Vector3d currentPosition = cameraPoses[currentPoseIndex].block<3, 1>(0, 3);
        glVertex3d(currentPosition.x(), currentPosition.y(), currentPosition.z());
        glEnd();
        glPointSize(3.0f);

        glfwSwapBuffers(window);
    }

    GLFWwindow* getWindow() { return window; }

    void processInput(GLFWwindow* window, bool isPaused, int& currentPoseIndex, const std::vector<Eigen::Matrix4d>& cameraPoses) {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
            if (isPaused) {
                cameraPosition += moveSpeed * cameraFront;
            } else {
                currentPoseIndex = std::min(currentPoseIndex + 1, static_cast<int>(cameraPoses.size()) - 1);
            }
        }
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
            if (isPaused) {
                cameraPosition -= moveSpeed * cameraFront;
            } else {
                currentPoseIndex = std::max(currentPoseIndex - 1, 0);
            }
        }
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
            Eigen::Vector3d right = cameraFront.cross(cameraUp).normalized();
            cameraPosition -= right * moveSpeed;
        }
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
            Eigen::Vector3d right = cameraFront.cross(cameraUp).normalized();
            cameraPosition += right * moveSpeed;
        }
    }

private:
    GLFWwindow* window;
    Eigen::Vector3d cameraPosition;
    Eigen::Vector3d cameraFront;
    Eigen::Vector3d cameraUp;
    float yaw, pitch;
    float lastX, lastY;
    bool firstMouse;
    float moveSpeed;
    float mouseSensitivity;

    static void mouse_callback_static(GLFWwindow* window, double xpos, double ypos) {
        Renderer3D* renderer = static_cast<Renderer3D*>(glfwGetWindowUserPointer(window));
        renderer->mouse_callback(xpos, ypos);
    }

    void mouse_callback(double xpos, double ypos) {
        if (firstMouse) {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos;
        lastX = xpos;
        lastY = ypos;

        xoffset *= mouseSensitivity;
        yoffset *= mouseSensitivity;

        yaw += xoffset;
        pitch += yoffset;

        if (pitch > 89.0f) pitch = 89.0f;
        if (pitch < -89.0f) pitch = -89.0f;

        updateCameraVectors();
    }

    static void scroll_callback_static(GLFWwindow* window, double xoffset, double yoffset) {
        Renderer3D* renderer = static_cast<Renderer3D*>(glfwGetWindowUserPointer(window));
        renderer->scroll_callback(xoffset, yoffset);
    }

    void scroll_callback(double xoffset, double yoffset) {
        moveSpeed += static_cast<float>(yoffset) * 0.01f;
        if (moveSpeed < 0.01f) moveSpeed = 0.01f;
        if (moveSpeed > 1.0f) moveSpeed = 1.0f;
    }

    void updateCameraVectors() {
        Eigen::Vector3d front;
        front.x() = cos(yaw * M_PI / 180.0) * cos(pitch * M_PI / 180.0);
        front.y() = sin(pitch * M_PI / 180.0);
        front.z() = sin(yaw * M_PI / 180.0) * cos(pitch * M_PI / 180.0);
        cameraFront = front.normalized();
    }
};



class Renderer2D {
public:
    Renderer2D() : window(nullptr), cameraPosition(0, 0), zoom(20.0), moveSpeed(0.5f) {}

    bool initialize(int width, int height, int x, int y) {
        window = glfwCreateWindow(width, height, "2D View", NULL, NULL);
        if (!window) {
            return false;
        }
        glfwSetWindowPos(window, x, y);
        glfwMakeContextCurrent(window);
        if (glewInit() != GLEW_OK) return false;

        glPointSize(3.0f);
        glfwSetWindowUserPointer(window, this);
        glfwSetScrollCallback(window, scroll_callback_static);
        return true;
    }

    void render(const std::vector<Eigen::Vector3d>& mapPoints, const std::vector<Eigen::Matrix4d>& cameraPoses, bool isPaused, int currentPoseIndex) {
        if (cameraPoses.empty()) return;

        glfwMakeContextCurrent(window);
        glClear(GL_COLOR_BUFFER_BIT);

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        if (!isPaused) {
            // Update camera position to follow the latest pose
            Eigen::Vector3d latestPosition = cameraPoses.back().block<3, 1>(0, 3);
            cameraPosition = Eigen::Vector2d(latestPosition.x(), latestPosition.z());
        }

        // Set up the view to follow the camera
        glOrtho(cameraPosition.x() - zoom, cameraPosition.x() + zoom,
                cameraPosition.y() - zoom * height / width, cameraPosition.y() + zoom * height / width,
                -1, 1);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Draw map points
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        for (const auto& point : mapPoints) {
            glVertex2d(point.x(), point.z());  // Use x and z for top-down view
        }
        glEnd();

        // Draw camera trajectory
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINE_STRIP);
        for (const auto& pose : cameraPoses) {
            Eigen::Vector3d position = pose.block<3, 1>(0, 3);
            glVertex2d(position.x(), position.z());  // Use x and z for top-down view
        }
        glEnd();

        // Draw current position
        glColor3f(0.0f, 0.0f, 1.0f);
        glPointSize(10.0f);
        glBegin(GL_POINTS);
        Eigen::Vector3d currentPosition = cameraPoses[currentPoseIndex].block<3, 1>(0, 3);
        glVertex2d(currentPosition.x(), currentPosition.z());
        glEnd();
        glPointSize(3.0f);

        glfwSwapBuffers(window);
    }

    GLFWwindow* getWindow() { return window; }

    void processInput(GLFWwindow* window, bool isPaused, int& currentPoseIndex, const std::vector<Eigen::Matrix4d>& cameraPoses) {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
            if (isPaused) {
                cameraPosition.y() -= moveSpeed;
            } else {
                currentPoseIndex = std::min(currentPoseIndex + 1, static_cast<int>(cameraPoses.size()) - 1);
            }
        }
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
            if (isPaused) {
                cameraPosition.y() += moveSpeed;
            } else {
                currentPoseIndex = std::max(currentPoseIndex - 1, 0);
            }
        }
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            cameraPosition.x() -= moveSpeed;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            cameraPosition.x() += moveSpeed;
    }

private:
    GLFWwindow* window;
    Eigen::Vector2d cameraPosition;
    double zoom;
    float moveSpeed;

    static void scroll_callback_static(GLFWwindow* window, double xoffset, double yoffset) {
        Renderer2D* renderer = static_cast<Renderer2D*>(glfwGetWindowUserPointer(window));
        renderer->scroll_callback(xoffset, yoffset);
    }

    void scroll_callback(double xoffset, double yoffset) {
        zoom -= yoffset;
        if (zoom < 1.0) zoom = 1.0;
        if (zoom > 100.0) zoom = 100.0;
    }
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
    // Initialize SLAM system with camera intrinsics
    double focal_length = 500.0;  // Replace with your camera's actual focal length
    cv::Point2d principal_point(videoWidth / 2, videoHeight / 2);  // Assume principal point is at the center of the image
    cv::Mat K = (cv::Mat_<double>(3, 3) <<
        focal_length, 0, principal_point.x,
        0, focal_length, principal_point.y,
        0, 0, 1);
    SLAMSystem slam(K);
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

    bool isPaused = false;
    int currentPoseIndex = 0;
    bool wasKeyPressed = false;

    while (!should_exit) {
        // Check for ESC key press in both 3D and 2D windows
        if (glfwGetKey(renderer3d.getWindow(), GLFW_KEY_ESCAPE) == GLFW_PRESS ||
            glfwGetKey(renderer2d.getWindow(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            should_exit = true;
            break;
        }

        // Check for 'P' key press to toggle pause
        if (glfwGetKey(renderer3d.getWindow(), GLFW_KEY_P) == GLFW_PRESS ||
            glfwGetKey(renderer2d.getWindow(), GLFW_KEY_P) == GLFW_PRESS) {
            if (!wasKeyPressed) {
                isPaused = !isPaused;
                wasKeyPressed = true;
                std::cout << (isPaused ? "Paused" : "Resumed") << std::endl;
            }
        } else {
            wasKeyPressed = false;
        }

        // Check if either window is closed
        if (glfwWindowShouldClose(renderer3d.getWindow()) || glfwWindowShouldClose(renderer2d.getWindow())) {
            should_exit = true;
            break;
        }

        // Get the latest map points and camera poses
        const auto& mapPoints = slam.getMapPoints();
        const auto& cameraPoses = slam.getCameraPoses();

        // Process input for both renderers
        renderer3d.processInput(renderer3d.getWindow(), isPaused, currentPoseIndex, cameraPoses);
        renderer2d.processInput(renderer2d.getWindow(), isPaused, currentPoseIndex, cameraPoses);

        // Update SLAM and video playback when not paused
        if (!isPaused) {
            FrameData frameData;
            if (frameQueue.pop(frameData)) {
                slam.processFrame(frameData.frame);
                currentPoseIndex = cameraPoses.size() - 1;

                // Draw tracked points on the frame
                cv::Mat displayFrame = frameData.frame.clone();
                const auto& currentKeypoints = slam.getCurrentKeypoints();
                for (const auto& kp : currentKeypoints) {
                    cv::rectangle(displayFrame,
                                  cv::Point(kp.pt.x - 2, kp.pt.y - 2),
                                  cv::Point(kp.pt.x + 2, kp.pt.y + 2),
                                  cv::Scalar(0, 0, 255), 2);
                }

                cv::imshow("Video", displayFrame);
            }
        }


        // Render both views
        renderer3d.render(mapPoints, cameraPoses, isPaused, currentPoseIndex);
        renderer2d.render(mapPoints, cameraPoses, isPaused, currentPoseIndex);

        // Check for ESC key press in OpenCV window
        int key = cv::waitKey(1);
        if (key == 27) {  // ESC key
            should_exit = true;
            break;
        } else if (key == 'p' || key == 'P') {
            isPaused = !isPaused;
            std::cout << (isPaused ? "Paused" : "Resumed") << std::endl;
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
