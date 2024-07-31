# JSLAM2: Real-time SLAM Visualization

## Overview

JSLAM2 is a real-time Simultaneous Localization and Mapping (SLAM) visualization tool. It processes video input and displays both 2D and 3D representations of the SLAM algorithm's output alongside the original video feed.

## Features

- Real-time video processing
- 3D visualization of SLAM results
- 2D map view
- Original video playback
- Multi-threaded design for efficient performance
- Aspect ratio preservation for video display

## Requirements

- C++17 compiler
- OpenCV 4.x
- OpenGL
- GLEW(included)
- GLFW3(included)
- CMake 3.28 or higher

## Project Structure

```
JSLAM2/
├── external/
│   ├── opencv/
│   ├── glew/
│   └── glfw/
├── src/
│   └── main.cpp
├── CMakeLists.txt
└── README.md
```

## Setup

1. Clone the repository:
   ```
   git clone https://github.com/johk3/JSLAM2.git
   cd JSLAM2
   ```

2. Install dependencies:
    - OpenCV, GLEW, and GLFW should be placed in the `external` directory.
    - Ensure the paths in `CMakeLists.txt` match your directory structure.

3. Build the project:
   ```
   mkdir build && cd build
   cmake ..
   make
   ```

## Usage

Run the program with a video file as an argument:

```
./JSLAM footage/video.mp4
```

The application will open three windows:
- 3D View (top-left): Displays the 3D reconstruction from SLAM
- 2D View (bottom-left): Shows a 2D map of the environment
- Video (right): Displays the original video input

## Controls

- ESC: Close the application
- The application automatically adjusts window sizes to fit your monitor

## Contributing

Contributions to JSLAM2 are welcome! Please feel free to submit pull requests, create issues or spread the word.

## License

[Insert your chosen license here]

## Contact

[Your Name] - [your.email@example.com]

Project Link: https://github.com/johk3/JSLAM2
