QT Project -  FaceAR Head Pose Estimation StaticLib based on CLM-framework (Now the project has moved to new repositories: OpenFace - https://github.com/TadasBaltrusaitis/OpenFace Created by TadasBaltrusaitis), eos, dlib.

This project could be used to select good frames from images sequences and estimate the head pose with opencv method: solvePnP and another method based on 2D image face geometrical relationship, then we can send the pose results(R, T) into 3D render engine, and complete the task such as Virtual Eye-Glass Wear-On AR Program.

This code has been tested on Ubuntu 14.04

This requires cmake, OpenCV 3.0.0 (or newer), tbb and boost.

Need to do the following:

Get newest GCC, done using: sudo apt-get update sudo apt-get install build-essential

Cmake: sudo apt-get install cmake

Get BLAS (for dlib) sudo apt-get install libopenblas-dev liblapack-dev

OpenCV 3.0.0, Based on tutorial from http://docs.opencv.org/trunk/doc/tutorials/introduction/linux_install/linux_install.html 4.1 Install OpenCV dependencies: sudo apt-get install git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev checkinstall

4.2 Download OpenCV 3.0.0 https://github.com/Itseez/opencv/archive/3.0.0-beta.zip

4.3 Unzip it and create a build folder there:

unzip 3.0.0-beta.zip
cd opencv-3.0.0-beta
mkdir build
cd build
4.4 Build it using: cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D BUILD_SHARED_LIBS=OFF .. make -j2 sudo make install

Get Boost: sudo apt-get install libboost1.55-all-dev alternatively: sudo apt-get install libboost-all-dev

Make the actual Glasses_PoseEstimation_SelectFrames and compile it using cd Glasses_PoseEstimation_SelectFrames_Use mkdir build cmake -D CMAKE_BUILD_TYPE=RELEASE .. make -j2 cd ..
