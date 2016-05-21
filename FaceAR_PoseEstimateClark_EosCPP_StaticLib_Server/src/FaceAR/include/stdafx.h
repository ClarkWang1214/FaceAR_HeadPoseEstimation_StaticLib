// Precompiled headers stuff

#ifndef __STDAFX_h_
#define __STDAFX_h_

// OpenCV stuff   opencv2/
#include <core/core.hpp>
#include <objdetect.hpp>
#include <calib3d.hpp>
#include <imgcodecs.hpp>
#include <imgproc.hpp>
#include <highgui/highgui.hpp>
#include <videoio/videoio.hpp>  // Video write  opencv2/
#include <videoio/videoio_c.h>  // Video write  opencv2/
//#include <viz/vizcore.hpp> ///*opencv2/*/

// IplImage stuff (get rid of it? TODO)
#include <core/core_c.h>
#include <imgproc/imgproc_c.h>

// C++ stuff
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <iomanip>

#define _USE_MATH_DEFINES
#include <math.h>

// dlib stuff
// Used for face detection
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>

// Boost stuff
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/program_options.hpp>

#include <tbb.h>

// glm
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#endif
