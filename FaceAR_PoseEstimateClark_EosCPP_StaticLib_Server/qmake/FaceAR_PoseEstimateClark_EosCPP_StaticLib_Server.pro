#-------------------------------------------------
#
# Project created by QtCreator 2015-03-22T19:52:02
#
#-------------------------------------------------

TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
TEMPLATE = lib
CONFIG += staticlib
CONFIG += release
unix {
    target.path = /usr/lib
    INSTALLS += target
}

QMAKE_CXXFLAGS += \
    -std=c++0x

DESTDIR = ../../libs

INCLUDEPATH += \
    /usr/local/include \
    /usr/local/include/opencv \
    /usr/local/include/opencv2

INCLUDEPATH += \
    /usr/include/boost \
    /usr/include/tbb \
    /usr/include/eigen3 \
    /usr/include/suitesparse \
    /usr/local/include/g2o



INCLUDEPATH += \
    ../src \
    ../src/run/include \
    ../src/FaceAR/include \
    ../src/FaceAR/src \
    ../src/PoseEstimation \
    ../src/eos/include \
    ../src/eos \
    ../src/GetGoodFrame \
#    ../src/3rdParty/dlib/include \
    ../src/3rdParty/glm-0.9.7.0 \
    ../src/3rdParty/cereal-1.1.1/include

# 与makefile的相对路径

#LIBS += \
#    ../../src/3rdParty/dlib/libdlib.a

LIBS += \
    -ltbb

LIBS += \
    -lboost_filesystem -lboost_system -lboost_thread -lboost_program_options

LIBS += -L/usr/local/lib -lg2o_core -lg2o_types_slam3d -lg2o_solver_csparse -lg2o_stuff -lg2o_csparse_extension -lg2o_types_sba


LIBS += \
    /usr/local/lib/libopencv_calib3d.so \
    /usr/local/lib/libopencv_objdetect.so \
    /usr/local/lib/libopencv_core.so \
    /usr/local/lib/libopencv_videoio.so \
    /usr/local/lib/libopencv_features2d.so \
    /usr/local/lib/libopencv_viz.so \
    /usr/local/lib/libopencv_highgui.so \
    /usr/local/lib/libopencv_xfeatures2d.so \
    /usr/local/lib/libopencv_imgcodecs.so \
    /usr/local/lib/libopencv_imgproc.so


#LIBS += \
#    -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc \
#    -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs \
#    -lopencv_videoio -lopencv_viz -lopencv_xfeatures2d -lopencv_features2d

HEADERS += \
    ../src/FaceAR/include/CCNF_patch_expert.h \
    ../src/FaceAR/include/DetectionValidator.h \
    ../src/FaceAR/include/Patch_experts.h \
    ../src/FaceAR/include/PAW.h \
    ../src/FaceAR/include/PDM.h \
    ../src/FaceAR/include/stdafx.h \
    ../src/FaceAR/include/SVR_patch_expert.h \
    ../src/FaceAR/include/FaceAR.h \
    ../src/FaceAR/include/FaceAR_core.h \
    ../src/FaceAR/include/FaceAR_utils.h \
    ../src/FaceAR/include/FaceARParameters.h \
    ../src/FaceAR/include/FaceARTracker.h \
    ../src/FaceAR/include/solvePnP.h \
    ../src/FaceAR/include/LandmarksImproved.h \
    ../src/dlib/image_processing/frontal_face_detector.h \
    ../src/dlib/opencv.h \
    ../src/dlib/image_processing/frontal_face_detector_abstract.h \
    ../src/dlib/image_processing/object_detector.h \
    ../src/dlib/image_processing/scan_fhog_pyramid.h \
    ../src/dlib/compress_stream.h \
    ../src/dlib/base64.h \
    ../src/dlib/opencv/cv_image.h \
    ../src/dlib/opencv/cv_image_abstract.h \
    ../src/dlib/opencv/to_open_cv.h \
    ../src/dlib/opencv/to_open_cv_abstract.h \
    ../src/PoseEstimation/PoseEstimation.h \
    ../src/GetGoodFrame/GetGoodFrame.h \
    ../src/GetGoodFrame/polyfit.hpp \
    ../src/GetGoodFrame/solve.h \
    ../src/eos/include/eos/core/Landmark.h \
    ../src/eos/include/eos/core/LandmarkMapper.h \
    ../src/eos/include/eos/fitting/detail/nonlinear_camera_estimation_detail.h \
    ../src/eos/include/eos/fitting/affine_camera_estimation.h \
    ../src/eos/include/eos/fitting/linear_shape_fitting.h \
    ../src/eos/include/eos/fitting/nonlinear_camera_estimation.h \
    ../src/eos/include/eos/morphablemodel/io/cvssp.h \
    ../src/eos/include/eos/morphablemodel/io/mat_cerealisation.h \
    ../src/eos/include/eos/morphablemodel/MorphableModel.h \
    ../src/eos/include/eos/morphablemodel/PcaModel.h \
    ../src/eos/include/eos/render/detail/render_affine_detail.h \
    ../src/eos/include/eos/render/detail/render_detail.h \
    ../src/eos/include/eos/render/detail/texture_extraction_detail.h \
    ../src/eos/include/eos/render/Mesh.h \
    ../src/eos/include/eos/render/render.h \
    ../src/eos/include/eos/render/render_affine.h \
    ../src/eos/include/eos/render/texture_extraction.h \
    ../src/eos/include/eos/render/utils.h \
    ../src/eos/eos_run.h \
    ../src/run/include/pre_main.h

SOURCES += \
    ../src/FaceAR/src/CCNF_patch_expert.cpp \
    ../src/FaceAR/src/DetectionValidator.cpp \
    ../src/FaceAR/src/Patch_experts.cpp \
    ../src/FaceAR/src/PAW.cpp \
    ../src/FaceAR/src/PDM.cpp \
    ../src/FaceAR/src/stdafx.cpp \
    ../src/FaceAR/src/SVR_patch_expert.cpp \
    ../src/FaceAR/src/FaceAR.cpp \
    ../src/FaceAR/src/FaceAR_utils.cpp \
    ../src/FaceAR/src/FaceARTracker.cpp \
    ../src/dlib/all/source.cpp \
    ../src/FaceAR/src/solvePnP.cpp \
    ../src/PoseEstimation/PoseEstimation.cpp \
    ../src/FaceAR/src/LandmarksImproved.cpp \
    ../src/eos/src/eos/core/LandmarkMapper.cpp \
    ../src/eos/src/eos/fitting/detail/nonlinear_camera_estimation_detail.cpp \
    ../src/eos/src/eos/fitting/affine_camera_estimation.cpp \
    ../src/eos/src/eos/fitting/linear_shape_fitting.cpp \
    ../src/eos/src/eos/fitting/nonlinear_camera_estimation.cpp \
    ../src/eos/src/eos/morphablemodel/io/cvssp.cpp \
    ../src/eos/src/eos/morphablemodel/MorphableModel.cpp \
    ../src/eos/src/eos/morphablemodel/PcaModel.cpp \
    ../src/eos/src/eos/render/detail/render_affine_detail.cpp \
    ../src/eos/src/eos/render/detail/render_detail.cpp \
    ../src/eos/src/eos/render/detail/texture_extraction_detail.cpp \
    ../src/eos/src/eos/render/Mesh.cpp \
    ../src/eos/src/eos/render/render.cpp \
    ../src/eos/src/eos/render/render_affine.cpp \
    ../src/eos/src/eos/render/texture_extraction.cpp \
    ../src/eos/src/eos/render/utils.cpp \
    ../src/eos/eos_run.cpp \
    ../src/GetGoodFrame/solve.cpp \
    ../src/GetGoodFrame/GetGoodFrame.cpp \
    ../src/run/src/pre_main.cpp  


#SOURCES += \
#    ../src/run/SimpleFaceAR/FaceAR_main.cpp

