release: DESTDIR = $$PWD/../bin
debug:   DESTDIR = $$PWD/../bin


TEMPLATE = lib
CONFIG += staticlib
QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS += -fext-numeric-literals
CONFIG += link_pkgconfig
unix: PKGCONFIG += pcl_registration-1.8
unix: PKGCONFIG += pcl_geometry-1.8
unix: PKGCONFIG += pcl_features-1.8
unix: PKGCONFIG += pcl_search-1.8
unix: PKGCONFIG += pcl_kdtree-1.8
unix: PKGCONFIG += pcl_filters-1.8
unix: PKGCONFIG += pcl_surface-1.8
unix: PKGCONFIG += pcl_octree-1.8
unix: PKGCONFIG += pcl_sample_consensus-1.8
unix: PKGCONFIG += pcl_segmentation-1.8
unix: PKGCONFIG += pcl_visualization-1.8
unix: PKGCONFIG += pcl_io-1.8
unix: PKGCONFIG += pcl_keypoints-1.8
unix: PKGCONFIG += pcl_tracking-1.8
unix: PKGCONFIG += flann

INCLUDEPATH += ../include
LIBS += -pthread

INCLUDEPATH += /usr/local/include/pcl-1.8/
INCLUDEPATH += /usr/include/eigen3/
INCLUDEPATH += /usr/include/boost/
INCLUDEPATH += /usr/local/include/vtk-7.0/

SOURCES += \
    ../src/viewpoint.cpp

HEADERS +=\
    ../include/viewpoint.h

LIBS += -lboost_date_time -lboost_signals -lboost_system -lboost_thread

LIBS += -L/usr/local/lib/vtk-7.0/ -lvtkCommonCore-7.0 -lvtkCommonExecutionModel-7.0 -lvtkFiltersSources-7.0 -lvtkCommonDataModel-7.0 -lvtkCommonMath-7.0 -lvtkRenderingCore-7.0 -lvtkRenderingLOD-7.0
