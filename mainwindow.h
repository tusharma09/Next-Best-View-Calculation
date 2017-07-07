#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
//#include <mathMatrix.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

//#include <mathMat.h>

#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/geometry/mesh_base.h>
#include <pcl/geometry/mesh_elements.h>
#include <pcl/geometry/mesh_traits.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/geometry/mesh_indices.h>
#include <pcl/geometry/get_boundary.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/common/geometry.h>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_btnFindNextPose_released();

private:
    Ui::MainWindow *ui;

    ///First Filter
    /*const*/ double _first_filter_distance = 0.004;
    ///
    ///Second Filter
    /*const*/ double _second_filter_distance = 0.001;
    ///
    ////Find directed boudaries
    /*const*/ float _penalty_tolerance = 0.3, _angle_tolerance = 50;
    /*const*/ int _min_length_for_boundary_edge = 5;
    ///
    ///Find View point at distance from object.. Minimum by specifications is 25 cm
    /*const*/ double _camera_distance = 0.250;
    ///
    ///GP3 radius distance- minimum length of edge
    /*const*/ double _gp3_radius = 0.007;
    ///
    /*const*/ double _join_boundaries_distance = 0.010;
    ///
    ///
    /*const*/ double _gc_smallRadius = 0.002, _gc_defaultBigRadius =0.004,  _gc_incBigRadius= 0.001, _gc_maxBigRadius = 0.010;
    double _gc_bigRadius = _gc_defaultBigRadius;
    ///
    ///overlapping; polynomial is estimated between 0 and 1, 1 being outside.
    /*const*/ double _overlap_coefficient = 0;
    ///
    ///stopping criteria while moving camera on camera path; less degrees takes viewpoint further
    double _camera_path_min_angle = 60;
    ///
};

#endif // MAINWINDOW_H
