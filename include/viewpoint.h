#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

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
#include <pcl/features/boundary.h>



class ViewPoint
{
    //////////////////// Distances in m //// allowed from c++11
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
    /*const*/ double _camera_distance = 0.25;
    ///
    ///GP3 radius distance- minimum length of edge
    /*const*/ double _gp3_radius = 0.007;
    ///
    /*const*/ double _join_boundaries_distance = 0.010;
    ///
    ///
    /*const*/ double _gc_smallRadius = 0.002, _gc_defaultBigRadius = 0.003,  _gc_incBigRadius= 0.001, _gc_maxBigRadius = 0.010;
    double _gc_bigRadius = _gc_defaultBigRadius;
    ///
    ///overlapping; polynomial is estimated between 0 and 1, 1 being outside.
    /*const*/ double _overlap_coefficient = 0.8;
    ///
    ///stopping criteria while moving camera on camera path; less degrees takes viewpoint further
    double _camera_path_min_angle = 50;


    typedef std::vector<std::vector<uint32_t> > Vector_VectorOfInt;

    struct Unidirected_Boundary
    {
        uint16_t direction;
        std::vector<uint32_t>  points;
        ///order of points and points_coordinates is not same, points coordinates is sorted by either x or y
        std::vector<pcl::PointXYZRGB>  points_coordinates;
        std::vector<pcl::PointXYZRGB> points_chain_for_first_point;
        std::vector<pcl::PointXYZRGB> points_chain_for_last_point;
        std::vector<double> axfirst,ayfirst,azfirst;////x= ax[0] + ax[1]*t + ax[2]*t²
        std::vector<double> axlast,aylast,azlast;

    };


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr global_cloud_normal;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud, source_cloud, scan_cloud;

    std::vector<Unidirected_Boundary> boundaries;

    Vector_VectorOfInt directed_boundary_edges, edges;

    pcl::PointXYZ x_axis, y_axis, z_axis;
    pcl::PointXYZRGB camera_point;
    int next_pose_direction = -1;
    std::vector<uint> angles_to_avoid;
    pcl::PointXYZ vector_downwards;
    uint32_t direction;
    std::vector<uint32_t> directed_boundary_edge;


    ///////////For display and debugging
    pcl::PolygonMesh global_triangles;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudJustForDisplay;
    std::vector<Unidirected_Boundary> global_boundaries;
    int click = 0;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    bool display = false, extendSurface=true, show_corners=true, show_chain_at_surface=true
            , show_base_point=true;
    bool show_line=true, show_chain_distance=true, show_camera_path=true, show_point_behind=true
            , show_camera_point=true, show_axis=true;
    ///////////For display and debugging


    void FilterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, double length /*= 0.001f*/, uint32_t MeanK /*= 50*/);

    void CalculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals);

    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> GreedyProjectionTriangulation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals);

    void GetBoundaryEdges(pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3);

    float AngleBetweenPoints(pcl::PointXYZRGB A, pcl::PointXYZRGB B, pcl::PointXYZRGB C, pcl::PointXYZRGB D/* = pcl::PointXYZRGB()*/, bool inXYplane/*=true*/);

    bool FindDirectedBoundries(uint32_t point0, uint32_t point1, float angle_sum, float penalty, uint32_t point2/* = 0*/, uint8_t is_around_0/* = 0*/);

    void GetChain(std::vector<pcl::PointXYZRGB>& chain, std::vector<pcl::PointXYZRGBNormal>& chain_normal, uint32_t numbers_of_points, bool is_with_normal, uint32_t direction);

    void FindDirectedBoundries();

    void FindDirectionOfBoundaries();

    void FindSetOfDirectedBoundariesWithMaxPoints();

    void JoinCloseBoundaries();

    double* PolynomialFit(double y[], double x[], int N /*= 10*/);

    void EstimateSurface();

    void FindViewPoint(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals);

    double GetAngleBetweenVectors(pcl::PointXYZ v1, pcl::PointXYZ v2);

    inline void RemoveBoundariesForAngle(int angle, int angle_range);

    void GetBoundaryPointsCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_points_cloud);

    void FilterEdges(pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_points_cloud);

    void ShowDirectedBoundaryPoints();


public:
    ViewPoint(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1);

    ////Sets the scan_cloud for boundary points filtration
    void SetScanCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    ////Set parameters for overall calculation of Next Best View
    /// param in: first_filter_distance: voxel filter length for mesh
    /// param in: second_filter_distance: voxel filter length to speed up further calculations of chains and viewpoint
    /// param in: gp3_radius: length of edge in mesh
    /// param in: join_boundaries_distance: join boundary sets if closer than this distance
    /// param in: camera_distance: distance of camera from the surface, should be > 0.2 m
    /// param in: overlap_coefficient: adjust overlap with the previous scan
    /// param in: camera_path_min_angle: angle to adjust the orientation of the NBV with the overlapped surface
    void ViewPointCalculationSettings(double first_filter_distance, double second_filter_distance
                                      , double gp3_radius, double join_boundaries_distance, double camera_distance
                                      , double overlap_coefficient, double camera_path_min_angle);


    ////Set parameters for finding boundary sets
    /// param in: angle_tolerance: maximum angle deviation allowed in the boundary set
    /// param in: penalty_tolerance: collective penalty on boundary set for edges with more than angle tolerance orientation;
    ///           in percentage of off edges in the boundary set between 0-1; if more than a limit boundary is restricted
    void SetParametersForDirectedBoundaries(float penalty_tolerance, float angle_tolerance ,int min_length_for_boundary_edge);


    ////Set parameters to find chain points
    /// param in: gc_smallRadius: points should be minimum this distance away
    /// param in: gc_defaultBigRadius: points should be maximum this distance away
    /// param in: gc_incBigRadius: increment to gc_defaultBigRadius if no point found
    /// param in: gc_maxBigRadius: maximum value gc_defaultBigRadius can be incremented
    void SetParametersForChain(double gc_smallRadius, double gc_defaultBigRadius, double  gc_incBigRadius, double gc_maxBigRadius);


    ////Restrict the movement in the given direction
    /// param in: angles_avoid: vector of directions(angles to the x-axis) to avoid
    void SetAnglesToAvoid(std::vector<uint> angles_to_avoid);


    ////Restrict the movement in the given direction
    /// param in: vector: direction(in the vector direction) to avoid
    void SetVectorDirectionToAvoid(pcl::PointXYZ vector);


    /////Find the Next Best View for the source_cloud
    /// Act as the main function of the class, controls the flow of algorithm
    /// param in: source_cloud: point cloud for which NBV to be found
    int FindNextViewPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud);


    ////Gets the Direction of the next movement as the angle to the x_axis in degrees
    int GetNextPoseDirection();


    ////Gets the Next Best View position
    pcl::PointXYZRGB GetCameraPoint();


    ////Return the angle of x-axis vector of the NBV from x-axis
    double GetXAxisRotation();


    ////Return the angle of y-axis vector of the NBV from y-axis
    double GetYAxisRotation();


    ////Return the angle of z-axis vector of the NBV from z-axis
    double GetZAxisRotation();


    ////Return the x-axis vector of the NBV
    pcl::PointXYZ GetXAxis();


    ////Return the y-axis vector of the NBV
    pcl::PointXYZ GetYAxis();


    ////Return the z-axis vector of the NBV
    pcl::PointXYZ GetZAxis();
};
