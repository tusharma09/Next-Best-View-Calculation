#include "viewpoint.h"

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;




////Constructor; initialises the cloud pointers
ViewPoint::ViewPoint(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1)
{
    source_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
    scan_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
    global_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
    global_cloud_normal = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

    viewer = viewer1;
    cloudJustForDisplay = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());
}


////Set parameters for overall calculation of Next Best View
/// param in: first_filter_distance: voxel filter length for mesh
/// param in: second_filter_distance: voxel filter length to speed up further calculations of chains and viewpoint
/// param in: gp3_radius: length of edge in mesh
/// param in: join_boundaries_distance: join boundary sets if closer than this distance
/// param in: camera_distance: distance of camera from the surface, should be > 0.2 m
/// param in: overlap_coefficient: adjust overlap with the previous scan
/// param in: camera_path_min_angle: angle to adjust the orientation of the NBV with the overlapped surface
void ViewPoint::ViewPointCalculationSettings(double first_filter_distance, double second_filter_distance
                                             , double gp3_radius, double join_boundaries_distance, double camera_distance
                                             , double overlap_coefficient, double camera_path_min_angle)
{
    _first_filter_distance = first_filter_distance;
    _second_filter_distance = second_filter_distance;
    _camera_distance = camera_distance;
    _gp3_radius = gp3_radius;
    _join_boundaries_distance = join_boundaries_distance;
    _overlap_coefficient = overlap_coefficient;
    _camera_path_min_angle = camera_path_min_angle;

}


////Set parameters for finding boundary sets
/// param in: angle_tolerance: maximum angle deviation allowed in the boundary set
/// param in: penalty_tolerance: collective penalty on boundary set for edges with more than angle tolerance orientation;
///           in percentage of off edges in the boundary set between 0-1; if more than a limit boundary is restricted
void ViewPoint::SetParametersForDirectedBoundaries(float penalty_tolerance, float angle_tolerance ,int min_length_for_boundary_edge)
{
    _penalty_tolerance = penalty_tolerance;
    _angle_tolerance = angle_tolerance;
    _min_length_for_boundary_edge = min_length_for_boundary_edge;
}


////Set parameters to find chain points
/// param in: gc_smallRadius: points should be minimum this distance away
/// param in: gc_defaultBigRadius: points should be maximum this distance away
/// param in: gc_incBigRadius: increment to gc_defaultBigRadius if no point found
/// param in: gc_maxBigRadius: maximum value gc_defaultBigRadius can be incremented
void ViewPoint::SetParametersForChain(double gc_smallRadius, double gc_defaultBigRadius, double  gc_incBigRadius, double gc_maxBigRadius)
{
    _gc_smallRadius = gc_smallRadius;
    _gc_defaultBigRadius = gc_defaultBigRadius;
    _gc_incBigRadius = gc_incBigRadius;
    _gc_maxBigRadius = gc_maxBigRadius;
    _gc_bigRadius = gc_defaultBigRadius;
}


////Restrict the movement in the given direction
/// param in: angles_avoid: vector of directions(angles to the x-axis) to avoid
void ViewPoint::SetAnglesToAvoid(std::vector<uint> _angles_to_avoid)
{
    //    for(size_t i = 0; i< _angles_to_avoid.size(); i++ )
    //    {
    //        angles_to_avoid.push_back(_angles_to_avoid[i]);
    //    }
    angles_to_avoid = _angles_to_avoid;
}


////Restrict te movement in the given direction
/// param in: vector: direction(in the vector direction) to avoid
void ViewPoint::SetVectorDirectionToAvoid(pcl::PointXYZ vector)
{
    vector_downwards = vector;
    pcl::PointXYZ x_axis(1,0,0), y_axis(0,1,0), z_axis(0,0,1);

    double angle_with_x, angle_with_y, angle_with_z;
    angle_with_x = GetAngleBetweenVectors(vector, x_axis);
    vector.z = 0;// need angle in xy plane
    angle_with_y = GetAngleBetweenVectors(vector, y_axis);
    angle_with_z = GetAngleBetweenVectors(vector, z_axis);


    if(angle_with_z > 45)
    {
        if(angle_with_y > 90)
            //                        angle_with_x = 360 - angle_with_x;
            angles_to_avoid.push_back(360 - angle_with_x);
        else
            angles_to_avoid.push_back(angle_with_x);
    }
    std::cout<<"Angle to downwards: "<<angles_to_avoid[angles_to_avoid.size()-1]<<std::endl;
}


////Outlier removal and voxel filtering on source_cloud
/// param in: length: length of the edge of the cubic voxel
/// param in: MeanK:  minimum number of neighbors for outlier removal
/// output: Fills global_cloud after filtering
void ViewPoint::FilterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, double length = 0.001f, uint32_t MeanK = 500)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (source_cloud);
    sor.setMeanK (MeanK);
    sor.setStddevMulThresh (1.0);
    sor.filter (*global_cloud);
    pcl::VoxelGrid<pcl::PointXYZRGB> vgf;
    vgf.setInputCloud (global_cloud);
    vgf.setLeafSize (length, length, length);
    vgf.filter (*global_cloud);
}


////Calculate surface normals of cloud using pcl::NormalEstimation with 50 neighbors
/// param in: cloud: input cloud surface normals
/// param out: cloudWithNormals: cloud with normals
void ViewPoint::CalculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals)
{

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>);
    // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr);

    cloudWithNormals->resize(cloud->size());

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> ne;
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (50);
    ne.compute (*normals);

    for(unsigned int i=0; i<normals->size(); i++)
    {
        pcl::PointNormal pn = normals->at(i);
        cloudWithNormals->points[i].x = cloud->points[i].x;
        cloudWithNormals->points[i].y = cloud->points[i].y;
        cloudWithNormals->points[i].z = cloud->points[i].z;
        cloudWithNormals->points[i].r = cloud->points[i].r;
        cloudWithNormals->points[i].g = cloud->points[i].g;
        cloudWithNormals->points[i].b = cloud->points[i].b;
        cloudWithNormals->points[i].normal_x = pn.normal_x;
        cloudWithNormals->points[i].normal_y = pn.normal_y;
        cloudWithNormals->points[i].normal_z = pn.normal_z;
    }
}

////Generate polymesh from point cloud using pcl::GreedyProjectionTriangulation
/// param in: cloud_with_normals: point cloud with the surface normals
/// output: object of pcl::GreedyProjectionTriangulation with all the parameters and input cloud loaded
/// to generate mesh, need to do GreedyProjectionTriangulation.reconstruct (pcl::PolygonMesh triangles);
pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> ViewPoint::GreedyProjectionTriangulation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals)
{
    //Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (_gp3_radius);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);

    return gp3;
}


////Collect all the boundary edges from the polymesh
/// param in: gp3: pcl::GreedyProjectionTriangulation object, Use GreedyProjectionTriangulation() to setup parameters and input for mesh
/// output: "edges" variable; one row of _edge represent- point_p_id: vector of conected_pointids_to_p
void ViewPoint::GetBoundaryEdges(pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3)
{
    pcl::PolygonMesh triangles;
    gp3.reconstruct(triangles);

    global_triangles = triangles;

    std::vector<int> states = gp3.getPointStates();

    std::vector< pcl::Vertices> polygons = triangles.polygons;

    for(std::vector< pcl::Vertices>::iterator i = polygons.begin(); i != polygons.end(); i++)
    {
        std::vector< uint32_t> v = i->vertices;

        bool is_0_boundary = states[v[0]] == gp3.BOUNDARY;
        bool is_1_boundary = states[v[1]] == gp3.BOUNDARY;
        bool is_2_boundary = states[v[2]] == gp3.BOUNDARY;

        if(v[0] > global_cloud->points.size() || v[1] > global_cloud->points.size() || v[2] > global_cloud->points.size() )
            std::cout<<"Weird"<<std::endl;

        ////two points are boundary points and connected, take them as edge
        if(is_0_boundary)
        {
            if (is_1_boundary && std::find(edges[v[0]].begin(), edges[v[0]].end(),v[1]) == edges[v[0]].end())
                edges[v[0]].push_back(v[1]);
            if (is_2_boundary && std::find(edges[v[0]].begin(), edges[v[0]].end(),v[2]) == edges[v[0]].end())
                edges[v[0]].push_back(v[2]);
        }
        if(is_1_boundary)
        {
            if (is_0_boundary && std::find(edges[v[1]].begin(), edges[v[1]].end(),v[0]) == edges[v[1]].end())
                edges[v[1]].push_back(v[0]);
            if (is_2_boundary && std::find(edges[v[1]].begin(), edges[v[1]].end(),v[2]) == edges[v[1]].end())
                edges[v[1]].push_back(v[2]);
        }
        if(is_2_boundary)
        {
            if (is_0_boundary && std::find(edges[v[2]].begin(), edges[v[2]].end(),v[0]) == edges[v[2]].end())
                edges[v[2]].push_back(v[0]);
            if (is_1_boundary && std::find(edges[v[2]].begin(), edges[v[2]].end(),v[1]) == edges[v[2]].end())
                edges[v[2]].push_back(v[1]);
        }

        //        std::cout<<std::distance( polygons.begin(), i ) << "  :  ";
        //        for(std::vector< uint32_t>::iterator j = v.begin(); j != v.end(); j++)
        //            std::cout<< *j << "     ";
        //        std::cout<<std::endl;
    }
}


////Angle between vectors AB and BC if point D is origin; Between AB and CD when D is not origin
/// param in: A,B,C,D: 3D points
/// param in: inXYplane: true-calculate angle in XY plane, camera point of view; false: calculate angle in 3d systems, all axes considered
/// output: angle in degrees
float ViewPoint::AngleBetweenPoints(pcl::PointXYZRGB A, pcl::PointXYZRGB B, pcl::PointXYZRGB C, pcl::PointXYZRGB D = pcl::PointXYZRGB(), bool inXYplane=true)
{
    float angle, res, v1mag, v2mag;
    //PointXYZ is just used as struct of xyz
    pcl::PointXYZ v1norm, v2norm, v1, v2;

    v1 = {B.x - A.x, B.y - A.y, B.z - A.z};

    if(D.x == 0 && D.y == 0 && D.z == 0)
        v2 = {C.x - B.x, C.y - B.y, C.z - B.z};
    else
        v2 = {D.x - C.x, D.y - C.y, D.z - C.z};

    if(inXYplane)
        v1.z = v2.z = 0;

    v1mag = sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
    v1norm = {v1.x / v1mag, v1.y / v1mag, v1.z / v1mag};

    v2mag = sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
    v2norm = {v2.x / v2mag, v2.y / v2mag, v2.z / v2mag};

    res = v1norm.x * v2norm.x + v1norm.y * v2norm.y + v1norm.z * v2norm.z;

    res = roundf( res * 1000.0 ) / 1000.0;

    angle = acos(res)*(180/M_PI);

    return angle;
}



/// Recurrsive function to collect boundaries in boundary set started from FindDirectedBoundries(); call itself with next edge points
/// param in: point0: starting of the edge
/// param in: point1: end of the edge
/// param in: angle_sum: collective angle deviation in the boundary set;carries sum of angles from previous edges, if more than tolerance then boundry is restricted
/// param in: penalty: collective penalty on boundary set for edges with more than angle tolerance orientation;
///                     ;in percentage of off edges in the boundary set between 0-1; if more than a limit boundary is restricted
/// param in: point2: end of the edge in case point1 was invalid in last iteration
/// param in: is_around_0: if the orientation is around 0°; 0-not set, 1-yes, 2-no
/// output: Valid boundary set found
bool ViewPoint::FindDirectedBoundries(uint32_t point0, uint32_t point1, float angle_sum, float penalty, uint32_t point2 = 0, uint8_t is_around_0 = 0)
{
    int current_point = point2 == 0 ? point1 : point2;

    for(u_int32_t j = 0; j != edges[current_point].size(); j++)
    {
        uint32_t next_point = edges[current_point][j];

        if(next_point == point0 || next_point == point1 || next_point == point2)
            continue;

        bool vertex_already_saved = false;
        //check if next_point is already saved, join if saved
        for(u_int32_t k = 0; !vertex_already_saved && k != directed_boundary_edges.size(); k++)
        {
            if(std::find(directed_boundary_edges[k].begin(), directed_boundary_edges[k].end(), next_point) != directed_boundary_edges[k].end())//&& next_point != (directed_boundary_edges[k]).back()
            {
                vertex_already_saved =true;

                uint32_t p0,p1,p2,p3;
                p0=directed_boundary_edges[k][0];
                p1=directed_boundary_edges[k][directed_boundary_edges[k].size()-1];
                p2=directed_boundary_edge[0];
                p3=directed_boundary_edge[directed_boundary_edge.size()-1];
                float angle= AngleBetweenPoints(global_cloud->points[p0],global_cloud->points[p1],global_cloud->points[p2],global_cloud->points[p3]);
                if(angle < _angle_tolerance /*|| angle > 180-_angle_tolerance*/)
                {
                    directed_boundary_edges[k].insert(directed_boundary_edges[k].end(), directed_boundary_edge.begin(), directed_boundary_edge.end());
                    directed_boundary_edge.clear();
                }
                return false;
            }
        }


        if(std::find(directed_boundary_edge.begin(), directed_boundary_edge.end(), next_point) == directed_boundary_edge.end())
        {
            float angle;

            if(point2 == 0)//current point is point1
                angle = AngleBetweenPoints(global_cloud->points[point0], global_cloud->points[current_point], global_cloud->points[next_point]);
            else//because of penalty point1  is not current_point but point2. edge(12) is skipped angle between (01) AND (cn) is calculated
                angle = AngleBetweenPoints(global_cloud->points[point0], global_cloud->points[point1], global_cloud->points[current_point], global_cloud->points[next_point]);

            if(is_around_0 == 0)
                is_around_0 = angle < 90 ? 1 : 2;

            if(_angle_tolerance < angle && angle < (180-_angle_tolerance))
                return false;

            if((_angle_tolerance*0.66) < angle && angle < (180-(_angle_tolerance*0.66)))
            {
                penalty++;

                if((double)penalty/directed_boundary_edge.size() > _penalty_tolerance )
                {
                    if(directed_boundary_edge.size() > _min_length_for_boundary_edge)
                        directed_boundary_edges.push_back(directed_boundary_edge);
                    return directed_boundary_edge.size() > _min_length_for_boundary_edge;
                }

                directed_boundary_edge.push_back(next_point);

                int length_of_boundary = directed_boundary_edge.size();

                bool is_success = FindDirectedBoundries(point0, current_point, angle_sum, penalty, next_point, is_around_0);
                if(!is_success)
                    directed_boundary_edge.erase(directed_boundary_edge.begin()+length_of_boundary, directed_boundary_edge.end());
                else
                    break;

            }
            else
            {
                if(is_around_0 == 1 && angle > 90)
                    angle_sum += -(180-angle);
                else if(is_around_0 == 2 && angle < 90)
                    angle_sum += -angle;
                else
                    angle_sum += (angle < 90 ? angle : 180-angle);


                if(fabs(angle_sum) > _angle_tolerance)
                {
                    if(directed_boundary_edge.size() > _min_length_for_boundary_edge)
                        directed_boundary_edges.push_back(directed_boundary_edge);
                    return directed_boundary_edge.size() > _min_length_for_boundary_edge;
                }

                int length_of_boundary = directed_boundary_edge.size();
                directed_boundary_edge.push_back(next_point);

                bool is_success = FindDirectedBoundries(current_point, next_point, angle_sum, penalty, 0 /*point2 as 0*/, is_around_0);
                if(!is_success)
                    directed_boundary_edge.erase(directed_boundary_edge.begin()+length_of_boundary, directed_boundary_edge.end());
                else
                    break;

            }
        }
    }

    return directed_boundary_edge.size() > _min_length_for_boundary_edge;
}


////Divide boundary edges in edges into different boundary sets; each boundary set has all the similar oriented boundary edges
/// output:Fills "directed_boundary_edges" with the boundary sets
void ViewPoint::FindDirectedBoundries()
{
    for(u_int32_t i = 0; i != edges.size(); i++)
    {
        if(edges[i].size() == 0)
            continue;

        for(u_int32_t j = 0; j != edges[i].size(); j++)
        {
            bool vertex_already_saved = false;
            //if i or j is already saved, skipped
            for(u_int32_t k = 0; !vertex_already_saved && k != directed_boundary_edges.size(); k++)
            {
                if(std::find(directed_boundary_edges[k].begin(), directed_boundary_edges[k].end(), i) != directed_boundary_edges[k].end()
                        || std::find(directed_boundary_edges[k].begin(), directed_boundary_edges[k].end(), edges[i][j]) != directed_boundary_edges[k].end() )
                    vertex_already_saved =true;
            }

            if(!vertex_already_saved )
            {
                directed_boundary_edge.push_back(i);
                directed_boundary_edge.push_back(edges[i][j]);

                if(edges[i][j] > global_cloud->points.size())
                    std::cout<<"Weird.. This shall never happen!"<<std::endl;

                FindDirectedBoundries(i, edges[i][j], 0, 0);
                directed_boundary_edge.clear();
            }
        }
    }
}


////Find direction as the angle of x-axis with the perpendicular outwards the boundary sets from directed_boundary_edges
/// output: "boundaries" is filled with the boundary set points and boundary set direction in degrees
void ViewPoint::FindDirectionOfBoundaries()
{
    for(Vector_VectorOfInt::iterator it = directed_boundary_edges.begin(); it != directed_boundary_edges.end(); it++)
    {
        std::vector<uint32_t> edges (*it);

        Unidirected_Boundary boundary;
        boundary.points.insert(boundary.points.end(), edges.begin(), edges.end());

        float dx1,dy1, dx2, dy2;

        // get rough slope
        dx1 = global_cloud->points[edges[0]].x - global_cloud->points[edges[edges.size()-1]].x;
        dx1 = dx1 == 0 ? 0.0000001 : dx1;
        dy1 = global_cloud->points[edges[0]].y - global_cloud->points[edges[edges.size()-1]].y;


        for(uint32_t i = 0; i < boundary.points.size(); i++)
        {
            boundary.points_coordinates.push_back(global_cloud->points[boundary.points[i]]);
        }

        if(fabs(dy1/dx1) < 1.5) // Must be 1.0 for 45° but gave results for 1.5- 45°.. check once again
            sort( boundary.points_coordinates.begin(), boundary.points_coordinates.end(), [](pcl::PointXYZRGB i, pcl::PointXYZRGB j) { return (i.x < j.x); } );
        else
            sort( boundary.points_coordinates.begin(), boundary.points_coordinates.end(), [](pcl::PointXYZRGB i, pcl::PointXYZRGB j) { return (i.y < j.y); } );

        // get exact slope
        dx1 = boundary.points_coordinates[boundary.points_coordinates.size()-1].x - boundary.points_coordinates[0].x;
        dx1 = dx1 == 0 ? 0.0000001 : dx1;
        dy1 = boundary.points_coordinates[boundary.points_coordinates.size()-1].y - boundary.points_coordinates[0].y;
        pcl::PointXYZRGB midPoint = boundary.points_coordinates[(boundary.points_coordinates.size()-1)/2];

        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud (global_cloud);

        int K = 20;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (! kdtree.nearestKSearch (midPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            std::cout<<midPoint<<std::endl;
            std::cerr<<"no neighbor found"<<std::endl;
        }
        else
        {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); i++)
            {
                pcl::PointXYZRGB p = global_cloud->points[ pointIdxNKNSearch[i] ];
                if(p.x == midPoint.x && p.y == midPoint.y && p.z==midPoint.z)
                {
                    pointIdxNKNSearch.erase(pointIdxNKNSearch.begin() + i);
                    pointNKNSquaredDistance.erase(pointNKNSquaredDistance.begin() + i);
                    break;
                }
            }

            std::vector<float> slope_difference;
            for (size_t i = 0; i < pointIdxNKNSearch.size (); i++)
            {
                dx2 =  global_cloud->points[ pointIdxNKNSearch[i] ].x - midPoint.x ;
                dy2 = global_cloud->points[ pointIdxNKNSearch[i] ].y -midPoint.y;
                dy2 = dy2 == 0 ? 0.0000001 : dy2;

                slope_difference.push_back(fabs((dy1/dx1) + (dx2/dy2)));// slope of perpendicular line to s = -1/s
            }

            int min_index = std::min_element(slope_difference.begin(), slope_difference.end()) - slope_difference.begin();

            //            for(int i=0;i < slope_difference.size();i++)
            //                std::cout<<slope_difference[i]<<std::endl;
            pcl::PointXYZRGB pointOnXAxis(0,0,0), pointOnYAxis(0,0,0);
            pointOnXAxis.x=1;
            pointOnYAxis.y=1;


            float angleWithX, angleWithY;
            angleWithX = AngleBetweenPoints(global_cloud->points[ pointIdxNKNSearch[min_index] ], midPoint, pcl::PointXYZRGB(0,0,0), pointOnXAxis);
            angleWithY = AngleBetweenPoints(global_cloud->points[ pointIdxNKNSearch[min_index] ], midPoint, pcl::PointXYZRGB(0,0,0), pointOnYAxis);

            //            std::cout<<angleWithY<< "  "<<angleWithX<<std::endl;
            if(angleWithY > 90)
                angleWithX = 360 - angleWithX;

            boundary.direction = angleWithX;
            boundaries.push_back(boundary);
        }
    }
}


////Find the boundary set with "maximum points from boundaries"
/// output: Keeps only boundary sets with similar direction
/// output: next_pose_direction gets direction for the next best view calculation
void ViewPoint::FindSetOfDirectedBoundariesWithMaxPoints()
{
    /////similar direction poll
    std::vector<uint32_t> direction_similar(boundaries.size());
    for(uint32_t i =0; i< boundaries.size(); i++)
    {
        for(uint32_t j = 0; j< boundaries.size(); j++)
        {
            float angle = 45;
            bool diff_lessthan_angle = (fabs(boundaries[j].direction - boundaries[i].direction) < angle);
            bool special_case_near_360 =(boundaries[j].direction < angle && boundaries[i].direction > (360-angle))
                    || (boundaries[j].direction > (360-angle) && boundaries[i].direction < angle );
            bool diff_lessthan_angle_special_case = ((360 - fabs(boundaries[i].direction - boundaries[j].direction)) < angle);

            if(!(!diff_lessthan_angle && !(special_case_near_360 && diff_lessthan_angle_special_case) ))
                direction_similar[i]+=boundaries[j].points.size();
        }
    }


    ////keep only the similar directed boundaries with max number points
    uint32_t max_index = std::max_element(direction_similar.begin(), direction_similar.end()) - direction_similar.begin();
    direction = boundaries[max_index].direction;
    for(uint32_t i = 0; i < boundaries.size(); i++)
    {
        float angle = 45;
        bool diff_lessthan_angle = (fabs(boundaries[max_index].direction - boundaries[i].direction) < angle);
        bool special_case_near_360 =(boundaries[max_index].direction < angle && boundaries[i].direction > (360-angle))
                || (boundaries[max_index].direction > (360-angle) && boundaries[i].direction < angle );
        bool diff_lessthan_angle_special_case = ((360 - fabs(boundaries[i].direction - boundaries[max_index].direction)) < angle);

        if(!diff_lessthan_angle && !(special_case_near_360 && diff_lessthan_angle_special_case) )
        {
            boundaries[i].points.clear();
        }
    }


    /////delete boundaries with 0 points left
    for(int32_t i = boundaries.size()-1; i >= 0; i--)
    {
        if(boundaries[i].points.size() == 0)
        {
            boundaries.erase(boundaries.begin() + i);
        }
    }

    for(uint32_t i =0; i < boundaries.size(); i++)
        boundaries[i].direction = direction;

    next_pose_direction = direction;
}


////Join boundary sets from "boundaries" which are closer than distance "_join_boundaries_distance"
/// output: modified "boundaries"
void ViewPoint::JoinCloseBoundaries()
{
    for(uint32_t i = 0; i < boundaries.size(); i++)
    {
        for(uint32_t j = 0; j < boundaries.size(); j++)
        {
            if(i==j || boundaries[j].points.size() == 0 ) continue;
            double distance = pcl::geometry::distance(boundaries[i].points_coordinates[0], boundaries[j].points_coordinates[boundaries[j].points_coordinates.size()-1]);

            if (distance < _join_boundaries_distance)
            {
                boundaries[j].points.insert(boundaries[j].points.end(), boundaries[i].points.begin(), boundaries[i].points.end());
                boundaries[j].points_coordinates.insert(boundaries[j].points_coordinates.end(), boundaries[i].points_coordinates.begin(), boundaries[i].points_coordinates.end());
                boundaries[j].points_chain_for_last_point.clear();
                boundaries[j].points_chain_for_last_point.insert(boundaries[j].points_chain_for_last_point.end(), boundaries[i].points_chain_for_last_point.begin(), boundaries[i].points_chain_for_last_point.end());
                boundaries[j].axlast =  boundaries[i].axlast;
                boundaries[j].aylast =  boundaries[i].aylast;
                boundaries[j].azlast =  boundaries[i].azlast;

                boundaries[i].points.clear();
                break;
            }
        }
    }

    /////delete boundaries with 0 points left
    for(int32_t i = boundaries.size()-1; i >= 0; i--)
    {
        if(boundaries[i].points.size() == 0)
        {
            boundaries.erase(boundaries.begin() + i);
        }
    }
}


////Least square curve fitting for data in y against parameters x
////Source: http://www.bragitoff.com/2015/09/c-program-for-polynomial-fit-least-squares/
/// param in: y: array of data to fit
/// param in: x: parameters for data in y in same order
/// param in: N: number of data points
/// output: pointer to coefficients of the polynomials after curve fitting
double* ViewPoint::PolynomialFit(double y[], double x[], int N = 10)
{
    static double a[3];
    // n is the degree of Polynomial
    int i,j,k, n=2;
    double X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i=0;i<2*n+1;i++)
    {
        X[i]=0;
        for (j=0;j<N;j++)
            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    double B[n+1][n+2];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i=0;i<=n;i++)
        for (j=0;j<=n;j++)
            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    for (i=0;i<n+1;i++)
    {
        Y[i]=0;
        for (j=0;j<N;j++)
            Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i=0;i<=n;i++)
        B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations


    for (i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<n;k++)
            if (B[i][i]<B[k][i])
                for (j=0;j<=n;j++)
                {
                    double temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }

    for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
        for (k=i+1;k<n;k++)
        {
            double t=B[k][i]/B[i][i];
            for (j=0;j<=n;j++)
                B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
        }
    for (i=n-1;i>=0;i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<n;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i]=a[i]-B[i][j]*a[j];
        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }

    return a;
}


////Gives the output point from the polynomial coefficients(surface estimation coefficients) for the given paramter
/// param in: ax: ax[0], ax[1] and ax[2] coefficients for x values
/// param in: ay: ay[0], ay[1] and ay[2] coefficients for y values
/// param in: az: az[0], az[1] and az[2] coefficients for z values
/// param in: parameter: parameter value for which points needs to be calculated
/// output: 3d point for the given parameter and surface estimation coefficients
pcl::PointXYZRGB GetPolynomialValue(std::vector<double> ax, std::vector<double> ay, std::vector<double> az, double parameter)
{
    pcl::PointXYZRGB p;
    p.x = ax[0] + ax[1]*parameter + ax[2]*std::pow(parameter,2) /*+ ax[3]*std::pow(parameter,3)*/;
    p.y = ay[0] + ay[1]*parameter + ay[2]*std::pow(parameter,2) /*+ ay[3]*std::pow(parameter,3)*/;
    p.z = az[0] + az[1]*parameter + az[2]*std::pow(parameter,2) /*+ az[3]*std::pow(parameter,3)*/;
    return p;
}


////Get the chain of the points for the first point sent in the chain in the given direction
/// param in: chain: vector of points; must contain point to start the chain of points
/// param in: chain_normal: vector of points; must contain point with normal to start the chain of points with normals
/// param in: number_of_points: number of points to find for the chain of points
/// param in: direction:  direction in which points of the chain needs to be find; in degrees
void ViewPoint::GetChain(std::vector<pcl::PointXYZRGB>& chain, std::vector<pcl::PointXYZRGBNormal>& chain_normal, uint32_t number_of_points, bool is_with_normal, uint32_t direction)
{
    bool chain_full = false;
    uint32_t angle_tol = 20;

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    std::vector<int> pointIds;
    std::vector<int> pointIdxSmallRadiusSearch, pointIdxBigRadiusSearch;
    std::vector<float> pointSmallRadiusSquaredDistance, pointBigRadiusSquaredDistance;
    kdtree.setInputCloud (global_cloud);

    while(!chain_full)
    {
        pcl::PointXYZRGB point;
        if(is_with_normal)
        {
            point.x = chain_normal[chain_normal.size()-1].x;
            point.y = chain_normal[chain_normal.size()-1].y;
            point.z = chain_normal[chain_normal.size()-1].z;
        }
        else
        {
            point.x = chain[chain.size()-1].x;
            point.y = chain[chain.size()-1].y;
            point.z = chain[chain.size()-1].z;
        }

        pointIds.clear();
        pointIdxSmallRadiusSearch.clear(); pointIdxBigRadiusSearch.clear();
        pointSmallRadiusSquaredDistance.clear(); pointBigRadiusSquaredDistance.clear();

        if (! kdtree.radiusSearch (point, _gc_smallRadius, pointIdxSmallRadiusSearch, pointSmallRadiusSquaredDistance) > 0 )
        {
            std::cerr<<"No neighbor found";
            return;
        }
        else
        {
            if (! kdtree.radiusSearch (point, _gc_bigRadius, pointIdxBigRadiusSearch, pointBigRadiusSquaredDistance) > 0 )
            {
                std::cerr<<"No neighbor found";
                return;
            }
            else
            {
                //pointIds.resize(pointIdxBigRadiusSearch.size() - pointIdxSmallRadiusSearch.size());
                for(uint32_t i=0; i<pointIdxBigRadiusSearch.size();i++)
                {
                    if(std::find(pointIdxSmallRadiusSearch.begin(), pointIdxSmallRadiusSearch.end(), pointIdxBigRadiusSearch[i]) !=  pointIdxSmallRadiusSearch.end())
                        continue;
                    else
                        pointIds.push_back(pointIdxBigRadiusSearch[i]);
                }
            }
        }


        if (pointIds.size() > 0)
        {
            std::vector<double> angles_difference;
            ////if exists do not consider
            for(uint32_t i=0; i<pointIds.size();i++)
            {
                pcl::PointXYZRGB pointOnXAxis(0,0,0), pointOnYAxis(0,0,0);
                pointOnXAxis.x=1;
                pointOnYAxis.y=1;

                float angleWithX, angleWithY;

                angleWithX = AngleBetweenPoints(global_cloud->points[ pointIds[i] ], point, pcl::PointXYZRGB(0,0,0), pointOnXAxis);
                angleWithY = AngleBetweenPoints(global_cloud->points[ pointIds[i] ], point, pcl::PointXYZRGB(0,0,0), pointOnYAxis);


                if(angleWithY > 90)
                    angleWithX = 360 - angleWithX;

                if(angleWithX > (360-angle_tol) && direction < angle_tol)
                {
                    angles_difference.push_back(fabs(angleWithX - direction+360));
                }
                else if (angleWithX < angle_tol && direction > (360-angle_tol))
                {
                    angles_difference.push_back(fabs((angleWithX+360) - direction));
                }
                else
                {
                    angles_difference.push_back(fabs(angleWithX - direction));
                }
            }

            int min_index = std::min_element(angles_difference.begin(), angles_difference.end()) - angles_difference.begin();

            if(angles_difference[min_index] < angle_tol)
            {
                if(is_with_normal)
                {
                    pcl::PointXYZRGBNormal p = global_cloud_normal->points[pointIds[min_index]];
                    chain_normal.push_back(p);
                }
                else
                {
                    pcl::PointXYZRGB p = global_cloud->points[pointIds[min_index]];
                    chain.push_back(p);
                }

                _gc_bigRadius = _gc_defaultBigRadius;
                if(chain.size() == number_of_points || chain_normal.size() == number_of_points)
                    chain_full=true;
            }
            else
            {
                _gc_bigRadius += _gc_incBigRadius;

                if(_gc_bigRadius > _gc_maxBigRadius)
                {
                    chain_full = true;
                }

                else if(_gc_bigRadius == _gc_maxBigRadius)
                    angle_tol *= 2;
            }
        }//if ends
        else
        {
            _gc_bigRadius += _gc_incBigRadius;

            if(_gc_bigRadius > _gc_maxBigRadius)
            {
                chain_full = true;
            }
        }
    }//while ends
}


////Estimate the surface for the boundary set in "boundaries" as the coeffcients for polynomials
/// output: update coefficients in "boundaries" for both the ends of the boundary set
void ViewPoint::EstimateSurface()
{
    for(std::vector<Unidirected_Boundary>::iterator it= boundaries.begin(); it != boundaries.end();it++)
    {
        if(display)
            for(int i=0; i < (*it).points_coordinates.size(); i++)
            {
                //                std::cout<< (*it).points_coordinates[j].x << "    " <<(*it).points_coordinates[j].y << std::endl;
                viewer->addSphere((*it).points_coordinates[i], 0.0005, "point with coordinates"  + std::to_string(i)+ std::to_string(it-boundaries.begin()));
                //                if(i==0 || i == ((*it).points_coordinates.size()-1) )
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "point with coordinates"  + std::to_string(i)+ std::to_string(it-boundaries.begin()));
            }

        pcl::PointXYZRGB firstPoint((*it).points_coordinates[2])///2 from first .. changed later
                ,lastPoint((*it).points_coordinates[(*it).points_coordinates.size()-3])///last but 2
                ;

        ////put first and last point in their chains
        (*it).points_chain_for_first_point.push_back(firstPoint);
        (*it).points_chain_for_last_point.push_back(lastPoint);

        const uint8_t number_of_points = 20;
        const uint8_t min_point_required = 5;
        ///find chain of points for first point
        {
            std::vector<pcl::PointXYZRGBNormal> empty_chain;
            for( uint32_t i =0; i < ((*it).points_coordinates.size()-1) && (*it).points_chain_for_first_point.size() < min_point_required+1; i++)
            {
                (*it).points_chain_for_first_point.clear();
                (*it).points_chain_for_first_point.push_back((*it).points_coordinates[i]);
                GetChain((*it).points_chain_for_first_point, empty_chain, number_of_points, false, (*it).direction);
            }
            for( uint32_t i =(*it).points_coordinates.size()-1; i >0 && (*it).points_chain_for_last_point.size() < min_point_required+1; i--)
            {
                (*it).points_chain_for_last_point.clear();
                (*it).points_chain_for_last_point.push_back((*it).points_coordinates[i]);
                GetChain((*it).points_chain_for_last_point, empty_chain, number_of_points, false, (*it).direction);
            }
        }
        if(display)
        {
            for (int i =0;i < (*it).points_chain_for_first_point.size(); i++)
            {
                viewer->addSphere((*it).points_chain_for_first_point[i], 0.0005, "firstchain_" + std::to_string(i)+ std::to_string(it-boundaries.begin()));
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5,"firstchain_" + std::to_string(i)+ std::to_string(it-boundaries.begin()));
            }

            for (int i =0;i < (*it).points_chain_for_last_point.size(); i++)
            {
                viewer->addSphere((*it).points_chain_for_last_point[i], 0.0005, "lastchain_" + std::to_string(i)+ std::to_string(it-boundaries.begin()));
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5,"lastchain_" + std::to_string(i)+ std::to_string(it-boundaries.begin()));
            }
        }

        /////////////////////////////////////////////////////////////////////
        ///  PolynomialFit first point
        if((*it).points_chain_for_first_point.size() > min_point_required)
        {
            uint32_t size = (*it).points_chain_for_first_point.size();

            double x[size];
            double y[size];
            double z[size];
            double t[size];
            double* a = new double(3);

            for (int i =0; i < size; i++)
            {
                pcl::PointXYZRGB p = (*it).points_chain_for_first_point[i];

                x[size-1-i] = p.x;
                y[size-1-i] = p.y;
                z[size-1-i] = p.z;
                t[i] = (double)i/(size-1);
            }

            a= PolynomialFit(x,t, size);
            (*it).axfirst.push_back(a[0]); (*it).axfirst.push_back(a[1]); (*it).axfirst.push_back(a[2]);

            a= PolynomialFit(y,t, size);
            (*it).ayfirst.push_back(a[0]); (*it).ayfirst.push_back(a[1]); (*it).ayfirst.push_back(a[2]);

            a= PolynomialFit(z,t, size);
            (*it).azfirst.push_back(a[0]); (*it).azfirst.push_back(a[1]); (*it).azfirst.push_back(a[2]);


            if(display && extendSurface)
            {
                for(uint32_t i =size; i < size*2; i++)
                {
                    double j = (double)i/(double)(size-1);
                    pcl::PointXYZRGB p = GetPolynomialValue((*it).axfirst, (*it).ayfirst, (*it).azfirst, j);
                    viewer->addSphere(p, 0.0005, "predicted first" + std::to_string(i) + std::to_string(it-boundaries.begin()));
                }
            }
        }


        /////////////////////////////////////////////////////////////////////
        ///  PolynomialFit last point
        if((*it).points_chain_for_last_point.size() > min_point_required)
        {
            uint32_t size = (*it).points_chain_for_last_point.size();
            double x[size];
            double y[size];
            double z[size];
            double t[size];
            double* a = new double(3);

            for (int i =0; i < size; i++)
            {
                pcl::PointXYZRGB p = (*it).points_chain_for_last_point[i];

                x[size-1-i] = p.x;
                y[size-1-i] = p.y;
                z[size-1-i] = p.z;
                t[i] = (double)i/(size-1);
            }

            a= PolynomialFit(x,t,size);
            (*it).axlast.push_back(a[0]); (*it).axlast.push_back(a[1]); (*it).axlast.push_back(a[2]);

            a= PolynomialFit(y,t,size);
            (*it).aylast.push_back(a[0]); (*it).aylast.push_back(a[1]); (*it).aylast.push_back(a[2]);

            a= PolynomialFit(z,t,size);
            (*it).azlast.push_back(a[0]); (*it).azlast.push_back(a[1]); (*it).azlast.push_back(a[2]);

            if(display  && extendSurface)
            {
                for(uint32_t i =size; i < size*2; i++)
                {
                    double j = (double)i/(double)(size-1);
                    pcl::PointXYZRGB p = GetPolynomialValue((*it).axlast, (*it).aylast, (*it).azlast, j);
                    viewer->addSphere(p, 0.0005, "predicted last" + std::to_string(i) + std::to_string(it-boundaries.begin()));
                }

            }
        }
    }//for
}


////Find the next best viewpoint for the boundary set in "boundaries"
/// param in: cloud_with_normals: cloud to be used to find chain of points
/// output: x-axis, y-axis, z-axis and camera_point updated for the next best view
void ViewPoint::FindViewPoint(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals)
{
    pcl::PointXYZRGB highest_boundary_point(0,0,0), lowest_boundary_point(0,0,0);
    std::vector<double> axH, ayH,azH, axL,ayL,azL;
    std::vector<pcl::PointXYZRGB> b_points;
    std::vector< std::vector< double> > coeffx, coeffy, coeffz;
    bool isSharpEnd=false;

    /////separate out points and coeff as there may be empty chains
    for(std::vector<Unidirected_Boundary>::iterator it= boundaries.begin(); it != boundaries.end();it++)
    {
        if((*it).axlast.size() > 0)
        {
            b_points.push_back((*it).points_coordinates[(*it).points_coordinates.size() - 1]);
            coeffx.push_back((*it).axlast);  coeffy.push_back((*it).aylast);   coeffz.push_back((*it).azlast);
        }

        if((*it).axfirst.size() > 0)
        {
            b_points.push_back((*it).points_coordinates[0]);
            coeffx.push_back((*it).axfirst);   coeffy.push_back((*it).ayfirst);   coeffz.push_back((*it).azfirst);
        }
    }

    ////find out highest and lowest boundary
    for(uint32_t i= 0; i< b_points.size();i++)
    {
        if((direction > 45 && direction < 135) || (direction > 225 && direction < 315))
        {////find according to x
            if((highest_boundary_point.x == 0 && highest_boundary_point.y == 0 && highest_boundary_point.z == 0)
                    || (highest_boundary_point.x < b_points[i].x))
            {
                highest_boundary_point = b_points[i];
                axH.clear(); axH.insert(axH.begin(),coeffx[i].begin(), coeffx[i].end());
                ayH.clear(); ayH.insert(ayH.begin(),coeffy[i].begin(), coeffy[i].end());
                azH.clear(); azH.insert(azH.begin(),coeffz[i].begin(), coeffz[i].end());
            }
            if((lowest_boundary_point.x == 0 && lowest_boundary_point.y == 0 && lowest_boundary_point.z == 0)
                    || (lowest_boundary_point.x > b_points[i].x))
            {
                lowest_boundary_point = b_points[i];
                axL.clear(); axL.insert(axL.begin(),coeffx[i].begin(), coeffx[i].end());
                ayL.clear(); ayL.insert(ayL.begin(),coeffy[i].begin(), coeffy[i].end());
                azL.clear(); azL.insert(azL.begin(),coeffz[i].begin(), coeffz[i].end());
            }
        }
        else
        {////find according to y
            if((highest_boundary_point.x == 0 && highest_boundary_point.y == 0 && highest_boundary_point.z == 0)
                    || (highest_boundary_point.y < b_points[i].y))
            {
                highest_boundary_point = b_points[i];
                axH.clear(); axH.insert(axH.begin(),coeffx[i].begin(), coeffx[i].end());
                ayH.clear(); ayH.insert(ayH.begin(),coeffy[i].begin(), coeffy[i].end());
                azH.clear(); azH.insert(azH.begin(),coeffz[i].begin(), coeffz[i].end());
            }
            if((lowest_boundary_point.x == 0 && lowest_boundary_point.y == 0 && lowest_boundary_point.z == 0)
                    || (lowest_boundary_point.y > b_points[i].y))
            {
                lowest_boundary_point = b_points[i];
                axL.clear(); axL.insert(axL.begin(),coeffx[i].begin(), coeffx[i].end());
                ayL.clear(); ayL.insert(ayL.begin(),coeffy[i].begin(), coeffy[i].end());
                azL.clear(); azL.insert(azL.begin(),coeffz[i].begin(), coeffz[i].end());
            }
        }
    }

    //////////estimate surface and normal to that
    /////find out four corner points
    pcl::PointXYZRGB corner1(0,0,0), corner2(0,0,0), corner3(0,0,0), corner4(0,0,0);
    double l = 0.5;
    if(1)
    {
        corner1 = GetPolynomialValue(axH, ayH, azH, l);

        corner2 = GetPolynomialValue(axL, ayL, azL, l);

        l = 2;

        corner3 = GetPolynomialValue(axH, ayH, azH, l);

        corner4 = GetPolynomialValue(axL, ayL, azL, l);
    }

    if(show_corners)
    {
        viewer->addSphere(corner1, 0.001, "corner1");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "corner1");
        viewer->addSphere(corner2, 0.001, "corner2");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1, "corner2");
        viewer->addSphere(corner3, 0.001, "corner3");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,  1, 1, 1, "corner3");
        viewer->addSphere(corner4, 0.001, "corner4");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "corner4");
    }


    ////camera path as ax, ay, az
    l = _overlap_coefficient;
    pcl::PointXYZRGB corner5(0,0,0), corner6(0,0,0),corner7(0,0,0), corner8(0,0,0);

    corner5 = GetPolynomialValue(axH, ayH, azH, l);

    corner6 = GetPolynomialValue(axL, ayL, azL, l);

    l = 1;

    corner7 = GetPolynomialValue(axH, ayH, azH, l);

    corner8 = GetPolynomialValue(axL, ayL, azL, l);


    /////isSharpEnd
    {
        float angle_at_top_corner, angle_at_bottom_corner;
        angle_at_top_corner = AngleBetweenPoints(pcl::PointXYZRGB(0,0,0), corner7, corner1, corner7, false);
        angle_at_bottom_corner = AngleBetweenPoints(pcl::PointXYZRGB(0,0,0), corner8, corner2, corner8, false);

        isSharpEnd = (angle_at_top_corner > 75 || angle_at_bottom_corner > 75) ? true : false;

        std::cout<< angle_at_top_corner << "   " << angle_at_bottom_corner <<"   " << (isSharpEnd ? "sharp end.. rotate camera": "no sharp end.. it is smooth") <<std::endl;
    }


    ////cross product of four points-normal
    ////PointXYZ is just used as struct of xyz
    pcl::PointXYZ v1, v2, normal_to_estimated_surface;
    ///normal_to_estimated_surface-normal to four corners
    {
        v1 = {corner4.x - corner1.x, corner4.y - corner1.y, corner4.z-corner1.z};
        v2 = {corner3.x - corner2.x, corner3.y - corner2.y, corner3.z-corner2.z};

        normal_to_estimated_surface.x = v1.y*v2.z - v1.z*v2.y;
        normal_to_estimated_surface.y = v1.z*v2.x - v1.x*v2.z;
        normal_to_estimated_surface.z = v1.x*v2.y - v1.y*v2.x;

        if(normal_to_estimated_surface.z > 0)
        {
            normal_to_estimated_surface.x = -normal_to_estimated_surface.x;
            normal_to_estimated_surface.y = -normal_to_estimated_surface.y;
            normal_to_estimated_surface.z = -normal_to_estimated_surface.z;
        }

        double mag = sqrt(pow(normal_to_estimated_surface.x,2) + pow(normal_to_estimated_surface.y,2)+ pow(normal_to_estimated_surface.z,2));
        normal_to_estimated_surface.x = normal_to_estimated_surface.x/mag;
        normal_to_estimated_surface.y = normal_to_estimated_surface.y/mag;
        normal_to_estimated_surface.z = normal_to_estimated_surface.z/mag;
    }


    if((direction > 225 && direction < 315) || (direction > 45 && direction < 135))
        sort( boundaries.begin(), boundaries.end(), [](Unidirected_Boundary i, Unidirected_Boundary j) { return (i.points_coordinates[0].x < j.points_coordinates[0].x); });
    else
        sort( boundaries.begin(), boundaries.end(), [](Unidirected_Boundary i, Unidirected_Boundary j) { return (i.points_coordinates[0].y < j.points_coordinates[0].y); });

    std::vector<pcl::PointXYZRGB> all_points_coordinates;
    for(uint32_t i =0; i < boundaries.size(); i++)
        all_points_coordinates.insert(all_points_coordinates.end(), boundaries[i].points_coordinates.begin(), boundaries[i].points_coordinates.end());


    ////chain for center point on boundary
    std::vector<pcl::PointXYZRGBNormal> chain;
    const uint8_t number_of_points = 10;
    const uint8_t min_point_required = 5;
    uint32_t center_point;
    ///find chain of points
    {
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        kdtree.setInputCloud((global_cloud));
        bool switch_side = false;
        std::vector<pcl::PointXYZRGB> empty_chain;
        uint32_t i =0;
        while(i < (all_points_coordinates.size()-1)/2 && chain.size() < min_point_required+1)
        {
            uint32_t size = all_points_coordinates.size();

            /////if not found, next runs will find both sides of center
            if(! kdtree.nearestKSearch (all_points_coordinates[(size/2)+ (switch_side? i: -i)], 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
                std::cerr<< "point not found"<<std::endl;
            else
            {
                chain.push_back(cloud_with_normals->points[pointIdxRadiusSearch[0]]);
                GetChain(empty_chain, chain, number_of_points, true, boundaries[0].direction);
            }

            ////switch between sides of center
            if(switch_side)
            {
                i++;
                switch_side = false;
            }
            else
                switch_side = true;
        }
        center_point = pointIdxRadiusSearch[0];
    }


    //////chain at 30 cm from surface
    double avg_nx=0, avg_ny=0, avg_nz=0;

    for(uint32_t i = 0; i< chain.size(); i++)
    {
        double mag = sqrt(pow(chain[i].normal_x,2) + pow(chain[i].normal_y,2)+ pow(chain[i].normal_z,2));
        avg_nx += chain[i].normal_x/mag;
        avg_ny += chain[i].normal_y/mag;
        avg_nz += chain[i].normal_z/mag;
    }

    avg_nx /= chain.size();
    avg_ny /= chain.size();
    avg_nz /= chain.size();


    std::vector<double> ax_surface,ay_surface,az_surface, ax_distance, ay_distance, az_distance;


    if(chain.size() < min_point_required)
        std::cerr<< "less neighbors found"<< std::endl;
    else
    {
        double x[chain.size()];
        double y[chain.size()];
        double z[chain.size()];
        double t[chain.size()];
        double* a = new double(3);

        //polynomial fit for chain at surface
        for(uint32_t i =0; i < chain.size(); i++)
        {
            pcl::PointXYZRGB p;
            p.x = chain[i].x;
            p.y = chain[i].y;
            p.z = chain[i].z;

            x[chain.size()-1-i] = p.x;
            y[chain.size()-1-i] = p.y;
            z[chain.size()-1-i] = p.z;
            t[i] = (double)i/(chain.size()-1);

            if(show_chain_at_surface)
            {
                viewer->addSphere(chain[i], 0.001, "chain at center" + std::to_string(i));
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1, "chain at center" + std::to_string(i));
            }
        }

        a= PolynomialFit(x,t, chain.size());
        ax_surface.push_back(a[0]); ax_surface.push_back(a[1]); ax_surface.push_back(a[2]);

        a= PolynomialFit(y,t, chain.size());
        ay_surface.push_back(a[0]); ay_surface.push_back(a[1]); ay_surface.push_back(a[2]);

        a= PolynomialFit(z,t, chain.size());
        az_surface.push_back(a[0]); az_surface.push_back(a[1]); az_surface.push_back(a[2]);

        for(uint32_t i =chain.size(); i < chain.size()*2; i++)
        {
            double j = (double)i/(double)(chain.size()-1);
            pcl::PointXYZRGB p = GetPolynomialValue(ax_surface, ay_surface, az_surface, j);
            //            p.x = ax_surface[0] + ax_surface[1]*j + ax_surface[2]*std::pow(j,2);
            //            p.y = ay_surface[0] + ay_surface[1]*j + ay_surface[2]*std::pow(j,2);
            //            p.z = az_surface[0] + az_surface[1]*j + az_surface[2]*std::pow(j,2);
            //            if(show_chain_at_surface)
            viewer->addSphere(p, 0.0005, "predicted center" + std::to_string(i));

        }


        pcl::PointXYZRGB base_point(0,0,0);
        l=2;
        base_point = GetPolynomialValue(ax_surface, ay_surface, az_surface, l);
        base_point.x = ax_surface[0] + ax_surface[1]*l + ax_surface[2]*std::pow(l,2);
        base_point.y = ay_surface[0] + ay_surface[1]*l + ay_surface[2]*std::pow(l,2);
        base_point.z = az_surface[0] + az_surface[1]*l + az_surface[2]*std::pow(l,2);

        if(show_base_point)
        {
            viewer->addSphere(base_point, 0.001, "center");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "center");
        }

        ////line passing through center and normal to plane--http://www.math.ucla.edu/~ronmiech/Calculus_Problems/32A/chap11/section5/716d3/716_3.html
        double ax_center[2],ay_center[2],az_center[2];
        ax_center[0] = base_point.x; ax_center[1]= normal_to_estimated_surface.x;
        ay_center[0] = base_point.y; ay_center[1]= normal_to_estimated_surface.y;
        az_center[0] = base_point.z; az_center[1]= normal_to_estimated_surface.z;


        ////show line
        if(show_line)
            for(uint32_t i =0; i < 20; i++)
            {
                double j = (double)i/(double)(100);
                pcl::PointXYZRGB point;
                point.x = ax_center[0] + ax_center[1]*j;
                point.y = ay_center[0] + ay_center[1]*j;
                point.z = az_center[0] + az_center[1]*j;
                viewer->addSphere(point, 0.0005, "normal" + std::to_string(i));
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1, "normal" + std::to_string(i));
            }


        //////point at 30 cm on normal line

        pcl::PointXYZRGB point_on_estimated_normal;double mag = sqrt(pow(normal_to_estimated_surface.x,2) + pow(normal_to_estimated_surface.y,2)+ pow(normal_to_estimated_surface.z,2));
        point_on_estimated_normal.x = base_point.x + _camera_distance*normal_to_estimated_surface.x;
        point_on_estimated_normal.y = base_point.y + _camera_distance*normal_to_estimated_surface.y;
        point_on_estimated_normal.z = base_point.z + _camera_distance*normal_to_estimated_surface.z;

        if(show_base_point)
        {
            viewer->addSphere(point_on_estimated_normal, 0.001, "point");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,  1, 1, 1, "point");
        }

        ////polynomial fit for chain at distance
        for(uint32_t i =0; i < chain.size(); i++)
        {
            pcl::PointXYZRGB p;
            p.x = chain[i].x + avg_nx*_camera_distance;
            p.y = chain[i].y + avg_ny*_camera_distance;
            p.z = chain[i].z + avg_nz*_camera_distance;

            x[chain.size()-1-i] = p.x;
            y[chain.size()-1-i] = p.y;
            z[chain.size()-1-i] = p.z;
            t[i] = (double)i/(chain.size()-1);

            if(show_chain_distance)
            {
                viewer->addSphere(p, 0.001, "chain at center distance" + std::to_string(i));
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1, "chain at center distance" + std::to_string(i));
            }
        }

        if(!isSharpEnd)
        {
            x[chain.size()-1] = point_on_estimated_normal.x;
            y[chain.size()-1] = point_on_estimated_normal.y;
            z[chain.size()-1] = point_on_estimated_normal.z;
            t[chain.size()-1] = 2;

            a= PolynomialFit(x,t, chain.size());
            ax_distance.push_back(a[0]); ax_distance.push_back(a[1]); ax_distance.push_back(a[2]);

            a= PolynomialFit(y,t, chain.size());
            ay_distance.push_back(a[0]); ay_distance.push_back(a[1]); ay_distance.push_back(a[2]);

            a= PolynomialFit(z,t, chain.size());
            az_distance.push_back(a[0]); az_distance.push_back(a[1]); az_distance.push_back(a[2]);
        }
        else
        {
            a= PolynomialFit(x,t, chain.size());
            ax_distance.push_back(a[0]); ax_distance.push_back(a[1]); ax_distance.push_back(a[2]);

            a= PolynomialFit(y,t, chain.size());
            ay_distance.push_back(a[0]); ay_distance.push_back(a[1]); ay_distance.push_back(a[2]);

            a= PolynomialFit(z,t, chain.size());
            az_distance.push_back(a[0]); az_distance.push_back(a[1]); az_distance.push_back(a[2]);
        }
    }



    /////find next camera point
    const uint8_t min_angle = _camera_path_min_angle;
    bool  vertical_movement = ((next_pose_direction > 65 && next_pose_direction < 115) || (next_pose_direction > 245 && next_pose_direction < 295 ));
    if(!isSharpEnd)
    {
        uint8_t max_parameter = 3;
        pcl::PointXYZRGB prev_p;
        for(uint32_t i = 0; i <= 10*max_parameter; i++)
        {
            double j = (double)i/(double)(10);
            pcl::PointXYZRGB p = GetPolynomialValue(ax_distance, ay_distance, az_distance, j);;
            //            p.x = ax_distance[0] + ax_distance[1]*j + ax_distance[2]*std::pow(j,2);
            //            p.y = ay_distance[0] + ay_distance[1]*j + ay_distance[2]*std::pow(j,2);
            //            p.z = az_distance[0] + az_distance[1]*j + az_distance[2]*std::pow(j,2);

            float angle1, angle2;
            angle1 = AngleBetweenPoints(p, corner1, corner7, corner1, false);
            angle2 = AngleBetweenPoints(p, corner2, corner8, corner2, false);
            //        std::cout<<angle1<< "  "<<angle2<<std::endl;
            //std::cout<<p<<std::endl<< corner1<< std::endl<<corner5<<std::endl;
            if(show_camera_path)
            {
                viewer->addSphere(p, 0.0005, "predicted camera" + std::to_string(i));
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "predicted camera" + std::to_string(i));
            }


            if(angle1 < min_angle || angle2 < min_angle || i == 10*max_parameter)
            {
                camera_point = p;
                pcl::PointXYZRGB p_surface = GetPolynomialValue(ax_surface, ay_surface, az_surface, j);
                //                p_surface.x = ax_surface[0] + ax_surface[1]*j + ax_surface[2]*std::pow(j,2);
                //                p_surface.y = ay_surface[0] + ay_surface[1]*j + ay_surface[2]*std::pow(j,2);
                //                p_surface.z = az_surface[0] + az_surface[1]*j + az_surface[2]*std::pow(j,2);

                double mag;
                z_axis = {p_surface.x - p.x, p_surface.y - p.y, p_surface.z - p.z};
                mag = sqrt(z_axis.x * z_axis.x + z_axis.y * z_axis.y + z_axis.z * z_axis.z);
                z_axis = {z_axis.x/mag, z_axis.y/mag, z_axis.z/mag};


                if(!vertical_movement)
                {
                    x_axis = {prev_p.x - p.x, prev_p.y - p.y, prev_p.z - p.z};
                    mag = sqrt(x_axis.x * x_axis.x + x_axis.y * x_axis.y + x_axis.z * x_axis.z);
                    x_axis = {x_axis.x/mag, x_axis.y/mag, x_axis.z/mag};
                }
                else
                {
                    y_axis = {prev_p.x - p.x, prev_p.y - p.y, prev_p.z - p.z};
                    mag = sqrt(y_axis.x * y_axis.x + y_axis.y * y_axis.y + y_axis.z * y_axis.z);
                    y_axis = {y_axis.x/mag, y_axis.y/mag, y_axis.z/mag};
                }


                if(x_axis.x < 0 && !vertical_movement)
                {
                    x_axis.x = -x_axis.x;
                    x_axis.y = -x_axis.y;
                    x_axis.z = -x_axis.z;
                }
                else if(y_axis.y < 0 && vertical_movement)
                {
                    y_axis.x = -y_axis.x;
                    y_axis.y = -y_axis.y;
                    y_axis.z = -y_axis.z;
                }
                break;
            }

            prev_p = p;
        }
    }
    else
    {
        pcl::PointXYZRGB point_behind(0,0,0);

        point_behind.x = global_cloud->points[center_point].x - (_camera_distance/2)*normal_to_estimated_surface.x;
        point_behind.y = global_cloud->points[center_point].y - (_camera_distance/2)*normal_to_estimated_surface.y;
        point_behind.z = global_cloud->points[center_point].z - (_camera_distance/2)*normal_to_estimated_surface.z;

        if(show_point_behind)
        {
            viewer->addSphere(point_behind, 0.001, "point_behind");
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1, "point_behind");
        }

        uint8_t max_parameter =5;
        pcl::PointXYZRGB prev_p1;
        for(uint32_t i =0; i <= 10*max_parameter; i++)
        {
            double j = (double)i/(double)(10);
            pcl::PointXYZRGB p0, p1;
            pcl::PointXYZ vec;
            double mag;

            p0 = GetPolynomialValue(ax_surface, ay_surface, az_surface, j);
            //            p0.x = ax_surface[0] + ax_surface[1]*j + ax_surface[2]*std::pow(j,2);
            //            p0.y = ay_surface[0] + ay_surface[1]*j + ay_surface[2]*std::pow(j,2);
            //            p0.z = az_surface[0] + az_surface[1]*j + az_surface[2]*std::pow(j,2);

            vec = {point_behind.x - p0.x, point_behind.y - p0.y, point_behind.z - p0.z};
            mag = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
            vec = {vec.x/mag, vec.y/mag, vec.z/mag};

            p1.x = point_behind.x - (_camera_distance*1.5)*vec.x;
            p1.y = point_behind.y - (_camera_distance*1.5)*vec.y;
            p1.z = point_behind.z - (_camera_distance*1.5)*vec.z;


            float angle1, angle2;
            angle1 = AngleBetweenPoints(p1, corner1, corner7, corner1, false);
            angle2 = AngleBetweenPoints(p1, corner2, corner8, corner2, false);
            //        std::cout<<angle1<< "  "<<angle2<<std::endl;
            //std::cout<<p<<std::endl<< corner1<< std::endl<<corner5<<std::endl;

            if(show_camera_path)
            {
                viewer->addSphere(p1, 0.0005, "predicted camera" + std::to_string(i));
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "predicted camera" + std::to_string(i));
            }

            if(angle1 < (min_angle - 10) || angle2 < (min_angle - 10) || i == 10*max_parameter)
            {
                double mag;

                camera_point = p1;
                z_axis = {vec.x, vec.y, vec.z};

                if(!vertical_movement)
                {
                    x_axis = {prev_p1.x - p1.x, prev_p1.y - p1.y, prev_p1.z - p1.z};
                    mag = sqrt(x_axis.x * x_axis.x + x_axis.y * x_axis.y + x_axis.z * x_axis.z);
                    x_axis = {x_axis.x/mag, x_axis.y/mag, x_axis.z/mag};
                }
                else
                {
                    y_axis = {prev_p1.x - p1.x, prev_p1.y - p1.y, prev_p1.z - p1.z};
                    mag = sqrt(y_axis.x * y_axis.x + y_axis.y * y_axis.y + y_axis.z * y_axis.z);
                    y_axis = {y_axis.x/mag, y_axis.y/mag, y_axis.z/mag};
                }

                if(x_axis.x < 0 && !vertical_movement)
                {
                    x_axis.x = -x_axis.x;
                    x_axis.y = -x_axis.y;
                    x_axis.z = -x_axis.z;
                }
                else if(y_axis.y < 0 && vertical_movement)
                {
                    y_axis.x = -y_axis.x;
                    y_axis.y = -y_axis.y;
                    y_axis.z = -y_axis.z;
                }
                break;
            }

            prev_p1 = p1;
        }
    }


    if(show_camera_point)
    {

        viewer->addSphere(camera_point, 0.005, "camera_point");
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1, 1, 1, "camera_point");
    }


    if(!vertical_movement)
    {
        y_axis.x = z_axis.y*x_axis.z - z_axis.z*x_axis.y;
        y_axis.y = z_axis.z*x_axis.x - z_axis.x*x_axis.z;
        y_axis.z = z_axis.x*x_axis.y - z_axis.y*x_axis.x;

        x_axis.x = y_axis.y*z_axis.z - y_axis.z*z_axis.y;
        x_axis.y = y_axis.z*z_axis.x - y_axis.x*z_axis.z;
        x_axis.z = y_axis.x*z_axis.y - y_axis.y*z_axis.x;
    }
    else
    {
        x_axis.x = y_axis.y*z_axis.z - y_axis.z*z_axis.y;
        x_axis.y = y_axis.z*z_axis.x - y_axis.x*z_axis.z;
        x_axis.z = y_axis.x*z_axis.y - y_axis.y*z_axis.x;

        y_axis.x = z_axis.y*x_axis.z - z_axis.z*x_axis.y;
        y_axis.y = z_axis.z*x_axis.x - z_axis.x*x_axis.z;
        y_axis.z = z_axis.x*x_axis.y - z_axis.y*x_axis.x;
    }

    uint angle_bw_x_n_downwards = GetAngleBetweenVectors(x_axis, vector_downwards);
    uint angle_bw_y_n_downwards = GetAngleBetweenVectors(y_axis, vector_downwards);
    std::cout<<"Angle x and y to downwards"<<angle_bw_x_n_downwards<<std::endl<<angle_bw_y_n_downwards<<std::endl;
    //    to keep camera vertical in world coordinates
    if(angle_bw_y_n_downwards < 100)
    {
        std::cout<<std::endl<<std::endl<<"Axes modified to keep arm up"<<std::endl<<std::endl;
        if(angle_bw_x_n_downwards > 90)
        {
            pcl::PointXYZ temp = y_axis;
            y_axis = x_axis;
            x_axis = pcl::PointXYZ(-temp.x, -temp.y, -temp.z);
        }
        else
        {
            pcl::PointXYZ temp = y_axis;
            y_axis = pcl::PointXYZ(-x_axis.x, -x_axis.y, -x_axis.z);
            x_axis = temp;
        }
    }

    std::cout<< "Camera point"<<camera_point<<std::endl;
    std::cout<<"z axis"<<z_axis<<std::endl;
    std::cout<<"x axis"<<x_axis<<std::endl;
    std::cout<<"y axis"<<y_axis<<std::endl;

    if(show_axis)
        for(uint32_t i =0; i < 5; i++)
        {
            double j = (double)i/(double)(100);
            pcl::PointXYZRGB point;

            point.x = camera_point.x + z_axis.x*j;
            point.y = camera_point.y + z_axis.y*j;
            point.z = camera_point.z + z_axis.z*j;
            viewer->addSphere(point, 0.001, "zaxis" + std::to_string(i));
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "zaxis" + std::to_string(i));

            point.x = camera_point.x + y_axis.x*j;
            point.y = camera_point.y + y_axis.y*j;
            point.z = camera_point.z + y_axis.z*j;
            viewer->addSphere(point, 0.001, "yaxis" + std::to_string(i));
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "yaxis" + std::to_string(i));

            point.x = camera_point.x + x_axis.x*j;
            point.y = camera_point.y + x_axis.y*j;
            point.z = camera_point.z + x_axis.z*j;
            viewer->addSphere(point, 0.001, "xaxis" + std::to_string(i));
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "xaxis" + std::to_string(i));
        }

}

////For debugging purposes.. not required
void ViewPoint::ShowDirectedBoundaryPoints()
{
    viewer->removeAllShapes(); viewer->addPolylineFromPolygonMesh(global_triangles);
    //                do{
    int boundary_set = click++;

    //    if(directed_boundary_edges.size() > 0)
    //    {
    //        //Show directed boundaries on point cloud or mesh as spheres
    //        //                remove duplicates
    //        sort(directed_boundary_edges[boundary_set].begin(), directed_boundary_edges[boundary_set].end() );
    //        directed_boundary_edges[boundary_set].erase( unique( directed_boundary_edges[boundary_set].begin(), directed_boundary_edges[boundary_set].end() ), directed_boundary_edges[boundary_set].end() );

    //        for(std::vector<uint32_t>::iterator i = directed_boundary_edges[boundary_set].begin(); i != directed_boundary_edges[boundary_set].end(); i++)
    //        {
    //            viewer->addSphere(cloudJustForDisplay->points[*i], 0.001, std::to_string(boundary_set) + std::to_string(*i));
    //        }


    //    }

    //                }while(click != directed_boundary_edges.size());

    //        int boundary_set = click++;
    //    //remove duplicates


    if(global_boundaries.size() > 0)
    {
        for(uint boundary_set = 0; boundary_set < global_boundaries.size(); boundary_set++)
        {
            for(std::vector<uint32_t>::iterator i = global_boundaries[boundary_set].points.begin(); i != global_boundaries[boundary_set].points.end(); i++)
            {
                viewer->addSphere(cloudJustForDisplay->points[*i], 0.005, "b " + std::to_string(*i));
                //                                std::cout<<*i<<std::endl;
            }
        }
    }



    if(click == global_boundaries.size())
        click=0;
}


////Remove boundary set with direction "angle" in "boundaries"
/// param in: angle: direction for boundary sets to be removed
/// param in: angle_range: remove boundary sets in this range around "angle"
inline void ViewPoint::RemoveBoundariesForAngle(int angle, int angle_range = 50)
{
    int min_angle, max_angle;
    max_angle = (angle + angle_range) > 360 ? (angle + angle_range) - 360 : (angle + angle_range);
    min_angle = (angle - angle_range) < 0 ? (angle - angle_range) + 360 : (angle - angle_range);
    std::cout<<"min angle: "<<min_angle<< "  max_angle: "<<max_angle<<std::endl;
    if(min_angle < 270)
    {
        for(std::vector<Unidirected_Boundary>::iterator it = boundaries.end(); it >= boundaries.begin(); it--)
        {
            if(((*it).direction > min_angle && (*it).direction < max_angle))
                boundaries.erase(it);
        }
    }
    else
    {
        for(std::vector<Unidirected_Boundary>::iterator it = boundaries.end(); it >= boundaries.begin(); it--)
        {
            if(((*it).direction > min_angle || (*it).direction < max_angle))
                boundaries.erase(it);
        }
    }
}


////Create a cloud of all the boundary points for cloud
/// param in: cloud: input cloud
/// param out: boundary_points_cloud: cloud of boundary points
void ViewPoint::GetBoundaryPointsCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_points_cloud)
{
    pcl::PointCloud<pcl::Boundary> pcl_bpoints;
    pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> boundEst;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normEst;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(cloud));
    normEst.setRadiusSearch(0.01);
    normEst.compute(*normals);

    boundEst.setInputCloud(cloud);
    boundEst.setInputNormals(normals);
    boundEst.setRadiusSearch(0.01);
    boundEst.setAngleThreshold(M_PI/2);
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    boundEst.compute(pcl_bpoints);

    for(int i = 0; i < pcl_bpoints.points.size(); i++)
    {
        if(pcl_bpoints[i].boundary_point > 0)
        {
            boundary_points_cloud->points.push_back(cloud->points[i]);
        }
    }
    boundary_points_cloud->width = boundary_points_cloud->points.size();
    boundary_points_cloud->height = 1;

    //    pcl::visualization::Cloudviewer viewer1("PCL viewer");

    //    viewer1.showCloud(boundary_points_cloud);

    //    while (!viewer1.wasStopped());

}


////Filter out boundary points which are not in the boundary_points_cloud
/// param in: boundary_points_cloud: point cloud of boundary points of the scan_cloud
/// output: updates "edges"
void ViewPoint::FilterEdges(pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_points_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_points_to_consider (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    std::vector<int> pointIds;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.setInputCloud (boundary_points_cloud);

    for(uint i = 0; i < edges.size(); i++)
    {
        if(edges[i].size() == 0)
            continue;

        else if (! kdtree.radiusSearch (global_cloud->points[i], 0.01, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            edges[i].clear();
        }
//        else
//        {
//            boundary_points_to_consider->points.push_back(global_cloud->points[i]);
//        }
    }
//    boundary_points_to_consider->width = boundary_points_to_consider->points.size();
//    boundary_points_to_consider->height = 1;

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D "));
//    viewer2->addPointCloud(boundary_points_to_consider);
//    viewer2->spin();
}


////Sets the scan_cloud for boundary points filtration
void ViewPoint::SetScanCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    scan_cloud = cloud;
}


/////Find the Next Best View for the source_cloud
/// Act as the main function of the class, controls the flow of algorithm
/// param in: source_cloud: point cloud for which NBV to be found
int ViewPoint::FindNextViewPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud)
{
    viewer->setBackgroundColor (0, 0, 0);
    //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    //      viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    //  viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    std::cerr << "Point cloud data: " << source_cloud->points.size () << " points" << std::endl;

    ////Filter CLoud
    FilterCloud(source_cloud, _first_filter_distance, 500);
    pcl::copyPointCloud(*global_cloud, *cloudJustForDisplay);
    std::cerr << "Point cloud data: " << global_cloud->points.size () << " points" << std::endl;


    //// Normal estimation*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    CalculateNormals(global_cloud, cloud_with_normals);


    ////mesh details
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3 = GreedyProjectionTriangulation(cloud_with_normals);


    ////store connecting boundary vertices as edges
    edges.resize( (global_cloud->points.size()));
    GetBoundaryEdges(gp3);
    ////print edges variable
    //    std::ofstream myfile;
    //    myfile.open("edges");
    //    for(Vector_VectorOfInt::iterator i = edges.begin(); i != edges.end(); i++)
    //    {
    //        //        std::cout<<std::distance( edges.begin(), i ) << "  :  ";
    //        //        for(std::vector< uint32_t>::iterator j = i->begin(); j != i->end(); j++)
    //        //            std::cout<< *j << "     ";
    //        //        std::cout<<std::endl;

    //        myfile << std::distance( edges.begin(), i ) << "  :  ";

    //        for(std::vector< uint32_t>::iterator j = i->begin(); j != i->end(); j++)
    //            myfile << *j << "     ";
    //        myfile <<std::endl;
    //    }
    //    myfile.close();


    if(scan_cloud->points.size() > 0)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_points_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        GetBoundaryPointsCloud(scan_cloud, boundary_points_cloud);
        FilterEdges(boundary_points_cloud);
    }


    ////find similar directed edges in edges
    FindDirectedBoundries();


    ////remove duplicate points in boundaries
    for(Vector_VectorOfInt::iterator it = directed_boundary_edges.begin(); it != directed_boundary_edges.end(); it++)
    {
        sort( (*it).begin(), (*it).end() );
        (*it).erase( unique( (*it).begin(), (*it).end() ), (*it).end() );
    }


    ////print directed boundaries
    for(Vector_VectorOfInt::iterator i = directed_boundary_edges.begin(); i != directed_boundary_edges.end(); i++)
    {
        //                     std::cout<<std::distance( directed_boundary_edges.begin(), i ) << "  :  "<< i->size()<<std::endl;
        //                     for(std::vector< uint32_t>::iterator j = i->begin(); j != i->end(); j++)
        //                         std::cout<< *j << "     ";
        //                     std::cout<<std::endl;
    }


    ////find direction of boundaries
    FindDirectionOfBoundaries();


    /////remove boundaries to avoid
    RemoveBoundariesForAngle(angles_to_avoid[angles_to_avoid.size()-1], 70);
    for(size_t i = 0; i<angles_to_avoid.size()-1; i++ )
    {
        RemoveBoundariesForAngle(angles_to_avoid[i], 50);
    }


    if(boundaries.size() == 0)
    {
        std::cerr<<"No boundary";
        return -1;
    }


    /////remove duplicate points
    for(int boundary_set =0; boundary_set< boundaries.size(); boundary_set++)
    {
        sort( boundaries[boundary_set].points.begin(), boundaries[boundary_set].points.end() );
        boundaries[boundary_set].points.erase( unique( boundaries[boundary_set].points.begin(), boundaries[boundary_set].points.end() ), boundaries[boundary_set].points.end() );
    }


    ////keep only the similar directed boundaries with max number points
    FindSetOfDirectedBoundariesWithMaxPoints();


    ////join close boundaries
    JoinCloseBoundaries();
    global_boundaries = boundaries;
    std::cout<<"Move camera in direction(angle to x axis) :"<< boundaries[0].direction<<std::endl;


    ////Filter CLoud
    FilterCloud(source_cloud, _second_filter_distance, 500);
    CalculateNormals(global_cloud, cloud_with_normals);
    *global_cloud_normal = *cloud_with_normals;


    //    /////Estimate surface towards maximum boundary points--  works on varibale boundary filled from main
    EstimateSurface();


    /////Find view point to cover estimated surface--works on varibale boundary filled from main and edited in EstimateBoundary
    FindViewPoint(cloud_with_normals);

    std::cout<<std::endl;

    //    int jkl=0;
    //    while (jkl++ < 30)
    //    {
    //        viewer->spinOnce (100);
    //        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    //    }
    viewer->addPolylineFromPolygonMesh(global_triangles);
    //    viewer->addSphere(pcl::PointXYZ(0,0,0), 0.001, "b");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "b");
    viewer->addCoordinateSystem(0.1);
    //     ShowDirectedBoundaryPoints();
    //            viewer->addPointCloud<pcl::PointXYZRGB> (cloudJustForDisplay, "cloud");
    viewer->spin();
    viewer->removeAllShapes();
    viewer->removeAllCoordinateSystems();
    viewer->spinOnce();

    return (0);
}


////Gets the Direction of the next movement as the angle to the x_axis in degrees
int ViewPoint::GetNextPoseDirection()
{
    return next_pose_direction;
}


////Gets the Next Best View position
pcl::PointXYZRGB ViewPoint::GetCameraPoint()
{
    return camera_point;
}


/////Get the angle between vectors v1 and v2
double ViewPoint::GetAngleBetweenVectors(pcl::PointXYZ v1, pcl::PointXYZ v2)
{
    double angle, res, v1mag, v2mag;
    //PointXYZ is just used as struct of xyz
    pcl::PointXYZ v1norm, v2norm;

    v1mag = sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
    v1norm = {v1.x / v1mag, v1.y / v1mag, v1.z / v1mag};

    v2mag = sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
    v2norm = {v2.x / v2mag, v2.y / v2mag, v2.z / v2mag};

    res = v1norm.x * v2norm.x + v1norm.y * v2norm.y + v1norm.z * v2norm.z;

    res = roundf( res * 1000.0 ) / 1000.0;

    angle = acos(res)*(180/M_PI);

    return angle;
}


////Return the angle of x-axis vector of the NBV from x-axis
double ViewPoint::GetXAxisRotation()
{
    pcl::PointXYZ current_x_axis(1,0,0);

    return  GetAngleBetweenVectors(current_x_axis, x_axis);
}


////Return the angle of y-axis vector of the NBV from y-axis
double ViewPoint::GetYAxisRotation()
{
    pcl::PointXYZ current_y_axis(0,1,0);

    return  GetAngleBetweenVectors(current_y_axis, y_axis);
}


////Return the angle of z-axis vector of the NBV from z-axis
double ViewPoint::GetZAxisRotation()
{
    pcl::PointXYZ current_z_axis(0,0,1);
    return  GetAngleBetweenVectors(current_z_axis, z_axis);
}


////Return the x-axis vector of the NBV
pcl::PointXYZ ViewPoint::GetXAxis()
{
    return x_axis;
}


////Return the y-axis vector of the NBV
pcl::PointXYZ ViewPoint::GetYAxis()
{
    return y_axis;
}


////Return the z-axis vector of the NBV
pcl::PointXYZ ViewPoint::GetZAxis()
{
    return z_axis;
}
