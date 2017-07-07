#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <MathLib/mathBasics.h>
#include <MathLib/mathVec.h>
#include <MathLib/mathFrame.h>
#include <string.h>

#include <viewpoint.h>
#include <iostream>

#include <libconfig.h++>
using namespace libconfig;


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D viewer"));


MainWindow::MainWindow(QWidget *parent) :  QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->dsbPenaltyTolerance->setValue(_penalty_tolerance);
    ui->dsbAngleTolerance->setValue(_angle_tolerance);
    ui->dsbMinimumLength->setValue(_min_length_for_boundary_edge);
    ui->dsbJoinBoundariesDistance->setValue(_join_boundaries_distance);

    ui->dsbChainSmallRadius->setValue(_gc_smallRadius);
    ui->dsbChainBigRadius->setValue(_gc_bigRadius);
    ui->dsbChainDefaultBigRadius->setValue(_gc_defaultBigRadius);
    ui->dsbChainIncRadius->setValue(_gc_incBigRadius);
    ui->dsbChainMaxBigRadius->setValue(_gc_maxBigRadius);


    ui->dsbCameraDistance->setValue(_camera_distance);
    ui->dsbOverlappingCoefficient->setValue(_overlap_coefficient);
    ui->dsbCameraAngleToCoeff->setValue(_camera_path_min_angle);
}


MainWindow::~MainWindow()
{
    delete ui;
}


Eigen::Matrix4d FrameToEigenMatrix( math::Frame4d transformation_rs2tool)
{
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

    //    for(int i =0; i< 4; i++)
    //        for(int j=0; j <4; j++)
    //            transform_setup(i,j) = transMatrix[j][i];

    transform(0,0) = transformation_rs2tool.N().X() ; transform(0,1) = transformation_rs2tool.O().X() ; transform(0,2) = transformation_rs2tool.A().X() ; transform(0,3) = transformation_rs2tool.P().X() ;
    transform(1,0) = transformation_rs2tool.N().Y() ; transform(1,1) = transformation_rs2tool.O().Y() ; transform(1,2) = transformation_rs2tool.A().Y() ; transform(1,3) = transformation_rs2tool.P().Y() ;
    transform(2,0) = transformation_rs2tool.N().Z() ; transform(2,1) = transformation_rs2tool.O().Z() ; transform(2,2) = transformation_rs2tool.A().Z() ; transform(2,3) = transformation_rs2tool.P().Z() ;
    transform(3,0) = transformation_rs2tool.N().W() ; transform(3,1) = transformation_rs2tool.O().W() ; transform(3,2) = transformation_rs2tool.A().W() ; transform(3,3) = transformation_rs2tool.P().W() ;

    return transform;
}


math::Frame4d GetTransformationMatrixFromFile(std::string filename)
{
    math::Frame4d transMatrix;
    ifstream f(filename);
    if (!f) {
        cout << "Cannot open file.\n";
    }

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            f >> transMatrix[j][i];
    f.close();
    return transMatrix;
}


std::vector<pcl::PointXYZ> GetMinMaxPoints()
{
    Config pos;

    std::string file_name = "SetUp.points";

    // Read the file. If there is an error, report it and exit.
    try
    {
        pos.readFile(file_name.c_str());
    }
    catch(const FileIOException &fioex)
    {
        cerr << "I/O error while reading file." << std::endl;
    }
    catch(const ParseException &pex)
    {
        cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << endl;
    }

    // Get the values.
    try
    {
        std::string type;
        pos.lookupValue("TYPE",  type);
        if(type != "MIN_MAX_POINTS")
        {
            throw new std::exception;
        }


        std::vector<pcl::PointXYZ> min_max_points;
        pcl::PointXYZ point;

        pos.lookupValue("MINx",  point.x);
        pos.lookupValue("MINy",  point.y);
        pos.lookupValue("MINz",  point.z);
        min_max_points.push_back(point);

        pos.lookupValue("MAXx",  point.x);
        pos.lookupValue("MAXy",  point.y);
        pos.lookupValue("MAXz",  point.z);
        min_max_points.push_back(point);

        pos.lookupValue("CORNERx",  point.x);
        pos.lookupValue("CORNERy",  point.y);
        pos.lookupValue("CORNERz",  point.z);
        min_max_points.push_back(point);

        return min_max_points;
    }
    catch(const SettingNotFoundException &nfex)
    {
        cerr << "File corrupted." << endl;
    }
}


void MainWindow::on_btnFindNextPose_released()
{
    std::string fileName = ui->txtFileName->toPlainText().toUtf8().constData();
    std::cout<<"Calculating for : "<< fileName <<std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    //// Fetch point cloud filename in arguments | Works with PCD and PLY files
    pcl::io::loadPCDFile (fileName + ".pcd", *source_cloud);

    math::Frame4d transform_matrix = GetTransformationMatrixFromFile(fileName);
    transform_matrix.P().X() /= 1000; transform_matrix.P().Y() /= 1000; transform_matrix.P().Z() /= 1000;
    transform_matrix.OrthoNormalize();
    Eigen::Matrix4d transform_eigen_matrix = FrameToEigenMatrix(transform_matrix.GetInv());
    pcl::transformPointCloud(*source_cloud, *global_cloud, transform_eigen_matrix);

    pcl::io::loadPCDFile ("output.pcd", *scan_cloud);
    pcl::transformPointCloud(*scan_cloud, *scan_cloud, transform_eigen_matrix);

    ViewPoint vp(viewer);

    pcl::VoxelGrid<pcl::PointXYZRGB> vgf;
    vgf.setInputCloud (scan_cloud);
    vgf.setLeafSize (0.005, 0.005, 0.005);
    vgf.filter (*scan_cloud);

    vp.SetScanCloud(scan_cloud);
    vp.ViewPointCalculationSettings( _first_filter_distance,  _second_filter_distance
                                     ,  _gp3_radius,  ui->dsbJoinBoundariesDistance->value(),  ui->dsbCameraDistance->value()
                                     ,  ui->dsbOverlappingCoefficient->value(),  ui->dsbCameraAngleToCoeff->value());

    vp.SetParametersForDirectedBoundaries( ui->dsbPenaltyTolerance->value(),  ui->dsbAngleTolerance->value() , ui->dsbMinimumLength->value());

    vp.SetParametersForChain( ui->dsbChainSmallRadius->value(),  ui->dsbChainDefaultBigRadius->value(),  ui->dsbChainMaxBigRadius->value(),  ui->dsbChainMaxBigRadius->value());


    std::vector<uint> ang;
    ang.push_back(269);
    //        ang.push_back(325);
    //    vp.SetAnglesToAvoid(ang);

    /////Find Vector towards ground
    math::Vec4d center_point_of_object(0,0,0,1), point_downwards(0,0,0,1);
    std::vector<pcl::PointXYZ> min_max_points = GetMinMaxPoints();
    center_point_of_object[0] = (min_max_points[0].x + min_max_points[1].x)/2;
    center_point_of_object[1] = (min_max_points[0].y + min_max_points[1].y)/2;
    center_point_of_object[2] = (min_max_points[0].z + min_max_points[1].z)/2;
    point_downwards = center_point_of_object;
    point_downwards[2] = min_max_points[0].z;
    center_point_of_object = transform_matrix.GetInv() * center_point_of_object;
    point_downwards = transform_matrix.GetInv() * point_downwards;
    pcl::PointXYZ vec_down(point_downwards[0] - center_point_of_object[0] , point_downwards[1] - center_point_of_object[1] , point_downwards[2] - center_point_of_object[2] );

    vp.SetVectorDirectionToAvoid(vec_down);


    vp.FindNextViewPoint(global_cloud);

    std::cout<<"done......"<<std::endl;

}
