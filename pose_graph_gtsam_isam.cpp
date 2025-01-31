#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>

// We want to use iSAM2 to solve the pose optimazation problem incrementally, so
// include iSAM2 here
#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using Sophus::SE3;
using Sophus::SO3;

/************************************************
 * 本程序演示如何用 gtsam 进行位姿图优化
 * sphere.g2o 是人工生成的一个 Pose graph，我们来优化它。
 * 与 g2o 相似，在 gtsam 中添加的是因子，相当于误差
 * **********************************************/

#define USE_ISAM 1


//闭环信息
typedef struct loop_t{
    int id1;
    int id2;
    gtsam::Rot3 R;
    gtsam::Point3 t;
    gtsam::Matrix mgtsam;
}Loop_;

std::vector<Loop_> getLoopInformation(const std::vector<Loop_>& loops_all, const int current_id);

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"Usage: pose_graph_gtsam sphere.g2o"<<endl;
        return 1;
    }
    ifstream fin ( argv[1] );
    if ( !fin )
    {
        cout<<"file "<<argv[1]<<" does not exist."<<endl;
        return 1;
    }

    // 从g2o文件中读取节点和边的信息，并将顶点与边存起来
    std::vector<gtsam::Pose3> pose_graph_poses;
    std::vector<Loop_> pose_graph_loops;

    int cntVertex=0, cntEdge = 0;
    cout<<"reading from g2o file"<<endl;
    
    while ( !fin.eof() )
    {
        string tag;
        fin>>tag;
        if ( tag == "VERTEX_SE3:QUAT" )
        {
            // 顶点
            gtsam::Key id;
            fin>>id;
            double data[7];
            for ( int i=0; i<7; i++ ) fin>>data[i];
            // 转换至gtsam的Pose3
            gtsam::Rot3 R = gtsam::Rot3::Quaternion ( data[6], data[3], data[4], data[5] );
            gtsam::Point3 t ( data[0], data[1], data[2] );
    
            pose_graph_poses.push_back(gtsam::Pose3 ( R,t ));
            cntVertex++;
        }
        else if ( tag == "EDGE_SE3:QUAT" )
        {
            // 边，对应到因子图中的因子
            gtsam::Matrix m = gtsam::I_6x6;     // 信息矩阵
            gtsam::Key id1, id2;
            fin>>id1>>id2;
            double data[7];
            for ( int i=0; i<7; i++ ) fin>>data[i];
            gtsam::Rot3 R = gtsam::Rot3::Quaternion ( data[6], data[3], data[4], data[5] );
            gtsam::Point3 t ( data[0], data[1], data[2] );
            for ( int i=0; i<6; i++ )
                for ( int j=i; j<6; j++ )
                {
                    double mij;
                    fin>>mij;
                    m ( i,j ) = mij;
                    m ( j,i ) = mij;
                }
                
            // g2o的信息矩阵定义方式与gtsam不同，这里对它进行修改
            gtsam::Matrix mgtsam = gtsam::I_6x6;
            mgtsam.block<3,3> ( 0,0 ) = m.block<3,3> ( 3,3 ); // cov rotation
            mgtsam.block<3,3> ( 3,3 ) = m.block<3,3> ( 0,0 ); // cov translation
            mgtsam.block<3,3> ( 0,3 ) = m.block<3,3> ( 0,3 ); // off diagonal
            mgtsam.block<3,3> ( 3,0 ) = m.block<3,3> ( 3,0 ); // off diagonal
            
            Loop_ loop;
            loop.id1 = id1;
            loop.id2 = id2;
            loop.R = R;
            loop.t = t;
            loop.mgtsam = mgtsam;
 
            pose_graph_loops.push_back(loop);

            cntEdge++;
        }
        if ( !fin.good() )
            break;
    }
    
    cout<<"read total "<<cntVertex<<" vertices, "<<cntEdge<<" edges."<<endl;    

    //构造因子图
    gtsam::NonlinearFactorGraph::shared_ptr graph ( new gtsam::NonlinearFactorGraph );  // gtsam的因子图
    gtsam::NonlinearFactorGraph::shared_ptr graph0 ( new gtsam::NonlinearFactorGraph );  // gtsam的因子图 
    gtsam::Values::shared_ptr initial ( new gtsam::Values ); // 初始值

    // SLAM过程,按照顺序增量添加观测

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);

    for(int i=0;i<pose_graph_poses.size();i++)
    {
        initial->insert( i+1,pose_graph_poses[i]);

        if(i==0)
        {
            //第一帧，添加先验位姿信息
            gtsam::noiseModel::Diagonal::shared_ptr priorModel = 
            gtsam::noiseModel::Diagonal::Variances (
                ( gtsam::Vector ( 6 ) <<1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 ).finished() 
            );
            graph->push_back(gtsam::PriorFactor<gtsam::Pose3>(1, pose_graph_poses[0], priorModel));
            graph0->push_back(gtsam::PriorFactor<gtsam::Pose3>(1, pose_graph_poses[0], priorModel));
            continue;
        }

        //添加连接关系
        std::vector<Loop_> loops;
        loops = getLoopInformation(pose_graph_loops,i);
        std::cout<<"Frame: "<< i <<" has connectivity size: "<<loops.size(); 
        for(auto iter = loops.begin(); iter != loops.end(); iter++)
        {
            // std::cout<<"id2: "<<iter->id2+1<<std::endl;
            /**添加关联**/
            // 高斯噪声模型
            gtsam::SharedNoiseModel model = gtsam::noiseModel::Gaussian::Information ( iter->mgtsam );   

            // 添加一个因子     
            // std::cout<<" Add factor: "<<iter->id1+1<<"  "<<iter->id2+1;
            gtsam::NonlinearFactor::shared_ptr factor ( 
                new gtsam::BetweenFactor<gtsam::Pose3> ( 
                    iter->id1+1,  
                    iter->id2+1, 
                    gtsam::Pose3 ( iter->R,iter->t ), model ) 
            );

            graph->push_back ( factor );
            graph0->push_back ( factor );
        }
        std::cout<<std::endl;


        //优化更新
        std::cout<<"isam2 update"<<std::endl;
        isam.update(*graph, *initial);
        isam.update();

        // Clear the factor graph and values for the next iteration
        // 特别重要，update以后，清空原来的约束。已经加入到isam2的那些会用bayes tree保管，你不用操心了。
        graph->resize(0);
        initial->clear();
    }

    // 写入 g2o 文件，同样伪装成 g2o 中的顶点和边，以便用 g2o_viewer 查看。
    gtsam::Values result = isam.calculateEstimate();
    // graph0中保留了连接关系
    writeG2o(*graph0, result, "result_gtsam_isam.g2o");

    return 0;
}


std::vector<Loop_> getLoopInformation(const std::vector<Loop_>& loops_all, const int current_id)
{
    std::vector<Loop_> loops_cur;

    for(int i=0;i<loops_all.size();i++)
    {
        Loop_ loop = loops_all[i];

        if(loop.id2 == current_id)
        {
            loops_cur.push_back(loop);
            continue;
        }
    }

    return loops_cur;

}