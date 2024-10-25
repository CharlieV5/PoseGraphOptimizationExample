#include <iostream>
#include <fstream>//文件读取头文件
#include <string>
#include <Eigen/Core>//Eigen核心模块
 
#include <g2o/core/base_vertex.h>//g2o顶点（Vertex）头文件 视觉slam十四讲p141用顶点表示优化变量，用边表示误差项
#include <g2o/core/base_binary_edge.h>//g2o边（edge）头文件
#include <g2o/core/block_solver.h>//求解器头文件
#include <g2o/core/optimization_algorithm_levenberg.h>//列文伯格——马尔夸特算法头文件
#include <g2o/solvers/eigen/linear_solver_eigen.h>
 
#include <sophus/se3.hpp>
 
using namespace std;
using namespace Eigen;
using Sophus::SE3d;//在SE3d子类中引用基类Sophus的成员
using Sophus::SO3d;//在SO3d子类中引用基类Sophus的成员
 
 
/************************************************
 * 本程序演示如何用g2o solver进行位姿图优化
 * sphere.g2o是人工生成的一个Pose graph，我们来优化它。
 * 尽管可以直接通过load函数读取整个图，但我们还是自己来实现读取代码，以期获得更深刻的理解
 * 本节使用李代数表达位姿图，节点和边的方式为自定义
 * **********************************************/
typedef Matrix<double, 6, 6> Matrix6d;
 
// 给定误差求J_R^{-1}的近似
Matrix6d JRInv(const SE3d &e) {
    Matrix6d J;//雅可比矩阵
    //视觉SLAM十四讲p272式(10.11)
    //              | φe(^)   ρe(^) |   | E   0 |        | φe(^)   ρe(^) |
    //Jr(-1)≈I+(1/2)|               | = |       | + (1/2)|               |
    //              |   0     φe(^) |   | 0   E |        |   0     φe(^) |
    J.block(0, 0, 3, 3) = SO3d::hat(e.so3().log());//E+(1/2)φe(^)
    J.block(0, 3, 3, 3) = SO3d::hat(e.translation());//(1/2)ρe(^)
    J.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);//0
    J.block(3, 3, 3, 3) = SO3d::hat(e.so3().log());//E+(1/2)φe(^)
    // J = J * 0.5 + Matrix6d::Identity();
    J = Matrix6d::Identity();    // try Identity if you want  用单位阵来近似雅可比矩阵
    return J;
}
 
// 李代数顶点
typedef Matrix<double, 6, 1> Vector6d;
 
class VertexSE3LieAlgebra : public g2o::BaseVertex<6, SE3d> //public表示公有继承；VertexSE3LieAlgebra是派生类，BaseVertex<3, Eigen::Vector3d>是基类
{
public://以下定义的成员变量和成员函数都是公有的
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW//解决Eigen库数据结构内存对齐问题
 
    virtual bool read(istream &is) override  //istream类是c++标准输入流的一个基类
    //可参照C++ Primer Plus第六版的6.8节
      {
        double data[7];//定义数组
        for (int i = 0; i < 7; i++)
            is >> data[i];
        setEstimate(SE3d(
            Quaterniond(data[6], data[3], data[4], data[5]),
            Vector3d(data[0], data[1], data[2])
        ));//Quaterniond表示四元数qw qx qy qz Vector3d表示平移向量元素tx ty tz
        return true;
    }
 
    virtual bool write(ostream &os) const override //ostream类是c++标准输出流的一个基类
    //可参照C++ Primer Plus第六版的6.8节
    {
        os << id() << " ";
        Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl;
        return true;
    }
 
    virtual void setToOriginImpl() override //virtual表示该函数为虚函数，override保留字表示当前函数重写了基类的虚函数
    {
        _estimate = SE3d();
    }
 
    // 左乘更新
    virtual void oplusImpl(const double *update) override 
    {
        Vector6d upd;
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = SE3d::exp(upd) * _estimate;
    }
};
 
// 两个李代数节点之边
class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, SE3d, VertexSE3LieAlgebra, VertexSE3LieAlgebra> {
public://以下定义的成员变量和成员函数都是公有的
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW//解决Eigen库数据结构内存对齐问题
 
    virtual bool read(istream &is) override  //istream类是c++标准输入流的一个基类
    //可参照C++ Primer Plus第六版的6.8节
    {
        double data[7];
        for (int i = 0; i < 7; i++)
            is >> data[i];
        Quaterniond q(data[6], data[3], data[4], data[5]);//四元数
        q.normalize();//归一化
        setMeasurement(SE3d(q, Vector3d(data[0], data[1], data[2])));
        for (int i = 0; i < information().rows() && is.good(); i++)
            for (int j = i; j < information().cols() && is.good(); j++) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }
 
    virtual bool write(ostream &os) const override 
    {
        VertexSE3LieAlgebra *v1 = static_cast<VertexSE3LieAlgebra *> (_vertices[0]);//定义指针v1
        VertexSE3LieAlgebra *v2 = static_cast<VertexSE3LieAlgebra *> (_vertices[1]);//定义指针v2
        os << v1->id() << " " << v2->id() << " ";
        SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";//输出四元数
 
        // information matrix 信息矩阵
        for (int i = 0; i < information().rows(); i++)
            for (int j = i; j < information().cols(); j++) {
                os << information()(i, j) << " ";
            }
        os << endl;
        return true;
    }
 
    // 误差计算与书中推导一致
    virtual void computeError() override {
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        //视觉SLAM十四讲p271式(10.4)
        //eij(^)=Δξij*In(Tij(-1)Ti(-1)Tj)^
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
        //_measurement.inverse() -> Tij(-1)
        //v1.inverse() -> Ti(-1)
        //Tj -> v2
        //.log()表示In()
    }
 
    // 雅可比计算
    virtual void linearizeOplus() override {
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        Matrix6d J = JRInv(SE3d::exp(_error));//使用TRInv()函数提供近似的Jr(-1)
        // 尝试把J近似为I？
        _jacobianOplusXi = -J * v2.inverse().Adj();//视觉SLAM十四讲式(10.9)
        _jacobianOplusXj = J * v2.inverse().Adj();//视觉SLAM十四讲式(10.10)
    }
};
 
int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: pose_graph_g2o_SE3_lie sphere.g2o" << endl;//输出使用方法
        return 1;
    }
    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }
 
    // 设定g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;//每个误差项优化变量维度为6，误差值维度为6
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;//线性求解器类型
    // 梯度下降方法，可以从GN, LM, DogLeg 中选
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 图模型
    optimizer.setAlgorithm(solver);   // 设置求解器
    optimizer.setVerbose(true);       // 打开调试输出
 
    int vertexCnt = 0, edgeCnt = 0; // 顶点和边的数量
 
    vector<VertexSE3LieAlgebra *> vectices;
    vector<EdgeSE3LieAlgebra *> edges;
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") //当数据是节点类型VERTEX_SE3:QUAT时
        {
            // 顶点
            VertexSE3LieAlgebra *v = new VertexSE3LieAlgebra();//指针v
            int index = 0;
            fin >> index;
            v->setId(index);//对边进行编号
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;//遍历所有节点
            vectices.push_back(v);
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {
            // SE3-SE3 边
            EdgeSE3LieAlgebra *e = new EdgeSE3LieAlgebra();//指针e
            int idx1, idx2;     // 关联的两个顶点
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);//遍历所有边
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
            edges.push_back(e);
        }
        if (!fin.good()) break;
    }
 
    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;//输出共有多少个顶点和边
 
    cout << "optimizing ..." << endl;//输出optimizing ...优化后
    optimizer.initializeOptimization();//优化过程初始化
    optimizer.optimize(30);//设置优化的迭代次数为30次
 
 
    cout << "saving optimization results ..." << endl;
 
    // 因为用了自定义顶点且没有向g2o注册，这里保存自己来实现
    // 伪装成 SE3 顶点和边，让 g2o_viewer 可以认出
    ofstream fout("result_g2o_lie.g2o");
    for (VertexSE3LieAlgebra *v:vectices) {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout);
    }
    for (EdgeSE3LieAlgebra *e:edges) {
        fout << "EDGE_SE3:QUAT ";
        e->write(fout);
    }
    fout.close();
    return 0;
}