#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    //生成四元数
    Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2);
    //四元数归一化***
    q1.normalize();
    //cout<<q1.coeffs().transpose()<<endl;//coeffs的输出顺序x,y,z,w
    Eigen::Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
    q2.normalize();
    //生成变换矩阵
    Eigen::Isometry3d T_wr1=Eigen::Isometry3d::Identity();
    T_wr1.rotate (q1);
    T_wr1.pretranslate (Eigen::Vector3d (0.7, 1.1, 0.2));
    Eigen::Isometry3d T_r1w=T_wr1.inverse();
    cout << "Transform matrix T_r1w= \n" << T_r1w.matrix() <<endl;
    Eigen::Isometry3d T_wr2=Eigen::Isometry3d::Identity();
    T_wr2.rotate(q2);
    T_wr2.pretranslate(Eigen::Vector3d (-0.1, 0.4, 0.8));
    cout << "Transform matrix T_wr2= \n" << T_wr2.matrix() <<endl;
    //生成点
    Eigen::Matrix<double,3,1> p_r1(0.5, -0.1, 0.2);
    Eigen::Matrix<double,3,1> p_r2=T_wr2*T_r1w*p_r1;
    cout<<"point in coordinate 2: \n"<<p_r2.transpose()<<endl;


}