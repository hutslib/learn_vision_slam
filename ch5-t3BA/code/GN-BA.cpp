#include <Eigen/Core>
#include <Eigen/Dense>
#include<fstream>
using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream p3dfile;
    ifstream p2dfile;
    p3dfile.open(p3d_file);
    p2dfile.open(p2d_file);
    if(!p3dfile.is_open()|!p2dfile.is_open())
    {
        cout << "Open file failure." << endl;
        return 1;
    }
    while(true)
    {
        double d[5];
        for(int i =0; i<2; i++)
        {
            p2dfile>>d[i];
        }
        for(int i=0; i<3; i++)
        {
            p3dfile>>d[i+2];
        }
        if(p2dfile.eof()|p3dfile.eof())
            break;
        Eigen::Vector2d p2(d[0],d[1]);
        Eigen::Vector3d p3(d[2],d[3],d[4]);
        p2d.push_back(p2);
        p3d.push_back(p3);
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti(Matrix3d::Identity(), Vector3d::Zero()); // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            Eigen::Vector3d pc(T_esti*p3d[i]);
//            cout<<pc<<endl;
            Eigen::Vector3d pu3(K*pc);
            Eigen::Vector2d pu2(pu3(0)/pu3(2),pu3(1)/pu3(2));
            Eigen::Vector2d e = pu2-p2d[i];
     	    // END YOUR CODE HERE

	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            double x = pc(0);
            double y = pc(1);
            double z = pc(2);
            double z_2 = z*z;
            // se3 默认平移在前，旋转在后！
            J <<-(fx/z), 0, fx*x/z_2,  fx*x*y/z_2, -(fx+fx*x*x/z_2), fx*y/z,
                    0, -fy/z, fy*y/z_2,fy+fy*y*y/z_2, -fy*x*y/z_2, -fy*x/z;
            // END YOUR CODE HERE
//            cout<<J<<endl;
            H += J.transpose() * J;
            b += -J.transpose() * e;
            cost +=  e.transpose()*e;
        }

	// solve dx 
        Vector6d dx;

        // START YOUR CODE HERE
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout <<"cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE
        T_esti = Sophus::SE3::exp(dx)*T_esti;

        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
