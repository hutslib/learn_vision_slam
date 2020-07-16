#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.h"
#include "sophus/se3.h"
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
using namespace std;

// path to trajectory file
string trajectory_file = "../compare.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>,vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>>);
void ICP(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_est, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_gro, Eigen::Matrix3d & R,Eigen::Vector3d & t);
int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_est;
    vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> poses_gro;
    // implement pose reading code
    // start your code here (5~10 lines)
    ifstream infile;
    infile.open(trajectory_file);
    if (!infile.is_open())
    {
        cout << "Open file failure." << endl;
        return 1;
    }
    while(true)
    {
        double date[16]={0};
        for(auto &d:date)
            infile>>d;
        if(infile.eof())
            break;
        Eigen::Quaterniond q_e(date[7],date[4],date[5],date[6]);
//        cout<<q.coeffs().transpose()<<endl;//coeffs的输出顺序x,y,z,w
        Eigen::Vector3d t_e(date[1], date[2], date[3]);
        Sophus::SE3 SE3_est(q_e, t_e);
        Eigen::Quaterniond q_g(date[15],date[12],date[13],date[14]);
        Eigen::Vector3d t_g(date[9],date[10],date[11]);
        Sophus::SE3 SE3_gro(q_g,t_g);
        poses_est.push_back(SE3_est);
        poses_gro.push_back(SE3_gro);


    }
    // end your code here

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    ICP(poses_est,poses_gro,R,t);
    Sophus::SE3 SE3_update(R,t);
    for(int i=0; i<poses_gro.size(); i++)
    {
        poses_gro[i] = SE3_update * poses_gro[i];
    }
    // draw trajectory in pangolin
     DrawTrajectory(poses_est,poses_gro);

    return 0;
}
void ICP(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_est,vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> poses_gro,Eigen::Matrix3d & R,Eigen::Vector3d & t)
{
    //计算中心值
    Eigen::Vector3d p_c1 = Eigen::Vector3d::Zero(),p_c2 = Eigen::Vector3d::Zero();
    int N_1 =0,N_2=0;
    for(int i =0; i<poses_est.size();i++)
    {
        Eigen::Vector3d p(poses_est[i].translation()[0],poses_est[i].translation()[1],poses_est[i].translation()[2]);
        p_c1 +=p;
        N_1+=1;
    }
    p_c1 = p_c1/N_1;
    for(int i=0; i<poses_gro.size();i++)
    {
        Eigen::Vector3d p(poses_gro[i].translation()[0],poses_gro[i].translation()[1],poses_gro[i].translation()[2]);
        p_c2 +=p;
        N_2+=1;
    }
    p_c2=p_c2/N_2;
    vector<Eigen::Vector3d> position1, position2;
    for(int i=0; i<N_1; i++)
    {
         position1.push_back(poses_est[i].translation()-p_c1);
         position2.push_back(poses_gro[i].translation()-p_c2);
    }
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N_1; i++ )
    {
        W += position1[i] * position2[i].transpose();
    }
    cout<<"W="<<W<<endl;
    //SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d  V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;
    R = U*(V.transpose());
    t= p_c1 - R*p_c2;
}
/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses_est,vector<Sophus::SE3,Eigen::aligned_allocator<Sophus::SE3>> poses_gro) {
    if (poses_est.empty()|poses_gro.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        //red estimatuon
        for (size_t i = 0; i < poses_est.size() -1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses_est[i], p2 = poses_est[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        //blue estimation
        for (size_t i =0; i<poses_gro.size()-1; i++)
        {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = poses_gro[i], p2 = poses_gro[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}