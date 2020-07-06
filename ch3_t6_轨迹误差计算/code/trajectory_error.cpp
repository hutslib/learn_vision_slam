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
string estimated_file = "../estimated.txt";
string groundtruth_file = "../groundtruth.txt";
typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> TrajectoryType;
void ReadTrajectory(TrajectoryType &,TrajectoryType &);
void DrawTrajectory(const TrajectoryType &,const TrajectoryType &);
double ComputeRmse(const TrajectoryType &, const TrajectoryType &);
int main(int argc, char **argv) {

    TrajectoryType est_poses;
    TrajectoryType gro_poses;
    //Read trajectory date
    ReadTrajectory(est_poses,gro_poses);
    // compute rmse
    cout<<ComputeRmse(est_poses, gro_poses)<<endl;
    // draw trajectory in pangolin
    DrawTrajectory(est_poses,gro_poses);
    return 0;
}
/*******************************************************************************************/
void ReadTrajectory(TrajectoryType &est_poses,TrajectoryType &gro_poses)
{
    ifstream e_file;
    ifstream g_file;
    e_file.open(estimated_file);
    g_file.open(groundtruth_file);
    if (!e_file.is_open()|!g_file.is_open())
    {
        cout << "Open file failure." << endl;
        return;
    }
    while(true)
    {
        //读取estimated的数据
        double e_date[8]={0};
        for(auto &e_d:e_date)
            e_file>>e_d;
        if(e_file.eof())
            break;
        Eigen::Quaterniond e_q(e_date[7],e_date[4],e_date[5],e_date[6]);
//        cout<<q.coeffs().transpose()<<endl;//coeffs的输出顺序x,y,z,w
        Eigen::Vector3d e_t(e_date[1], e_date[2], e_date[3]);
        Sophus::SE3 SE3_est(e_q, e_t);
        est_poses.push_back(SE3_est);

        //读取groundtruth的数据
        double g_date[8]={0};
        for(auto &g_d:g_date)
            g_file>>g_d;
        if(g_file.eof())
            break;
        Eigen::Quaterniond g_q(g_date[7],g_date[4],g_date[5],g_date[6]);
//        cout<<g_q.coeffs().transpose()<<endl;//coeffs的输出顺序x,y,z,w
        Eigen::Vector3d g_t(g_date[1], g_date[2], g_date[3]);
        Sophus::SE3 SE3_gro(g_q, g_t);
        gro_poses.push_back(SE3_gro);

    }
}
/*******************************************************************************************/
void DrawTrajectory(const TrajectoryType &est_poses,const TrajectoryType &gro_poses) {
    if (est_poses.empty() | gro_poses.empty()) {
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
        for (size_t i = 0; i < est_poses.size() - 1; i++) {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = est_poses[i], p2 = est_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < gro_poses.size() - 1; i++) {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = gro_poses[i], p2 = gro_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
/*******************************************************************************************/
double ComputeRmse(const TrajectoryType & est_poses, const TrajectoryType & gro_poses)
{
    double rmse=0;
    for(int i=0; i<est_poses.size(); i++)
    {
        auto est = est_poses[i];
        auto gro = gro_poses[i];
        double error = (est.inverse() * gro).log().norm();
        rmse += error * error;
    }
    rmse = rmse/double(est_poses.size());
    rmse = sqrt(rmse);
    return rmse;
}
