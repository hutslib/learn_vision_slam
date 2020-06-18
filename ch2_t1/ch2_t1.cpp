#include <iostream>
using namespace std;
#include <Eigen/Core>
#include <Eigen/Dense>
#define MATRIX_SIZE 100
int main(int argc, char** argv)
{
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> Matrix_A;
    Eigen::Matrix<double, MATRIX_SIZE, 1> Matrix_b;
    Eigen::Matrix<double, MATRIX_SIZE, 1> x;
    //增广矩阵
    Eigen::Matrix<double, MATRIX_SIZE, 1 + MATRIX_SIZE> Matrix_B;
     while(true)
     {
        //生成 Matrix_A * x = Matrix_b
        //Matrix_A<<1,0,0,1,0,0,1,0,0;
        Matrix_A = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
        Matrix_A = Matrix_A*Matrix_A.transpose();//保证半正定，保证为实对称 cholesky需要对称正定矩阵
        //Matrix_b<<1,2,6;
        Matrix_b = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);
        Matrix_B << Matrix_A, Matrix_b;
        //cout << Matrix_B << endl;
        //判断矩阵解的情况
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(Matrix_A);
        Eigen::Index rank_A = svd_A.rank();
        cout << "the rank of Matrix_A:" << rank_A << endl;
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_B(Matrix_B);
        Eigen::Index rank_B = svd_B.rank();
        cout << "the rank of Matrix_B:" << rank_B << endl;
        if ((rank_A == rank_B) && (rank_A== MATRIX_SIZE))
            break;
    }
    // 直接求逆
    x = Matrix_A.inverse()*Matrix_b;
    cout<<"result:  "<<x.transpose()<<endl;

    //使用cholesky分解方法求解方程
    x = Matrix_A.ldlt().solve(Matrix_b);
    cout<<"using cholesky:  "<<x.transpose()<<endl;
//    cout<<Matrix_A*x<<endl;

    //使用QR分解方法求解方程
    x = Matrix_A.colPivHouseholderQr().solve(Matrix_b);
    cout<<"using QR:  "<<x.transpose()<<endl;
//    cout<<Matrix_A*x<<endl;

}