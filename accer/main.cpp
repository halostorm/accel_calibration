#include<iostream>
#include "glog/logging.h"
#include "ceres/ceres.h"
#include <vector>
#include <fstream>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace std;

struct EllipsoidResidual
{
    /*
     * x, y, z 分别为加速度计的三轴观测值
    */
    EllipsoidResidual(double x, double y, double z):_x(x), _y(y), _z(z){}

    /*
     *pEllipsoidParameters：0-2分别为a1、a2、a3,    3-5分别为b1、b2、b3
    */
    template <typename T> bool operator () (const T * const pEllipsoidParameters,  T * residual) const
    {
        residual[0] =  (pEllipsoidParameters[0]*T(_x )+pEllipsoidParameters[3])* (pEllipsoidParameters[0]*T(_x )+pEllipsoidParameters[3])+
           (pEllipsoidParameters[1]*T(_y)+pEllipsoidParameters[4])*(pEllipsoidParameters[1]*T(_y)+pEllipsoidParameters[4]) +
            (pEllipsoidParameters[2]*T(_z )+ pEllipsoidParameters[5])* (pEllipsoidParameters[2]*T(_z )+ pEllipsoidParameters[5])-T(9.7893*9.7893) ;
        return true;
    }
private:
    const double _x;
    const double _y;
    const double _z;
};

struct Point3D
{
    double x;
    double y;
    double z;
};

/*
 * 用于解算加速度计校准参数的类
 * m_EllipsoidParameters：参数的迭代初始值
 * m_bInitParameters:参数是否已经进行初始化
 * m_data：观测值
 * m_options：解算方式选项，可以进行设置
 * m_summary：解算的报告信息
 *
 * 使用方式：
 * （）设置参数的初始值（m_EllipsoidParameters），并标记m_bInitParameters为true
 * （）设置观测值m_data(个数大于)
 * （）调用SolveParameters()函数
 * （）得到结果m_EllipsoidParameters与校准报告信息m_summary
*/
struct EllipsoidFittingSolver
{
    EllipsoidFittingSolver()
    {
        m_options;
        m_options.max_num_iterations = 1000;
        m_options.linear_solver_type = ceres::DENSE_QR;
        m_options.minimizer_progress_to_stdout = true;
        m_bInitParameters = true;
    }

    bool SolveParameters()
    {
        if (m_data.size() < 6 || !m_bInitParameters)
        {
            return false;
        }

        Problem problem;
        const int nObservations = m_data.size();
        for (int i = 0; i < nObservations; ++i)
        {
            problem.AddResidualBlock(new AutoDiffCostFunction<EllipsoidResidual, 1, 6>(
                new EllipsoidResidual(m_data[i].x, m_data[i].y, m_data[i].z)),
                NULL,
                m_EllipsoidParameters);
        }

        Solve(m_options, &problem, &m_summary);
        return true;
    }

    bool                        m_bInitParameters;
    double                      m_EllipsoidParameters[6];
    std::vector<Point3D>        m_data;
    Solver::Options             m_options;
    Solver::Summary             m_summary;
};


int CountLines(char *filename)
{
    ifstream ReadFile;
    int n = 0;
    char line[512];
    ReadFile.open(filename, ios::in);//ios::in?表示以只读的方式读取文件
    if (ReadFile.fail())//文件打开失败:返回0
    {
        return 0;
    }
    else//文件存在
    {
        while (!ReadFile.eof())
        {
            ReadFile.getline(line, 512, '\n');
            n++;
        }
        return n;
    }
    ReadFile.close();
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);

    EllipsoidFittingSolver efs;
    double dInit[6] = {1,1,1
                       ,0.5,0.5,0.5};
    memcpy(efs.m_EllipsoidParameters, dInit, sizeof(double)*6);
    efs.m_bInitParameters = true;

  fprintf(stderr,"ReadFile begins.\n");
    ifstream infile;  //打开文件

        infile.open("E:/accer校准/accer/4l.txt");
        if (!infile) {
            std::cout << "Opening the file failed!\n";
            exit(1);
        }
        int nCount=-1;
        nCount+=CountLines("E:/accer校准/accer/4l.txt");
        if(nCount<0)
        {
            fprintf(stderr,"ReadFile Error!\n");
            return -1;
        }
        else
            fprintf(stderr,"ReadFile Completed.!\n");
        std::cout << "nCount为" << CountLines("E:/accer校准/accer/4l.txt") << endl;
        double *dbSrcData = new double[nCount*3]; //为动态数组分配内存

            for (int i = 0; i < nCount*3; i++) {

                infile >> dbSrcData[i]; //存Acc第一个姿态数据的二维数组
            }

            infile.close();

    for (int i=0; i<nCount; i++)
    {
        Point3D pt;
        pt.x = dbSrcData[i*3+0];
        pt.y = dbSrcData[i*3+1];
        pt.z = dbSrcData[i*3+2];

        efs.m_data.push_back(pt);
}

    free(dbSrcData);

    if(efs.SolveParameters())
    {
        fprintf(stdout,"Solver Success!\n");

        fprintf(stdout,"a1=%lf a2=%lf a3=%lf\n",efs.m_EllipsoidParameters[0],efs.m_EllipsoidParameters[1],efs.m_EllipsoidParameters[2]);
        fprintf(stdout,"b1=%lf b2=%lf b3=%lf\n",efs.m_EllipsoidParameters[3],efs.m_EllipsoidParameters[4],efs.m_EllipsoidParameters[5]);
        fprintf(stdout,"%s\n",efs.m_summary.FullReport().c_str());
    }
    else
        fprintf(stdout,"Solver Failed!\n");

    return 0;
}
