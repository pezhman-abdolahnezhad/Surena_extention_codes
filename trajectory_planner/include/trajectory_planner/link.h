#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <math.h>

using namespace std;
using namespace Eigen;

class link {
    public:
        link (short int ID_, Vector3d a_, Vector3d b_, link* parent_);
        Matrix3d hat(Vector3d omega);
        Matrix3d rodrigues(Vector3d omega, float q);
        Matrix4d forwardKin();
        void setQ(float q);
    private:
        short int ID_;
        Vector3d a_;
        Vector3d b_;
        Vector3d p_;
        float q_;
        float dq_;
        float ddq_;
        Matrix3d R_;
        link* parent_;
        Matrix4d T;
};
