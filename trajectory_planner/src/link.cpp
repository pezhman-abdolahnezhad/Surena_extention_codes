#include "../include/trajectory_planner/link.h"


link::link(short int ID, Vector3d a, Vector3d b, link* parent){
    this->ID_ = ID;
    this->a_ = a;
    this->b_ = b;
    this->parent_ = parent;
    cout << "link object has been created"<<endl;
}

void link::setQ(float q){
    this->q_ = q;
}

Matrix3d link::hat(Vector3d omega){
    Matrix3d omegaHat;
    omegaHat<<0.0,-omega(2),omega(1),
              omega(2),0.0,-omega(0),
              -omega(1),omega(0),0.0;
    return omegaHat;
}

Matrix3d link::rodrigues(Vector3d omega, float q){
    Matrix3d omegaHat = this->hat(omega);
    Matrix3d result = Matrix3d::Identity(3, 3) + sin(q)*omegaHat + (omegaHat*omegaHat)*(1-cos(q));
    return result;
}

Matrix4d link::forwardKin(){
    if (this->ID_ == 0){
        this->p_ << 0.0, 0.0, 0.0;
        this->R_ << 1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0;
    T = MatrixXd::Identity(4,4) ;
    return T;
    }
    else{
        this->parent_->forwardKin();
        this->p_ = this->parent_->R_ * this->b_ + this->parent_->p_;
        this->R_ = this->parent_->R_ * rodrigues(this->a_, this->q_);    
    }
    
    T << this->R_(0, 0), this->R_(0, 1), this->R_(0, 2), this->p_(0),
         this->R_(1, 0), this->R_(1, 1), this->R_(1, 2), this->p_(1),
         this->R_(2, 0), this->R_(2, 1), this->R_(2, 2), this->p_(2),
         0.0, 0.0, 0.0, 1;
    return T;
}
/*
int main(int argc, char** argv){

    cout<<"mamad"<<endl;
    Vector3d a1(0.0, 0.0, 1.0);
    Vector3d a2(1.0, 0.0, 0.0);
    Vector3d a3(0.0, 1.0, 0.0);
    Vector3d a4(0.0, 1.0, 0.0);
    Vector3d a5(0.0, 1.0, 0.0);
    Vector3d a6(1.0, 0.0, 0.0);
    Vector3d a7(0.0, 0.0, 1.0);
    Vector3d a8(1.0, 0.0, 0.0);
    Vector3d a9(0.0, 1.0, 0.0);
    Vector3d a10(0.0, 1.0, 0.0);
    Vector3d a11(0.0, 1.0, 0.0);
    Vector3d a12(1.0, 0.0, 0.0);
    Vector3d b1(0.0, -0.115, 0.0);
    Vector3d b2(0.0, 0.0, 0.0);
    Vector3d b3(0.0, 0.0, 0.0);
    Vector3d b4(0.0, 0.0, -0.36);
    Vector3d b5(0.0, 0.0, -0.37);
    Vector3d b6(0.0, 0.0, 0.0);
    Vector3d b7(0.0, 0.115, 0.0);
    Vector3d b8(0.0, 0.0, 0.0);
    Vector3d b9(0.0, 0.0, 0.0);
    Vector3d b10(0.0, 0.0, -0.36);
    Vector3d b11(0.0, 0.0, -0.37);
    Vector3d b12(0.0, 0.0, 0.0);
    link pelvis(0 , VectorXd::Ones(3), VectorXd::Ones(3), nullptr);
    link hipYawR(1, a1, b1, &pelvis);
    hipYawR.setQ(0.0);
    link hipRollR(2, a2, b2, &hipYawR);
    hipRollR.setQ(0.0);
    link hipPitchR(3, a3, b3, &hipRollR);
    hipPitchR.setQ(0.0);
    link kneeR(4, a4, b4, &hipPitchR);
    kneeR.setQ(0.0);
    link anklePitchR(5, a5, b5, &kneeR);
    anklePitchR.setQ(0.0);
    link ankleRollR(6, a6, b6, &anklePitchR);
    ankleRollR.setQ(0.0);
    link hipYawL(7, a7, b7, &pelvis);
    hipYawL.setQ(0.0);
    link hipRollL(8, a8, b8, &hipYawL);
    hipRollL.setQ(0.0);
    link hipPitchL(9, a9, b9, &hipRollL);
    hipPitchL.setQ(0.0);
    link kneeL(10, a10, b10, &hipPitchL);
    kneeL.setQ(0.0);
    link anklePitchL(11, a11, b11, &kneeL);
    anklePitchL.setQ(0.0);
    link ankleRollL(12, a12, b12, &anklePitchL);
    ankleRollL.setQ(0.0);
    Matrix4d mamad;
    mamad = ankleRollL.forwardKin();
    cout << mamad << endl;
    return 0;
}*/