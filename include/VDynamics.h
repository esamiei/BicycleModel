//
// Created by Ethan on 9/27/21.
//

#ifndef BICYCLEMODEL_LATERALDYNAMIC_VEHICLEDYNAMIC_H
#define BICYCLEMODEL_LATERALDYNAMIC_VEHICLEDYNAMIC_H

#include<iostream>
#include<vector>
#include<Eigen/Dense>


typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;


class VehicleDynamics{
private:
    float m,Iz,vx,caf,car,lf,lr,delta;
    float psiDotDes;
    Mat A;
    Vec B;
    Mat C;
    Mat B_psiDot;
    Mat I;
    Vec x0,x;    //state variables
    float dt;

public:
    VehicleDynamics(float m, float Iz, float caf, float car, float vx, 
                    float lf, float lr, Vec x0, float dt);
    void VehicleModel(float delta,float PsiDotDes);
    Vec getter_states();

};




#endif //BICYCLEMODEL_LATERALDYNAMIC_VEHICLEDYNAMIC_H
