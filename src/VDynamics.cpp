//
// Created by Ethan on 9/27/21.
//

#include "VDynamics.h"


VehicleDynamics::VehicleDynamics(float m, float Iz, float caf, float car,
                                 float vx,float lf, float lr,Vec x0, float dt):
                                 m(m),Iz(Iz),caf(caf),car(car),
                                 vx(vx),lf(lf),lr(lr),x0(x0), dt(dt){
                                    A=Mat::Zero(4,4);
                                    B=Vec::Zero(4);
                                    C=Mat::Zero(4,4);
                                    B_psiDot=Vec::Zero(4);
                                    x=x0;
                                    I.setIdentity(4,4);
                                 }

void VehicleDynamics::VehicleModel(float delta,float PsiDotDes){
    A(0,1)=1;
    
    A(1,1)=-2*(caf+car)/(m*vx);
    A(1,2)=2*(caf+car)/(m);
    A(1,3)=-2*(caf*lf-car*lr)/(m*vx);
    
    A(2,3)=1;

    A(3,1)=-2*(caf*lf-car*lr)/(Iz*vx);
    A(3,2)=2*(caf*lf-car*lr)/(Iz);
    A(3,3)=-2*(caf*lf*lf+car*lr*lr)/(Iz*vx);


    B(1)=2*caf/m;
    B(3)=2*caf*lf/Iz;

    B_psiDot(1)=(-2*(caf*lf-car*lr)/(m*vx))-vx;
    B_psiDot(3)=-2*(caf*lf*lf-car*lr*lr)/(Iz*vx);

    x=(A*dt+I)*x+(B*delta*dt)+(B_psiDot*psiDotDes*dt);

}

Vec VehicleDynamics::getter_states(){
    return x;
}


