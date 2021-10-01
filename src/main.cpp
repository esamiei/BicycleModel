#include <iostream>
#include<vector>
#include <math.h>  
#include "VDynamics.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;



int main() {

    float m=1573;
    float Iz=2873;
    float lf=1.1;
    float lr=1.58;
    float caf=80000;
    float car=80000;
    float vx=30;
    
    float t0{0};
    float t1{5};
    float dt{0.01};
    Vec x0(4);
    x0<<0,0,0,0;

    std::vector<double>time;
    std::vector<double>cte;
    std::vector<double>PsiDotDes;

    
    VehicleDynamics S1(m,Iz,caf,car,vx,lf,lr,x0,dt);

    Vec K(4);
    K<<1,0.09,1.1,0.1;
    Vec x=x0;   


    for(int i=0;i<std::ceil((t1-t0)/dt);i++){
        time.push_back(i*dt);    
        if (time[i]<1){PsiDotDes.push_back(0);}
        else{PsiDotDes.push_back(vx/1000);}

    float delta=-K.transpose()*x;

    S1.VehicleModel(delta,PsiDotDes[i]);
    x=S1.getter_states();
    
    cte.push_back(x[0]);
    std::cout<<x<<std::endl;
    std::cout<<" \n";
    }
    

    plt::title("Bicycle model system");
    plt::plot(time,cte);
    plt::legend();
    plt::show();


    return 0;
}
