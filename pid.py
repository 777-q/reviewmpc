#include<iostream>
#include<vector>
#include<cmath>
using namespace std;
class Vehicle{
public:
    Vehicle(double position,double v,double a):position(position),v(v),a(a){};
    void updatestate(double a,double dt){
        v+=a*dt;
        position+=v*dt;
    }
    double getposition(){
        return position;
    }
    double getvelocity(){
        return v;
        
    }
    double geta(){
        return a;
    }
private:
    double position;
    double v;
    duuble a;
};
class pidcontroller{
public:
    pidcontroller(double kp,double ki,double kd):kp(kp),ki(ki),kd(kd){};
    double output_acel(double target_v,double velocity,double dt){
        double error=target_v-velocity;
        inter=min(maxinter,inter+error*dt);
        double derive=(error-last_error)/dt;
        double output_a=kp*error+ki*inter+kd*derive;
        last_error=error;
        return output_a; 
    }
private:
    double kp;
    double ki;
    double kd;
    double inter=0.0;
    double last_erro=0.0;
    double maxinter=1.0;
};
class pidcontroller2{
public:
    pidcontroller(double kp,double ki,double kd):kp(kp),ki(ki),kd(kd){};
    double output_acel(double target_position,double position,double dt){
        double error=target_position-position;
        inter=min(maxinter,inter+error*dt);
        double derive=(error-last_error)/dt;
        double output_v=kp*error+ki*inter+kd*derive;
        last_error=error;
        return output_v; 
    }
private:
    double kp;
    double ki;
    double kd;
    double inter=0.0;
    double last_erro=0.0;
    double maxinter=1.0;
};
int main(){
    Vehicle Veh(0.0,0.0,0.0);
    double target_position=5.0;
    double target_velocity=5.0;
    pidcontroller2 pidv(1.0,1.0,1.0);
    pidcontroller pida(1.0,1.0,1.0);
    double dt=0.1;
    for(int i=0;i<100;i++){
        double position=Veh.getposition();
        double currentv=Veh.getvelocity();
        double output_v=pidv.output_acel(target_position,position,dt);
        double accel=pida.output_acel(target_velocity+output_v,currentv,dt);
        Veh.updatestate(accel,dt);
        cout<<i<<position<<output_v<<endl;
    }
    return 0;
}
