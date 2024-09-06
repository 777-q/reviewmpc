#include<iostream>
#include<vector>
#include<Eigen>
#include<cmath>
#include<numeric>
#include"mpc_osqp"
using namespace std;
using namespace Eigen;
class Vehicle{
public:
    Vehicle(double x,double y,double theta,double speed,double L):state(4,1),L(L){
        state<<x,y,theta,speed;
    }
    void updatestate(const double& steer,const double& accel,double dt){
        double x=state(0,0);
        double y=state(1,0);
        ...
        x+=speed*cos(theta)*dt;
        ..
        state<<x,y,theta,speed;
        
    }
    MatrixXd getstate(){
        return state;
    }
private:
    MatrixXd state;
    double L;
}；
class Referencepath{
public:
struct Point{
    double x;
    double y;
    double theta;
    double speed;
}
Referencepath(const vector<Point>& points):Points(points){};
int getindex(const MatrixXd& currentstate){
    double dis_min=nueric_limits<double>::max();
    int index=0;
    for(int i=0;i<points.size();i++){
        double dis=(Vector2d(Points[i].x,Points[i].y),Vector2d(currentstate(0,0),currentstate(1,0))).squareNorm();
        if(dis<dis_min){
            dis=dis_min;
            index=i;
        }
        
    }
    
    return index;
}
private:
vector<Point> Points;

};
class Mpccontroller{
public:
    Mpccontroller(int state_dim,int control_dim,int N,double max_steer,double min_steer,double max_accel,double min_accel,
    double max_speed,double min_speed):...{
        matrix_q=MatrixXd::Identity(state_dim,state_dim);
        matrix_r=MatrixXd::Identity(control_dim,control_dim);
    };
    MatrixXd getrefstate(const VectorXd& currentstate,const Referencepath::Point target_point,const Referencepath& points){
        MatrixXd ref_traj(state_dim,N);
        return ref_traj;
    }
    bool solve(const Vehicle& Veh,const Referencepath& refpath,const vector<double>& control_cmd){
        MatrixXd currentstate=Veh.getstate();
        int index=refpath.getindex(currentstate);
        Referencepath::Point target_point=refpath.points[index];
        MatrixXd ref_traj=getrefstate(currentstate,target_point,Points);
        MatrixXd matrix_a=MatrixXd::Zero(state_dim*N,state_dim);
        MatrixXd matrix_b=MatrixXd::Zero(control_dim*N,state_dim);
        for(int i=0i<N;i++){
            
        }
        MatrixXd lower_bound(control_dim,1);
        lower_bound<<nueric_limits<double>:: min(),min_steer;
        MatrixXd upper_bound(control_dim,1);
        MatrixXd lower_state_bound(state_dim,1);
        
        Mpcosqp mpcosqp();
        vector<double> future_state(state_dim*(N+1),0);
        vector<double> control_cmd(control_dim*(N),0);
        if(!mpcosqp.solve(&future_state,&control_cmd))
        {
            cout<<"solve failed"<<endl;
            return false;
        }
        else{
            // Vector<double> control_ouput;
            // control_ouput={control_cmd[0],control_cmd[1]};
            return true;
        }
    }
    
private:
    int state_dim;
    int control_dim;
    int N;
    double max_steer;
    double min_steer;
    double max_accel;
    double min_accel;
    double max_speed;
    double min_speed;
    MatrixXd matrix_q;
    MatrixXd matrix_r;
    
};
int main(){
    vector<Referencepath::Point> path_points={{0,0,0,0},{}
};
Referencepath refpath(path_points);
Vehicle Veh(0.0,0.0,0.0,1.0,2.5);
int state=4;
int control=2;
int N=10;
double dt=0.01;
double max_speed=;

Mpccontroller mpc(state,control,N,);
double t=0.0;
double length=100;
Vector<double> contorl_cmd(control*N,0);
for(;t<length;t++){
    MatrixXd currentstate=Veh.getstate();
    
    if(mpc.solve(Veh,refpath,contorl_cmd)){
        Veh.updatestate(contorl_cmd[0],contorl_cmd[1],dt);
        
    }
    else{
        cout<<"fail"<<endl;
    }
    
}
return 0;
}

