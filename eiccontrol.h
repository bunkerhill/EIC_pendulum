#ifndef  EICCONTROL_H
#define  EICCONTROL_H

#include "desired_trj.h"
#include "pendulum_plant.h"

class eiccontrol
{
public:
	eiccontrol(pendulum_plant, desired_trj);
	~eiccontrol();
	double external_track_control(Vector4d x, double t, double kp, double kd);
	double solveBEM(Vector4d x, double ddtheta_design);
	double internal_stablize_BEM(Vector4d x, double angle_bem, double kp, double kd);

private:
	desired_trj trj2follow;
	pendulum_plant model;

};






double eiccontrol::external_track_control(Vector4d x, double t,double kp, double kd) 
{
	double y0 = trj2follow.get_desired_trj(t);
	double y1 = trj2follow.get_desired_trj_1dot(t);
	double y2 = trj2follow.get_desired_trj_2dot(t);

	double theta = x(0), dtheta = x(2);
	double ddtheta_design = y2 - kp*(theta-y0) - kd*(dtheta-y1);
	return ddtheta_design;
}


double eiccontrol::solveBEM(Vector4d x, double ddtheta_design)
{   
	double bem=model.balance_equilibrium_manifold(x, ddtheta_design);
	return bem;

}


double eiccontrol::internal_stablize_BEM(Vector4d x, double angle_bem, double kp, double kd) {
	double alpha = x(1), dalpha = x(3);
	double ddalpha_design = -kp*(alpha-angle_bem) - kd*(dalpha-0);
	// inverse dynamics controller
	// q_acc= M^{-1}(Bu-C-G) 
	// q_acc= linear_term *u -  const_term
	Vector2d linear_term = model.get_Mass(x).colPivHouseholderQr().solve(model.get_B());
	Vector2d const_term = model.get_Mass(x).colPivHouseholderQr().solve(model.get_C(x)+model.get_G(x));
	double control_input = (ddalpha_design + const_term(1)) / (linear_term(1));
	return control_input;
}



eiccontrol::eiccontrol(pendulum_plant md , desired_trj trj): model(md), trj2follow(trj)
{
}

eiccontrol::~eiccontrol()
{
}




#endif // ! EICCONTROL_H
