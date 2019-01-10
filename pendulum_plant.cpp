#include "stdafx.h"
#include "pendulum_plant.h"


pendulum_plant::pendulum_plant() {
	
    Kg = 70;
	Jr = 0.000998291014166666;
	Jp = 0.00119873080145833;
	Dp = 0.0024;
	Dr = 0.0024;
	Lp = 0.3365;
	Lr = 0.2159;
	Mp = 0.1270;
	Mr = 0.2570;
	Rm = 2.6000;
	g = 9.81;
	km = 0.0077;
	kt = 0.0077;
	
};


Vector4d pendulum_plant::simulate(Vector4d x, double u ) {
	double theta = x(0), alpha = x(1), dtheta = x(2), dalpha = x(3);
	
	//  ddq=M^{-1}(Bu-C-G)
	Vector2d B = get_B();
	Vector2d Tau=B*u;
	Matrix2d M = get_Mass(x);
	Vector2d C = get_C(x);
	Vector2d G = get_G(x);
	Vector2d acc = M.colPivHouseholderQr().solve(Tau-G-C);

	Vector4d dx_dt(dtheta,dalpha,acc(0),acc(1));
	return dx_dt;
};


Matrix2d pendulum_plant::get_Mass(Vector4d x) {

	double theta = x(0), alpha = x(1), dtheta = x(2), dalpha = x(3);

	Matrix2d M(2, 2);
	M(0, 0) = Mp*Lr*Lr + 0.25*Mp*Lp*Lp - 0.25* Mp*Lp*Lp * cos(alpha)*cos(alpha) + Jr;
	M(0, 1) = -0.5 * Mp*Lp*Lr*cos(alpha);
	M(1, 0) = M(0, 1);
	M(1, 1) = Jp + 0.25 * Mp*Lp*Lp;

	return M;
};


Vector2d pendulum_plant::get_G(Vector4d x) {
	double alpha = x(1);
	Vector2d G(0, -0.5*Mp*Lp*g*sin(alpha) );
	return G;
};

Vector2d pendulum_plant::get_C(Vector4d x) {
	double theta = x(0), alpha = x(1), dtheta = x(2), dalpha = x(3);
	Vector2d C;
	C(0)= 0.5*Mp*Lp*Lp*sin(alpha)*cos(alpha)*dtheta*dalpha + 0.5 * Mp*Lp*Lr*sin(alpha)*dalpha*dalpha
		+ Dr*dtheta + Kg*Kg * kt*km / Rm*dtheta;

	C(1) = -0.25 * Mp*Lp*Lp * cos(alpha)*sin(alpha)*dtheta*dtheta + Dp*dalpha;
	return C;
};

Vector2d pendulum_plant::get_B() {
	Vector2d B(Kg*kt / Rm, 0);
	return B;
};

double pendulum_plant::balance_equilibrium_manifold(Vector4d x, double ddtheta_design) {

	// m21 ddtheta + m22 ddalpha +C_2 +G_2 =0;
	// substitute in ddtheta=ddtheta_design  ddalpha=dalpha=0, solve alpha (approximate)
	//-1 / 2 * Mp*Lp*Lr*cos(0)*ddtheta_design  -1 / 4 * Mp*Lp*Lp * cos(0)*(alpha)*dtheta*dtheta - 1 / 2 * Mp*Lp*g*(alpha)=0
	double dtheta = x(2);
	double denom = (0.25*Mp*Lp*Lp*dtheta*dtheta + 0.5*Mp*Lp*g);
	double nom = -0.5*Mp*Lp*Lr*ddtheta_design;
	double alpha_BEM = -0.5*Mp*Lp*Lr*ddtheta_design/(0.25*Mp*Lp*Lp*dtheta*dtheta+0.5*Mp*Lp*g)  ;
	return alpha_BEM;
};