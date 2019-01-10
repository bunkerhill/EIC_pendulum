#pragma once

#ifndef PENDULUM_PLANT_H
#define PENDULUM_PLANT_H

#include <Eigen/Dense>
using Eigen::Vector4d;
using Eigen::Vector2d;
using Eigen::Matrix2d;

class pendulum_plant {

private:
	double Kg;
	double Jr  ;
	double Jp  ;
	double Dp  ;
	double Dr  ;
	double Lp  ;
	double Lr  ;
	double Mp  ;
	double Mr  ;
	double Rm  ;
	double g ;
	double km ;
	double kt;


public:
	pendulum_plant();
    Vector4d simulate(Vector4d , double );
	Matrix2d get_Mass(Vector4d);
	Vector2d get_G(Vector4d);
	Vector2d get_C(Vector4d);
	Vector2d get_B();
	double balance_equilibrium_manifold(Vector4d x, double ddtheta_design);
};





#endif
