// EIC.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <Eigen/Dense>
#include <iostream>
#include "pendulum_plant.h"
#include "eiccontrol.h"
#include <fstream>
using namespace std;
using Eigen::Vector4d;
using Eigen::MatrixXd;
int main()
{
	double dt = 0.01;
	pendulum_plant pendulum;
	// x=[theta alpha dtheta dalpha];
	Vector4d x_ini(0.1, 0.1, 0.1, 0.1);
	Vector4d x = x_ini;
	// initialize desired trjectory
	desired_trj trj(0.5,1);
	// initialize controller
	eiccontrol controller(pendulum,trj);
	double ddtheta_design, angle_bem, control_input;
	Vector4d dxdt;
	int steps = 6000;
	MatrixXd x_collect(4,steps);
	MatrixXd desired_trj_collect(2, steps);
	MatrixXd bem_collect(1, steps);
	MatrixXd time_collect(1, steps);
	for (int i = 0; i < steps; i++) {
		time_collect(i) = i*dt;
		 x_collect.col(i) = x;
		 desired_trj_collect(0, i) = trj.get_desired_trj(i*dt);
		 desired_trj_collect(1, i) = trj.get_desired_trj_1dot(i*dt);

		 ddtheta_design = controller.external_track_control(x, i*dt, 4.0, 1.8);
		 angle_bem = controller.solveBEM(x, ddtheta_design);
		 bem_collect(i) = angle_bem;
		 control_input = controller.internal_stablize_BEM(x, angle_bem, 50, 10);
		 dxdt = pendulum.simulate(x, control_input);
		 x = x + dxdt*dt;
		// cout <<x_collect.col(i) << endl;
	}
	
	ofstream output_file("sim.txt",ios::out|ios::trunc );
	output_file << x_collect << endl;
	output_file << desired_trj_collect << endl;
	output_file << bem_collect<<endl;
	output_file << time_collect;
	return 0;
}

