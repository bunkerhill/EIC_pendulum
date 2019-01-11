#ifndef DESIRED_TRJ_H
#define DESIRED_TRJ_H

class desired_trj
{
public:
	desired_trj(double, double);
	~desired_trj();
	double get_Amp();
	double get_Omega();
	double get_desired_trj(double t);
	double get_desired_trj_1dot(double t);
	double get_desired_trj_2dot(double t);

private:
	double amp;
	double omega;

};

desired_trj::desired_trj(double Amp, double Omega)
{
	this->amp = Amp;
	this->omega = Omega;
}

desired_trj::~desired_trj()
{
}


double desired_trj::get_Amp() {
	return amp;
}

double desired_trj::get_Omega() {
	return omega;
}

double desired_trj::get_desired_trj(double t) {
	return amp*sin(omega*t);
}

double desired_trj::get_desired_trj_1dot(double t) {
	return amp*cos(omega*t)*omega;
}

double desired_trj::get_desired_trj_2dot(double t) {
	return -amp*sin(omega*t)*omega*omega;
}
#endif // !DESIRED_TRJ_H
