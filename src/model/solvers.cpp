#include <tansa/model.h>


VectorXd runge_kutta_4(VectorXd (*f)(double, VectorXd), VectorXd y0, double t0, double tn) {

	double h = tn - t0; // The step size
	double halfH = h / 2.0;
	VectorXd k1 = f(t0, y0);
	VectorXd k2 = f(t0 + halfH, y0 + halfH*k1);
	VectorXd k3 = f(t0 + halfH, y0 + halfH*k2);
	VectorXd k4 = f(t0 + h, y0 + h*k3);

	return y0 + (h / 6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4);
}
