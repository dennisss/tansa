#include <tansa/model.h>

/*
	Battery
	-------

	Parameters
	- Number of cells
	- Internal resistance

	State
	- Internal voltage
	- Terminal voltage
	- Current


	A battery also has an array of loads which we can model as resistances
	-> The terminal voltage is defined by the loads


	We have the following coefficients x^9 ... x^0 of the terminal voltage:




	Internal resistance of 0.02 ohms
*/


namespace tansa {


Battery::Battery(const DataObject &desc) {



}


void Battery::update(BatteryState &s, const Time &t) {
	// Integrate forward current
}


double Battery::voltage(const BatteryState &s) {

	// coefficients of v(percent = p) = c_0 p^0 + c_1 p^1 + ... of a standard lipo cell
	Matrix<double, 10, 1> coeffs;
	coeffs << 4.19746956e+00, -2.05847935e+00, 2.77923013e+01,
			  -3.12926262e+02, 1.83591641e+03, -6.08866393e+03,
			  1.18914470e+04, -1.35410348e+04, 8.30922936e+03,
			  -2.12224582e+03;





}


}
