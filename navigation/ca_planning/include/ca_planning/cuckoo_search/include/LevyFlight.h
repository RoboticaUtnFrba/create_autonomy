/*
	Description:
		Function produce stochastic values, which distributed by Levy stable distribution.
		For this purpose used Mantegna algorithm.
*/

#ifndef LEVY_FLIGHT
#define LEVY_FLIGHT

#include <valarray>
#include <map>
#include <utility>

#include <tbb/parallel_for.h>
#include <tbb/critical_section.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include <time.h>

struct StochasticBuffer
{
	StochasticBuffer(double number, bool use) :
		second_number(number), use_second(use) {};
	double second_number;
	bool use_second = false;
};

class LevyFlight
{
public:
	static std::valarray<double> GetValue(double lambda, unsigned int dimension = 1);
protected:
	static double GetNormalDistribution(double mue, double sigma);
	static std::pair<double, double> BoxMullerTransform(double mue, double sigma);
private:
	LevyFlight() = delete;
	LevyFlight(LevyFlight&) = delete;
	LevyFlight& operator=(LevyFlight&) = delete;
};

#endif // !LEVY_FLIGHT

