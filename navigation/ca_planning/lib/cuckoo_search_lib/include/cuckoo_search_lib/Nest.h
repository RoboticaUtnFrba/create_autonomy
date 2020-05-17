/*
	Description:
		This class saves the current solution and information for
		getting next solution.
*/


#ifndef NEST
#define NEST

#include "FunctionHelper.h"

#include <valarray>
#include <vector>
#include <random>
#include <exception>
#include <stdexcept>
#include <algorithm>
#include <fstream>
#include <iomanip>

using Egg = std::valarray<double>;

class Nest
{
public:
	Nest() {};
	Nest(const ObjectiveFunction& func, double lambda = 0.3);
	Nest(const ObjectiveFunction& func, const Egg& host_nest, double lambda = 0.3);
	Nest(const ObjectiveFunction& func, const Bounds& bounds, const Egg& host_nest, double lambda = 0.3);
	Nest(const ObjectiveFunction& func, const std::vector<Bounds>& bounds, const Egg& host_nest, double lambda = 0.3);
	Nest& operator=(const Nest& nest);
	Nest(const Nest& nest);

	bool operator<(const Nest& rs) const;
	bool operator>(const Nest& rs) const;
	bool operator<=(const Nest& rs) const;
	bool operator>=(const Nest& rs) const;

	inline double GetFitness() const { return m_fitness; };
	inline std::valarray<double> GetSolutions() const { return m_solutions; };
	inline std::valarray<double> GetAlpha() const { return m_alpha; };
	inline double GetLambda() const { return m_lambda; };
	inline std::vector<Bounds> GetBounds() const { return m_bounds; };

	inline void SetBounds(const std::vector<Bounds>& bounds) { m_bounds = bounds; };
	void SetLambda(double lambda);
	void SetAlpha(double alpha);
	void SetAlpha(const std::valarray<double>& alpha);
	void SetBounds(const Bounds& bounds) { m_bounds = std::vector<Bounds>(m_solutions.size(), bounds); };

	friend std::ostream& operator<<(std::ostream& stream, Nest& nest);

private:
	Egg						m_solutions;
	double					m_fitness;
	double					m_lambda;
	std::valarray<double>	m_alpha;
	std::vector<Bounds>		m_bounds;

	void GenerateInitialSolutions();
	void BoundedSolutions();
};

using CompareFitness = std::function<bool(const Nest&, const Nest&)>;

#endif // !NEST
