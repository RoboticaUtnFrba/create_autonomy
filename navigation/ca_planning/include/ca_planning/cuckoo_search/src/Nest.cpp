#include "Nest.h"


Nest::Nest(const ObjectiveFunction& func, double lambda)
{
	m_bounds = func.GetBounds();
	GenerateInitialSolutions();
	m_fitness = func(m_solutions);
	m_alpha = std::valarray<double>(1.0, m_solutions.size());
};

Nest::Nest(const ObjectiveFunction& func, const Egg& host_nest, double lambda) :
	 m_solutions(host_nest), m_lambda(lambda)
{
	m_bounds = func.GetBounds();
	BoundedSolutions();
	m_fitness = func(m_solutions);
	m_alpha = std::valarray<double>(1.0, m_solutions.size());
};

Nest::Nest(const ObjectiveFunction& func, const Bounds& bounds, const Egg& host_nest, double lambda):
	m_solutions(host_nest), m_lambda(lambda)
{
	m_bounds = std::vector<Bounds>(m_solutions.size(), bounds);
	BoundedSolutions();
	m_fitness = func(m_solutions);
	m_alpha = std::valarray<double>(1.0, m_solutions.size());
};

Nest::Nest(const ObjectiveFunction& func, const std::vector<Bounds>& bounds, const Egg& host_nest, double lambda):
	m_solutions(host_nest), m_bounds(bounds), m_lambda(lambda)
{
	BoundedSolutions();
	m_fitness = func(m_solutions);
	m_alpha = std::valarray<double>(1.0, m_solutions.size());
};

Nest::Nest(const Nest& nest)
{
	if (&nest == this)
		return;
	m_solutions = nest.m_solutions;
	m_bounds = nest.m_bounds;
	m_alpha = nest.m_alpha;
	m_fitness = nest.m_fitness;
};

Nest& Nest::operator=(const Nest& nest)
{
	if (&nest == this)
		return *this;

	m_solutions = nest.m_solutions;
	m_bounds = nest.m_bounds;
	m_alpha = nest.m_alpha;
	m_fitness = nest.m_fitness;
	return *this;
};

bool Nest::operator<(const Nest& rs) const
{
	return (this->m_fitness < rs.m_fitness);
};

bool Nest::operator>(const Nest& rs) const
{
	return (this->m_fitness > rs.m_fitness);
};

bool Nest::operator<=(const Nest& rs) const
{
	return (*this < rs || this->m_fitness == rs.m_fitness);
};

bool Nest::operator>=(const Nest& rs) const
{
	return (*this > rs || this->m_fitness == rs.m_fitness);
};

void Nest::SetLambda(double lambda)
{
	if ((lambda <= 0.1) || (lambda >= 2))
		throw std::exception("Lambda must be in range [0.1, 1.99]\n");
	m_lambda = lambda;
};

void Nest::SetAlpha(double alpha)
{
	SetAlpha(std::valarray<double>(alpha, m_solutions.size()));
};

void Nest::SetAlpha(const std::valarray<double>& alpha)
{
	if (std::find_if(std::begin(alpha), std::end(alpha), [](double elem) {return (elem <= 0.0); }) != std::end(alpha))
		throw std::exception("Alpha must be more than 0\n");
	m_alpha = alpha;
};

void Nest::GenerateInitialSolutions()
{
	m_solutions = std::valarray<double>(m_bounds.size());
	for (size_t i = 0; i < m_bounds.size(); ++i)
	{
		m_solutions[i] = m_bounds[i].lower_bound + rand() * (m_bounds[i].upper_bound - m_bounds[i].lower_bound) / double(RAND_MAX + 1.0);
	}
};

void Nest::BoundedSolutions()
{
	for (size_t i = 0; i < m_bounds.size(); ++i)
	{
		if (m_solutions[i] < m_bounds[i].lower_bound)
			m_solutions[i] = m_bounds[i].lower_bound;
		if (m_solutions[i] > m_bounds[i].upper_bound)
			m_solutions[i] = m_bounds[i].upper_bound;
	}
};

std::ostream& operator<<(std::ostream& stream, Nest& nest)
{
	//stream << "Fitness: " << nest.m_fitness << "\n";
	stream << "Solution: [";
	stream << std::fixed;
	for (size_t i = 0; i < nest.m_solutions.size() - 1; ++i)
	{
		stream << std::setprecision(6) << nest.m_solutions[i] << ", ";
	}
	stream << nest.m_solutions[nest.m_solutions.size() - 1] << "]\n";
	return stream;
	//stream << "Lambda: " << nest.m_lambda << "\n\n";
	//return stream;
};
