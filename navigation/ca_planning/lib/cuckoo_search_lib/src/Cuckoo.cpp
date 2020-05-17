#include "cuckoo_search_lib/Cuckoo.h"

//Standard fly
Nest Cuckoo::MakeFlight(const Nest& nest)
{
	Egg new_solution = GetNewSolution(nest);
	return Nest(m_function, new_solution, nest.GetLambda());
};

	//** THE BEST RESULT **//
/*	Make fly, divide this path on 2 equal path and check
	3 points (start position, middle and end point), where solution is better	*/
/*Nest Cuckoo::MakeFlight(const Nest& nest)
{
	Egg new_solution = GetNewSolution(nest);
	Egg curr_best = new_solution;
	Egg step = (new_solution - nest.GetSolutions()) / 2.0;
	//#pragma omp parallel for
	for (int i = 0; i < 1; ++i)
	{
		new_solution -= 2.0 * step;
		if (m_function(new_solution) < m_function(curr_best))
		{
			curr_best = new_solution;
		}
	}

	return Nest(m_function, curr_best, nest.GetLambda());
};*/

/*Nest Cuckoo::MakeFlight(const Nest& nest)
{
	Egg curr_best = GetNewSolution(nest);
	double best_value = m_function(curr_best);
	for (int i = 1; i < 10; ++i)
	{
		Egg new_solution = GetNewSolution(nest);
		if (m_function(new_solution) <= best_value)
		{
			best_value = m_function(new_solution);
			curr_best = new_solution;
		}
	}
};*/

Nest Cuckoo::MakeFlight(const Nest& nest, Bounds& bounds)
{
	Egg new_solution = GetNewSolution(nest);
	return Nest(m_function, bounds, new_solution, nest.GetLambda());
};

Nest Cuckoo::MakeFlight(const Nest& nest, std::vector<Bounds>& bounds)
{
	Egg new_solution = GetNewSolution(nest);
	return Nest(m_function, bounds, new_solution, nest.GetLambda());
};

Egg Cuckoo::GetNewSolution(const Nest& nest)
{
	std::valarray<double> old_solution = nest.GetSolutions();
	return old_solution + nest.GetAlpha() * LevyFlight::GetValue(nest.GetLambda(), static_cast<unsigned int>(old_solution.size()));
};

Nest LazyCuckoo::MakeFlight(const Nest &nest)
{
	Egg new_solution = GetNewSolution(nest);
	Egg curr_best = new_solution;
	Egg step = (new_solution - nest.GetSolutions()) / 2.0;
	for (int i = 0; i < 2; ++i)
	{
		new_solution -= step;
		if (m_function(new_solution) < m_function(curr_best))
		{
			curr_best = new_solution;
		}
	}

	return Nest(m_function, curr_best, nest.GetLambda());
};

Nest LazyCuckoo::MakeFlight(const Nest & nest, Bounds & bounds)
{
	return Nest();
};

Nest LazyCuckoo::MakeFlight(const Nest & nest, std::vector<Bounds>& bounds)
{
	return Nest();
};
