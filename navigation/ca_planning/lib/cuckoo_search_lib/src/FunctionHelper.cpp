#include "cuckoo_search_lib/FunctionHelper.h"

#include <iostream>


ObjectiveFunction::ObjectiveFunction(std::function<double(std::valarray<double>)> function, unsigned int dimensions, Bounds bounds, std::string function_name) :
	m_function(function), m_dimensions(dimensions), m_function_name(function_name)
{
	m_bounds = std::vector<Bounds>(m_dimensions, bounds);
};

ObjectiveFunction::ObjectiveFunction(std::function<double(std::valarray<double>)> function, unsigned int dimensions, std::vector<Bounds> bounds, std::string function_name):
	m_function(function), m_dimensions(dimensions), m_bounds(bounds), m_function_name(function_name) { };

double ObjectiveFunction::operator()(const std::valarray<double>& args) const
{
	if (args.size() != m_dimensions) {
		const std::string ex("Dimensions in current function and amount of args isn't equal\n");
		// throw std::exception(ex);
		std::cout << ex << std::endl;
	}

	if (args.size() != m_bounds.size()) {
		const std::string ex("The number of bounds isn't equal amount of args\n");
		// throw std::exception(ex);
		std::cout << ex << std::endl;
	}

	return m_function(args);
};

void ObjectiveFunction::SetDimensions(unsigned int new_dimension)
{
	m_dimensions = new_dimension;
	m_bounds = std::vector<Bounds>(m_dimensions, m_bounds[0]);
};

