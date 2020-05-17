/*
	Description:
		Objective function class contains function, number of function dimensions, its bounds and name.
*/

#ifndef FUNCTION_HELPER
#define FUNCTION_HELPER

#include <functional>
#include <valarray>
#include <vector>
#include <exception>
#include <string>

using StopCritearian = std::function<bool()>;

using StatisticsHandler = std::function<void()>;

struct Bounds
{
	double lower_bound;
	double upper_bound;
};

class ObjectiveFunction
{
public:
	ObjectiveFunction() { };
	ObjectiveFunction(std::function<double(std::valarray<double>)> function, unsigned int dimensions, Bounds bounds, std::string function_name = "NaN");
	ObjectiveFunction(std::function<double(std::valarray<double>)> function, unsigned int dimensions, std::vector<Bounds> bounds, std::string function_name = "NaN");

	double operator()(const std::valarray<double>& args) const;

	void SetDimensions(unsigned int new_dimension);
	inline void SetName(std::string new_name) { m_function_name = new_name; }
	inline void ChangeFunction(std::function<double(std::valarray<double>)> new_function) { m_function = new_function; };
	inline void SetBounds(Bounds bounds) { m_bounds = std::vector<Bounds>(m_dimensions, bounds); };
	inline void SetBounds(std::vector<Bounds> bounds){ m_bounds = bounds; };

	inline unsigned int GetNumberOfDimensions() const { return m_dimensions; };
	inline std::function<double(std::valarray<double>)> GetFunction() const { return m_function; };
	inline std::vector<Bounds> GetBounds() const { return m_bounds; };
	inline std::string GetName() const { return m_function_name; };

private:
	std::function<double(std::valarray<double>)>	m_function;
	unsigned int m_dimensions;
	std::vector<Bounds> m_bounds;
	std::string 	m_function_name;
};

#endif // !FUNCTION_HELPER

