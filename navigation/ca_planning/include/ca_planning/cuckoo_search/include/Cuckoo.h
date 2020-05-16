/*
	Description:
		This class gets nest (solution) and make flight via Levy Flights.
*/


#ifndef CUCKOO
#define CUCKOO

#include "LevyFlight.h"
#include "FunctionHelper.h"
#include "Nest.h"

#include <valarray>
#include <vector>


class Cuckoo
{
public:
	Cuckoo(ObjectiveFunction func) :
		m_function(func) {};
	virtual Nest MakeFlight(const Nest& nest);
	virtual Nest MakeFlight(const Nest& nest, Bounds& bounds);
	virtual Nest MakeFlight(const Nest& nest, std::vector<Bounds>& bounds);

	inline ObjectiveFunction GetFunction() const { return m_function; };
	inline void SetFunction(ObjectiveFunction func) { m_function = func; };

protected:
	ObjectiveFunction m_function;

	Egg GetNewSolution(const Nest& nest);
};

class LazyCuckoo : public Cuckoo
{
public:
	LazyCuckoo(ObjectiveFunction func) :
		Cuckoo(func) {};
	virtual Nest MakeFlight(const Nest& nest);
	virtual Nest MakeFlight(const Nest& nest, Bounds& bounds);
	virtual Nest MakeFlight(const Nest& nest, std::vector<Bounds>& bounds);
};

#endif // !CUCKOO

