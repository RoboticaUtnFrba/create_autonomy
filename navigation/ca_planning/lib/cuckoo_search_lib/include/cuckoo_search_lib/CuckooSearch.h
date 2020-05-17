/*
	Description:
		This file contains 3 classes:
		Lambda - save minimum and maximum as posible value of lambda.
		Step - save n-dimensional vector with max and min steps.
		CuckooSearch - main class, which implements the same named algorithm
		with two modifications:
		a) managed step (alpha), which related by current
		number of generation and change by this law:
		a(t) = a(0) * delta^t;
		delta = (a(min) / a(max))^(1 / k), where k - maximum number of iterations, t - current iteration.

		b) variable lambda which depends on current cuckoo position - cuckoos with better
		fitness have max lambda and can explore solution more precisely.
		Lambda calculates for formula:
		lambda(i) = lambda(max) - (i * (lambda(max) - lambda(min))) / (m - 1), where
		lambda(i) - is lambda for i-cuckoo, i - cuckoo position, m - amount of nests.
*/


#ifndef CUCKOO_SEARCH
#define CUCKOO_SEARCH

#include "FunctionHelper.h"
#include "LevyFlight.h"
#include "Cuckoo.h"
#include "Nest.h"

#include <functional>
#include <vector>
#include <random>
#include <algorithm>
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>

#include <tbb/parallel_for.h>
#include <tbb/critical_section.h>
#include <tbb/parallel_sort.h>

using SetOfNests = std::vector<Nest>;

class Lambda
{
public:
	Lambda() {};
	Lambda(double lambda) :
		m_min_lambda(lambda), m_max_lambda(lambda) {};
	Lambda(double min_lambda, double max_lambda) :
		m_min_lambda(min_lambda), m_max_lambda(max_lambda) {};

	inline void SetMinLambda(double min_lambda) { m_min_lambda = min_lambda; };
	inline void SetMaxLambda(double max_lambda) { m_max_lambda = max_lambda; };

	inline double GetMinLambda() const { return m_min_lambda; };
	inline double GetMaxLamda() const { return m_max_lambda; };

private:
	double m_min_lambda;
	double m_max_lambda;
};

class Step
{
public:
	Step() {};
	Step(double step, unsigned int size = 1) :
		m_min_step(std::valarray<double>(step, size)), m_max_step(std::valarray<double>(step, size)) {};
	Step(double min_step, double max_step, unsigned int size = 1) :
		m_min_step(std::valarray<double>(min_step, size)), m_max_step(std::valarray<double>(max_step, size)) {};
	Step(std::valarray<double> step) :
		m_min_step(step), m_max_step(step) {};
	Step(std::valarray<double> min_step, std::valarray<double> max_step) :
		m_min_step(min_step), m_max_step(max_step) {};

	inline void SetMinStep(double min_step) { m_min_step = min_step; };
	inline void SetMaxStep(double max_step) { m_max_step = max_step; };
	inline void SetMinStep(std::valarray<double> min_step) { m_min_step = min_step; };
	inline void SetMaxStep(std::valarray<double> max_step) { m_max_step = max_step; };

	inline std::valarray<double> GetMinStep() const { return m_min_step; };
	inline std::valarray<double> GetMaxStep() const { return m_max_step; };

private:
	std::valarray<double> m_min_step;
	std::valarray<double> m_max_step;
};

class CuckooSearch
{
public:
	CuckooSearch(ObjectiveFunction func, unsigned amount_of_nests = 32, Step step = 1.0, Lambda lambda = {0.3, 1.99}, double prob = 0.25,
		unsigned max_generations = 10000, StopCritearian stop_criterian = []() {return true; }, bool use_lazy_cuckoo = false);
	~CuckooSearch();

	std::valarray<double> FindMax();
	std::valarray<double> FindMin();
	std::valarray<double> FindMax(Lambda lambda, Step step, double prob, StopCritearian stop_criterian = []() {return true; });
	std::valarray<double> FindMin(Lambda lambda, Step step, double prob, StopCritearian stop_criterian = []() {return true; });

	inline double GetCurrentBestValue() const { return m_best_ever.GetFitness(); };
	inline Nest GetCurrentBestNest() const { return m_best_ever; };
	inline unsigned GetMaxGenerations() const { return m_max_generations; };
	inline unsigned GetCurrentGeneration() const { return m_current_generation; };
	inline Lambda GetLambda() const { return m_lambda; };
	inline Step GetStep() const { return m_step; };
	inline double GetAbandonProbability() const { return m_abandon_probability; };
	inline ObjectiveFunction GetObjectiveFunction() const { return m_objective_function; };
	inline StopCritearian GetStopCriterian() const { return m_stop_criterian; };
	inline SetOfNests GetCurrentSetOfNests() const { return m_nests; };
	inline unsigned GetNumberOfNests() const { return m_amount_of_nests; };
	inline bool IsLazyCuckoo() const { return m_use_lazy_cuckoo; };

	inline void SetNumberOfNests(unsigned int nests) { m_amount_of_nests = nests; };
	inline void SetStopCriterian(StopCritearian stop_criterian) { m_stop_criterian = stop_criterian; };
	inline void SetLamda(const Lambda& lambda) {m_lambda = lambda;};
	inline void SetStep(const Step& step) { m_step = step; };
	inline void SetAbandonProbability(double probability){ m_abandon_probability = probability; };
	inline void SetStatisticsHandler(StatisticsHandler* handler) { m_statistics_handler = *handler; };
	inline void SetMaxGenerations(unsigned int generations) { m_max_generations = generations; };

	void UseLazyCuckoo();
	void UseStandartCuckoo();

protected:
	unsigned int m_amount_of_nests;
	SetOfNests m_nests;
	Nest m_best_ever;
	Cuckoo* m_cuckoo;
	ObjectiveFunction m_objective_function;
	StopCritearian m_stop_criterian;
	CompareFitness m_cmp_fitness;
	unsigned int m_max_generations;
	unsigned int m_current_generation;

	Lambda m_lambda;
	Step m_step;
	std::valarray<double>	m_delta_step;
	double m_abandon_probability;
	bool m_use_lazy_cuckoo;

	StatisticsHandler		m_statistics_handler;


	void GenerateInitialPopulation();
	std::valarray<double> GetSolution();
	void AbandonNests();
	void RankNests();
	void RecalculateStep();
	void RecalculateLambdas();


};


#endif // !CUCKOO_SEARCH

