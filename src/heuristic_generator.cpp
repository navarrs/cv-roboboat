#include "planning.h"

using namespace Planner;

/***********************************************************
						 HEURISTICS
************************************************************/
/* 	@method: get_delta(...)
	@brief: gets the absolute value of src - dst */
coord Heuristic::get_delta(coord src, coord dst) {
	return {abs(src.r - dst.r), abs(src.c - dst.c)};
}

/* 	@method: get_delta(...)
	@brief: gets the manhattan distance between s
			rc and dst. */
uint Heuristic::manhattan(coord src, coord dst) {
	 auto delta = std::move(get_delta(src, dst));
	 return (uint) (delta.r + delta.c);
}

/* 	@method: get_delta(...)
	@brief: gets the euclidian distance between s
			rc and dst. */
uint Heuristic::euclidian(coord src, coord dst) {
	 auto delta = std::move(get_delta(src, dst));
	 return (uint) (sqrt((pow(delta.r, 2) + pow(delta.c, 2))));
}

/* 	@method: get_delta(...)
	@brief: gets the octagonal distance between src and dst */
uint Heuristic::octagonal(coord src, coord dst) {
	auto delta = std::move(get_delta(src, dst));
	return (uint) ((delta.r + delta.c) - std::min(delta.r, delta.c));
}