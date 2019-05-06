#include "planning.h"

using namespace Planner;

/*************************************************
				PATH GENERATOR
 *************************************************/
/* Constructor */
PathGenerator::PathGenerator() {
	set_diagonal_movement(false);
	set_heuristic(&Heuristic::manhattan);
	directions = {
		{ 0,  1},  // Right 
		{ 1,  0},  // Down 
		{ 0, -1},  // Left
		{-1,  0},  // Up
        {-1, -1},  // Up-left
        { 1,  1},  // Down-right
        {-1,  1},  // Up-right
        { 1, -1}   // Down-left 
	};
	src = dst = {INT_MAX, INT_MAX};
}

/* 	@method: is_coord_valid(...)
	@brief: checks if coord is out of the world's range. 
	@param: coord
	@returns: 
		true if the coord is within the world's bounds, 
		false otherwise */
bool PathGenerator::is_coord_valid(coord co) {
	return co.r >= 0 && co.r < GRID_WIDTH && co.c >= 0 && co.c < GRID_HEIGHT;
}

/* 	@method: is_coord_blocked(...)
	@brief: checks if coord is an obstacle 
	@param:
		grid (const vector<vector<int> >) map
		x (int) the x coordinate of the coord.
		y (int) the y coordinate of the coord.
	@returns: 
		true if the coord is blocked, 
		false otherwise */
bool PathGenerator::is_coord_blocked(const std::vector<std::vector<int> >&grid, coord co) {
	return grid[co.r][co.c];
}

/* 	@method: is_coord_destination(...)
	@brief: checks if coord is destination
	@param:
		r (int) - r coordinate to compare
		c (int) - c coordinate to compare
		d (coord) - destinatino 
	@returns: 
		true if the coord is blocked, 
		false otherwise */
bool PathGenerator::is_coord_destination(coord s, coord d) {
	return s == d;
}

/* 	@method: set_heuristic(...)
	@brief: sets type of heuristic to use for pathfinding.
			Current heuristics: euclidean, manhattan and
			octagonal.
	@param: 
		h - heuristic_function to use */
void PathGenerator::set_heuristic(heuristic_func h) {
	heuristic = std::bind(h, std::placeholders::_1, std::placeholders::_2); 
}

/* 	@method: set_diagonal_movement(...)
	@brief: sets type of heuristic to use for pathfinding
			current heuristics: euclidean, manhattan and
			octagonal.
	@param:
		enable (bool) - move in 8 directions or 4 */
void PathGenerator::set_diagonal_movement(bool enable_full_movement) {
	num_directions = (enable_full_movement ? 8 : 4);
}

/*************************************************
			   PATH FINDING METHODS
 *************************************************/

/* @method: compute_path(...)
   @brief: Produces final path. */
void PathGenerator::compute_path(std::vector<std::vector<node> > &nodes, std::stack<coord> &path) {
	path = std::stack<coord>();
	coord temp = dst;
	while(!(nodes[temp.r][temp.c].parent == temp)) {
		path.push(temp);
		temp = nodes[temp.r][temp.c].parent;
	}
	path.push(temp);
}

void PathGenerator::a_star_search(std::vector<std::vector<int> > grid, std::stack<coord> &path) {

	// Assert that neither source nor destination are out of range
	if (!is_coord_valid(src)) {
		std::cout << "Source is invalid." << std::endl;
		return;
	}
	if (!is_coord_valid(dst)) {
		std::cout << "Destination is invalid." << std::endl;
		return;
	}

	// Assert that neither source or destination are blocked
	if (is_coord_blocked(grid, src) || is_coord_blocked(grid, dst)) {
		std::cout << "Either source or destination are blocked." << std::endl;
		return;
	}

	// Assert that source is not destination 
	if (is_coord_destination(src, dst)) {
		std::cout << "Already at destination." << std::endl;
		return;
	}

	// Initialize the closed list
	bool closed_list[GRID_HEIGHT][GRID_WIDTH];
	memset(closed_list, false, sizeof(closed_list));

	// Initialize path 
	std::vector<std::vector<node> > nodes;
	int r, c;
	for (r = 0; r < GRID_HEIGHT; r++) {
		std::vector<node> n;
		for (c = 0; c < GRID_WIDTH; c++) {
			n.push_back({{-1,-1}, INT_MAX});
		}
		nodes.push_back(n);
	}

	// Initialize the parameters of starting node
	r = src.r;
	c = src.c;
	nodes[r][c].h = 0;
	nodes[r][c].parent = {r, c};

	node *temp = new node({src, 0}); 
	node_list open_list;
	open_list.insert(temp);
	bool found_dst = false;

	while(!open_list.empty()) {
		// Get top node
		temp = *open_list.begin();
		coord current_coord = temp->parent;

		// Erase from list of open nodes and add it to closed nodes
		open_list.erase(open_list.begin());
		r = current_coord.r;
		c = current_coord.c;
		closed_list[r][c] = true;
		
		// Generate all successors
		uint h_temp;
		for (uint i = 0; i < num_directions; i++) {
			coord move = current_coord + directions[i];

			// Assert that coordinate is valid
			if (is_coord_valid(move)) {
				// If its destination, finish
				if (is_coord_destination(move, dst)) {
					nodes[move.r][move.c].parent = {r, c};
					std::cout << "Found destination" << std::endl;
					found_dst = true;
					compute_path(nodes, path);
					return;
				} else if (!closed_list[move.r][move.c] && !is_coord_blocked(grid, move)) {
				// Otherwise, compute heuristic and insert to open if we get a better heuristic. 
					h_temp = heuristic(move, dst);
					if (nodes[move.r][move.c].h == INT_MAX || nodes[move.r][move.c].h > h_temp) {
						temp = new node({move, h_temp});
						open_list.insert(temp);
						nodes[move.r][move.c].h = h_temp;
						nodes[move.r][move.c].parent = {r, c};
					}
				}
			}
		}
	}

	if (!found_dst) {
		std::cout << "Could not find destination." << std::endl;
		path = std::stack<coord>();
		return;
	}
}

