Team name- Lumpus

# Life-long planning

A repository to implement the D* Lite search algorithm in a pacman environment. The agent only sees its neighboring squares,
and can store information about the locations it has visited, and the structure of the walls surrounding those locations.

If we use traditional planning algorithms like A* to plan, we have to run them each time our robot changes its state. This is 
redundant, and can be avoided by using an iterative planning approach like D* lite, which uses information computed in the previous
iterations, reducing the need to recompute.

The objective was to compare the performance of D* Lite with A* search and A* search- Tunnel Vission(TV) to corroborate that D* Lite 
performs better than A* search for big and complex enough mazes as descrobed in [this paper on D* Lite](http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf) 
We present our results in terms of the number of nodes expanded for reaching the goal state and the final score achieved in the Pacman domain. 

Number of nodes expanded by the algorithms discussed above for different mazes sizes and complexities were as tabulated below.
| Maze | D* Lite | A* search | A* search (TV) |
| ------------- | ------------- | ------------- | ------------- |
| bigMaze | 10979 | 54160 | 52085 |
| mediumMaze | 3251 | 8842 | 9481 |
| SmallMaze | 669 | 548 | 582 |
| tinyMaze | 122 | 74 | 46 |

The final score achieved using the three search algortihms and different mazes were-
| Maze | D* Lite | A* search | A* search (TV) |
| ------------- | ------------- | ------------- | ------------- |
| bigMaze | 92 | 100 | 70 |
| mediumMaze | 428 | 420 | 420 |
| SmallMaze | 473 | 473 | 471 |
| tinyMaze | 502 | 502 | 502 |

To run the A* algorithm, execute the following line from inside the search folder

python pacman.py -l bigMaze -z  -p SearchAgent -a fn=astar2,heuristic=manhattanHeuristic

You can replace 'astar2' with 'dstar' or 'astartv' to run those algorithms

To change the maze, replace 'bigMaze' with another maze like 'mediumMaze'

A* search implemetation in Pacman domain video- https://drive.google.com/file/d/1dJxdAXtt_TP36qseuFwgjHao1YwLrx1H/view?usp=sharing

A* search- Tunnel Vision implemetation in Pacman domain video- https://drive.google.com/file/d/12yHSP5GARA5iG13dDJndBeEIUmZIvLVx/view?usp=sharing

D* Lite implemetation in Pacman domain video- https://drive.google.com/file/d/1YwP7NWzpVlkjzU_yhGjMebTtfalB7fww/view?usp=sharing

## Authors
* ** Sagar Khar- Research & Solution formulation **
* ** Laukik Mujumda- Code Implementation **
* ** Malay Tushar Nagda- Testing and Bug fixes **
* ** Prabal Bijoy Dutta- Analysis & Report Generation **
