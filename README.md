Team name- Lumpus

Topic- Life-long planning

Team members-
Sagar Khar,
Laukik Mujumdar,
Malay Tushar Nagda,
Prabal Bijoy Dutta

Contributions-
Research & Solution formulation: Sagar Khar,
Code Implementation: Laukik Mujumdar,
Testing and Bug fixes: Malay Tushar Nagda,
Analysis & Report Generation: Prabal Bijoy Dutta


A repository to implement the D* lite search algorithm in a pacman environment. The agent only sees its neighboring squares,
and can store information about the locations it has visited, and the structure of the walls surrounding those locations.

If we use traditional planning algorithms like A* to plan, we have to run them each time our robot changes its state. This is 
redundant, and can be avoided by using an iterative planning approach like D* lite, which uses information computed in the previous
iterations, reducing the need to recompute.

To run the A* algorithm, execute the following line from inside the search folder

python pacman.py -l bigMaze -z  -p SearchAgent -a fn=astar2,heuristic=manhattanHeuristic

You can replace 'astar2' with 'dstar' or 'astartv' to run those algorithms

To change the maze, replace 'bigMaze' with another maze like 'mediumMaze'

A* search implemetation in Pacman domain video- https://drive.google.com/file/d/1dJxdAXtt_TP36qseuFwgjHao1YwLrx1H/view?usp=sharing

A* search- Tunnel Vision implemetation in Pacman domain video- https://drive.google.com/file/d/12yHSP5GARA5iG13dDJndBeEIUmZIvLVx/view?usp=sharing

D* Lite implemetation in Pacman domain video- https://drive.google.com/file/d/1YwP7NWzpVlkjzU_yhGjMebTtfalB7fww/view?usp=sharing
