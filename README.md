A repository to implement the D* lite search algorithm in a pacman environment. The agent only sees its neighboring squares,
and can store information about the locations it has visited, and the structure of the walls surrounding those locations.

If we use traditional planning algorithms like A* to plan, we have to run them each time our robot changes its state. This is 
redundant, and can be avoided by using an iterative planning approach like D* lite, which uses information computed in the previous
iterations, reducing the need to recompute.
