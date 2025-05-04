Solution of exercise 3.15 described in the book "Artificial Intelligence: Modern Approach" by S. Russel and P. Norvig  
scene_example.png contains an example scene taken from the book with defined starting and goal points and polygons, which serve as obstacles for the agent

The program generates starting and goal points and finds approximately the best path between them using the selected graph traversal algorithm

User interface:  
Press 'G' to generate a new pair of start/goal points (first pair is set)  
Press 'H' to switch between informed search method (A*, default) and uninformed (BFS)  
(Note that BFS algorithm may still randomly generate the same path as A*)