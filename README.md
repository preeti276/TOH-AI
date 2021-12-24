# Tower Of Hanoi 

Solving the Tower of Hanoi problem using search algorithms

We will solve the famous Tower of Hanoi problem by creating an efficient representation of the problem to be used with the search algorithms. 
The search algorithms used are A* search and Breadth-first search.
A* search algorithm will be implemented with different hueristics, including the pattern database hueristic. The details of each hueristic and corresponding results are presented in report.
Furthermore, the algorithms are created for a general n-disc problem and the corresponding results with different values of n are also presented in report.


### How to run the code

To run the code, the following command needs to be run on the console/command line with the necessary arguments:<br>
python -m < m-value > --heuristic < Heuristic name > < n-value > Tower_of_Hanoi.py

--help can be used to access values of arguments
<n-value> : "Number of discs"
"--heuristic": <m-value> :'disksNotOnRightmost','distancefromlastpeg','Blind','PDB', default="disksNotOnRightmost"
"-m" : <m-value> : m<=n, default=0 , This argument is only needed for PDB
