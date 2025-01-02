# Sprintern-Cadence
This repository is for cadence sprinterns


# Steiner Tree Algorithm Implementation

## **Overview**
This project implements a Steiner Tree algorithm to minimize the total wirelength connecting a set of input points using Manhattan (rectilinear) distances. The goal is to compute the optimal connections for a given set of terminal points, possibly adding Steiner points, while following the constraints of the rectilinear geometry.

The program takes an input file containing Cartesian coordinates of points and outputs a file containing the resulting Steiner tree, including Steiner points and edges.

---

## **Features**
- Implements the **Hanan grid** to identify potential Steiner points.
- Utilizes **Kruskal’s MST algorithm** to generate a Minimum Spanning Tree (MST).
- Includes a **1-Steiner heuristic** to further optimize the rectilinear MST.
- Supports modular and multithreaded code design for efficiency.
- Fully documented and follows good coding practices.

---

## **File Structure**
The project structure is as follows:

project/
│
├── src/                     # Source files
│   ├── steiner.cpp          # Main program file
│   ├── utils.h              # Utility functions and definitions (if any)
│   └── Makefile             # Build instructions
│
├── data/                    # Input and output files
│   ├── example.in           # Sample input file
│   └── example.out          # Corresponding output file
│
├── README.md                # Project documentation
├── pseudocode/              # Folder for pseudocode drafts
│   ├── initial_pseudocode.md
│
└── docs/                    # Additional resources or references

---

## **Input File Format**
The input file is a text file (e.g., `example.in`) with the following format:
1. The first line contains the number of terminal points (**N**).
2. Each of the next **N** lines contains the **x** and **y** coordinates of a point.

### Example:

4
0 0
10 0
0 10
10 10

---

## **Output File Format**
The output file (e.g., `example.out`) includes:
1. A list of all points (terminal and Steiner points).
2. A list of edges in the Steiner tree and their weights (Manhattan distances).

### Example:

Points:
0 0
10 0
0 10
10 10
5 5

Edges:
(0, 0) -> (5, 5) : 10
(10, 0) -> (5, 5) : 10
(0, 10) -> (5, 5) : 10
(10, 10) -> (5, 5) : 10

---

## **Setup Instructions**
### **Prerequisites**
- A C++ compiler (e.g., `g++` or `clang`).
- A terminal or command prompt.
- (Optional) A text editor like VS Code for editing and testing the code.

### **Building the Code**
1. Navigate to the project directory:
   ```bash
   cd project/src

	2.	Use the provided Makefile to build the project:

make

This will compile the steiner.cpp file and generate an executable called steiner.

Running the Program
	1.	Run the program with an input file and specify an output file:

./steiner ../data/example.in ../data/example.out

	•	Input File: Contains terminal points in the format described above.
	•	Output File: The program will generate this file with the Steiner tree details.

	2.	View the generated output file:

cat ../data/example.out

Implementation Plan

Pseudocode
	1.	Read Input:
	•	Parse the input file to read the number of points and their coordinates.
	•	Store the points in a data structure (e.g., vector of points).
	2.	Generate Hanan Grid:
	•	Construct a grid by drawing horizontal and vertical lines through the terminal points.
	•	Identify intersections as candidate Steiner points.
	3.	Build MST:
	•	Use Kruskal’s algorithm to create a Minimum Spanning Tree (MST) for the terminal points.
	•	Store the MST edges and weights.
	4.	1-Steiner Heuristic:
	•	Iteratively introduce candidate Steiner points from the Hanan grid.
	•	Recalculate the MST with the new points.
	•	If the total wirelength decreases, retain the Steiner point.
	5.	Write Output:
	•	Output all points (terminals and Steiner points) to the output file.
	•	Write the edges of the Steiner tree with their weights.

Milestones

Project Timeline

Date	Task
Jan 2-3	Familiarize with concepts and project requirements.
Jan 6-8	Draft pseudocode and read input data.
Jan 9-10	Implement data structures, MST, and Hanan grid.
Jan 13-14	Add 1-Steiner heuristic for optimization.
Jan 15-17	Finalize code, generate output, and prepare presentation.

Resources
	•	Concepts:
	•	Manhattan Distance: Wikipedia
	•	Hanan Grid: GeeksforGeeks
	•	Steiner Tree Problem: YouTube
	•	Algorithms:
	•	Kruskal’s MST: GeeksforGeeks
	•	1-Steiner Heuristic: ResearchGate

Future Improvements
	•	Implement parallelization to handle larger datasets more efficiently.
	•	Explore advanced heuristics for better Steiner point placement.
	•	Integrate a GUI or visualization tool (e.g., Python’s matplotlib) for better representation of results.

Contributors
	•	Arohan Shrestha
	•	Viswa Kotra
    •	Lambert Ike
    •	Julia High
    •	Ishita Kumari



License

This project is licensed under the MIT License - see the LICENSE file for details.

---
