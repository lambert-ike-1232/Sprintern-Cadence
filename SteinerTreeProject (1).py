#steiner_tree_project/
#│
#├── c++/
#│   ├── steiner_tree.cpp       # Core algorithm implementation
#│   ├── input.txt              # Input: Cartesian coordinates
#│   ├── output.txt             # Output: Points and edges
#│   ├── Makefile               # Build instructions
#│
#├── python/
#│   ├── visualize.py           # Matplotlib visualization script
#│   ├── output.txt             # Input: C++ program output
#│
#└── test_cases/
#    ├── small_test.txt         # Test case: Small dataset
#    ├── medium_test.txt        # Test case: Medium dataset
#    ├── large_test.txt         # Test case: Large dataset

import matplotlib.pyplot as plt
import networkx as nx

# Function to read output file and parse data
def read_output(file_path):
    """
    Reads the output file and parses points and edges.

    :param file_path: Path to the output file
    :return: (points, edges, wirelength)
    """
    points = []
    edges = []
    total_wirelength = 0.0
    
    with open(file_path, 'r') as file:
        lines = file.readlines()
        mode = None
        
        for line in lines:
            line = line.strip()
            if line.startswith("# Points"):
                mode = "points"
                continue
            elif line.startswith("# Edges"):
                mode = "edges"
                continue
            elif line.startswith("# Total Wirelength"):
                total_wirelength = float(line.split(":")[1].strip())
                break
            
            if mode == "points" and line:
                x, y, label = line.split(",")
                points.append((float(x), float(y), label.strip()))
            elif mode == "edges" and line:
                start, end, wirelength = line.split(" ")
                edges.append((tuple(map(float, start.split(","))), 
                              tuple(map(float, end.split(","))), 
                              float(wirelength)))
    
    return points, edges, total_wirelength

# Function to visualize the Steiner Tree
def visualize_tree(points, edges, total_wirelength):
    """
    Visualizes the Steiner tree using Matplotlib.

    :param points: List of points [(x, y, label)]
    :param edges: List of edges [((x1, y1), (x2, y2), wirelength)]
    :param total_wirelength: Total wirelength of the tree
    """
    # Create a plot
    plt.figure(figsize=(10, 10))
    
    # Plot points
    for x, y, label in points:
        color = 'blue' if label == "Steiner" else 'red'
        plt.scatter(x, y, color=color, label=label, s=100, edgecolors='black')
        plt.text(x + 0.1, y + 0.1, f"({x}, {y})", fontsize=9, color='black')
    
    # Plot edges
    for start, end, wirelength in edges:
        x_values = [start[0], end[0]]
        y_values = [start[1], end[1]]
        plt.plot(x_values, y_values, color='black', linewidth=1, linestyle='--')
    
    # Add title and annotations
    plt.title(f"Steiner Tree Visualization\nTotal Wirelength: {total_wirelength}", fontsize=14)
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.grid(True)
    plt.legend(["Steiner Point", "Original Node"], loc="upper right")
    
    # Show the plot
    plt.show()

# Main execution
if __name__ == "__main__":
    # Replace with the path to your output file
    output_file_path = "output.txt"
    
    # Read output and visualize
    points, edges, total_wirelength = read_output(output_file_path)
    visualize_tree(points, edges, total_wirelength)


#VERY IMPORTANT
# Before you run the program make sure to put both the visualize.py file and the output.txt file within the same path on your computer to ensure that it can read and write the text.#
# Points
#x1, y1, Original
#x2, y2, Steiner
#...

# Edges
#x1,y1 x2,y2 wirelength
#x3,y3 x4,y4 wirelength
#...

# Total Wirelength: <value>


