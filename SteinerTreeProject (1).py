import matplotlib.pyplot as plt
import networkx as nx


def read_input(file_path):
    """
    Reads the input file and parses sink points.

    :param file_path: Path to the input file
    :return: List of sink points [(x, y, label)]
    """
    points = []
    
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            line = line.strip()
            if line.startswith("sink"):
                _, id, x, y = line.split()
                points.append((float(x), float(y), "Sink"))
    
    return points

# Function to calculate Manhattan distance
def manhattan_distance(point1, point2):
    """
    Computes the Manhattan distance between two points.

    :param point1: (x1, y1)
    :param point2: (x2, y2)
    :return: Manhattan distance
    """
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])

# Function to generate Steiner points and edges
def generate_steiner_tree(sinks):
    """
    Generates Steiner points, edges, and calculates the total wirelength.

    :param sinks: List of sink points [(x, y, label)]
    :return: points, edges, total_wirelength
    """
    # Generate Steiner points (example logic for intersections)
    unique_x = sorted(set([point[0] for point in sinks]))
    unique_y = sorted(set([point[1] for point in sinks]))
    
    steiner_points = []
    id_counter = 1000  # Start IDs for Steiner points

    for x in unique_x:
        for y in unique_y:
            if (x, y) not in [(sink[0], sink[1]) for sink in sinks]:
                steiner_points.append((x, y, "Steiner"))
    
    # Combine sinks and Steiner points
    all_points = sinks + steiner_points
    
    # Create edges and calculate total wirelength
    edges = []
    total_wirelength = 0.0
    
    for i in range(len(all_points)):
        for j in range(i + 1, len(all_points)):
            point1 = all_points[i]
            point2 = all_points[j]
            wirelength = manhattan_distance((point1[0], point1[1]), (point2[0], point2[1]))
            edges.append(((point1[0], point1[1]), (point2[0], point2[1]), wirelength))
            total_wirelength += wirelength
    
    return all_points, edges, total_wirelength

# Function to write output file
def write_output(file_path, points, edges, total_wirelength):
    """
    Writes the points, edges, and total wirelength to an output file.

    :param file_path: Path to the output file
    :param points: List of points [(x, y, label)]
    :param edges: List of edges [((x1, y1), (x2, y2), wirelength)]
    :param total_wirelength: Total wirelength of the tree
    """
    with open(file_path, 'w') as file:
        file.write("# Points\n")
        for x, y, label in points:
            file.write(f"{x},{y},{label}\n")
        
        file.write("\n# Edges\n")
        for start, end, wirelength in edges:
            file.write(f"{start[0]},{start[1]} {end[0]},{end[1]} {wirelength:.1f}\n")
        
        file.write(f"\n# Total Wirelength: {total_wirelength:.1f}\n")

# Function to visualize the Steiner Tree
def visualize_tree(points, edges, total_wirelength):
    """
    Visualizes the Steiner tree using Matplotlib.

    :param points: List of points [(x, y, label)]
    :param edges: List of edges [((x1, y1), (x2, y2), wirelength)]
    :param total_wirelength: Total wirelength of the tree
    """
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
    plt.title(f"Steiner Tree Visualization\nTotal Wirelength: {total_wirelength:.1f}", fontsize=14)
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.grid(True)
    plt.legend(["Steiner Point", "Original Node"], loc="upper right")
    plt.show()

# Main execution
if __name__ == "__main__":
    # Replace these with your input and output file paths
    input_file_path = "input.txt"
    output_file_path = "output.txt"
    
    # Read sinks from input file
    sinks = read_input(input_file_path)
    
    # Generate Steiner tree
    points, edges, total_wirelength = generate_steiner_tree(sinks)
    
    # Write to output file
    write_output(output_file_path, points, edges, total_wirelength)
    
    # Visualize the tree
    visualize_tree(points, edges, total_wirelength)