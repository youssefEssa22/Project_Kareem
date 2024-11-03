import java.util.*;
public class PuzzleSolver {
    private static final int[][] goalState = {
        {0, 1, 2},
        {3, 4, 5},
        {6, 7, 8}
    };

    static class Node {
        int[][] state;
        int x, y;
        Node parent;
        int depth, cost;

        Node(int[][] state, int x, int y, Node parent, int depth, int cost) {
            this.state = copyState(state);
            this.x = x;
            this.y = y;
            this.parent = parent;
            this.depth = depth;
            this.cost = cost;
        }
    }

    private static int[][] copyState(int[][] state) {
        int[][] copy = new int[3][3];
        for (int i = 0; i < 3; i++) {
            System.arraycopy(state[i], 0, copy[i], 0, 3);
        }
        return copy;
    }

    private static boolean isGoal(int[][] state) {
        return Arrays.deepEquals(state, goalState);
    }

    public List<Node> getSolutionPathBFS(int[][] initialState, int blankX, int blankY) {
        Queue<Node> queue = new LinkedList<>();
        Set<String> visited = new HashSet<>();
        queue.add(new Node(initialState, blankX, blankY, null, 0, 0));
        visited.add(Arrays.deepToString(initialState));

        while (!queue.isEmpty()) {
            Node node = queue.poll();
            if (isGoal(node.state)) return buildSolutionPath(node);

            for (Node neighbor : getNeighbors(node)) {
                if (!visited.contains(Arrays.deepToString(neighbor.state))) {
                    queue.add(neighbor);
                    visited.add(Arrays.deepToString(neighbor.state));
                }
            }
        }
        return null;
    }

    public List<Node> getSolutionPathDFS(int[][] initialState, int blankX, int blankY) {
        Stack<Node> stack = new Stack<>();
        Set<String> visited = new HashSet<>();
        stack.push(new Node(initialState, blankX, blankY, null, 0, 0));
        visited.add(Arrays.deepToString(initialState));

        while (!stack.isEmpty()) {
            Node node = stack.pop();
            if (isGoal(node.state)) return buildSolutionPath(node);

            for (Node neighbor : getNeighbors(node)) {
                if (!visited.contains(Arrays.deepToString(neighbor.state))) {
                    stack.push(neighbor);
                    visited.add(Arrays.deepToString(neighbor.state));
                }
            }
        }
        return null;
    }

    public List<Node> getSolutionPathAStar(int[][] initialState, int blankX, int blankY, boolean useManhattan) {
        PriorityQueue<Node> openList = new PriorityQueue<>(Comparator.comparingInt(n -> n.cost));
        Set<String> closedList = new HashSet<>();
        openList.add(new Node(initialState, blankX, blankY, null, 0, 0));

        while (!openList.isEmpty()) {
            Node node = openList.poll();
            if (isGoal(node.state)) return buildSolutionPath(node);

            closedList.add(Arrays.deepToString(node.state));

            for (Node neighbor : getNeighbors(node)) {
                if (!closedList.contains(Arrays.deepToString(neighbor.state))) {
                    neighbor.cost = node.depth + heuristic(neighbor.state, useManhattan);
                    openList.add(neighbor);
                    closedList.add(Arrays.deepToString(neighbor.state));
                }
            }
        }
        return null;
    }

    private List<Node> buildSolutionPath(Node goalNode) {
        List<Node> path = new ArrayList<>();
        for (Node node = goalNode; node != null; node = node.parent) {
            path.add(node);
        }
        Collections.reverse(path);
        return path;
    }

    private List<Node> getNeighbors(Node node) {
        List<Node> neighbors = new ArrayList<>();
        int[][] moves = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

        for (int[] move : moves) {
            int newX = node.x + move[0], newY = node.y + move[1];
            if (isValidMove(newX, newY)) {
                int[][] newState = copyState(node.state);
                newState[node.x][node.y] = newState[newX][newY];
                newState[newX][newY] = 0;
                neighbors.add(new Node(newState, newX, newY, node, node.depth + 1, node.cost + 1));
            }
        }
        return neighbors;
    }

    private int heuristic(int[][] state, boolean useManhattan) {
        int h = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                int value = state[i][j];
                if (value != 0) {
                    int goalX = value / 3, goalY = value % 3;
                    if (useManhattan) {
                        h += Math.abs(i - goalX) + Math.abs(j - goalY);
                    } else {
                        h += Math.sqrt(Math.pow(i - goalX, 2) + Math.pow(j - goalY, 2));
                    }
                }
            }
        }
        return h;
    }

    private boolean isValidMove(int x, int y) {
        return x >= 0 && x < 3 && y >= 0 && y < 3;
    }

    private void printSolution(List<Node> path) {
        if (path == null) {
            System.out.println("No solution found.");
            return;
        }
        System.out.println("Solution path:");
        for (Node node : path) {
            printState(node.state);
            System.out.println();
        }
        System.out.println("Path length: " + (path.size() - 1));
    }

    private void printState(int[][] state) {
        for (int[] row : state) {
            for (int cell : row) System.out.print(cell + " ");
            System.out.println();
        }
    }

    // Method to get user input for the initial puzzle configuration
    private static int[][] getUserInput() {
        Scanner scanner = new Scanner(System.in);
        int[][] initialState = new int[3][3];
        Set<Integer> seen = new HashSet<>();

        System.out.println("Enter the numbers for the puzzle (0 to 8) in row-major order:");
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                int num;
                while (true) {
                    System.out.print("Enter number for position (" + i + ", " + j + "): ");
                    num = scanner.nextInt();
                    if (num >= 0 && num <= 8 && !seen.contains(num)) {
                        seen.add(num);
                        break;
                    } else {
                        System.out.println("Invalid input. Please enter a unique number between 0 and 8.");
                    }
                }
                initialState[i][j] = num;
            }
        }
        return initialState;
    }

    public static void main(String[] args) {
        PuzzleSolver solver = new PuzzleSolver();

        // Get the user's input for the initial puzzle configuration
        int[][] initialState = getUserInput();

        // Find the blank tile position (where 0 is located)
        int blankX = -1, blankY = -1;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (initialState[i][j] == 0) {
                    blankX = i;
                    blankY = j;
                    break;
                }
            }
        }

        // Now solve the puzzle using different algorithms
        System.out.println("Solving with BFS:");
        List<Node> bfsSolution = solver.getSolutionPathBFS(initialState, blankX, blankY);
        solver.printSolution(bfsSolution);

        System.out.println("\nSolving with DFS:");
        List<Node> dfsSolution = solver.getSolutionPathDFS(initialState, blankX, blankY);
        solver.printSolution(dfsSolution);

        System.out.println("\nSolving with A* (Manhattan):");
        List<Node> aStarManhattanSolution = solver.getSolutionPathAStar(initialState, blankX, blankY, true);
        solver.printSolution(aStarManhattanSolution);

        System.out.println("\nSolving with A* (Euclidean):");
        List<Node> aStarEuclideanSolution = solver.getSolutionPathAStar(initialState, blankX, blankY, false);
        solver.printSolution(aStarEuclideanSolution);;}
}