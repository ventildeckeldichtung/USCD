/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	HashMap<GeographicPoint,Vertex> map1;
	HashSet<GeographicPoint> edges;
	
	Integer numedges;
	
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		map1 = new HashMap<GeographicPoint,Vertex>();
		edges = new HashSet<GeographicPoint>();
		numedges = 0;
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return map1.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		HashSet<GeographicPoint> vertices = new HashSet<GeographicPoint>();
		for(GeographicPoint p:map1.keySet()) {
			vertices.add(p);
		}
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numedges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	/*
	 * Implementation: First, check if the location is already marked as a vertex in the map. If not, create a new Vertex object
	 * and add it to map1.
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(location.equals(null)) {
			return false;
		}
		else if (!map1.containsKey(location)) {
			map1.put(location, new Vertex(location));
			
			return true;
		}
		else {
			return false;
		}
		
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	/*
	 * First, check if both point are already in the graph. 
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		

		try {
			if(map1.containsKey(from) && map1.containsKey(to) && length > 0) {
				//System.out.println(from + " " + " " + to + " " + roadName + " " + roadType + " " + length);
				Edge e1 = new Edge(new Vertex(from), new Vertex(to), roadName, roadType, length);
				numedges ++;
				//Edge e2 = new Edge(new Vertex(to), new Vertex(from), roadName, roadType, length);
				map1.get(from).addNeighbor(e1);
				//map1.get(to).addNeighbor(e2);
			}
		}
		catch (IllegalArgumentException e) {
			System.out.println(e);
		}
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		/*
		 * return an empty list if one of the coordinates is not on the map or if 
		 * one of the coordinates is null. Else, start the search. 
		 */
		
		HashMap<GeographicPoint,GeographicPoint> parentMap;
		if(!map1.containsKey(start) || !map1.containsKey(goal)) {
			System.out.println("Either start or goal coordinates are not on the map! ");
			return new LinkedList<GeographicPoint>();
		}
		else if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		else {
			parentMap = bfsSearch(start, goal, nodeSearched);
			
		}
		//If a path has been found (i.e. the goal is in the parentMap) then reconstruct the path. else return an empty list
		if(parentMap.containsKey(goal)) {
			//System.out.println(reconstructPath(parentMap,  start, goal));
			return reconstructPath(parentMap,  start, goal);
		}
		else {
			System.out.println("no path found! ");
			return null;
		}
		
	}
	/*
	 * bfsSearch: returns the path of the BFS search. 
	 */
	private HashMap<GeographicPoint,GeographicPoint> bfsSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched){
		
		Vertex startpoint = map1.get(start);
		Vertex goalpoint = map1.get(goal);
		HashMap<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		Queue<Vertex> explore = new LinkedList<Vertex>();
		explore.add(startpoint);
		while(!explore.isEmpty()) {
			//Take the next vertex from the list. If it is the goal, exit.
			Vertex curr = explore.remove();
			curr.setVisited();
			nodeSearched.accept(curr.getLocation());
			if(curr.equals(goalpoint)) {
				//parentMap.put(curr.getLocation(), v.getLocation());
				break;
			}
			//for each of curr's neighbors which have not been visited yet:
			HashSet<GeographicPoint> nb = curr.getNeighbors();
			for(GeographicPoint p:nb) {	
				Vertex v = map1.get(p);
				if(!v.getVisited()) {
					v.setVisited();
					parentMap.put(v.getLocation(), curr.getLocation());
					explore.add(v);
				}
			}
		}
		return parentMap;
	}
	/*
	 * Reconstruct the path from the parentMap. Start with the last element. 
	 */
	private LinkedList<GeographicPoint> reconstructPath(HashMap<GeographicPoint,GeographicPoint> parentMap, GeographicPoint start, GeographicPoint goal) {
		//System.out.println(parentMap);
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		GeographicPoint curr = goal;
		
		while(curr != null) {
			path.addFirst(curr);
			//System.out.println(curr);
			curr = parentMap.get(curr);
		}
		//path.addFirst(start);
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		HashMap<GeographicPoint,GeographicPoint> parentMap;
		if(!map1.containsKey(start) || !map1.containsKey(goal)) {
			System.out.println("Either start or goal coordinates are noton the map! ");
			return new LinkedList<GeographicPoint>();
		}
		else if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		else {
			parentMap = dijkstraSearch(start, goal, nodeSearched);
			
		}
		//If a path has been found (i.e. the goal is in the parentMap) then reconstruct the path. else return an empty list
		if(parentMap.containsKey(goal)) {
			//System.out.println(reconstructPath(parentMap,  start, goal));
			return reconstructPath(parentMap,  start, goal);
		}
		else {
			System.out.println("no path found! ");
			return null;
		}
		
	}

	private HashMap<GeographicPoint,GeographicPoint> dijkstraSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched){
		Vertex startpoint = map1.get(start);
		Vertex goalpoint = map1.get(goal);
		HashMap<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		PriorityQueue<Vertex> pq =new PriorityQueue<Vertex>();
		startpoint.setWeight(0.0);
		pq.add(startpoint);
		while(!pq.isEmpty()) {
			Vertex curr = pq.remove();	
			nodeSearched.accept(curr.getLocation());
			//System.out.println(curr.getNeighbors());
			if(!curr.getVisited()) {
				curr.setVisited();
				if(curr.getLocation() == goalpoint.getLocation()) {
					break;
				}
				//get the list of streets from the current edge.
				List<Edge> streets = curr.getStreets();
				//For each of the Neighbors, the distance to the start node is the weight of the current node plus the
				//road length of the current node to that node. 
				for(Edge street:streets) {	
					Vertex v = map1.get(street.getEnd().getLocation());		
					Double distance = street.getLength() + curr.getWeight();
					if(!v.getVisited()) {
						//if the current distance to v is shorter than the old distance to v
						if(distance < v.getWeight()) {
							v.setWeight(distance);	
							pq.add(v);
							parentMap.put(v.getLocation(), curr.getLocation());
						}
					}
				}
			}
		}
		
		return parentMap;
	}
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		HashMap<GeographicPoint,GeographicPoint> parentMap;
		if(!map1.containsKey(start) || !map1.containsKey(goal)) {
			System.out.println("Either start or goal coordinates are noton the map! ");
			return new LinkedList<GeographicPoint>();
		}
		else if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		else {
			parentMap = aStarSearchSearch(start, goal, nodeSearched);
			
		}
		//If a path has been found (i.e. the goal is in the parentMap) then reconstruct the path. else return an empty list
		if(parentMap.containsKey(goal)) {
			//System.out.println(reconstructPath(parentMap,  start, goal));
			return reconstructPath(parentMap,  start, goal);
		}
		else {
			System.out.println("no path found! ");
			return null;
		}
		
		
	}
	private HashMap<GeographicPoint,GeographicPoint> aStarSearchSearch(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched){
		Vertex startpoint = map1.get(start);
		Vertex goalpoint = map1.get(goal);
		HashMap<GeographicPoint,GeographicPoint> parentMap = new HashMap<GeographicPoint,GeographicPoint>();
		PriorityQueue<Vertex> pq =new PriorityQueue<Vertex>();
		startpoint.setWeight(0.0);
		pq.add(startpoint);
		while(!pq.isEmpty()) {
			Vertex curr = pq.remove();	
			nodeSearched.accept(curr.getLocation());
			System.out.println(curr.getLocation());
			if(!curr.getVisited()) {
				curr.setVisited();
				if(curr.getLocation() == goalpoint.getLocation()) {
					break;
				}
				//get the list of streets from the current edge.
				List<Edge> streets = curr.getStreets();
				//For each of the Neighbors, the distance to the start node is the weight of the current node plus the
				//road length of the current node to that node. 
				for(Edge street:streets) {	
					Vertex v = map1.get(street.getEnd().getLocation());		
					Double distance = street.getLength() + curr.getWeight();
					Double heuristicdistance = street.getEnd().getLocation().distance(goal);
					Double weight = heuristicdistance + distance;
					if(!v.getVisited()) {
						//if the current distance to v is shorter than the old distance to v
						if(weight < v.getWeight()) {
							v.setWeight(weight);	
							pq.add(v);
							parentMap.put(v.getLocation(), curr.getLocation());
						}
					}
				}
			}
		}
		
		return parentMap;
	}
	
	/*
	 * Takes a list of GeographicPoints as input. Returns a path (which is represented as a list of geopoints which 
	 * travels trough all the vertices in an efficient way.
	 */
	public List<GeographicPoint> salesman(List<GeographicPoint> targets,  Consumer<GeographicPoint> nodeSearched){
		
		return null;
	}
	/*
	 * A helper function to evaluate the quality of the path. It returns the length of the path.
	 */
	public double pathLength(List<GeographicPoint> path) {
		GeographicPoint p1 = new GeographicPoint(12.3,11.3);
		
		return 0;
	}
	/*
	 * Add a method to print the Map for testing purposes.
	 */
	@Override
	public String toString() {
		
		return "";
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		List<GeographicPoint> testroute = firstMap.dijkstra(new GeographicPoint(1.0,1.0), new GeographicPoint(8.0,-1.0));
		System.out.println(testroute);
		//System.out.println(firstMap.map1.get(new GeographicPoint(4.0,1.0)).getNeighbors());
		//firstMap.addVertex(new GeographicPoint(33.9,12.8));
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		/*
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute3 = simpleTestMap.dijkstra(testStart,testEnd);
		System.out.println(testroute3);
		
		
		
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
