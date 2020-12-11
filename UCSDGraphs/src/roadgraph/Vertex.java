package roadgraph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import geography.GeographicPoint;

/*
 * The vertex Class has a location and a list of streets. The streets are represented as edge objects
 * whose end is another vertex. The Vertex itself is made comparable to simplify implementation of Djikstra search.
 * the weight of the graph is initilaised with maximum value
 */
public class Vertex implements Comparable<Vertex>{
	private GeographicPoint location;
	private List<Edge> streets;
	private boolean visited;
	private Double weight;
	public Vertex(GeographicPoint location) {
		this.location = location;
		visited = false;
		streets = new ArrayList<Edge>();
		this.weight = Double.MAX_VALUE;
	}	
	public Vertex(GeographicPoint location, List<Edge> streets) {
		this.location = location;
		this.streets = streets;
		streets = new ArrayList<Edge>();
		visited = false;
	}
	public void addNeighbor(Edge street) {
		streets.add(street);
	}
	/*
	 * this function takes a vertex as input and returns the neighbors of this Vertex.
	 */
	public HashSet<GeographicPoint> getNeighbors(){
		HashSet<GeographicPoint> nb1 = new HashSet<GeographicPoint>();
		for(Edge e:streets) {
			nb1.add(e.getEnd().getLocation());
		}
		return nb1;
	}
	public List<Edge> getStreets() {
		return this.streets;
	}
	public Double getWeight() {
		return this.weight;
	}
	public void setWeight(Double weight) {
		this.weight = weight;
	}
	public GeographicPoint getLocation() {
		return this.location;
	}
	public void setVisited() {
		this.visited = true;
	}
	public boolean getVisited() {
		return this.visited;
	}
	public boolean equals(Vertex v) {
		if(this.location == v.location) {
			return true;
		}
		else {
			return false;
		}
	}
	
	
	@Override
	public String toString() {
		StringBuilder str = new StringBuilder();
		str.append(location.x + " : " + location.y + " " + visited);
		str.append(System.getProperty("line.separator"));
		for(Edge e:streets) {
			str.append("  ->"+e);
		}
		
		
		return str.toString();
	}
	@Override
	public int compareTo(Vertex v1) {
		if(v1.getWeight() > this.getWeight()) {
			return -1;
		}
		else if(v1.getWeight() < this.getWeight()) {
			return 1;
		}
		else {
			return 0;
		}
	}
}
