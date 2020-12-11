package roadgraph;

import geography.GeographicPoint;

/*
 * 
 */
public class Edge {
	private String roadname;
	private String roadtype;
	private Vertex start;
	private Vertex end;
	private double length;
	
	public Edge(Vertex start, Vertex end, String roadname, String roadtype, double length) {
		this.start = start;
		this.end = end;
		this.roadname = roadname;
		this.roadtype = roadtype;
		this.length = length;
		
		
	}
	public String getRoadame() {
		return roadname;
	}
	public String getRoadtype() {
		return roadtype;
	}
	public Vertex getStart() {
		return start;
	}
	public Vertex getEnd() {
		return end;
	}
	public double getLength() {
		return this.length;
	}
	@Override
	public String toString() {
		return roadname + " to " + end;
	}

}
