package roadgraph;
import geography.GeographicPoint;
import java.util.List;
import java.util.ArrayList;
import java.util.Comparator;

//This class is used to create the vertices for the graph every Map Node is the vertex
//in the graph

public class MapNode 
{
	//location of Map Node and edges connected with it are the data variables for the class
	private GeographicPoint location;
	private List<MapEdges> edges;
	private double cost,hcost,tcost;
	public MapNode(GeographicPoint location)
	{
		//creating a Map Node.
		edges=new ArrayList<>();
		this.location=location;
		cost=(double)Integer.MAX_VALUE;
		hcost=(double)Integer.MAX_VALUE;
	}	
	
	public GeographicPoint getLocation()
	{
		return this.location;
	}
	//method to addEdge to the Node by whose object this method is called upon
	public void addEdge(MapEdges e)
	{
		this.edges.add(e);
	}
	
	//method to set time cost for a star time method
	public void settCost(double tcost)
	{
		this.tcost=tcost;
	}
	
	//method to get tcost
	public double gettCost()
	{
		return this.tcost;
	}
	//method to return the edges connected to the node whose object calls the method.
	public List<MapEdges> getEdge()
	{
		return this.edges;
	}
	
	//method to get the cost of the node
	public double getCost()
	{
		return this.cost;
	}
	
	//method to set hCost
	public double gethcost()
	{
		return hcost;
	}
	
	//method to get hCost
	public void sethcost(MapNode goal)
	{
		hcost=this.location.distance(goal.getLocation());
	}
	
	public void sethcost()
	{
		hcost=(double)Integer.MAX_VALUE;
	}
	
	//method to set the cost of the node
	public void setCost(double cost)
	{
		this.cost=cost;
	}
}