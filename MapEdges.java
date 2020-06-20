package roadgraph;
import geography.GeographicPoint;


//This class is to create the edges that form the graph
public class MapEdges
{
	//start,end,streetName,type and distance defines an edge
	private MapNode start,end;
	private String streetName,streetType;
	private double distance,speed;
	
	public MapEdges(MapNode start,MapNode end,String streetName,String streetType,double distance)
	{
		this.start=start;
		this.end=end;
		this.streetType=streetType;
		this.streetName=streetName;
		this.distance=distance;
		setSpeed();
	}
	
	public void setSpeed()
	{
		switch(streetType)
		{
			case "motorway":
				speed=180.0;break;
			case "motorway_link":
				speed=120.0;break;
			case "secondary":
				speed=80.0;break;
			case "unclassified":
				speed=70.0;break;
			case "primary":
				speed=80.0;break;
			case "residential":
				speed=50.0;break;
			case "trunk":
				speed=80.0;break;
			case "trunk_link":
				speed=70.0;break;
			case "tertiary":
				speed=80.0;break;
			case "living_street":
				speed=30.0;break;
			default:
				speed=50.0;
		}
	}
	
	public double getSpeed()
	{
		return speed;
	}
	
	public String getStreetType()
	{
		return streetType;
	}
	
	
	//this method returns the other end of the edge whose object calls the method
	public MapNode getAnotherEnd(MapNode mp)
	{
		if(mp.equals(start))
			return end;
		else if(end.equals(mp))
			return start;
		else 
			throw new IllegalArgumentException();
	}
	
	public double getDistance() {
		return this.distance;
	}
	
	public double getTime()
	{
		return (distance/speed);
	}
	
	public void setDistance(double val)
	{
		this.distance=val;
	}

}