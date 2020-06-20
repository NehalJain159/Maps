/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.Set;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.HashMap;
import java.util.HashSet;
import java.util.function.Consumer;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Collections;
import java.util.Comparator;

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
	//Map data structure used to have constant time complexity in searching Node
	private Map<GeographicPoint,MapNode> graph;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		graph=new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return graph.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		Set<GeographicPoint> ans=new HashSet<>();
		ans.addAll(graph.keySet());
		return ans;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		int ans=0;
		for(Map.Entry<GeographicPoint,MapNode> e:graph.entrySet())
			ans+=e.getValue().getEdge().size();
			
		return ans;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(location==null || graph.containsKey(location))return false;
		MapNode tmp=new MapNode(location);
		graph.put(location,tmp);
		return true;
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
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		
		try {
			if(!graph.containsKey(from) || !graph.containsKey(to) || length<=0)
				throw new IllegalArgumentException();
		}
		catch(IllegalArgumentException e) {}
		
		MapNode x=graph.get(from);
		MapNode y=graph.get(to);
		
		MapEdges xx=new MapEdges(x,y,roadName,roadType,length);
		
		x.addEdge(xx);
		
		
		
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
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startNode = graph.get(start);
		MapNode endNode = graph.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (endNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}
		nodeSearched.accept(start);
		//hashset to maintain the nodes already visited
		HashSet<MapNode> vis=new HashSet<>();
		//queue for doing BFS
		Queue<MapNode> q=new LinkedList<>();
		//for tracing the path hashmap is used
		HashMap<MapNode,MapNode> hm=new HashMap<>();
		
		q.add(startNode);
		vis.add(startNode);
		
		
		while(!q.isEmpty())
		{
			MapNode tmp=q.remove();
			nodeSearched.accept(tmp.getLocation());
			if(tmp.equals(endNode))
			{
				return path(startNode,endNode,hm);
			}
			
			for(MapEdges e:tmp.getEdge())
			{
				MapNode end=e.getAnotherEnd(tmp);
				if(!vis.contains(end))
				{
					vis.add(end);
					nodeSearched.accept(end.getLocation());
					hm.put(end,tmp);
					q.offer(end);
				}
			}	
		}
		return null;
	}
	
	private List<GeographicPoint> path(MapNode s,MapNode e,HashMap<MapNode,MapNode> hm)
	{
		List<GeographicPoint> ans=new LinkedList<>();
		MapNode tmp=e;
		
		while(tmp!=null && !tmp.equals(s))
		{
			ans.add(tmp.getLocation());
			tmp=hm.get(tmp);
		}
		ans.add(s.getLocation());
		Collections.reverse(ans);
		return ans;
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
	 *   
	 *  
	 */
	
	//method to initiate the distance as infinite
	public void initiate()
	{
		for(Map.Entry<GeographicPoint, MapNode> e:graph.entrySet())
		{
			MapNode tmp=e.getValue();
			tmp.setCost((double)Integer.MAX_VALUE);
			tmp.sethcost();
			tmp.settCost((double)Integer.MAX_VALUE);
		}
	}
	
	
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		initiate();
		if(start==null || goal==null)
		{
			System.out.println("Cannot find path:Information glitch");
			return null;
		}
		
		if(start==goal)
		{
			System.out.println("Start and end nodes are the same");
			return null;
		}
		
		MapNode startNode=graph.get(start);
		MapNode endNode=graph.get(goal);
		//for finding out the smallest
		PriorityQueue<MapNode> pq=new PriorityQueue<MapNode>(new Compare());
		//To maintain visited nodes
		HashSet<GeographicPoint> vis=new HashSet<>();
		//To have parent array
		HashMap<MapNode,MapNode> par=new HashMap<>();
		//to check whether we have found solution or not.
		int count=0;
		
		startNode.setCost(0.0);
		pq.offer(startNode);
		
		while(!pq.isEmpty())
		{
			
			MapNode tmp=pq.poll();
			double cost=tmp.getCost();
			
			if(!vis.contains(tmp.getLocation()))
			{
				vis.add(tmp.getLocation());
				
				nodeSearched.accept(tmp.getLocation());
				++count;
				//System.out.println(tmp.getLocation());
				if(goal.equals(tmp.getLocation()))
				{
					System.out.println("Djikstra :"+count);
					return path(startNode,endNode,par);
				}
				for(MapEdges e:tmp.getEdge())
				{
					MapNode gp=e.getAnotherEnd(tmp);
					if(!vis.contains(gp.getLocation()))
					{
						double check=cost+e.getDistance();
						if(gp.getCost()>check)
						{
							gp.setCost(check);
							par.put(gp,tmp);
							pq.add(gp);
						}	
					}
				}
			}
		}
		return null;
		
	}
	
	//comparator for djikstra's algo
	public class Compare implements Comparator<MapNode>
	{
		public int compare(MapNode a,MapNode b)
		{
			if(a.getCost()<b.getCost())
				return -1;
			else if(a.getCost()>b.getCost())
				return 1;
			else
				return 0;
			
		}
	}
	
	//comparator for A-star algo
	public class ACompare implements Comparator<MapNode>
	{
		public int compare(MapNode a,MapNode b)
		{
			if(a.getCost()+a.gethcost()<b.getCost()+b.gethcost())
				return -1;
			else if(a.getCost()+a.gethcost()>b.getCost()+b.gethcost())
				return 1;
			else
				return 0;
		}
	}
	
	//comparator for A-star time algo
		public class TCompare implements Comparator<MapNode>
		{
			public int compare(MapNode a,MapNode b)
			{
				if(a.getCost()+a.gettCost()<b.getCost()+b.gettCost())
					return -1;
				else if(a.getCost()+a.gettCost()>b.getCost()+b.gettCost())
					return 1;
				else
					return 0;
			}
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
		initiate();
		if(start==null || goal==null)
		{
			System.out.println("Cannot find path:Information glitch");
			return null;
		}
		
		if(start==goal)
		{
			System.out.println("Start and end nodes are the same");
			return null;
		}
		
		MapNode startNode=graph.get(start);
		MapNode endNode=graph.get(goal);
		//for finding out the smallest
		PriorityQueue<MapNode> pq=new PriorityQueue<MapNode>(new ACompare());
		//To maintain visited nodes
		HashSet<GeographicPoint> vis=new HashSet<>();
		//To have parent array
		HashMap<MapNode,MapNode> par=new HashMap<>();
		//to check whether we have found solution or not.
		int count=0;
		
		startNode.setCost(0.0);
		startNode.sethcost(startNode);
		pq.offer(startNode);
		
		while(!pq.isEmpty())
		{
			MapNode tmp=pq.poll();
			double cost=tmp.getCost();
			
			if(!vis.contains(tmp.getLocation()))
			{
				vis.add(tmp.getLocation());
				nodeSearched.accept(tmp.getLocation());
				
				++count;
				//System.out.println(tmp.getLocation());
				if(endNode.equals(tmp))
				{
					System.out.println("Astar :"+count);
					return path(startNode,endNode,par);
				}
				for(MapEdges e:tmp.getEdge())
				{
					MapNode gp=e.getAnotherEnd(tmp);
					if(!vis.contains(gp.getLocation()))
					{
						double check=cost+e.getDistance();
						if(gp.getCost()>check)
						{
							gp.setCost(check);
							gp.sethcost(endNode);
							par.put(gp,tmp);
							pq.offer(gp);
						}
					}
				}
			}
		}
		return null;
		
	}
	
	public List<GeographicPoint> searchByTime(GeographicPoint start, 
			 GeographicPoint goal)
	{
		Consumer<GeographicPoint> temp = (x) -> {};
        return searchByTime(start, goal, temp);
	}
	
	//method to search the best route in an even effective way by computing the time required
	public List<GeographicPoint> searchByTime(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if(start==null || goal==null)
		{
			System.out.println("Path cannot be found out");
			return null;
		}
		if(start==goal)
		{
			System.out.println("Starting and ending are same");
			return null;
		}
		
		MapNode startNode=graph.get(start);
		MapNode endNode=graph.get(goal);
		
		if(startNode==null || endNode==null)
		{
			System.out.println("Path cannot be found out");
			return null;
		}
		
		//for storing nodes
		PriorityQueue<MapNode> pq=new PriorityQueue<>(new Compare());
		//for storing visited locations
		HashSet<GeographicPoint> vis=new HashSet<>();
		//for parent map
		HashMap<MapNode,MapNode> par=new HashMap<>();
		int count=0;
		double time=0;
		//here the cost of our priority queue would be time
		initiate();
		startNode.setCost(0.0);
		pq.offer(startNode);
		
		while(!pq.isEmpty()) 
		{
			MapNode curr=pq.remove();
			double cost=curr.getCost();
			if(!vis.contains(curr.getLocation()))
			{
				vis.add(curr.getLocation());
				nodeSearched.accept(curr.getLocation());
				time+=curr.getCost();
				++count;
				
				if(goal.equals(curr.getLocation()))
				{
					System.out.println("Time required :"+time+" "+endNode.getCost());
					System.out.println("Nodes visited :"+count);
					return path(startNode,endNode,par);
				}
				
				for(MapEdges e:curr.getEdge())
				{
					MapNode gp=e.getAnotherEnd(curr);
					if(!vis.contains(gp.getLocation()))
					{
						double check=cost+e.getTime();
						if(gp.getCost()>check)
						{
							gp.setCost(check);
							pq.offer(gp);
							par.put(gp,curr);
						}
					}
				}
					
			}
		}
		return null;
	}
	
	
	public List<GeographicPoint> AstarTime(GeographicPoint start, 
			 GeographicPoint goal)
	{
		Consumer<GeographicPoint> temp = (x) -> {};
       return AstarTime(start, goal, temp);
	}
	
	//method to search the best route in an even effective way by computing the time required
		public List<GeographicPoint> AstarTime(GeographicPoint start, 
												 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
		{
			if(start==null || goal==null)
			{
				System.out.println("Path cannot be found out");
				return null;
			}
			if(start==goal)
			{
				System.out.println("Starting and ending are same");
				return null;
			}
			
			MapNode startNode=graph.get(start);
			MapNode endNode=graph.get(goal);
			
			if(startNode==null || endNode==null)
			{
				System.out.println("Path cannot be found out");
				return null;
			}
			
			//for storing nodes
			PriorityQueue<MapNode> pq=new PriorityQueue<>(new TCompare());
			//for storing visited locations
			HashSet<GeographicPoint> vis=new HashSet<>();
			//for parent map
			HashMap<MapNode,MapNode> par=new HashMap<>();
			int count=0;
			double time=0;
			//here the cost of our priority queue would be time
			initiate();
			startNode.setCost(0.0);
			pq.offer(startNode);
			
			while(!pq.isEmpty()) 
			{
				MapNode curr=pq.remove();
				double cost=curr.getCost();
				if(!vis.contains(curr.getLocation()))
				{
					vis.add(curr.getLocation());
					nodeSearched.accept(curr.getLocation());
					time+=curr.getCost();
					++count;
					
					if(goal.equals(curr.getLocation()))
					{
						System.out.println("Time required :"+time+" "+endNode.getCost());
						System.out.println("Nodes visited :"+count);
						return path(startNode,endNode,par);
					}
					
					for(MapEdges e:curr.getEdge())
					{
						MapNode gp=e.getAnotherEnd(curr);
						if(!vis.contains(gp.getLocation()))
						{
							double check=cost+e.getTime();
							if(gp.getCost()>check)
							{
								gp.setCost(check);
								gp.sethcost(endNode);
								gp.settCost(gp.gethcost()/180.0);
								pq.offer(gp);
								par.put(gp,curr);
							}
						}
					}
						
				}
			}
			return null;
		}
		
		
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testMap.dijkstra(testStart,testEnd);
		testMap.aStarSearch(testStart,testEnd);
				
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// Use this code in Week 3 End of Week Quiz 
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
	}
	
}
