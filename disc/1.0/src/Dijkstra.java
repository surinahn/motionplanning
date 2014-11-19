/* Dijkstra.java
 *
 *  Description:
 *  		This program is used for path finding
 *  		in the RRT implementation.
 *  		
 *  Author: Surin Ahn (June 2012)
 *  -- supervised by Professor Yap
 *
 ***************************************************/

import java.util.*;
import java.awt.geom.Point2D;

//The Vertex "template"
class Vertex implements Comparable<Vertex> {
    public List<Ed> adjacencies;	//an array list of all outgoing edges from a node
    public double minDistance = Double.POSITIVE_INFINITY;	//sets the initial minimum distance from source to vertex to infinity
    public Vertex parent;
    //store coordinates
    public double x;
    public double y;

    public Vertex(double xCoord, double yCoord) {
        adjacencies = new ArrayList<Ed>();
        x = xCoord;
        y = yCoord;
    }

    public void addE(Ed Ed) {
        adjacencies.add(Ed);
    }

    public int compareTo(Vertex other) {
        return Double.compare(minDistance, other.minDistance);
    }

}

//The Edge "template"
class Ed {
    public final Vertex target;
    public final double weight;

    public Ed(Vertex argTarget, double argWeight) {
        target = argTarget;
        weight = argWeight;
    }
}


public class Dijkstra {

    public void computePaths(Vertex source) {
        source.minDistance = 0.;
        PriorityQueue<Vertex> vertexQueue = new PriorityQueue<Vertex>();
        vertexQueue.add(source);

        while (!vertexQueue.isEmpty()) {
            Vertex u = vertexQueue.poll();

            // Visit each Ed extending from vertex u
            for (Ed Ed : u.adjacencies) {
                Vertex v = Ed.target;
                double weight = Ed.weight;
                double distanceThroughU = u.minDistance + weight;
                if (distanceThroughU < v.minDistance) {
                    vertexQueue.remove(v);
                    v.minDistance = distanceThroughU;
                    v.parent = u;
                    vertexQueue.add(v);
                }
            }
        }
    }

    //lists the shortest path to a node by visiting the parents in reverse order
    public List<Point2D> getShortestPathTo(Vertex target) {
        List<Vertex> path = new ArrayList<Vertex>();
        List<Point2D> pointPath = new ArrayList<Point2D>();
       
        for (Vertex vertex = target; vertex != null; vertex = vertex.parent)
            path.add(vertex);

        Collections.reverse(path);
        for (Vertex v : path)
        	pointPath.add(new Point2D.Double(v.x, v.y));
        
        return pointPath;
    }
}