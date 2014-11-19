import java.awt.geom.Point2D;
import java.util.*;


class Node implements Comparable<Node>{
	public ArrayList<Node> neighbors;
	public int F;
	public int G;
	public int H;
	public Point2D point;
	Node parent;
	
	public Node(Point2D p){
		neighbors = new ArrayList<Node>();
		point = p;
	}
	
	public void addNeighbor(Node n){
		neighbors.add(n);
	}

    public int compareTo(Node other) {
        return Double.compare(F, other.F);
    }
}

public class AStar{
	
	public LinkedList<Point2D> computePath(Node start, Node target){
		ArrayList<Node> closedset = new ArrayList<Node>();
		PriorityQueue<Node> openset = new PriorityQueue<Node>();
		openset.add(start);
		start.G = 0;
		start.F = start.G + H(start, target);
		
		while(!openset.isEmpty()){
			Node current = openset.peek();
			if(current.equals(target)){
				return findPath(target);
			}
			else{
				openset.remove(current);
				closedset.add(current);
				
				for(Node neighbor:current.neighbors){
					if(closedset.contains(neighbor)){
						continue;
					}
					int tentative_G = current.G + (int)current.point.distance(neighbor.point);
					
					if (!openset.contains(neighbor) || tentative_G < neighbor.G){
						openset.add(neighbor);
						neighbor.parent = current;
						neighbor.G = tentative_G;
						neighbor.F = neighbor.G + H(neighbor, target);
						
					}
				}
			}
		}
		return null;
	}
	
	public int H(Node start, Node target){
		start.H = (int)(start.point.distance(target.point));
		return start.H;
	}
    
	public LinkedList<Point2D> findPath(Node target) {
        LinkedList<Point2D> pointPath = new LinkedList<Point2D>();
       
        for (Node node = target; node != null; node = node.parent)
            pointPath.add(node.point);

        Collections.reverse(pointPath);
        
        return pointPath;
    }
}