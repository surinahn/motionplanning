/*
 * Author: Surin Ahn
 */

import java.awt.geom.Point2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Area;
import java.util.*;

public class Quadtree{
    
    private Box root;
    private LinkedList<Box> BFSList; 
    private ArrayList<Box> randomList;
    private PriorityQueue<Box> GBFQ;
    private ArrayList<Box> leaves;
    private ArrayList<Box> free;
    private ArrayList<Box> stuck;
    private ArrayList<Box> mixed;
    private ArrayList<Box> underEpsilon;
    private ArrayList<Area> obstacles;
    private int epsilon;
    private int strategy;
    private enum Status {FREE, STUCK, MIXED};
    private double robot_radius;
    private UnionFind<Box> uf;
    private Point2D start;
    private Point2D target;
    private LinkedList<Point2D> path;
    private long expansionTime;
    private long pathTime;
    private long endExpansion;
    private long startExpansion;

    
    public Quadtree(double length, ArrayList<Area> o, double robotRadius, Point2D s, Point2D t) {
        obstacles = new ArrayList<Area>();
        obstacles = o;
        free = new ArrayList<Box>();
        stuck = new ArrayList<Box>();
        underEpsilon = new ArrayList<Box>();
        mixed = new ArrayList<Box>();
        robot_radius = robotRadius;
        BFSList = new LinkedList<Box>();
        randomList = new ArrayList<Box>();
        GBFQ = new PriorityQueue<Box>();
        path = new LinkedList<Point2D>();
        target = t;
        root = new Box(0, 0, length);
        leaves = new ArrayList<Box>();
        leaves.add(root);
        uf = new UnionFind<Box>();
        start = s;
    }
    
    
    //Exact FindPath from paper
    public boolean findPath() {
        startExpansion = System.currentTimeMillis();
        while (boxContaining(start).status != Status.FREE) {
        		if (boxContaining(start).split() == false){
            		endExpansion = System.currentTimeMillis();
            		expansionTime = endExpansion-startExpansion;
        			return false;
        		}
        }
        while (boxContaining(target).status != Status.FREE){
        	if (boxContaining(target).split() == false){
        		endExpansion = System.currentTimeMillis();
        		expansionTime = endExpansion-startExpansion;
        		return false;
        	}
        }
        
        if (strategy == 0){
        	return BFS();
        }
        else if (strategy == 1){
        	return random();
        }
        else if (strategy == 2){
        	return GBF();
        }
		return false;
    }
    
    boolean BFS(){
        Box startbox = boxContaining(start);
        startbox.node = new Node(start);
        Box targetbox = boxContaining(target);
        targetbox.node = new Node(target);
        while(!uf.sameSet(startbox.record, targetbox.record)){
        	if (BFSList.isEmpty()){
        		endExpansion = System.currentTimeMillis();
        		expansionTime = endExpansion-startExpansion;
        		return false;
        	}
                BFSList.poll().split();
        }
        endExpansion = System.currentTimeMillis();
        expansionTime = endExpansion-startExpansion;
        
        for (Box b:leaves){
        	if (b.status == Status.FREE){
            	for(Box n:b.N){
            		if(n.status == Status.FREE)
                		b.node.addNeighbor(n.node);
            	}
            	for(Box s:b.S){
            		if(s.status == Status.FREE)
            		b.node.addNeighbor(s.node);
            	}
            	for(Box e:b.E){
            		if(e.status == Status.FREE)
            		b.node.addNeighbor(e.node);
            	}
            	for(Box w:b.W){
            		if(w.status == Status.FREE)
            		b.node.addNeighbor(w.node);
            	}
        	}
        } 
        for (Box b:leaves){
        	if (b.status == Status.FREE){
            	for(Box n:b.N){
            		if(n.status == Status.FREE)
                		b.node.addNeighbor(n.node);
            	}
            	for(Box s:b.S){
            		if(s.status == Status.FREE)
            		b.node.addNeighbor(s.node);
            	}
            	for(Box e:b.E){
            		if(e.status == Status.FREE)
            		b.node.addNeighbor(e.node);
            	}
            	for(Box w:b.W){
            		if(w.status == Status.FREE)
            		b.node.addNeighbor(w.node);
            	}
        	}
        } 
        AStar astar = new AStar();
        long startTime = System.currentTimeMillis();
        path = astar.computePath(startbox.node, targetbox.node);
        long endTime = System.currentTimeMillis();
        pathTime = endTime-startTime;
        return true;
    }
    
    boolean random(){
        Box startbox = boxContaining(start);
        startbox.node = new Node(start);
        Box targetbox = boxContaining(target);
        targetbox.node = new Node(target);
        while(!uf.sameSet(startbox.record, targetbox.record)){
        	if (randomList.isEmpty()){
        		endExpansion = System.currentTimeMillis();
        		expansionTime = endExpansion-startExpansion;
        		return false;
        	}
        		Random r = new Random();
                randomList.get(r.nextInt(randomList.size())).split();
        }
        endExpansion = System.currentTimeMillis();
        expansionTime = endExpansion-startExpansion;
        for (Box b:leaves){
        	if (b.status == Status.FREE){
            	for(Box n:b.N){
            		if(n.status == Status.FREE)
                		b.node.addNeighbor(n.node);
            	}
            	for(Box s:b.S){
            		if(s.status == Status.FREE)
            		b.node.addNeighbor(s.node);
            	}
            	for(Box e:b.E){
            		if(e.status == Status.FREE)
            		b.node.addNeighbor(e.node);
            	}
            	for(Box w:b.W){
            		if(w.status == Status.FREE)
            		b.node.addNeighbor(w.node);
            	}
        	}
        } 
        AStar astar = new AStar();
        long startTime = System.currentTimeMillis();
        path = astar.computePath(startbox.node, targetbox.node);
        long endTime = System.currentTimeMillis();
        pathTime = endTime-startTime;
        return true;
    }
   
    boolean GBF(){
    	Box startbox = boxContaining(start);
        Box targetbox = boxContaining(target);
        
        GBFQ.add(startbox);
        startbox.inQ = true;
        startbox.record = new Record<Box>(startbox);
        targetbox.record = new Record<Box>(targetbox);
        
        while(!uf.sameSet(startbox.record, targetbox.record)){
        	if (GBFQ.isEmpty()){
        		endExpansion = System.currentTimeMillis();
        		expansionTime = endExpansion-startExpansion;
        		return false;
        	}
        	Box nextBox = GBFQ.peek();
        	if (nextBox.status==Status.FREE){
        		Box b = GBFQ.poll();
        		b.addToUF();
            	for(Box n:b.N){
            		if (n.inQ == false){
            			GBFQ.add(n);
            			n.inQ = true;
            		}
            	}
            	for(Box s:b.S){
            		if (s.inQ == false){
            			GBFQ.add(s);
            			s.inQ = true;
            		}
            	}
            	for(Box e:b.E){
            		if (e.inQ == false){
            			GBFQ.add(e);
            			e.inQ = true;
            		}
            	}
            	for(Box w:b.W){
            		if (w.inQ == false){
            			GBFQ.add(w);
            			w.inQ = true;
            		}
            	}
        	}
        	
        	else if (nextBox.status==Status.MIXED){
    			Box b = GBFQ.poll();
        		if (b.split() != false){
        		for (Record<Box> r: uf.getRecords()){
        			Box s = r.getName();
        			if (b.Q1.inQ == false && b.Q1.adjacentTo(s)){
        				GBFQ.add(b.Q1);
        				b.Q1.inQ = true;
        			}
        			if (b.Q2.inQ == false && b.Q2.adjacentTo(s)){
        				GBFQ.add(b.Q2);
        				b.Q2.inQ = true;
        			}
        			if (b.Q3.inQ == false && b.Q3.adjacentTo(s)){
        				GBFQ.add(b.Q3);
        				b.Q3.inQ = true;
        			}
        			if (b.Q4.inQ == false && b.Q4.adjacentTo(s)){
        				GBFQ.add(b.Q4);
        				b.Q4.inQ = true;
        			}
        		}
        	}
        	}
        }
        endExpansion = System.currentTimeMillis();
        expansionTime = endExpansion-startExpansion;
        LinkedList<Point2D> pointPath = new LinkedList<Point2D>();
        long startTime = System.currentTimeMillis();
        pointPath.add(target);
        for (Box node = targetbox.pathParent; node != startbox; node = node.pathParent)
            pointPath.add(new Point2D.Double(node.x + node.length/2, node.y + node.length/2));
        pointPath.add(start);
        Collections.reverse(pointPath);
        path = pointPath;
        long endTime = System.currentTimeMillis();
        pathTime = endTime-startTime;
        return true;
    }
    
    public void setEpsilon(int e) {
        epsilon = e;
    }
    
    public void setStrategy(int s){
    	strategy = s;
    }
    
    public ArrayList<Box> getUnderEpsilon(){
    	return underEpsilon;
    }
    
    public ArrayList<Box> getFree(){
        return free;
    }
    
    public ArrayList<Box> getStuck(){
    	return stuck;
    }
    
    public ArrayList<Box> getMixed(){
    	for (Box b:leaves){
    		if (b.status == Status.MIXED){
    			mixed.add(b);
    		}
    	}
    	return mixed;
    }
    
    public ArrayList<Box> getLeaves(){
        return leaves;
    }
    
    public LinkedList<Point2D> getPath(){
    	return path;
    }
    
    public long getExpansionTime(){
    	return expansionTime;
    }
    public long getPathTime(){
    	return pathTime;
    }
    private Box boxContaining(Point2D p){
    	Box container_box = null;
    	for (int i = 0; i < leaves.size(); i++){
    		Box leaf = leaves.get(i);
    		if (((p.getX() >= leaf.x) && (p.getX() <= leaf.x + leaf.length)) && ((p.getY() >= leaf.y) && (p.getY() <= leaf.y + leaf.length))){
    			container_box = leaves.get(i);
    			break;
    		}
    	}
    	return container_box;
    } 
    
    
    class Box implements Comparable<Box> {

        Box parent;
        Box pathParent;
        Box Q1, Q2, Q3, Q4;	//children
        double x;
        double y;
        double length;
        double priority;
        Status status;
        LinkedList<Box> N, S, E, W;	//adjacency lists
        Record<Box> record;  //for the Union-Find data structure
        Node node;	//for the graph
        boolean inQ;
 
        Box(double xCoord, double yCoord, double l){
           x = xCoord;
           y = yCoord;
           length = l;
           if (strategy==2) priority = (new Point2D.Double(x + length/2, y + length/2)).distance(target);
           N = new LinkedList<Box>();
           S = new LinkedList<Box>();
           E = new LinkedList<Box>();
           W = new LinkedList<Box>();
           this.classify();
           if (status == Status.STUCK){
        	   stuck.add(this);
           }
           else if (status == Status.FREE){
        	   free.add(this);
        	   if (strategy==0 || strategy==1) node = new Node(new Point2D.Double(x + length/2, y + length/2));
           }
           else{
        	   if (strategy==0 || strategy==1){
            	   BFSList.add(this);
            	   randomList.add(this);
        	   }
           }
        }
        
        public int compareTo(Box other) {
            return Double.compare(priority, other.priority);
        }
        
        //if the max depth is reached, returns false
        //else, divides the box into four sub-boxes and returns true
        boolean split() {
            if (length < epsilon) {
            	BFSList.remove(this);
            	randomList.remove(this);
            	underEpsilon.add(this);
                return false;
            }
            else {                   
            	//modify coordinates according to the quadrant number
            	Q1 = new Box(x+(length/2), y, length/2);  
            	Q2 = new Box(x, y, length/2);
            	Q3 = new Box(x, y+(length/2), this.length/2);
            	Q4 = new Box(x+(length/2), y+(length/2), this.length/2);

            	Q1.parent = this;
            	Q2.parent = this;
            	Q3.parent = this;
            	Q4.parent = this;
            	
            	Q1.W.add(Q2);
            	Q1.S.add(Q4);
            	Q2.E.add(Q1);
            	Q2.S.add(Q3);
            	Q3.N.add(Q2);
            	Q3.E.add(Q4);
            	Q4.N.add(Q1);
            	Q4.W.add(Q3);
            	
            	Q1.updateAdjacencies();
            	Q2.updateAdjacencies();
            	Q3.updateAdjacencies();
            	Q4.updateAdjacencies();
            	if (Q1.status == Status.FREE){
            		if (strategy==0 || strategy==1) Q1.addToUF();
            	}
            	leaves.add(Q1);
            	if(Q2.status == Status.FREE){
            		if (strategy==0 || strategy==1)Q2.addToUF();
            	}
            	leaves.add(Q2);
            	if(Q3.status == Status.FREE){
            		if (strategy==0 || strategy==1)Q3.addToUF();
            	}
            	leaves.add(Q3);
            	if(Q4.status == Status.FREE){
            		if (strategy==0 || strategy==1)Q4.addToUF();
            	}
            	leaves.add(Q4);
            	leaves.remove(this); //removes the current box from leaves since it now has children
            	N.clear();
            	S.clear();
            	E.clear();
            	W.clear();
            	BFSList.remove(this);
            	randomList.remove(this);
            }
            return true;
        }
        
        //The box predicate--determines whether boxes are stuck, mixed or free
        void classify() {
        	for (int o = 0; o < obstacles.size(); o++){
	        	Point2D box_center = new Point2D.Double(x + (length/2), y + (length/2));
	        	double box_radius = box_center.distance(new Point2D.Double(x,y));
	        	
	        	double outerDomain_radius = box_radius + robot_radius;
	        	Ellipse2D outerDomain = new Ellipse2D.Double(box_center.getX() - outerDomain_radius, 
	        			box_center.getY() - outerDomain_radius, outerDomain_radius*2, outerDomain_radius*2);
	        	
	        	Area od = new Area(outerDomain); 
	        	od.intersect(obstacles.get(o));  //sets od equal to the intersection of od and the obstacle
	
	        	if (od.isEmpty()){
	        		if (o==obstacles.size()-1){
	        			status = Status.FREE;
	        			return;
	        		}
	        		continue;
	        	}
	        	
	        	if (robot_radius > box_radius){
	            	double innerDomain_radius = robot_radius - box_radius;
	            	Ellipse2D innerDomain = new Ellipse2D.Double(box_center.getX() - (innerDomain_radius), 
	            			box_center.getY() - (innerDomain_radius), 
	            			innerDomain_radius*2, innerDomain_radius*2);
	            	
	            	Area id = new Area(innerDomain);
	            	id.intersect(obstacles.get(o));
	        		
	            	if (!id.isEmpty()){
		        		status = Status.STUCK;
		        		return;
	        		}
	        	}
	            
	        	status = Status.MIXED;
	            return;
        	}
        }
        
        void addToUF(){
    		record = new Record<Box>(this);
    		uf.addRecord(record);
    		ArrayList<Record<Box>> records = uf.getRecords();
    		for (Record<Box> r:records){
    			Box b = r.getName();
    			if (this.adjacentTo(b)){
    				if(!uf.sameSet(record, r)){
        				uf.Union(record, r);
        				pathParent = b;
    				}
    			}
    		}
        }
        
        boolean adjacentTo(Box b){
        	if (N.contains(b) || S.contains(b) || E.contains(b) || W.contains(b)){
        		return true;
        	}
        	return false;
        }
        
        void updateAdjacencies(){
        	if (parent.Q1 == this){
        		for(Box a:parent.N){
        			if ((a.x > this.x - a.length) && (a.x < this.x + this.length)){
        				this.N.add(a);
        				a.S.add(this);
        			}
        			a.S.remove(parent);
        		}
        		for(Box a:parent.E){
        			if ((a.y > this.y - a.length) && (a.y < this.y + this.length)){
        				this.E.add(a);
        				a.W.add(this);
        			}
        			a.W.remove(parent);
        		}
        	}
        	else if(parent.Q2 == this){
        		for (Box a:parent.N){
        			if((a.x > this.x - a.length) && (a.x < this.x + this.length)){
        				this.N.add(a);
        				a.S.add(this);
        			}
        			a.S.remove(parent);
        			
        		}
        		for(Box a: parent.W){
        			if((a.y > this.y - a.length) && (a.y < this.y + this.length)){
        				this.W.add(a);
        				a.E.add(this);
        			}
        			a.E.remove(parent);
        			
        		}
        		
        	}
        	else if (parent.Q3 == this){
        		for(Box a: parent.W){
        			if((a.y > this.y - a.length) && (a.y < this.y + this.length)){
        				this.W.add(a);
        				a.E.add(this);
        			}
        			a.E.remove(parent);
        			
        		}
        		for(Box a: parent.S){
        			if((a.x > this.x - a.length) && (a.x < this.x + this.length)){
        				this.S.add(a);
        				a.N.add(this);
        			}
        			a.N.remove(parent);
        		}
        		
        	}
        	else if (parent.Q4 == this){
        		for(Box a:parent.E){
        			if((a.y > this.y - a.length) && (a.y < this.y + this.length)){
        				this.E.add(a);
        				a.W.add(this);
        			}
        			a.W.remove(parent);
        		}
        		for(Box a: parent.S){
        			if ((a.x > this.x - a.length) && (a.x < this.x + this.length)){
        				this.S.add(a);
        				a.N.add(this);
        			}
        			
        			a.N.remove(parent);
        		}
        		
        	}
        }
	}
        
}
