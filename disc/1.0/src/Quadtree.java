/*
 * @author Surin Ahn
 */

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Area;
import java.util.*;


public class Quadtree{
    
    private Box root;
    private LinkedList<Box> boxQ; //the queue used in subdivision; will later be changed to a priority queue
    private ArrayList<Box> leaves;
    private ArrayList<Box> free;
    private ArrayList<Box> blocked;
    private ArrayList<Box> underEpsilon;
    private int epsilon;

    
    public Quadtree(double l) {
        root = new Box(l);
        boxQ = new LinkedList<Box>();
        leaves = new ArrayList<Box>();
        leaves.add(root);
        free = new ArrayList<Box>();
        blocked = new ArrayList<Box>();
        underEpsilon = new ArrayList<Box>();
        boxQ.add(root);
    }
    
    public void setEpsilon(int t) {
        epsilon = t;
    }
    
    public ArrayList<Box> getUnderEpsilon(){
    	return underEpsilon;
    }
    
    public ArrayList<Box> getFree(){
        return free;
    }
    
    public ArrayList<Box> getBlocked(){
    	return blocked;
    }
    
    public ArrayList<Box> getLeaves(){
        return leaves;
    }
    
    private Box boxContaining(Point2D p){
    	Box container_box = null;
    	for (int i = 0; i < leaves.size(); i++){
    		if (leaves.get(i).rectangle.contains(p)){
    			container_box = leaves.get(i);
    		}
    	}
    	return container_box;
    }
    
    //The entire subdivision procedure
    public void subdivide(ArrayList<Area> obstacles, double robot_radius) {
        
        while (!boxQ.isEmpty()) {
            for (int o = 0; o < obstacles.size(); o++) {
                
                if (boxQ.peek().classify(obstacles.get(o), robot_radius).equals("BLOCKED")) {
                    blocked.add(boxQ.poll());
                    break;
                }
                else if (boxQ.peek().classify(obstacles.get(o), robot_radius).equals("MIXED")) {
                    boxQ.poll().split();
                    break;
                }
                
                else if (boxQ.peek().classify(obstacles.get(o), robot_radius).equals("FREE")) {
                    //determine if there are no more obstacles to check for intersections
                    if (o==obstacles.size()-1) {
                        free.add(boxQ.poll());
                    }
                    else {
                        continue;
                    }
                }
            }
        }
        
    }
    
    class Box{

        Box parent;
        Box[] children;
        double x;
        double y;
        double length;
        Rectangle2D rectangle;
 
        Box(double l){
           x = 0;
           y = 0;
           length = l;
           children = new Box[4];
        }
        
        Rectangle2D getRect(){
        	return rectangle;
        }
        
        //if the max depth is reached, returns false
        //else, divides the box into four sub-boxes and returns true
        boolean split() {
            if (length < epsilon) {
            	underEpsilon.add(this);
                return false;
            }
            else {
            	leaves.remove(this); //removes the current box from leaves since it now has children 
            	rectangle = null;
            	boxQ.remove(this);
                for(int i = 0; i<4; i++) {
                    children[i] = new Box(this.length/2);
                    children[i].parent = this;
                    
                    //modify coordinates according to the quadrant number
                    if (i == 0) {
                        children[i].x = x + (length/2);
                        children[i].y = y;
                    }
                    else if (i==1) {
                        children[i].x = x;
                        children[i].y = y;
                    }
                    else if (i==2) {
                        children[i].x = x;
                        children[i].y = y + (length/2);
                    }
                    else {
                        children[i].x = x + (length/2);
                        children[i].y = y + (length/2);
                    }
                    
                    leaves.add(children[i]);
                    children[i].rectangle = new Rectangle2D.Double(children[i].x, children[i].y, children[i].length, children[i].length);
                    boxQ.add(children[i]);
                }
                return true;
            }
            
        }
        
        //The box predicate--determines whether boxes are blocked, mixed or free
        String classify(Area obstacle, double robot_radius) {
            
        	Point2D box_center = new Point2D.Double(x + (length/2), y + (length/2));
        	double box_radius = box_center.distance(new Point2D.Double(x,y));
        	
        	double outerDomain_radius = box_radius + robot_radius;
        	Ellipse2D outerDomain = new Ellipse2D.Double(box_center.getX() - outerDomain_radius, 
        			box_center.getY() - outerDomain_radius, outerDomain_radius*2, outerDomain_radius*2);
        	
        	Area od = new Area(outerDomain); 
        	od.intersect(obstacle);  //sets od equal to the intersection of od and the obstacle

        	if (od.isEmpty()){  
        		return "FREE";
        	}
        	
        	else if (robot_radius > box_radius){
            	double innerDomain_radius = robot_radius - box_radius;
            	Ellipse2D innerDomain = new Ellipse2D.Double(box_center.getX() - (innerDomain_radius), 
            			box_center.getY() - (innerDomain_radius), 
            			innerDomain_radius*2, innerDomain_radius*2);
            	
            	Area id = new Area(innerDomain);
            	id.intersect(obstacle);
        		
            	if (!id.isEmpty()){
        			return "BLOCKED";
        		}
        	}
            return "MIXED";
        }
        
    }

}