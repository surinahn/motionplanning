/* RRT.java
 *
 *  Description:
 *  		This is an implementation of a rapidly-exploring 
 *  		random tree that attempts to connect the start and 
 *  		target configurations and then find a path for the
 *  		robot using Dijkstra's Algorithm.
 *
 *  How to Use:
 *		Using the Makefile here, just type 
 *			> make RRT
 *		This compiles and runs the program.
 *  		
 *  Author: Surin Ahn (June 2012)
 *  -- supervised by Professor Yap
 *
 ***************************************************/
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.*;
import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.StringTokenizer;

import javax.swing.*;

public class RRT{
	JFrame frame;
	DrawPanel p;
	int XDIM;
	int YDIM;
	
	static double EPSILON;
	static int NUMNODES;
	static int nodeCount = 1; 
	
	ArrayList<Line2D> lines;
	HashMap<Point2D, Node> nodes;
	static HashMap<String, Point2D> obstaclePoints;
    static ArrayList<Area> obstacles;
    static Robot robot;
    Point2D start;
    static Point2D target;
    static String file;
    static String dir;
    static double radius;
    static double startx;
    static double starty;
    static double goalx;
    static double goaly;
    static AStar astar;
    List<Point2D> path;
    boolean pathFound;
	boolean treeExpanding = true;
	static long endTime = 0;
    
	public static void main (String[] args){
		EPSILON = 10.0;
		NUMNODES = 5000;
		file = "input2.txt";
		dir = "../inputData/";
		radius = 40;
		startx = 50;
		starty = 50;
		goalx = 400;
		goaly = 500;
		
		if (args.length>0) file = args[0];
		if (args.length>1) dir = args[1];
		if (args.length>2) radius = Double.parseDouble(args[2]);
		if (args.length>3) EPSILON = Double.parseDouble(args[3]);
		if (args.length>4) startx = Double.parseDouble(args[4]);
		if (args.length>5) starty = Double.parseDouble(args[5]);
		if (args.length>6) goalx = Double.parseDouble(args[6]);
		if (args.length>7) goaly = Double.parseDouble(args[7]);
		
		RRT rrt = new RRT();
		rrt.setUp();
		long startTime = System.currentTimeMillis();
		rrt.run(); 
		if (endTime != 0) System.out.println(endTime - startTime);
	}
	
	public void setUp(){
		//GUI:
		XDIM = 600;
		YDIM = 600;
		frame = new JFrame("Rapidly-Exploring Random Tree");
		frame.setSize(XDIM, YDIM);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		p = new DrawPanel();
		frame.getContentPane().add(p);
		frame.setVisible(true);
		
		lines = new ArrayList<Line2D>();
		nodes = new HashMap<Point2D, Node>();
		robot = new Robot(startx, starty, radius);
		start = new Point2D.Double(startx, starty);
		nodes.put(start, new Node(start)); 
		
		obstaclePoints = new HashMap<String, Point2D>();
		obstacles = new ArrayList<Area>();
		target = new Point2D.Double(goalx, goaly);
		
		astar = new AStar();
		readInput(new File (dir + file));
	}
	
	public void run(){
		while (treeExpanding){
			if (nodeCount > NUMNODES){
				treeExpanding = false;
				System.out.println("Reached node limit");
			}
			else{
				//check to see if a node in the RRT is near the target
				for(Point2D point : nodes.keySet()){
					if (point.distance(target) < radius){
						nodes.put(target, new Node(new Point2D.Double(target.getX(), target.getY())));
						nodes.get(point).addNeighbor(nodes.get(target));
						lines.add(new Line2D.Double (point, target));
						treeExpanding = false;
						
						path = astar.computePath(nodes.get(start), nodes.get(target));
						pathFound = true;
						endTime = System.currentTimeMillis();
						p.repaint();
						return;
					}
				}
				
				if (treeExpanding){
					Point2D rand = new Point2D.Double(Math.random()*600.0, Math.random()*600.0);
					Point2D nn = start;
					//find the node nearest to the random point
					for (Point2D pt:nodes.keySet()){
						if (pt.distance(rand) < nn.distance(rand)){
							nn = pt;
						}
					}
					
					Point2D newnode = stepFromTo(nn, rand);
					Ellipse2D outerDomain = new Ellipse2D.Double(newnode.getX() - robot.getRadius(), 
			          		 newnode.getY() - robot.getRadius(), robot.getRadius()*2, robot.getRadius()*2);
		
					for (int o = 0; o < obstacles.size(); o++){
						Area od = new Area(outerDomain); 
			        	od.intersect(obstacles.get(o));
						
			        	if (!od.isEmpty()){
							break;
						}
						else{
							if (o==obstacles.size()-1) {
								nodes.put(newnode, new Node(new Point2D.Double(newnode.getX(), newnode.getY())));
								nodes.get(nn).addNeighbor(nodes.get(newnode));
								lines.add(new Line2D.Double(nn, newnode));
								nodeCount++;
		                    }
		                    else {
		                        continue;
		                    }
		
						}
					}
				}
			}
		}
	}
	
	public  Point2D stepFromTo(Point2D p1, Point2D p2){
		if (p1.distance(p2) < EPSILON){
			return p2;
		}
		else{
			double theta = Math.atan2((p2.getY()-p1.getY()), (p2.getX()-p1.getX()));
		    return new Point2D.Double(p1.getX() + EPSILON*Math.cos(theta), p1.getY() + EPSILON*Math.sin(theta));
		}
	}
	
	//read in obstacle info
    public static void readInput(File f){
    	try {
			BufferedReader reader = new BufferedReader(new FileReader(f));
			String line = null;
			while (true){
				line = reader.readLine();
				StringTokenizer st = null;
				if (line == null) continue;
				else st = new StringTokenizer(line);
				
				while(st.hasMoreTokens()){
					String token = st.nextToken();
					if (token.contains("#")){
						break;
					}
					//define a point
					else if (token.contentEquals("p:")){
						String name = st.nextToken();
						double x = Double.parseDouble(st.nextToken());
						double y = Double.parseDouble(st.nextToken());
						Point2D p = new Point2D.Double(x, y);
						obstaclePoints.put(name, p);
						continue;
					}

					//define an obstacle
					else if (token.contentEquals("o:")){
						ArrayList<Double> xPoints = new ArrayList<Double>();
						ArrayList<Double> yPoints = new ArrayList<Double>();
						while (st.hasMoreTokens()){
							Point2D p = obstaclePoints.get(st.nextToken());
							xPoints.add(p.getX());
							yPoints.add(p.getY());
						}
						Path2D polygon = new Path2D.Double(Path2D.WIND_EVEN_ODD, xPoints.size());
						polygon.moveTo(xPoints.get(0), yPoints.get(0));
						for (int index = 1; index < xPoints.size(); index++) {
			                polygon.lineTo(xPoints.get(index), yPoints.get(index));
						}
						polygon.closePath();
				        Area poly = new Area(polygon);
				        obstacles.add(poly);
				        continue;
					}
					
					else if (token.contentEquals("end")){
						return;
					}
				}	

			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			System.exit(0);
		} catch (IOException ex){
			ex.printStackTrace();
			System.exit(0);
		}
    }
	
	class DrawPanel extends JPanel {
        public void paintComponent(Graphics g) {
             super.paintComponent(g); 
             Graphics2D g2 = (Graphics2D) g;
             p.setBackground(Color.black);
             
             if (pathFound){
            	//draw obstacles
                 g2.setColor(Color.red);
                 for (Area a: obstacles){
                	 g2.fill(a);
                 }
                 
                 //draw RRT
                 g2.setColor(Color.white);
                 for (Line2D l: lines){
                	 g2.draw(l);
                 }
                 
                 //draw the path
    	             g2.setColor(Color.magenta);
    	             for (int i = 1; i < path.size(); i++){
    	            	 Line2D l = new Line2D.Double(path.get(i-1), path.get(i));
    	            	 g2.draw(l);
    	             }
                 
                 //draw target configuration
                 g2.setStroke(new BasicStroke(2));
                 g2.setColor(Color.yellow);
                 Ellipse2D tar = new Ellipse2D.Double(target.getX() - robot.getRadius(), 
                		 target.getY() - robot.getRadius(), robot.getRadius()*2, robot.getRadius()*2);
                 g2.draw(tar);
                 g2.setColor(Color.orange);
                 g2.draw(new Line2D.Double(target.getX(), target.getY(), target.getX(), target.getY()));
                 
                 
                 //draw robot
                 g2.draw(new Line2D.Double(robot.getX(), robot.getY(), robot.getX(), robot.getY()));
                 g2.setColor(Color.green);
                 Ellipse2D r = new Ellipse2D.Double(robot.getX() - robot.getRadius(), 
                		 robot.getY() - robot.getRadius(), robot.getRadius()*2, robot.getRadius()*2);
                 g2.draw(r);
             }
             
        }
	}
}