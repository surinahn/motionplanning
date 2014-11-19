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
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.StringTokenizer;

import javax.swing.*;

public class RRT implements ActionListener{
	JFrame frame;
	Panel p;
	int XDIM;
	int YDIM;
	Timer timer;
	
	double EPSILON;
	int NUMNODES;
	int nodeCount = 1; 
	
	ArrayList<Line2D> lines;
	HashMap<Point2D, Vertex> nodes;
	HashMap<String, Point2D> obstaclePoints;
    ArrayList<Area> obstacles;
    Robot robot;
    Point2D start;
    Point2D target;
    
    Dijkstra dijkstra;
    List<Point2D> path;
    int i = 0;	//keeps track of where the robot is in the path
    boolean pathFound;
	boolean treeExpanding = true;
    
	public static void main (String[] args){
		RRT rrt = new RRT();
		rrt.run();
	}
	
	public void run(){
		//GUI:
		XDIM = 600;
		YDIM = 600;
		frame = new JFrame("Rapidly-Exploring Random Tree");
		frame.setSize(XDIM, YDIM);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		p = new Panel();
		frame.getContentPane().add(p);
		frame.setVisible(true);
        
		//Assign values:
		EPSILON = 10.0;
		NUMNODES = 5000;
		lines = new ArrayList<Line2D>();
		
		robot = new Robot(70, 400, 45);
		nodes = new HashMap<Point2D, Vertex>();
		start = new Point2D.Double(robot.getX(), robot.getY());
		nodes.put(start, new Vertex(start.getX(), start.getY())); 
		
		obstaclePoints = new HashMap<String, Point2D>();
		obstacles = new ArrayList<Area>();
		target = new Point2D.Double(500, 70);
		
		dijkstra = new Dijkstra();
		
		readInput(new File("../inputData/input.txt"));
		timer = new Timer(2, this);
		timer.start();
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

	public void actionPerformed(ActionEvent e) {
		if (treeExpanding){
			if (nodeCount > NUMNODES){
				treeExpanding = false;
			}
			else{
				//check to see if a node in the RRT is near the target
				for(Point2D point : nodes.keySet()){
					if (point.distance(target) < EPSILON){
						nodes.put(target, new Vertex(target.getX(), target.getY()));
						nodes.get(point).addE(new Ed(nodes.get(target), point.distance(target)));
						lines.add(new Line2D.Double (point, target));
						p.repaint();
						treeExpanding = false;
						
						dijkstra.computePaths(nodes.get(start));
						path = dijkstra.getShortestPathTo(nodes.get(target));
						pathFound = true;
						p.repaint();
						break;
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
								nodes.put(newnode, new Vertex(newnode.getX(), newnode.getY()));
								nodes.get(nn).addE(new Ed(nodes.get(newnode), nn.distance(newnode)));
								lines.add(new Line2D.Double(nn, newnode));
								nodeCount++;
		                    }
		                    else {
		                        continue;
		                    }
		
						}
					}
				}
					p.repaint();
			}
		}
		
		if (pathFound){
			if (i < path.size() - 1){
				robot.setCoords(path.get(i+1).getX(), path.get(i+1).getY());
				p.repaint();
				i++;
			}
		}
	}
	
	//read in obstacle info
    public void readInput(File f){
    	try {
			BufferedReader reader = new BufferedReader(new FileReader(f));
			String line = null;
			while ((line = reader.readLine()) != null){
				StringTokenizer st = new StringTokenizer(line);
				String token = st.nextToken();
				if (token.contains("#")){
					continue;
				}	
				//define a point
				else if (token.contentEquals("p:")){
					String name = st.nextToken();
					double x = Double.parseDouble(st.nextToken());
					double y = Double.parseDouble(st.nextToken());
					Point2D p = new Point2D.Double(x, y);
					obstaclePoints.put(name, p);
				}
				//define and obstacle
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
	
	class Panel extends JPanel {
        public void paintComponent(Graphics g) {
             super.paintComponent(g); 
             Graphics2D g2 = (Graphics2D) g;
             p.setBackground(Color.black);
             
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
             if(pathFound){
	             g2.setColor(Color.magenta);
	             for (int i = 1; i < path.size(); i++){
	            	 Line2D l = new Line2D.Double(path.get(i-1), path.get(i));
	            	 g2.draw(l);
	             }
             }
             
             //draw target configuration
             g2.setStroke(new BasicStroke(2));
             g2.setColor(Color.yellow);
             Ellipse2D tar = new Ellipse2D.Double(target.getX() - robot.getRadius(), 
            		 target.getY() - robot.getRadius(), robot.getRadius()*2, robot.getRadius()*2);
             g2.draw(tar);
             g2.setStroke(new BasicStroke(6));
             g2.setColor(Color.orange);
             g2.draw(new Line2D.Double(target.getX(), target.getY(), target.getX(), target.getY()));
             
             
             //draw robot
             g2.draw(new Line2D.Double(robot.getX(), robot.getY(), robot.getX(), robot.getY()));
             g2.setStroke(new BasicStroke(2));
             g2.setColor(Color.green);
             Ellipse2D r = new Ellipse2D.Double(robot.getX() - robot.getRadius(), 
            		 robot.getY() - robot.getRadius(), robot.getRadius()*2, robot.getRadius()*2);
             g2.draw(r);
        }
	}
}