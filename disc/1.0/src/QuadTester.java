/* QuadTester.java
 *
 *  Description:
 *  		This program reads a description of an environment
 *  		from an input file, for a disc robot.
 *
 *  		The environment is contained in a box B_0, and we
 *  		keep subdividing the box until the size of boxes is less than
 *  		some epsilon, or the box is free or stuck, or until we have
 *  		found a path from start to goal.
 *
 *  		We then display the subdivision boxes and their status
 *  		(free/stuck/mixed/epsilon).
 *
 *  How to Use:
 *		Using the Makefile here, just type 
 *			> make QuadTester
 *		This compiles and runs the program.
 *  		
 *  Author: Surin Ahn (June 2012)
 *  -- supervised by Professor Yap
 *
 ***************************************************/
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.geom.*;
import java.awt.geom.Point2D;
import java.util.*;
import javax.swing.*;
import java.io.*;


public class QuadTester extends JFrame implements ActionListener{
    
    Quadtree tree;
    Robot robot;
    Point2D target;
    HashMap<String, Point2D> points;
    ArrayList<Area> obstacles;
    ArrayList<Quadtree.Box> leaves;
    ArrayList<Quadtree.Box> blocked;

    // GUI:
    JFrame frame;
    Drawing d;
    JPanel rightPanel;
    JPanel filePanel;
    JPanel radiusPanel;
    JPanel startPanel;
    JPanel targPanel;
    JPanel epsilonPanel;
    JPanel buttonPanel;
    JTextField fileField;
    JTextField radiusField;
    JTextField xStartField;
    JTextField yStartField;
    JTextField xTargField;
    JTextField yTargField;
    JTextField epsilonField;
    JLabel fileLabel;
    JLabel radiusLabel;
    JLabel startLabel;
    JLabel targLabel;
    JLabel epsilonLabel;
    JButton start;
    JButton clear;
    boolean start_clicked=false;
    
    // main method
    public static void main(String[] args) {
    	QuadTester t = new QuadTester();
        t.setGUI();
    }
    
    // run method
    public void setGUI() {
    	frame = new JFrame("Quadtree");
        frame.setSize(800, 680);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        frame.setLayout(new BorderLayout());
        rightPanel = new JPanel();
        
        fileField = new JTextField("../inputData/input.txt", 8);
        radiusField = new JTextField("45", 4);
        xStartField = new JTextField("70", 4);
        yStartField = new JTextField("400", 4);
        xTargField = new JTextField("500", 4);
        yTargField = new JTextField("70", 4);
        epsilonField = new JTextField("10", 4);
        
        rightPanel.setLayout(new GridLayout(0,1));

        filePanel = new JPanel();
        fileLabel = new JLabel("File:");
        filePanel.add(fileLabel);
        filePanel.add(fileField);
        radiusPanel = new JPanel();
        radiusLabel = new JLabel("Radius:");
        radiusPanel.add(radiusLabel);
        radiusPanel.add(radiusField);
        startPanel = new JPanel();
        startLabel = new JLabel("Start:");
        startPanel.add(startLabel);
        startPanel.add(xStartField);
        startPanel.add(yStartField);
        targPanel = new JPanel();
        targLabel = new JLabel("Target:");
        targPanel.add(targLabel);
        targPanel.add(xTargField);
        targPanel.add(yTargField);
        rightPanel.add(filePanel);
        rightPanel.add(radiusPanel);
        rightPanel.add(startPanel);
        rightPanel.add(targPanel);
        
        buttonPanel = new JPanel();
        start = new JButton("Start");
        start.addActionListener(this);
        clear = new JButton("Clear");
        clear.addActionListener(this);
        buttonPanel.add(start);
        buttonPanel.add(clear);
        rightPanel.add(buttonPanel);

        frame.getContentPane().add(rightPanel, BorderLayout.EAST);
        d = new Drawing();
        frame.getContentPane().add(d, BorderLayout.CENTER);
        frame.setVisible(true);
    }  
    
    public void readInput(File f){
    	try {
    		points.clear();
    		obstacles.clear();
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
					points.put(name, p);
				}
				//define an obstacle
				else if (token.contentEquals("o:")){
					ArrayList<Double> xPoints = new ArrayList<Double>();
					ArrayList<Double> yPoints = new ArrayList<Double>();
					while (st.hasMoreTokens()){
						Point2D p = points.get(st.nextToken());
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
    
    class Drawing extends JPanel {
        public void paintComponent(Graphics g) {
             super.paintComponent(g); 
             Graphics2D g2 = (Graphics2D) g;
             g2.setStroke(new BasicStroke(2));
             
             if(start_clicked){
             //fill mixed boxes
             g2.setColor(Color.gray);
             for (int i = 0; i < leaves.size(); i++) {
                 g2.fill(leaves.get(i).getRect());
             }
             
             //fill blocked boxes
             g2.setColor(Color.red);
             for (int i = 0; i < blocked.size(); i++){
            	 g2.fill(blocked.get(i).getRect());
             }
             
             //fill free boxes
             g2.setColor(Color.green);
             for (int i = 0; i < tree.getFree().size(); i++) {
            	 g2.fill(tree.getFree().get(i).getRect());   
             }
             
             //fill underEpsilon boxes
             g2.setColor(Color.yellow);
             for (int i = 0; i < tree.getUnderEpsilon().size(); i++){
            	 g2.fill(tree.getUnderEpsilon().get(i).getRect());
             }
             
             //outline boxes
             g2.setColor(Color.black);
             for (int i = 0; i < leaves.size(); i++) {
                     g2.draw(leaves.get(i).getRect());
             }
             
             //draw obstacles
             g2.setColor(Color.white);
             for (int i = 0; i < obstacles.size(); i++) {
                 g2.draw(obstacles.get(i));
             }
             
             //draw target configuration
             g2.setStroke(new BasicStroke(4));
             g2.setColor(Color.black);
             g2.draw(new Line2D.Double(target.getX(), target.getY(), target.getX(), target.getY()));
             g2.setStroke(new BasicStroke(2));
             g2.setColor(Color.blue);
             Ellipse2D tar = new Ellipse2D.Double(target.getX() - robot.getRadius(), 
            		 target.getY() - robot.getRadius(), robot.getRadius()*2, robot.getRadius()*2);
             g2.draw(tar);
             
             //draw robot
             Ellipse2D r = new Ellipse2D.Double(robot.getX() - robot.getRadius(), 
            		 robot.getY() - robot.getRadius(), robot.getRadius()*2, robot.getRadius()*2);
             g2.draw(r);
             g2.setColor(Color.white);
             g2.setStroke(new BasicStroke(4));
             g2.draw(new Line2D.Double(robot.getX(), robot.getY(), robot.getX(), robot.getY()));
             }
             
             else{
            	 g2.draw(new Rectangle(0,0,frame.getHeight()-80, frame.getHeight()-80));
             }
        }
    }

	@Override
	public void actionPerformed(ActionEvent e) {
        
		if(e.getSource()==start){
	    	tree = new Quadtree(frame.getHeight()-80);
	    	points = new HashMap<String, Point2D>();
	        obstacles = new ArrayList<Area>(); 
	        leaves = new ArrayList<Quadtree.Box>();
	        blocked = new ArrayList<Quadtree.Box>();
			
			readInput(new File (fileField.getText()));
			robot = new Robot(Integer.parseInt(xStartField.getText()), Integer.parseInt(yStartField.getText()), Integer.parseInt(radiusField.getText()));
			target = new Point2D.Double(Integer.parseInt(xTargField.getText()), Integer.parseInt(yTargField.getText()));
	    	tree.setEpsilon(Integer.parseInt(epsilonField.getText()));
        
			tree.subdivide(obstacles, robot.getRadius());
        
			leaves = tree.getLeaves();
			blocked = tree.getBlocked();
			start_clicked=true;
			d.repaint();
		}
		
		else if (e.getSource()==clear){
			start_clicked=false;
			d.repaint();
		}
	}
}
    
 
