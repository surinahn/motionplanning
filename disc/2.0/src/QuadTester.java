/* QuadTester.java
 *
 *  Description:
 *  		This program reads a description of an environment
 *  		from an input file, for a disc robot.
 *
 *  		The environment is contained in a box B_0, and we
 *  		keep subdividing the box until the size of boxes is less than
 *  		some epsilon, or the box is free or stuck, or until we have
 *  		found a path from runAgain to goal.
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
    
    static Quadtree tree;
    static Robot robot;
    static Point2D startConfig;
    static Point2D target;
    static HashMap<String, Point2D> points;
    static ArrayList<Area> obstacles;
    static ArrayList<Quadtree.Box> leaves, stuck, free, mixed, underEpsilon;
    static LinkedList<Point2D> path;
    static final int rootSize = 600;
    

    // GUI:
    static JFrame frame;
    static Drawing d;
    static JPanel rightPanel, filePanel, stratPanel, 
    radiusPanel, configPanel, buttonPanel, pathPanel;
    static JTextField fileField, dirField,
    		radiusField, epsilonField,
		xStartField, yStartField, xTargField, yTargField;
    static JComboBox stratList;
    static String[] stratStrings = {"BFS", "Random", "GBF"};
    JLabel fileLabel, dirLabel, stratLabel, radiusLabel, startLabel, targLabel, epsilonLabel;
    static JTextArea statistics;
    JButton runAgain, clear;
    static boolean start_clicked;
    
    // main method
    public static void main(String[] args) {
        fileField = new JTextField("input2.txt", 8);
        dirField = new JTextField("../inputData/", 8);
        radiusField = new JTextField("40", 4);
        epsilonField = new JTextField("4", 4);
        xStartField = new JTextField("50", 4);
        yStartField = new JTextField("50", 4);
        xTargField = new JTextField("400", 4);
        yTargField = new JTextField("500", 4);
        stratList = new JComboBox(stratStrings);
        stratList.setSelectedIndex(0);	//BFS is the default strategy


	if (args.length>0) fileField = new JTextField(args[0], 8);
	if (args.length>1) dirField = new JTextField(args[1], 8);
	if (args.length>2) stratList.setSelectedIndex(Integer.parseInt(args[2]));
	if (args.length>3) radiusField = new JTextField(args[3], 4);
	if (args.length>4) epsilonField = new JTextField(args[4], 4);
	if (args.length>5) xStartField = new JTextField(args[5], 4);
	if (args.length>6) yStartField = new JTextField(args[6], 4);
	if (args.length>7) xTargField = new JTextField(args[7], 4);
	if (args.length>8) yTargField = new JTextField(args[8], 4);
	
    	QuadTester t = new QuadTester();
        t.setGUI();
        run();
    }
    
    // run method
    public void setGUI() {
    	frame = new JFrame("Quadtree");
        frame.setSize(1000, 700);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        frame.setLayout(new BorderLayout());
        rightPanel = new JPanel();

        fileLabel = new JLabel("File:");
        dirLabel = new JLabel("   Directory:");
        stratLabel = new JLabel("Search Strategy:");
        radiusLabel = new JLabel("Radius:");
        epsilonLabel = new JLabel("   Epsilon:");
        startLabel = new JLabel("Start:");
        targLabel = new JLabel("   Target:");
        statistics = new JTextArea(30, 16);
        
        rightPanel.setLayout(new GridLayout(6, 1, 0, 0));
        
        filePanel = new JPanel();
        filePanel.add(fileLabel);
        filePanel.add(fileField);
        filePanel.add(dirLabel);
        filePanel.add(dirField);
        stratPanel = new JPanel();
        stratPanel.add(stratLabel);
        stratPanel.add(stratList);
        radiusPanel = new JPanel();
        radiusPanel.add(radiusLabel);
        radiusPanel.add(radiusField);
        radiusPanel.add(epsilonLabel);
        radiusPanel.add(epsilonField);
        configPanel = new JPanel();
        configPanel.add(startLabel);
        configPanel.add(xStartField);
        configPanel.add(yStartField);
        configPanel.add(targLabel);
        configPanel.add(xTargField);
        configPanel.add(yTargField);
        pathPanel = new JPanel();
        pathPanel.add(statistics);
        rightPanel.add(filePanel);
        rightPanel.add(stratPanel);
        rightPanel.add(radiusPanel);
        rightPanel.add(configPanel);
        rightPanel.add(pathPanel); 
        
        buttonPanel = new JPanel();
        runAgain = new JButton("Run Again");
        runAgain.addActionListener(this);
        clear = new JButton("Clear");
        clear.addActionListener(this);
        buttonPanel.add(runAgain);
        buttonPanel.add(clear);
        buttonPanel.setBorder(BorderFactory.createEmptyBorder(20, 0, 0, 0));
        rightPanel.add(buttonPanel);

        frame.getContentPane().add(rightPanel, BorderLayout.EAST);
        d = new Drawing();
        frame.getContentPane().add(d, BorderLayout.CENTER);
        frame.setVisible(true);
    }  
    
    public static void readInput(File f){
    	try {
    		points.clear();
    		obstacles.clear();
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
						points.put(name, p);
						continue;
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
    
    class Drawing extends JPanel {
        public void paintComponent(Graphics g) {
             super.paintComponent(g); 
             Graphics2D g2 = (Graphics2D) g;
             g2.setStroke(new BasicStroke(2));
             
             if(start_clicked){
            	 //fill mixed boxes
            	 g2.setColor(Color.yellow);
            	 for (int i = 0; i < mixed.size(); i++) {
            		 Quadtree.Box m = mixed.get(i);
                     g2.fill(new Rectangle2D.Double(m.x, m.y, m.length, m.length));
                 }
	             //fill stuck boxes
	             g2.setColor(Color.red);
	             for (int i = 0; i < stuck.size(); i++){
	            	 Quadtree.Box s = stuck.get(i);
	            	 g2.fill(new Rectangle2D.Double(s.x, s.y, s.length, s.length));
	             }
	             
	             //fill free boxes
	             g2.setColor(Color.green);
	             for (int i = 0; i < free.size(); i++) {
	            	 Quadtree.Box f = free.get(i);
	            	 g2.fill(new Rectangle2D.Double(f.x, f.y, f.length, f.length));   
	             }
	             
	             //fill underEpsilon boxes
	             g2.setColor(Color.gray);
	             for (int i = 0; i < underEpsilon.size(); i++){
	            	 Quadtree.Box u = underEpsilon.get(i);
	            	 g2.fill(new Rectangle2D.Double(u.x, u.y, u.length, u.length));
	             }
	             
	             for (int i = 0; i < leaves.size(); i++) {
	            	 Quadtree.Box leaf = leaves.get(i);
	                 //outline boxes
	                 g2.setColor(Color.black);
	                 g2.draw(new Rectangle2D.Double(leaf.x, leaf.y, leaf.length, leaf.length));
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
	             
	             g2.setColor(Color.magenta);
	             for (int i = 1; i < path.size(); i++){
	            	 Line2D l = new Line2D.Double(path.get(i-1), path.get(i));
	            	 g2.draw(l);
	             }
             }
             
             else g2.draw(new Rectangle(0,0,rootSize, rootSize));
        }
    }

	@Override
	public void actionPerformed(ActionEvent e) {
        
		if(e.getSource()==runAgain){
			run();
		}
		
		else if (e.getSource()==clear){
			start_clicked=false;
			d.repaint();
		}
	}
	
	public static void run(){
        obstacles = new ArrayList<Area>();
        leaves = new ArrayList<Quadtree.Box>();
        stuck = new ArrayList<Quadtree.Box>();
        mixed = new ArrayList<Quadtree.Box>();
        points = new HashMap<String, Point2D>();
        path = new LinkedList<Point2D>();
        readInput(new File (dirField.getText() + fileField.getText()));
		startConfig = new Point2D.Double(Double.parseDouble(xStartField.getText()), Double.parseDouble(yStartField.getText()));
		robot = new Robot(startConfig.getX(), startConfig.getY(), Double.parseDouble(radiusField.getText()));
		target = new Point2D.Double(Double.parseDouble(xTargField.getText()), Double.parseDouble(yTargField.getText()));
    	tree = new Quadtree(rootSize, obstacles, robot.getRadius(), startConfig, target); 
	
    	tree.setEpsilon(Integer.parseInt(epsilonField.getText()));
    	tree.setStrategy(stratList.getSelectedIndex());

       	boolean pathStatus = tree.findPath();
    	
    	leaves = tree.getLeaves();
    	stuck = tree.getStuck();
   		free = tree.getFree();
   		mixed = tree.getMixed();
   		underEpsilon = tree.getUnderEpsilon();
   		path = tree.getPath();
   		start_clicked=true;
   		long expansionTime = tree.getExpansionTime();
   		long pathTime = tree.getPathTime();
		statistics.setText("");
		
		if(pathStatus == true){
			statistics.append("Path Found!\n");
		}
		else{
			statistics.append("No Path\n");
		}
		statistics.append("Free: " + Integer.toString(free.size()) + "\n");
		statistics.append("Stuck: " + Integer.toString(stuck.size()) + "\n");
		statistics.append("Mixed: " + Integer.toString(mixed.size()) + "\n");
		statistics.append("Under Epsilon: " + Integer.toString(underEpsilon.size()) + "\n");
		statistics.append("Expansion Time: " + expansionTime + " ms\n");
		statistics.append("Pathfinding Time: " + pathTime + " ms");
		d.repaint();
	}
}