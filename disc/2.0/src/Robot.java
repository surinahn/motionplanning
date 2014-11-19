

public class Robot{
	private double x;
	private double y;
	private double radius;
	
	public Robot(double xStart, double yStart, double r){
		x = xStart;
		y = yStart;
		radius = r;
	}
	
	public double getX(){
		return x;
	}
	public double getY(){
		return y;
	}
	
	public double getRadius(){
		return radius;
	}
	public void setCoords(double xpos, double ypos){
		x = xpos;
		y = ypos;
	}
}