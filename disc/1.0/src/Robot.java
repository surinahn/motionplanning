

public class Robot{
	private double x;
	private double y;
	private double radius;
	
	public Robot(int startX, int startY, int r){
		x = startX;
		y = startY;
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