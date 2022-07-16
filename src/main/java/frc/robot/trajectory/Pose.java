package frc.robot.trajectory;

public class Pose {
    
    private double x;
    private double y;
    private double theta;

    public Pose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double getX() { 
        return x; 
    }

    public double getY() { 
        return y; 
    }

    public double getHeading() { 
        return theta; 
    }

    public void setX(double x) { 
        this.x = x; 
    }

    public void setY(double y) { 
        this.y = y; 
    }

    public void setHeading(double theta) { 
        this.theta = theta; 
    }

    public void translate(double dx, double dy) {
        x += dx;
        y += dy;
    }

    public double distanceTo (Pose anotherPos) {
        double dx = x - anotherPos.getX();
        double dy = y - anotherPos.getY();
        
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }

    
}
