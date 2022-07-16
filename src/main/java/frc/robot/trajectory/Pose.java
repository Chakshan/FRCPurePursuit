package frc.robot.trajectory;

import java.util.ArrayList;

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

    public double length() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public void normalize() {
        double dist = length();
        x /= dist;
        y /= dist;
    }

    public void rescale(double dist) {
        normalize();
        x *= dist;
        y *= dist;
    }

    public Pose dif(Pose anotherPos) {
        return new Pose(anotherPos.getX() - x, anotherPos.getY() - y, 0.0);
    }

    public Pose add(Pose anotherPos) {
        return new Pose(anotherPos.getX() + x, anotherPos.getY() + y, 0.0);
    }

    // things to note for injectPath
    // using Pose's is kinda pointless cuz we're rlly not taking advantage of hte heading component
    // so ig ignore those values until we figure how to take advantage of them

    public static ArrayList<Pose> injectPath(ArrayList<Pose> path, double spacing) {
        ArrayList<Pose> newPath = new ArrayList<Pose>();


        for(int i = 0; i < (int)path.size() - 1; i++) {
            Pose curDif = path.get(i).dif(path.get(i + 1)); // getting the actual vector/path between the two adjacent waypoints
            int pointsOnPath = (int)Math.ceil(curDif.length() / spacing); // counting amt of points on the path
            curDif.rescale(spacing); // rescales so that the length of the vector is "spacing" instead of whatever it is rn
            for(int j = 0; j < pointsOnPath; j++) { // inject points for however many points need to be between them
                newPath.add(path.get(i)); // add the point
                Pose newPos = path.get(i).add(curDif); // update it so its an extra spacing distance away
                path.set(i, newPos); // update the new point we add is that distance
            }
        }

        newPath.add(path.get((int)path.size() - 1)); // add the last point

        return newPath;
    }
    
}
