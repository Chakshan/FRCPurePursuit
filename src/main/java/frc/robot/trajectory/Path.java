// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectory;

import java.util.ArrayList;

/** Add your docs here. */
public class Path {
    private ArrayList<Pose> path;

    public Path(ArrayList<Pose> path) {
        this.path = path;
    }

    public Pose getPose(int ind) {
        return path.get(ind);
    }

    public void add(Pose pose) {
        path.add(pose);
    }

    public void modify(int ind, Pose pose) {
        path.set(ind, pose);
    }

    public int getSize() {
        return (int)path.size();
    }

    public Path getCopy() {
        Path ret = new Path(this.path);
        return ret;
    }

    public void modifyInd(int poseInd, int dimInd, double value) {
        Pose modify = path.get(poseInd);
        if(dimInd == 0) modify.setX(value);
        if(dimInd == 1) modify.setY(value);
        path.set(poseInd, modify);
    }

    public static Path injectPath(Path path, double spacing) {
        Path newPath = new Path(new ArrayList<Pose>());


        for(int i = 0; i < path.getSize() - 1; i++) {
            Pose curDif = path.getPose(i).dif(path.getPose(i + 1)); // getting the actual vector/path between the two adjacent waypoints
            int pointsOnPath = (int)Math.ceil(curDif.length() / spacing); // counting amt of points on the path
            curDif.rescale(spacing); // rescales so that the length of the vector is "spacing" instead of whatever it is rn
            for(int j = 0; j < pointsOnPath; j++) { // inject points for however many points need to be between them
                newPath.add(path.getPose(i)); // add the point
                Pose newPos = path.getPose(i).add(curDif); // update it so its an extra spacing distance away
                path.modify(i, newPos); // update the new point we add is that distance
            }
        }

        newPath.add(path.getPose((int)path.getSize() - 1)); // add the last point

        return newPath;
    }

    public static Path smoothPath(Path path, double weight_data, double weight_smooth, double tolerance) {
        Path newPath = path.getCopy();
        // higher weight_smooth -> more curvature
        // optimal value 0.75 -> 0.98
        // weight_data = 1 - weight_smooth
        // tolerance = 0.001
        // this is for best results according to paper

        double change = tolerance;
		while(change >= tolerance)
		{
			change = 0.0;
			for(int i=1; i<path.getSize()-1; i++)
				for(int j=0; j<2; j++)
				{
					double aux = newPath.getPose(i).getInd(j);
					newPath.getPose(i).setInd(j, newPath.getPose(i).getInd(j) + weight_data * (path.getPose(i).getInd(j) - newPath.getPose(i).getInd(j)) + weight_smooth * (newPath.getPose(i-1).getInd(j) + newPath.getPose(i+1).getInd(j) - (2.0 * newPath.getPose(i).getInd(j))));
					newPath.modifyInd(i, j, newPath.getPose(i).getInd(j) + weight_data * (path.getPose(i).getInd(j) - newPath.getPose(i).getInd(j)) + weight_smooth * (newPath.getPose(i-1).getInd(j) + newPath.getPose(i+1).getInd(j) - (2.0 * newPath.getPose(i).getInd(j))));
                    // question does this actually modify the initial arraylist????? i forgor how java works tbh
                    change += Math.abs(aux - newPath.getPose(i).getInd(j));	
				}					
		}


        return newPath;
    }

}
