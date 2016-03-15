package com.roboclub.robobuggy.map;

import Jama.Matrix;

/**
 * 
 * @author Trevor Decker
 * Representation of a so2 point (x,y,orientation) 
 *
 */
public class So2Pose {
	private Point location;
	private double orientation;

	/**
	 * 
	 * @param newLocation the new location
	 * @param newOrientation the new orientation
	 */
	public So2Pose(Point newLocation,double newOrientation){
		this.location = newLocation;
		this.orientation = newOrientation;
	}
	
	/**
	 * 
	 * @param x x coord of the point
	 * @param y y coord of the point
	 * @param newOrientation the new orientation
	 */
	public So2Pose(double x,double y,double newOrientation){
		location = new Point(x, y);
		orientation = newOrientation;
	}
	
	/**
	 * 
	 * @param postPose the pose that is being applied to the right of the expresion
	 * @return the new So2Pose TODO
	 */
	public So2Pose mult(So2Pose postPose){
		double[][] aM = {{Math.cos(orientation), -Math.sin(orientation), getX()},
				         {Math.sin(orientation), Math.cos(orientation),getY()},
				         {0,0,1}};
		double[][] bM = {{Math.cos(postPose.orientation), -Math.sin(postPose.orientation), postPose.getX()},
		         		{Math.sin(postPose.orientation), Math.cos(postPose.orientation),postPose.getY()},
		         		{0,0,1}};
		Matrix a = new Matrix(aM);
		Matrix b = new Matrix(bM);
		Matrix c = a.times(b);

		
		return new So2Pose(c.get(0, 2), c.get(1,2), Math.atan2(c.get(1, 0), c.get(0, 0)));
		
	}
	
	
	/**
	 * updates the values of the pose
	 * @param newPoint the new se2 point to be set
	 * @param newOrientation the new orientation
	 */
	public void updatePoint(Point newPoint, double newOrientation){
		this.orientation = newOrientation;
		this.location = newPoint;

	}
	
	/**
	 * returns the most recent position (se2 point) value 
	 * @return se2 Point
	 */
	public Point getSe2Point(){
		return location;
	}
	
	/**
	 * 
	 * @return the x coordinate of the of the se2 position of the pose 
	 */
	public double getX(){
			return location.getX();
	}
	
	/**
	 * 
	 * @return the y coordinate of the se2 position of the pose 
	 */
	public double getY(){
			return location.getY();
	}
	
	/**
	 * 
	 * @return the orientation of the pose
	 */
	public double getOrientation(){
		return orientation;
	}

}