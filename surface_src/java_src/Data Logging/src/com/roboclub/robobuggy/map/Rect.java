package com.roboclub.robobuggy.map;

public class Rect {
	private Point uR;
	private Point uL;
	private Point lR;
	private Point lL;
	
	public Rect(Point uR_, Point uL_, Point lR_, Point lL_) {
		this.uR = uR_;
		this.uL = uL_;
		this.lR = lR_;
		this.lL = lL_;
	}
	
	public boolean within(Point marker) {
		if (marker != null) {
			return (marker.getX() >= this.uL.getX()) && 
					(marker.getX() <= this.uR.getX()) &&
					(marker.getY() >= this.lL.getY()) && 
					(marker.getY() <= this.uL.getY());
		}
		
		return false;
	}
}