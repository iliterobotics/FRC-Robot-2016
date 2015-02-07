package org.usfirst.frc.team1885.robot.comms;

public class VisionMessage extends Message {
	private static final long serialVersionUID = -8096425948256627421L;
	private double distance;
	
	public double getDistance(){
		return distance;
	}
	
	public void setDistance(double d){
		distance = d;
	}
	
	public long getSerialID() {
		return serialVersionUID;
	}
	
	@Override
	public String toString() {
		return "VisionMessage [distance=" + distance + "]";
	}
	
}