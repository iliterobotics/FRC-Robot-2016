package org.usfirst.frc.team1885.robot.comms;

import org.usfirst.frc.team1885.robot.common.type.ServerMessageType;

public class VisionMessage extends Message {
	private static final long serialVersionUID = -8096425948256627421L;
	private double distance;

	public VisionMessage() {
		super(ServerMessageType.VISION_MSG);
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double d) {
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