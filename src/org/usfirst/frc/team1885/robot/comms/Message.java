package org.usfirst.frc.team1885.robot.comms;

import java.io.Serializable;

public abstract class Message implements Serializable/* In a Bottle, Yeah */ {

	private static final long serialVersionUID = -9192040333938840230L;
	
	public long getSerialID() {
		return serialVersionUID;
	}

	
}
