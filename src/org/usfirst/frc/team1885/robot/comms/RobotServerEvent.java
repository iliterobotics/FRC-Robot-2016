package org.usfirst.frc.team1885.robot.comms;

public class RobotServerEvent {
	private String message;
	public RobotServerEvent( String m ){
		message = m;
	}
	/**
	 * @return the message
	 */
	public String getMessage() {
		return message;
	}
	/**
	 * @param message the message to set
	 */
	public void setMessage( String message ) {
		this.message = message;
	}
}
