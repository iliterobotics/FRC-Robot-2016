package org.usfirst.frc.team1885.robot.comms;

public class RobotServerEvent {
	private Message message;
	public RobotServerEvent( Message m ){
		message = m;
	}
	/**
	 * @return the message
	 */
	public Message getMessage() {
		return message;
	}
	/**
	 * @param message the message to set
	 */
	public void setMessage( Message m ) {
		this.message = m;
	}
}
