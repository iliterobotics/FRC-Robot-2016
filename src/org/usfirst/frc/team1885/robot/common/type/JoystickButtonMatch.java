package org.usfirst.frc.team1885.robot.common.type;

public class JoystickButtonMatch {
	private RobotJoystickType joystickType;
	private int port;
	
	public JoystickButtonMatch(RobotJoystickType joystickType, int port) {
		this.joystickType = joystickType;
		this.port = port;
	}
	
	public RobotJoystickType getJoystickType() {
		return joystickType;
	}
	
	public int getPort() {
		return port;
	}
}
