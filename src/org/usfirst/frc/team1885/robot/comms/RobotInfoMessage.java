package org.usfirst.frc.team1885.robot.comms;

import org.usfirst.frc.team1885.robot.common.type.ServerMessageType;

public class RobotInfoMessage extends Message {

	public RobotInfoMessage() {
		super(ServerMessageType.ROBOTINFO_MSG);
	}

}
 