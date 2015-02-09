package org.usfirst.frc.team1885.robot.comms;

import java.io.Serializable;

import org.usfirst.frc.team1885.robot.common.type.ServerMessageType;

public class Message implements Serializable/* In a Bottle! Yeah */{
	private final ServerMessageType messageType;
	private static final long serialVersionUID = -9192040333938840230L;
	private final long currentTimeStamp;

	public Message(ServerMessageType msg) {
		messageType = msg;
		currentTimeStamp = System.currentTimeMillis();
	}

	public ServerMessageType getType() {
		return messageType;
	}

	public long getSerialID() {
		return serialVersionUID;
	}

	public long getCurrentTimeStamp() {
		return currentTimeStamp;
	}

}
