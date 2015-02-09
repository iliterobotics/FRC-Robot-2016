package org.usfirst.frc.team1885.robot.comms;

import org.usfirst.frc.team1885.robot.common.type.ServerMessageType;

/*  */
public class CameraDataService implements RobotServerListener {
	private static CameraDataService instance = null;
	private double distance;

	public CameraDataService() {
		RobotServer.getInstance().addListener(this);
	}

	public static CameraDataService getInstance() {
		if (instance == null)
			instance = new CameraDataService();
		return instance;
	}

	public void setDistance(double d) {
		distance = d;
	}

	@Override
	public void receivedServerEvent(RobotServerEvent event) {
		if (event.getMessage().getType() == ServerMessageType.VISION_MSG) {
			distance = ((VisionMessage) (event.getMessage())).getDistance();
			System.out.println("Distance updated to : " + distance);
		}
	}
}