package org.usfirst.frc.team1885.robot.comms;

import java.util.ArrayList;

import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.Relay;


public class DataTelemetryService {
	private RobotControl rc = RobotControl.getInstance();
	private TelemetryMessage tm = new TelemetryMessage();
	
	public void setRelays() {
		ArrayList<Relay> temp = rc.getRelay();
		for(int i = 0; i < 4; i++){
			if(temp.get(i).get().equals(0))
				tm.setRelays(i, 0.0);
			else if(temp.get(i).get().equals(1))
				tm.setRelays(i, 1.0);
			else if(temp.get(i).get().equals(2))
				tm.setRelays(i, 2.0);
			else if(temp.get(i).get().equals(3))
				tm.setRelays(i, 3.0);
		}
	}
	
	
}
