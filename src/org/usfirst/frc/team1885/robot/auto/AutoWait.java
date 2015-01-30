package org.usfirst.frc.team1885.robot.auto;

public class AutoWait implements AutoCommand{
	private double miliseconds;
	private double startTime;
	public AutoWait(double inputMilliseconds) {
		miliseconds = inputMilliseconds;
		startTime = System.currentTimeMillis();
	}
	public boolean execute() {
		if(System.currentTimeMillis() >= startTime + miliseconds) {
			return true;
		}
		else {
			return false;
		}
	}
	public void reset() {
	}

}
