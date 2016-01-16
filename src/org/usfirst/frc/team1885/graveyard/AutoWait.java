package org.usfirst.frc.team1885.graveyard;

public class AutoWait extends AutoCommand{
	private double miliseconds;
	private double startTime;
	
	public AutoWait(double inputMilliseconds) {
		miliseconds = inputMilliseconds;
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
	public boolean updateOutputs() {
		return true;
	}
	

	public boolean init() {
		startTime = System.currentTimeMillis();
		return true;
	}

}
