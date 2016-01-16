package org.usfirst.frc.team1885.graveyard;

public class AutoCanGrabber extends AutoCommand{
	private boolean grabberState;
	
	public AutoCanGrabber(boolean g) {
		grabberState = g;
	}
	public boolean init() {
		reset();
		CanGrabber.getInstance().update(grabberState);
		return true;
	}

	public boolean execute() {
		CanGrabber.getInstance().update(grabberState);
		return true;
	}

	public boolean updateOutputs() {
		CanGrabber.getInstance().updateOutputs();
		return true;
	}

	
	public void reset() {
	}

}
