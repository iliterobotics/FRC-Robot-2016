package org.usfirst.frc.team1885.graveyard;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class AutoClaw extends AutoCommand{
	private boolean rotationState;
	private boolean extensionState;
	private boolean pinchState;
	
	public AutoClaw(boolean rotationState, boolean extensionState, boolean pinchState ) {
		this.rotationState = rotationState;
		this.extensionState = extensionState;
		this.pinchState = pinchState;
	}
	
	public boolean execute() {
		ClawControl.getInstance().updateClaw(rotationState, extensionState, pinchState);
		return true;
	}
	public void reset() {
//		rotationState = false;
//		extensionState = false;
//		pinchState = false;
		
	}

	public boolean init() {
		reset();
		return true;
	}

	public boolean updateOutputs() {
		ClawControl.getInstance().updateOutputs();
        return true;
	}

	

}
