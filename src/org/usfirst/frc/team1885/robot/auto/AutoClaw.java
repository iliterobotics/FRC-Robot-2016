package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.manipulator.ClawControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

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
		rotationState = false;
		extensionState = false;
		pinchState = false;
		
		RobotControl.getInstance().updateWristRotationPneumatics(Value.kReverse);
		RobotControl.getInstance().updateWristExtensionPneumatics(Value.kReverse);
		RobotControl.getInstance().updateGrabberPneumatics(pinchState);
	}

	public boolean init() {
		reset();
		return true;
	}

	public boolean updateOutputs() {
		if (ClawControl.getInstance().clawWristRotationState()) {
			RobotControl.getInstance().updateWristRotationPneumatics(Value.kForward);
    	} else{
    		RobotControl.getInstance().updateWristRotationPneumatics(Value.kReverse);
        }
        
        if (ClawControl.getInstance().clawWristExtentionState()) {
        	RobotControl.getInstance().updateWristExtensionPneumatics(Value.kForward);
    	} else{
    		RobotControl.getInstance().updateWristExtensionPneumatics(Value.kReverse);
        }
        
        RobotControl.getInstance().updateGrabberPneumatics(ClawControl.getInstance().clawPinchState());
        return true;
	}

	

}
