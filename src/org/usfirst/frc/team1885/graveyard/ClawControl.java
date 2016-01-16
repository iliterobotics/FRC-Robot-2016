package org.usfirst.frc.team1885.graveyard;

import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.modules.Module;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ClawControl implements Module
{
	private static ClawControl instance;
	private boolean rotationState;
	private boolean extensionState;
	private boolean pinchState;
	
	private boolean prevWristToggleState = false;
	private boolean prevElbowToggleState = false;
	private boolean prevClawState = false;
	
	protected ClawControl(){
		rotationState = false;
		extensionState = false;
		pinchState = false;
	}
	public static ClawControl getInstance(){
	    if(instance == null){
	        instance = new ClawControl();
	    }
	    return instance;
	}
	public void updateClaw(){
		
	        if (DriverInputControl.getInstance().getButton(RobotButtonType.WRIST_ROTATION) && !prevWristToggleState) {
	            rotationState = !rotationState;
	    	}
	        
	        if (DriverInputControl.getInstance().getButton(RobotButtonType.WRIST_EXTENSION) && !prevElbowToggleState) {
	            extensionState = !extensionState;
	    	}
	        
	        if (DriverInputControl.getInstance().getButton(RobotButtonType.CLAW) && !prevClawState) {
	            pinchState = !pinchState;
	    	}
	        
	        prevWristToggleState = DriverInputControl.getInstance().getButton(RobotButtonType.WRIST_ROTATION);
	        prevElbowToggleState = DriverInputControl.getInstance().getButton(RobotButtonType.WRIST_EXTENSION);
	        prevClawState = DriverInputControl.getInstance().getButton(RobotButtonType.CLAW);
	}
	public void updateClaw(boolean rotationState, boolean extensionState, boolean pState){
		this.rotationState = rotationState;
		this.extensionState = extensionState;
		this.pinchState = pState;
	}
	public boolean clawWristRotationState(){
	    return rotationState;
	}
	public boolean clawWristExtentionState(){
	    return extensionState;
	}
	public boolean clawPinchState(){
	    return pinchState;
	}
	public void updateOutputs() {
		if(extensionState) {
			RobotControl.getInstance().updateWristExtensionPneumatics(Value.kForward);
		} else {
			RobotControl.getInstance().updateWristExtensionPneumatics(Value.kReverse);
		}
		
		if(rotationState) {
			RobotControl.getInstance().updateWristRotationPneumatics(Value.kForward);
		} else {
			RobotControl.getInstance().updateWristRotationPneumatics(Value.kReverse);
		}
		
		RobotControl.getInstance().updateGrabberPneumatics(pinchState);

    }
	@Override
	public void update() {
		updateClaw();
		
	}
}
