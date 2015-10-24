package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControl;

public class CanGrabber implements Module {

    private static CanGrabber instance;
    private boolean grabberState;
    
    private boolean prevGrabberButtonState = false;

    protected CanGrabber(){
        grabberState = false;
    }
    public static CanGrabber getInstance() {
        if (instance == null) {
            instance = new CanGrabber();
        }
        return instance;
    }

    public void update() {
    	if (DriverInputControl.getInstance().getButton(RobotButtonType.CAN_BURGLAR) && !prevGrabberButtonState) {
    		grabberState = !grabberState;
    	}
    	
    	prevGrabberButtonState = DriverInputControl.getInstance().getButton(RobotButtonType.CAN_BURGLAR);
    }
    
    public void reset() {
    	grabberState = false;
    }
    public void update( boolean state ) {
        grabberState = state;
    }

   public void updateOutputs() {
        RobotControl.getInstance().updateCanBurglarPneumatics( grabberState );
    }

}
