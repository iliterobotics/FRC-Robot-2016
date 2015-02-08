package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick;

public class ToteLift {

    private final int toteHeight = 42; /* psuedoheight */
    public static final double DEFAULT_LIFT_SPEED = .5;
    private static ToteLift instance;
    private double liftSpeed;
    private MotorState state;
    private PID distanceControlLoop;
    private double distanceTraveled;
    private final double DEAD_ZONE = .1;
    private boolean isIncrementing;
    private boolean isBraked;

    protected ToteLift() {
        this.state = MotorState.STOP;
        distanceControlLoop = new PID(0.01, 0.00001, 0);
        isBraked = false;
        liftSpeed = 0;
        isIncrementing = false;
    }
    public static ToteLift getInstance() {
        if (instance == null) {
            instance = new ToteLift();
        }
        return instance;
    }
    public void setMotorState(MotorState state) {
        this.state = state;
    }
    public MotorState getMotorState() {
        return state;
    }
    public double getSpeed() {
        return liftSpeed;
    }
    public void updateLift() {
    	    	
        if (DriverInputControl.getInstance().getButton(
                RobotButtonType.TOTE_LIFT_INCREMENT)
                && !isIncrementing || isIncrementing) {
            state = MotorState.UP;
            this.isIncrementing = !incrementLift(42, 4.20);

        } else if (DriverInputControl.getInstance().getPressureButton(
                        RobotButtonType.TOTE_LIFT) > DEAD_ZONE) {

            state = MotorState.UP;
        } else if (DriverInputControl.getInstance().getPressureButton(
        		RobotButtonType.TOTE_LIFT) < -DEAD_ZONE) {

            state = MotorState.DOWN;
        } else if(DriverInputControl.getInstance().getButton(
        		RobotButtonType.HARD_STOP)) {
        	state = MotorState.STOP;
    	} else{
            state = MotorState.STOP;
        }
        
        if(state == MotorState.STOP) {
        	isBraked = true;
        } else {
        	isBraked = false;
        }
        
        if (state == MotorState.UP) {
            liftSpeed = DEFAULT_LIFT_SPEED;
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
                stop();
            }
        } else if (state == MotorState.DOWN) {
            liftSpeed = -DEFAULT_LIFT_SPEED;
        } else {
        	stop();
        }
    }
    public void updateLift(double speed) {
        liftSpeed = speed;
        if (speed > 0) {
            state = MotorState.UP;
        } else if (speed < 0) {
            state = MotorState.DOWN;
        } else {
            state = MotorState.STOP;
        }
        
        if(state == MotorState.STOP) {
        	isBraked = true;
        } else {
        	isBraked = false;
        }
        
        if (state == MotorState.UP) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.TOTE_UPPER_LIMIT_SWITCH).get()) {
                stop();
            }
        }
    }
    public boolean incrementLift(double distance, double error) {

        distanceTraveled = SensorInputControl.getInstance()
                .getEncoder(SensorType.TOTE_ENCODER).getDistance();
        if (Math.abs(distanceTraveled - distance) <= error) {
            this.reset();
            this.stop();
            return true;
        } else {
            liftSpeed = distanceControlLoop.getPID(
                    distance,
                    SensorInputControl.getInstance()
                            .getEncoder(SensorType.TOTE_ENCODER).getDistance());
            updateLift(liftSpeed);

            return false;
        }
    }
    public void reset() {
        distanceControlLoop.reset();
        SensorInputControl.getInstance().getEncoder(SensorType.TOTE_ENCODER)
                .reset();
    }
    public void cycle() {
        if (state == MotorState.UP) {
            liftSpeed = DEFAULT_LIFT_SPEED;
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.TOTE_LOWER_LIMIT_SWITCH).get()) {
                stop();
            }
        }
    }
    public void stop() {
        state = MotorState.STOP;
        liftSpeed = 0;
    }
    
    public void updateOutputs() {
    	RobotControl.getInstance().updateToteMotor(liftSpeed);
    	RobotControl.getInstance().updateToteStop(!isBraked);
    }
	@Override
	public String toString() {
		return "ToteLift [toteHeight=" + toteHeight + ", liftSpeed="
				+ liftSpeed + ", state=" + state + ", distanceControlLoop="
				+ distanceControlLoop + ", distanceTraveled="
				+ distanceTraveled 
				+ ", DEAD_ZONE=" + DEAD_ZONE + ", isIncrementing="
				+ isIncrementing + ", isBraked=" + isBraked + "]";
	}
    
    
}