package org.usfirst.frc.team1885.robot.modules.lift;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.SensorInputControl;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControl;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class RecycleBinLift implements Module{

    public static final double DEFAULT_LIFT_SPEED = .5;
    private static RecycleBinLift instance;
    private double liftSpeed;
    private final double DEAD_ZONE = .1;
    private boolean hasBin;
    private MotorState state;

    protected RecycleBinLift() {
        this.state = MotorState.OFF;
        hasBin = false;
        liftSpeed = 0;
        
        // TODO fix me
//        SensorInputControl.getInstance().getEncoder(SensorType.RECYCLE_BIN_ENCODER).setDistancePerPulse(1/360);
    }
    public static RecycleBinLift getInstance() {
        if (instance == null) {
            instance = new RecycleBinLift();
        }
        return instance;
    }
    public void setMotorState(MotorState state) {
        this.state = state;
    }
    public MotorState getMotorState() {
        return state;
    }
    public boolean hasBin() {
        return hasBin;
    }
    public double getSpeed() {
        return liftSpeed;
    }
    public void updateLift() {
    	
        if (DriverInputControl.getInstance().getPressureButton(
                RobotButtonType.RECYCLE_BIN_LIFT) > DEAD_ZONE) {
            state = MotorState.REVERSE;
            liftSpeed = DriverInputControl.getInstance().getPressureButton(
                    RobotButtonType.RECYCLE_BIN_LIFT);
        } else if (DriverInputControl.getInstance().getPressureButton(
                RobotButtonType.RECYCLE_BIN_LIFT) < -DEAD_ZONE) {
            state = MotorState.FORWARD;
            liftSpeed = DriverInputControl.getInstance().getPressureButton(
                    RobotButtonType.RECYCLE_BIN_LIFT);

        } else {
        	state = MotorState.OFF;
        	liftSpeed = 0;
        }
        updateLift(liftSpeed);
    }
    public void updateLift(double speed) {
        liftSpeed = speed;
        if (speed > 0) {
            state = MotorState.REVERSE;
        } else if (speed < 0) {
            state = MotorState.FORWARD;
        } else {
            state = MotorState.OFF;
        }
        
        if (state == MotorState.FORWARD) {
            if (SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_UPPER_LIMIT).get()) {
                hasBin = true;
                stop();
            }
        }
        
        if (state == MotorState.REVERSE) {
            if (!SensorInputControl.getInstance()
                    .getLimitSwitch(SensorType.RECYCLE_BIN_LOWER_LIMIT).get()) {
                hasBin = false;
                stop();
            }
        }
    }
    
    public Value getBinState() {
    	if(this.state == MotorState.FORWARD) {
    		return Value.kForward;
    	} else if(this.state == MotorState.REVERSE) {
    		return Value.kReverse;
    	} else {
    		return Value.kOff;
    	}
    }

    public void stop() {
        state = MotorState.OFF;
        liftSpeed = 0;
    }
    
    public void updateOutputs() {
    	RobotControl.getInstance().updateRecycleMotor(-liftSpeed);
    }
	@Override
	public void update() {
		updateLift();
	}
}