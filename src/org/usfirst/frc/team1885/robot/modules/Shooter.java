package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class Shooter implements Module {

    private static Shooter instance;

    private final double SHOOTER_SPEED = 1;
    private final double TWIST_SPEED = .6;
    private final double TILT_SPEED = .3;
    private final double TILT_BRAKE = .1;
    private double flywheelSpeedLeft;
    private MotorState leftState;
    private double flywheelSpeedRight;
    private MotorState rightState;
    private double twistSpeed;
    private MotorState twistState;
    private double tiltSpeed;
    private MotorState tiltState;
    private DriverInputControlSRX driverInputControl;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
    protected Shooter() {
        this.leftState = MotorState.OFF;
        this.rightState = MotorState.OFF;
        this.twistState = this.tiltState = MotorState.OFF;
        flywheelSpeedLeft = flywheelSpeedRight = twistSpeed = tiltSpeed = 0;
        driverInputControl = DriverInputControlSRX.getInstance();
    }
    public MotorState getLeftMotorState() {
        return leftState;
    }
    public MotorState getRightMotorState() {
        return rightState;
    }
    public double getLeftSpeed() {
        return flywheelSpeedLeft;
    }
    public double getRightSpeed() {
        return flywheelSpeedRight;
    }
    public MotorState getTwistState() {
        return twistState;
    }
    public MotorState getTiltState() {
        return tiltState;
    }
    public void updateShooter() {
        // TODO modify values after testing for direction
        if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_OUT)) {
            flywheelSpeedLeft = -SHOOTER_SPEED;
            flywheelSpeedRight = SHOOTER_SPEED;
        } else if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_IN)) {
            flywheelSpeedLeft = SHOOTER_SPEED * .5;
            flywheelSpeedRight = -SHOOTER_SPEED * .5;
        } else {
            flywheelSpeedLeft = 0;
            flywheelSpeedRight = 0;
        }
        updateShooter(flywheelSpeedLeft, flywheelSpeedRight);
    }
    public void updateShooter(double speedLeft, double speedRight) {
        if (speedLeft > 0) {
            leftState = MotorState.REVERSE;
        } else if (speedLeft < 0) {
            leftState = MotorState.FORWARD;
        } else {
            leftState = MotorState.OFF;
        }
        if (speedRight > 0) {
            rightState = MotorState.REVERSE;
        } else if (speedRight < 0) {
            rightState = MotorState.FORWARD;
        } else {
            rightState = MotorState.OFF;
        }
    }
    public void updateTilt(){
        tiltSpeed = TILT_BRAKE;
        double tilt = driverInputControl.getShooterTilt();
        DriverStation.reportError("\nTilt::" + tilt, false);
        if(tilt > 0){
            tiltSpeed = TILT_SPEED + TILT_BRAKE;
        } else if( tilt < 0 ){
            tiltSpeed = -TILT_SPEED + TILT_BRAKE;
        }
        updateTilt(tiltSpeed);
    }
    public void updateTilt(double speed){
        if (speed > 0) {
            tiltState = MotorState.REVERSE;
        } else if (speed < 0) {
            tiltState = MotorState.FORWARD;
        } else {
            tiltState = MotorState.OFF;
        }
    }
    public void updateTwist(){
        twistSpeed = 0;
        double twist = driverInputControl.getShooterTwist();
        DriverStation.reportError("\nTwist::" + twist, false);
        if(twist > 0){
            twistSpeed = TWIST_SPEED;
        } else if( twist < 0 ){
            twistSpeed = -TWIST_SPEED;
        }
        updateTwist(twistSpeed);
    }
    public void updateTwist(double speed){
        if (speed > 0) {
            twistState = MotorState.REVERSE;
        } else if (speed < 0) {
            twistState = MotorState.FORWARD;
        } else {
            twistState = MotorState.OFF;
        }
    }
    public void reset() {
        rightState = leftState = MotorState.OFF;
        flywheelSpeedRight = flywheelSpeedLeft = 0;
    }

    public void updateOutputs() {
        RobotControlWithSRX.getInstance()
                .updateFlywheelShooter(getLeftSpeed(), getRightSpeed());
        RobotControlWithSRX.getInstance()
                .updateShooterTilt(tiltSpeed);
        RobotControlWithSRX.getInstance()
                .updateShooterTwist(twistSpeed);
    }
    @Override
    public void update() {
        DriverStation.reportError("\nUpdating shooter", false);
        updateShooter();
        updateTilt();
        updateTwist();
        updateOutputs();
    }

}
