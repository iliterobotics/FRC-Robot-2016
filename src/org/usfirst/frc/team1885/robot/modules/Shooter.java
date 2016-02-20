package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class Shooter implements Module {

    private static Shooter instance;
    public static final double HIGH_GOAL_ANGLE = 45.0;
    private final double SHOOTER_SPEED = 1;
    private final double TWIST_SPEED = .3;
    private final double TILT_SPEED = .2;
    private final double TILT_BRAKE = .1;
    private final double TILT_LIMIT_UPPER = 45;
    private final double TILT_LIMIT_LOWER = 5;
    private final double TWIST_BOUND_RIGHT = (1024.0 / GEAR_RATIO_TWIST) / (360 / 45);
    private final double TWIST_BOUND_LEFT = (1024.0 / GEAR_RATIO_TWIST) / (360 / 45);
    //public static final double GEAR_RATIO_TILT = 1.0 / 3;
    public static final double GEAR_RATIO_TWIST = 3.0 / 7;
    private double flywheelSpeedLeft;
    private MotorState leftState;
    private double flywheelSpeedRight;
    private MotorState rightState;
    private double twistSpeed;
    private MotorState twistState;
    private double tiltSpeed;
    private final double INITIAL_TWIST;
    private MotorState tiltState;
    private double totalTilt;
    private DriverInputControlSRX driverInputControl;
    private boolean containerState, kickerState;
    private SensorInputControlSRX sensorControl;

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
        totalTilt = 0;
        driverInputControl = DriverInputControlSRX.getInstance();
        containerState = kickerState = false;
        sensorControl = SensorInputControlSRX.getInstance();
        INITIAL_TWIST = sensorControl.getEncoderPos(
                SensorType.SHOOTER_TWIST_ENCODER) * GEAR_RATIO_TWIST;
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
            flywheelSpeedLeft = SHOOTER_SPEED * .4;
            flywheelSpeedRight = -SHOOTER_SPEED * .4;
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
    public void updateTilt() {     
        updateTilt(tiltCheck(driverInputControl.getShooterTilt()));
    }
    public void updateTilt(double speed) {
        if (speed > 0) {
            tiltState = MotorState.REVERSE;
        } else if (speed < 0) {
            tiltState = MotorState.FORWARD;
        } else {
            tiltState = MotorState.OFF;
        }
        DriverStation.reportError("\n Tilt Speed: " + speed, false);
        RobotControlWithSRX.getInstance().updateShooterTilt(speed);
    }
    public void updateTwist() {
        twistSpeed = 0;
        int twistInput = driverInputControl.getShooterTwist();
        double encoder = getTwistEncoder();
        if (sensorControl.getZeroedPotentiometer(
                SensorType.SHOOTER_TILT_POTENTIOMETER) >= 45) {
                if (twistInput > 0) {
                    twistSpeed = ( encoder > -TWIST_BOUND_RIGHT /*&& encoder > -TWIST_BOUND_LEFT*/ ) ? -TWIST_SPEED : twistSpeed;
                } else if (twistInput < 0) {
                    twistSpeed = ( /*encoder < TWIST_BOUND_RIGHT &&*/ encoder < TWIST_BOUND_LEFT ) ? TWIST_SPEED : twistSpeed;
                }
        } /*else {
            DriverStation.reportError("Below Tilt threshold", false);
                if (twistInput > 0) {
                    if(encoderTwist > -TWIST_BOUND_LEFT)
                        twistSpeed = -TWIST_SPEED;
                } else if (twistInput < 0) {
                    if(encoderTwist < TWIST_BOUND_LEFT)
                        twistSpeed = TWIST_SPEED;
                }
        }*/
        updateTwist(twistSpeed);
    }
    public double getTwistEncoder() {
        return sensorControl.getZeroedEncoder(SensorType.SHOOTER_TWIST_ENCODER)
                * GEAR_RATIO_TWIST;
    }
    public void updateTwist(double speed) {
        if (speed > 0) {
            twistState = MotorState.REVERSE;
        } else if (speed < 0) {
            twistState = MotorState.FORWARD;
        } else {
            twistState = MotorState.OFF;
        }
        RobotControlWithSRX.getInstance().updateShooterTwist(speed);
    }
    public void reset() {
        rightState = leftState = MotorState.OFF;
        flywheelSpeedRight = flywheelSpeedLeft = 0;
    }
    public void updateContainer() {
        if (DriverInputControlSRX.getInstance()
                .getButton(RobotButtonType.SHOOTER_CONTAINER_TOGGLE)) {
            containerState = !containerState;
            DriverStation.reportError("Containing: " + containerState + "\n",
                    false);
        }
    }
    public void updateKicker() {
        if (DriverInputControlSRX.getInstance().getButton(
                RobotButtonType.SHOOTER_KICKER_KICK) && !kickerState) {
            kickerState = true;
            DriverStation.reportError("Kicking", false);
        } else if (DriverInputControlSRX.getInstance().getButton(
                RobotButtonType.SHOOTER_KICKER_RETRACT) && kickerState) {
            kickerState = false;
            DriverStation.reportError("Retracting", false);
        }
    }
    public void updateOutputs() {
        RobotControlWithSRX.getInstance().updateFlywheelShooter(getLeftSpeed(),
                getRightSpeed());
        RobotControlWithSRX.getInstance().updateShooterTilt(tiltSpeed);
        RobotControlWithSRX.getInstance().updateShooterTwist(twistSpeed);
        // RobotControlWithSRX.getInstance()
        // .updateSingleSolenoid(RobotPneumaticType.SHOOTER_CONTAINER,
        // containerState);
        // RobotControlWithSRX.getInstance()
        // .updateSingleSolenoid(RobotPneumaticType.SHOOTER_KICKER,
        // kickerState);
    }
    @Override
    public void update() {
        updateShooter();
        updateTilt();
        updateTwist();
        //updateOutputs();
        StringBuilder output = new StringBuilder();
        output.append("\nTilt Raw: " + sensorControl.getAnalogGeneric(SensorType.SHOOTER_TILT_POTENTIOMETER));
        output.append("\nTilt Zero: " + sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER));
        output.append("\nTwist Raw: " + sensorControl.getEncoderPos(SensorType.SHOOTER_TWIST_ENCODER));
        output.append("\nTwist Zero: " + sensorControl.getZeroedEncoder(SensorType.SHOOTER_TWIST_ENCODER));
        DriverStation.reportError(output.toString(), false);
    }
    public boolean position(double angle) {
        boolean isInPosition = true;
        tiltSpeed = TILT_BRAKE;
        double currentAngle = sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER);
        double angleDifference = currentAngle - angle;
        int divisor = 1;
        if (Math.abs(angleDifference) < 5) {
            divisor++;
        }
        if (Math.abs(angleDifference) > 1.0 ) {
            isInPosition = false;
        }
        if (angleDifference < 0) {            
            tiltSpeed = tiltCheck(1);
        } else if (angleDifference > 0) {
            tiltSpeed = tiltCheck(-1);
        }
        tiltSpeed /= divisor;
        if (isInPosition) {
            tiltSpeed = TILT_BRAKE;
        }
        updateTilt(tiltSpeed);
        return isInPosition;
    }
    public double tiltCheck(int tiltInput) {
        double tiltSpeed = TILT_BRAKE;
        totalTilt = sensorControl
                .getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER);
        if (tiltInput > 0) {
            if (totalTilt < TILT_LIMIT_UPPER) {
                tiltSpeed = TILT_SPEED + TILT_BRAKE;
            }
        } else if (tiltInput < 0) {
            if (totalTilt > TILT_LIMIT_LOWER) {
                tiltSpeed = -TILT_SPEED + TILT_BRAKE;
            }
        }
        return tiltSpeed;
    }
}
