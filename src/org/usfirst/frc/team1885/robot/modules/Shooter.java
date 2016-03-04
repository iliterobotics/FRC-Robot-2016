package org.usfirst.frc.team1885.robot.modules;

import java.util.Map;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DriverStation;
//TODO add @depricated tags (or alternate documentation) to all methods no longer being used
public class Shooter implements Module {

    private static final int FLYWHEEL_MIN_SPEED = 29000;
    private static Shooter instance;
    public static final double HIGH_GOAL_ANGLE = 130.0;
    public static final double LOW_GOAL_ANGLE = 12.0;
    public static final double ANGLE_ERROR = 1;
    public static final int TICK_ERROR = 10;
    public static final double SHOOTER_SPEED = 1;
    public static final double TWIST_SPEED = .3;
    public static final double TILT_SPEED = .2;
    private static final double TILT_BRAKE = .05;
    private static final double TILT_LIMIT_UPPER = 130;
    private static final double TILT_LIMIT_LOWER = 0;
    public static final double GEAR_RATIO_TWIST = 3.0 / 7;
    private static final double TWIST_BOUND_RIGHT = (1024.0 / GEAR_RATIO_TWIST) / (360 / 45);
    private static final double TWIST_BOUND_LEFT = (1024.0 / GEAR_RATIO_TWIST) / (360 / 45);
    // public static final double GEAR_RATIO_TILT = 1.0 / 3;
    private double flywheelSpeedLeft;
    private MotorState leftState;
    private double flywheelSpeedRight;
    private MotorState rightState;
    private double twistSpeed;
    private MotorState twistState;
    // private double tiltSpeed;
    private final double INITIAL_TWIST;
    private MotorState tiltState;
    private DriverInputControlSRX driverInputControl;
    private boolean isHeld, kickerState;
    private SensorInputControlSRX sensorControl;
    private double relativeTiltAngle;
    private double toTwistTicks;

    private double leftFlyWheelSpeedOffset;
    private double rightFlyWheelSpeedOffset;

    private boolean reseting;
    private boolean twistDone;

    private static final double TILT_P = 5.0;
    private static final double TILT_I = 0.0;
    private static final double TILT_D = 0.0;

    private static final double TWIST_P = 0.7;
    private static final double TWIST_I = 0;
    private static final double TWIST_D = 0;

    private static final double SPIN_P = 3;
    private static final double SPIN_I = 0.0001;
    private static final double SPIN_D = 0.0;

    private boolean previousReset;
    private boolean previousLow;
    private boolean previousHigh;
    private boolean previousFlywheel;
    private double tiltPosition;
    private double trueTiltAngle;
    private double relativeTwistAngle;
    private double twistPosition;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
    private Shooter() {
        sensorControl = SensorInputControlSRX.getInstance();

        leftState = rightState = twistState = tiltState = MotorState.OFF;

        driverInputControl = DriverInputControlSRX.getInstance();
        isHeld = kickerState = false;

        INITIAL_TWIST = sensorControl.getEncoderPos(SensorType.SHOOTER_TWIST_ENCODER);
        
        Map<RobotMotorType, CANTalon> talonList = RobotControlWithSRX.getInstance().getTalons();

        //Setting up Tilt Talon
        talonList.get(RobotMotorType.SHOOTER_TILT).changeControlMode(TalonControlMode.Position);
        talonList.get(RobotMotorType.SHOOTER_TILT).setFeedbackDevice(FeedbackDevice.AnalogPot);
        talonList.get(RobotMotorType.SHOOTER_TILT).setPID(TILT_P, TILT_I, TILT_D);
        
        //Setting up Twist Talon
        talonList.get(RobotMotorType.SHOOTER_TWIST).changeControlMode(TalonControlMode.Position);
        talonList.get(RobotMotorType.SHOOTER_TWIST).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talonList.get(RobotMotorType.SHOOTER_TWIST).setPID(TWIST_P, TWIST_I, TWIST_D);
        
        //Setting up Flywheel Left Talon
        talonList.get(RobotMotorType.FLYWHEEL_LEFT).changeControlMode(TalonControlMode.Speed);
        talonList.get(RobotMotorType.FLYWHEEL_LEFT).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talonList.get(RobotMotorType.FLYWHEEL_LEFT).setPID(SPIN_P, SPIN_I, SPIN_D);
        
        //Setting up Flywheel Right Talon
        talonList.get(RobotMotorType.FLYWHEEL_RIGHT).changeControlMode(TalonControlMode.Speed);
        talonList.get(RobotMotorType.FLYWHEEL_RIGHT).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talonList.get(RobotMotorType.FLYWHEEL_RIGHT).setPID(SPIN_P, SPIN_I, SPIN_D);
    }
    
    public void init(){
        relativeTiltAngle = sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER);
    }
    // Get telemetry data
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
    
    // updates shooter fly wheels
    public void updateShooter() {
        // TODO modify values after testing for direction
//        DriverStation.reportError("\n\nLeft Encoder:: " + sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_LEFT_ENCODER) + "\nRight Encoder:: " + sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER), false);

        if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_OUT)) {
            flywheelSpeedLeft = -SHOOTER_SPEED;
            flywheelSpeedRight = SHOOTER_SPEED;
            if (sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER) < 10) {
                setToTiltValue(13);
                updateTiltPosition();
                ActiveIntake.getInstance().intakeUp();
            }
            if (Math.abs(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_LEFT_ENCODER)) >= FLYWHEEL_MIN_SPEED && Math.abs(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER)) >= FLYWHEEL_MIN_SPEED) {
                isHeld = false;
            }

            leftFlyWheelSpeedOffset = -1;
            rightFlyWheelSpeedOffset = 1;

            // int vleft =
            // Math.abs(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_LEFT_ENCODER));
            // int vright =
            // Math.abs(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER));
            //
            // if(vleft > vright){
            // leftSpinnerPID.setScalingValue(vright);
            // double difference = leftSpinnerPID.getPID(vright, vleft)/2;
            // leftFlyWheelSpeedOffset -= difference;
            // rightFlyWheelSpeedOffset-= difference;
            // }else if(vright > vleft){
            // rightSpinnerPID.setScalingValue(vleft);
            // double difference = leftSpinnerPID.getPID(vleft, vright)/2;
            // leftFlyWheelSpeedOffset += difference;
            // rightFlyWheelSpeedOffset+= difference;
            // }

            // DriverStation.reportError("\nR flywheel velocity:" + vleft,
            // false);
            // DriverStation.reportError("\nL flywheel velocity:" + vright,
            // false);
            //
            // DriverStation.reportError("\nR flywheel power:" +
            // (flywheelSpeedLeft + leftFlyWheelSpeedOffset), false);
            // DriverStation.reportError("\nL flywheel power:" +
            // (flywheelSpeedRight + rightFlyWheelSpeedOffset), false);
        } else if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_IN)) {
            flywheelSpeedLeft = SHOOTER_SPEED * .6;
            flywheelSpeedRight = -SHOOTER_SPEED * .6;
            leftFlyWheelSpeedOffset = 0;
            rightFlyWheelSpeedOffset = 0;
            isHeld = false;
            setToTiltValue(13);
            updateTiltPosition();
        } else {
            flywheelSpeedLeft = 0;
            flywheelSpeedRight = 0;
            leftFlyWheelSpeedOffset = 0;
            rightFlyWheelSpeedOffset = 0;
            isHeld = true;
        }

        updateShooter(flywheelSpeedLeft + leftFlyWheelSpeedOffset, flywheelSpeedRight + rightFlyWheelSpeedOffset);
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
    public void setToTiltValue(double angle) {
        relativeTiltAngle = angle;
    }
    // updates tilt telemetry
    public void updateTilt() {
        int userTiltDirection = driverInputControl.getShooterTilt();
        // DriverStation.reportError("\nDirection:: " + direction, false);

        this.relativeTiltAngle += userTiltDirection * .25;

        updateTiltPosition();
    }
    public boolean updateTiltPosition() {
        boolean isInPosition = true;
        // tiltSpeed = TILT_BRAKE;
        double currentAngle = sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER);

        this.relativeTiltAngle = this.boundTilt(this.relativeTiltAngle);

        this.tiltPosition = (this.relativeTiltAngle + sensorControl.getInitialTiltPosition()) * (1024 / 360.0);
        // DriverStation.reportError("\ntiltPosition: " + tiltPosition +
        // "\nrelativeTiltAngle: " + relativeTiltAngle
        // +"\nInitial Tilt:: " + SensorInputControlSRX.INITIAL_TILT_POSITION,
        // false);
        isInPosition = (currentAngle > relativeTiltAngle - ANGLE_ERROR) && (currentAngle < relativeTiltAngle + ANGLE_ERROR);

        updateTilt(tiltPosition);

        return isInPosition;
    }
    /**
     * Ensures the tilt does not go beyond the set bounds
     * 
     * @param tiltInputAngle
     *            angle to possibly go to
     * @return the corrected angle to go to
     */
    public double boundTilt(double tiltInputAngle) {
        double realTiltAngle = tiltInputAngle;

        double totalTwist = sensorControl.getZeroedEncoder(SensorType.SHOOTER_TWIST_ENCODER);

        if (realTiltAngle > TILT_LIMIT_UPPER) {
            realTiltAngle = TILT_LIMIT_UPPER;
        }
        if (realTiltAngle < TILT_LIMIT_LOWER) {
            realTiltAngle = TILT_LIMIT_LOWER;
        }
        return realTiltAngle;
    }
    public void updateTilt(double position) {
        if (position > 0) {
            tiltState = MotorState.REVERSE;
        } else if (position < 0) {
            tiltState = MotorState.FORWARD;
        } else {
            tiltState = MotorState.OFF;
        }
    }
    
    // updates twist telemetry
    public void updateTwist() {
        double userTwistDirection = driverInputControl.getShooterTwist();
        // DriverStation.reportError("\nDirection:: " + direction, false);

        this.relativeTwistAngle += userTwistDirection * .25;

        updateTwistPosition();
    }
    private boolean updateTwistPosition() {
        boolean isInPosition = true;
        // tiltSpeed = TILT_BRAKE;
        double currentAngle = sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER);

        this.relativeTwistAngle = this.boundTwist(this.relativeTwistAngle);

        this.twistPosition = (this.relativeTwistAngle + sensorControl.getInitialTwistPosition()) * (1024 / 360.0);

        isInPosition = (currentAngle > relativeTwistAngle - ANGLE_ERROR) && (currentAngle < relativeTwistAngle + ANGLE_ERROR);

        updateTilt(tiltPosition);

        return isInPosition; 
    }
    /**
     * Ensures the twist does not go beyond the set bounds
     * 
     * @param twistInputAngle
     *            angle to possibly go to
     * @return the corrected angle to go to
     */
    public double boundTwist(double twistInputAngle) {
        double realTwistAngle = twistInputAngle;
        return realTwistAngle;
    }
    public void updateTwist(double position) {
        if (position > 0) {
            twistState = MotorState.REVERSE;
        } else if (position < 0) {
            twistState = MotorState.FORWARD;
        } else {
            twistState = MotorState.OFF;
        }
    }
    
    public void reset() {
        rightState = leftState = MotorState.OFF;
        flywheelSpeedRight = flywheelSpeedLeft = 0;
    }
    
    public void updateKicker() {
        if (DriverInputControlSRX.getInstance().getButton(RobotButtonType.SHOOTER_KICKER_KICK) && !kickerState) {
            kickerState = true;
//            DriverStation.reportError("Kicking", false);
        } else if (DriverInputControlSRX.getInstance().getButton(RobotButtonType.SHOOTER_KICKER_RETRACT) && kickerState) {
            kickerState = false;
//            DriverStation.reportError("Retracting", false);
        }
    }
    
    @Override
    public void update() {
        updateShooter();
        updateKicker();
        updateTilt();
        updateTwist();
    }
    
    public void updateOutputs() {
//         DriverStation.reportError("\n Tilt Position: " + tiltPosition, false);
//         DriverStation.reportError("\nTo Tilt Angle: " + relativeTiltAngle,false);
//         DriverStation.reportError("\n Raw Pot: " + sensorControl.getAnalogGeneric(SensorType.SHOOTER_TILT_POTENTIOMETER) *(1024.0/360), false);
//         DriverStation.reportError("\n", false);
         //        RobotControlWithSRX.getInstance().updateFlywheelShooter(flywheelSpeedLeft, flywheelSpeedRight);
//        RobotControlWithSRX.getInstance().updateShooterTilt(tiltPosition);
//        RobotControlWithSRX.getInstance().updateShooterTwist(twistSpeed);
//        RobotControlWithSRX.getInstance().updateSingleSolenoid(RobotPneumaticType.SHOOTER_CONTAINER, isHeld);
        // DriverStation.reportError("\nContainer State:: " + isHeld, false);
        // // RobotControlWithSRX.getInstance()
        // // .updateSingleSolenoid(RobotPneumaticType.SHOOTER_KICKER,
        // // kickerState);
    }
}