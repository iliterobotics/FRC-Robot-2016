package org.usfirst.frc.team1885.robot.modules;

import java.util.Map;

import org.usfirst.frc.team1885.robot.auto.AutoAimShooter;
import org.usfirst.frc.team1885.robot.auto.AutoShooterTilt;
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

    private static final int FLYWHEEL_MIN_SPEED = 850;
    public static final double TILT_MOVEMENT_PROPORTION = 0.45;
    private static final double TWIST_MOVEMENT_PROPORTION = 4;
    private static Shooter instance;
    public static final double HIGH_GOAL_ANGLE = 130.0;
    public static final double LOW_GOAL_ANGLE = 12.0;
    public static final double ANGLE_ERROR = 1;
    public static final int TICK_ERROR = 10;
    public static final double SHOOTER_SPEED = 1;
    private static final boolean OPEN = false;
    private long lastLaunchCheck;
    private static final double FIRE_DELAY = 2000;
    private static final double INTAKE_PROP = 0.5;
    public static final double TWIST_SPEED = .3;
    public static final double TILT_SPEED = .2;
    private static final double STATIC_TILT_LIMIT_UPPER = 140;
    private double TILT_LIMIT_UPPER;
    private double TILT_LIMIT_LOWER;
    private final static double LOW_GOAL_TILT_BOUND = 30;
    private final static double LOW_GOAL_TILT = 13;
    private final static double HIGH_GOAL_INTAKE_TILT_BOUND = 80;
    private final static double HIGH_GOAL_INTAKE_TILT = 56;
    private final static double HIGH_GOAL_CAM_TILT = 138;
    private final static double TILT_THRESHOLD = 50;
    public final static double LOWER_TILT_COLLISION = 20;
    public final static double UPPER_TILT_COLLISION = 90;
    public static final double GEAR_RATIO_TWIST = 3.0 / 7;
    private static final double TWIST_LEVEL_THRESHOLD = 90; //tilt degree at which the bounds of the twist are changed
    private static final double TWIST_BOUND_HIGH_RIGHT = -40;
    private static final double TWIST_BOUND_HIGH_LEFT = 40;
    private static final double TWIST_BOUND_LOW_RIGHT = -30;
    private static final double TWIST_BOUND_LOW_LEFT = 30;
    // public static final double GEAR_RATIO_TILT = 1.0 / 3;
    private double flywheelSpeedLeft;
    private MotorState leftState;
    private double flywheelSpeedRight;
    private MotorState rightState;
    private MotorState twistState;
    // private double tiltSpeed;
    private MotorState tiltState;
    private DriverInputControlSRX driverInputControl;
    private boolean isHeld;
    private SensorInputControlSRX sensorControl;
    private double relativeTiltAngle;

    private double leftFlyWheelSpeedOffset;
    private double rightFlyWheelSpeedOffset;

    private boolean reseting;
    private boolean twistDone;
    
    private boolean isAutoTilt;
    private AutoShooterTilt autoShooterTilt;
    
    private static final double TILT_P = 5;
    private static final double TILT_I = 0.0001;
    private static final double TILT_D = 0;

    private static final double TWIST_P = 2;
    private static final double TWIST_I = 0;
    private static final double TWIST_D = 0;

    private static final double SPIN_P = 1.0;
    private static final double SPIN_I = 0.0;
    private static final double SPIN_D = 0.0;

    private boolean previousReset;
    private double tiltPosition;
    private double relativeTwistAngle;
    private double twistPosition;
    private boolean isAiming; //checks if we are using autonomous to aim - using camera vision
    private AutoAimShooter autoAimShooter;

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
        isHeld = !OPEN;

        Map<RobotMotorType, CANTalon> talonList = RobotControlWithSRX
                .getInstance().getTalons();

        // Setting up Tilt Talon
        talonList.get(RobotMotorType.SHOOTER_TILT)
                .changeControlMode(TalonControlMode.Position);
        talonList.get(RobotMotorType.SHOOTER_TILT)
                .setFeedbackDevice(FeedbackDevice.AnalogPot);
        talonList.get(RobotMotorType.SHOOTER_TILT).setPID(TILT_P, TILT_I,
                TILT_D);
        talonList.get(RobotMotorType.SHOOTER_TILT).reverseSensor(true);

        // Setting up Twist Talon
        talonList.get(RobotMotorType.SHOOTER_TWIST)
                .changeControlMode(TalonControlMode.Position);
        talonList.get(RobotMotorType.SHOOTER_TWIST)
                .setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talonList.get(RobotMotorType.SHOOTER_TWIST).setPID(TWIST_P, TWIST_I,
                TWIST_D);

        // Setting up Flywheel Left Talon
         talonList.get(RobotMotorType.FLYWHEEL_LEFT).changeControlMode(TalonControlMode.Speed);
         talonList.get(RobotMotorType.FLYWHEEL_LEFT).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talonList.get(RobotMotorType.FLYWHEEL_LEFT)
                .changeControlMode(TalonControlMode.PercentVbus);
        talonList.get(RobotMotorType.FLYWHEEL_LEFT).setPID(SPIN_P, SPIN_I,
                SPIN_D);
//                 RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).configEncoderCodesPerRev(1024);

        // Setting up Flywheel Right Talon
         talonList.get(RobotMotorType.FLYWHEEL_RIGHT).changeControlMode(TalonControlMode.Speed);
         talonList.get(RobotMotorType.FLYWHEEL_RIGHT).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talonList.get(RobotMotorType.FLYWHEEL_RIGHT)
                .changeControlMode(TalonControlMode.PercentVbus);
        talonList.get(RobotMotorType.FLYWHEEL_RIGHT).setPID(SPIN_P, SPIN_I,
                SPIN_D);
        lastLaunchCheck = System.currentTimeMillis();
        isAutoTilt = false;
        // RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).configEncoderCodesPerRev(1024);
    }

    public void init() {
        TILT_LIMIT_LOWER = sensorControl.getInitialTiltPosition() / 1024.0 / 360;
        TILT_LIMIT_UPPER = TILT_LIMIT_LOWER + STATIC_TILT_LIMIT_UPPER;
        autoShooterTilt = new AutoShooterTilt(LOW_GOAL_TILT);
        autoAimShooter = new AutoAimShooter();
        this.twistPosition = sensorControl.getInitialTwistPosition();
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
        // DriverStation.reportError("\nRight Encoder:: " +
        // sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER),false);

        flywheelSpeedLeft = flywheelSpeedRight = 0;
        isHeld = !OPEN;

        if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_OUT)) {
            fire();
        } else{
            lastLaunchCheck = System.currentTimeMillis();
        }
        if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_IN)) {
            flywheelSpeedLeft = SHOOTER_SPEED * INTAKE_PROP;
            flywheelSpeedRight = SHOOTER_SPEED * INTAKE_PROP;
            isHeld = OPEN;
            setToTiltValue(LOW_GOAL_TILT);
            updateTiltPosition();
        }

        // DriverStation.reportError("\n Left Speed:: " + flywheelSpeedLeft + "
        // Right Speed:: " + flywheelSpeedRight, false);
        // DriverStation.reportError("\n\n Left:: " +
        // RobotControlWithSRX.getInstance().getTalons()
        // .get(RobotMotorType.FLYWHEEL_LEFT)
        // .get() +
        // " Right:: " +
        // RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).get(),
        // false);
//        DriverStation.reportError("\nContained:: " + isHeld, false);
        updateShooter(flywheelSpeedLeft, flywheelSpeedRight);
    }
    public boolean fire(){
        initiateLaunch();
        lockAim();
        return launch();
    }
    /**
     * Starts the flywheels, sets both motors to shooting speed
     */
    public void initiateLaunch(){
        flywheelSpeedLeft = -SHOOTER_SPEED;
        flywheelSpeedRight = -SHOOTER_SPEED;
    }
    /**
     * Locks the aim to the closes set position: low goal, high goal toward intake, or high goal toward the battery
     * @return the tilt angle being locked on to
     */
    public double lockAim(){
        if (this.relativeTiltAngle < LOW_GOAL_TILT_BOUND) {
            setToTiltValue(LOW_GOAL_TILT);
            ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeUp);
        } else if (this.relativeTiltAngle < HIGH_GOAL_INTAKE_TILT_BOUND) {
            setToTiltValue(HIGH_GOAL_INTAKE_TILT);
            ActiveIntake.getInstance().setIntakeSolenoid(ActiveIntake.intakeDown);
        } else {
            setToTiltValue(HIGH_GOAL_CAM_TILT);
        }
        return this.relativeTiltAngle;
    }
    /**
     * Releases the container after a delay to allow the motors to speed up
     * @return true if the container is opened
     */
    public boolean launch(){
        if (System.currentTimeMillis() - lastLaunchCheck > FIRE_DELAY) {
            isHeld = OPEN;
            return true;
            // DriverStation.reportError("\nFire", false);
        }
        return false;
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
    public double getRelativeTilt(){
        return relativeTiltAngle;
    }
    public void setToTiltValue(double angle) {
        relativeTiltAngle = angle;
    }
    public boolean updateTilt() {
        int userTiltDirection = driverInputControl.getShooterTilt();
//        DriverStation.reportError("\n" + Math.abs((((RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).get() - sensorControl.getInitialTiltPosition())/ (1024.0/360.0)) - this.relativeTiltAngle)), false);
//        DriverStation.reportError(" " + isAutoTilt, false);
        if(isAutoTilt)
            isAutoTilt = userTiltDirection == 0;
        if(isAiming)
            isAiming = userTiltDirection == 0;
        this.relativeTiltAngle += userTiltDirection * TILT_MOVEMENT_PROPORTION;
        
        return updateTiltPosition();
    }
    public boolean updateTiltPosition() {
        boolean isInPosition = true;
        int input = driverInputControl.getPOVButton(RobotButtonType.AIM);
        if(input > 0){
            isAutoTilt = true;  
            switch(input){
                case 90: autoShooterTilt = new AutoShooterTilt(HIGH_GOAL_CAM_TILT); isAiming = true; break;
                case 180: autoShooterTilt = new AutoShooterTilt(LOW_GOAL_TILT); break;
                case 270: autoShooterTilt = new AutoShooterTilt(HIGH_GOAL_INTAKE_TILT); break;
                default:
            }
            autoShooterTilt.init();
        }
        if(isAutoTilt){
            isAutoTilt = !autoShooterTilt.execute();
        }
        if(isAiming){
//            isAiming = !autoAimShooter.execute();
            setToTwistValue(45);
        }
        double currentAngle = sensorControl
                .getAnalogGeneric(SensorType.SHOOTER_TILT_POTENTIOMETER);

        this.relativeTiltAngle = this.boundTilt(this.relativeTiltAngle);

        this.tiltPosition = (this.relativeTiltAngle * (1024 / 360.0))
                + sensorControl.getInitialTiltPosition();
         DriverStation
         .reportError(
         "\ntiltPosition: " + tiltPosition
         + "\nrelativeTiltAngle: " + relativeTiltAngle
         + "\nCurrent Tilt:: "
         + RobotControlWithSRX.getInstance().getTalons()
         .get(RobotMotorType.SHOOTER_TILT).get(),
         false);
        isInPosition = (currentAngle > relativeTiltAngle - ANGLE_ERROR)
                && (currentAngle < relativeTiltAngle + ANGLE_ERROR);

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
        if (realTiltAngle > TILT_LIMIT_UPPER) {
            tiltInputAngle = TILT_LIMIT_UPPER;
        }
        if (realTiltAngle < TILT_LIMIT_LOWER) {
            tiltInputAngle = TILT_LIMIT_LOWER;
        }
        if(!ActiveIntake.getInstance().isDown()){
            if(realTiltAngle < TILT_THRESHOLD){
                tiltInputAngle = realTiltAngle > LOWER_TILT_COLLISION ? LOWER_TILT_COLLISION : tiltInputAngle;
            }else if(realTiltAngle > TILT_THRESHOLD){
                tiltInputAngle = realTiltAngle < UPPER_TILT_COLLISION ? UPPER_TILT_COLLISION : tiltInputAngle;
            }
        }
        return tiltInputAngle;
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

    public void setToTwistValue(double angle) {
        relativeTwistAngle = angle;
    }
    public boolean updateTwist() {
        double userTwistDirection = 0/*driverInputControl.getShooterTwist()*/;
        
            this.relativeTwistAngle += userTwistDirection
                    * TWIST_MOVEMENT_PROPORTION;
//            DriverStation.reportError("\nUpdating Twist:: " + this.relativeTwistAngle, false);
        return updateTwistPosition();
    }
    public boolean updateTwistPosition() {
        boolean isInPosition = true;
        double currentAngle = sensorControl.getZeroedEncoder(
                SensorType.SHOOTER_TWIST_ENCODER) / (4096 / 360.0);

        this.relativeTwistAngle = this.boundTwist(this.relativeTwistAngle);

        this.twistPosition = ((this.relativeTwistAngle * (4096 / 360.0))
                + sensorControl.getInitialTwistPosition()) / GEAR_RATIO_TWIST;

        isInPosition = (currentAngle > relativeTwistAngle - ANGLE_ERROR)
                && (currentAngle < relativeTwistAngle + ANGLE_ERROR);
        
        

        updateTwist(twistPosition);
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
        if(this.relativeTiltAngle > TWIST_LEVEL_THRESHOLD){
            twistInputAngle = twistInputAngle < TWIST_BOUND_HIGH_RIGHT ? TWIST_BOUND_HIGH_RIGHT
                    : twistInputAngle;

            twistInputAngle = twistInputAngle > TWIST_BOUND_HIGH_LEFT ? TWIST_BOUND_HIGH_LEFT
                    : twistInputAngle;
        } else{
            twistInputAngle = 0;
//            twistInputAngle = twistInputAngle < TWIST_BOUND_LOW_RIGHT ? TWIST_BOUND_LOW_RIGHT
//                    : twistInputAngle;
//
//            twistInputAngle = twistInputAngle > TWIST_BOUND_LOW_LEFT ? TWIST_BOUND_LOW_LEFT
//                    : twistInputAngle;
        }
//        DriverStation.reportError("\nInput:: " + twistInputAngle + "  Real Angle:: " + twistInputAngle, false);
        return twistInputAngle;
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
    
    public void resetPosition(){
        isAutoTilt = true;
        autoShooterTilt = new AutoShooterTilt(0);
        setToTwistValue(sensorControl.getInitialTwistPosition());
    }

    public void reset() {
        rightState = leftState = MotorState.OFF;
        flywheelSpeedRight = flywheelSpeedLeft = 0;
        setToTiltValue(sensorControl.getInitialTiltPosition());
        setToTwistValue(sensorControl.getInitialTwistPosition());
    }

    @Override
    public void update() {
        updateShooter();
        updateTilt();
        updateTwist();
    }

    public void updateOutputs() {
        // DriverStation.reportError("\n Tilt Position: " + tiltPosition,
        // false);
        // DriverStation.reportError("\nTo Tilt Angle: " +
        // relativeTiltAngle,false);
        // DriverStation.reportError("\n Raw Pot: " +
        // sensorControl.getAnalogGeneric(SensorType.SHOOTER_TILT_POTENTIOMETER),
        // false);
        // DriverStation.reportError("\n", false);
        // DriverStation.reportError("\n Left Output:: " + flywheelSpeedLeft *
        // FLYWHEEL_MIN_SPEED + " Right Output:: " + flywheelSpeedRight *
        // FLYWHEEL_MIN_SPEED, false);
        // RobotControlWithSRX.getInstance().updateFlywheelShooter(flywheelSpeedLeft
        // * FLYWHEEL_MIN_SPEED, flywheelSpeedRight * FLYWHEEL_MIN_SPEED);
        RobotControlWithSRX.getInstance()
                .updateFlywheelShooter(flywheelSpeedLeft, flywheelSpeedRight);
        RobotControlWithSRX.getInstance().updateShooterTilt(tiltPosition);
         RobotControlWithSRX.getInstance().updateShooterTwist(twistPosition);
//         DriverStation.reportError("\nIntended Position:: " + twistPosition + " Actual Position" + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).get(), false);
         RobotControlWithSRX.getInstance().updateSingleSolenoid(RobotPneumaticType.SHOOTER_CONTAINER, isHeld);
        // DriverStation.reportError("\nContainer State:: " + isHeld, false);
        // // RobotControlWithSRX.getInstance()
        // // .updateSingleSolenoid(RobotPneumaticType.SHOOTER_KICKER,
        // // kickerState);
    }
}