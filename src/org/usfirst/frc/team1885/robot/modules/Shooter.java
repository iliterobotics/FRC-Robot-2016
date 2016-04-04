package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.auto.AutoShooterTilt;
import org.usfirst.frc.team1885.robot.auto.AutoShooterTwist;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RelayType;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotMotorType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;
import org.usfirst.frc.team1885.serverdata.ShooterDataClient;

import dataclient.robotdata.vision.HighGoal;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;

//TODO add @depricated tags (or alternate documentation) to all methods no longer being used
public class Shooter implements Module {

    private static final int FLYWHEEL_MAX_SPEED = 775;
    public static final double TILT_MOVEMENT_PROPORTION = 0.6;
    public static final double TWIST_MOVEMENT_PROPORTION = 0.8;
    private static Shooter instance;
    public static final double HIGH_GOAL_ANGLE = 130.0;
    public static final double LOW_GOAL_ANGLE = 12.0;
    public static final double BATTER_SHOT_ANGLE = 120.0;
    public static final double ANGLE_ERROR = 1;
    public static final int TICK_ERROR = 10;
    public double shooterSpeed;
    private static double [] shooterSpeedTable = {0.70};
    private static final double BALL_FACTOR = 1.0;
    private static final boolean OPEN = false;
    private long lastLaunchCheck;
    private static final double FIRE_DELAY = 2000;
    private static final double INTAKE_PROP = 0.7;
    public static final double TWIST_SPEED = .3;
    public static final double TILT_SPEED = .2;
    public static final double STATIC_TILT_LIMIT_UPPER = 135.0;
    private double TILT_LIMIT_UPPER;
    private double TILT_LIMIT_LOWER;
    private final static double LOW_GOAL_TILT_BOUND = 30;
    public final static double LOW_GOAL_TILT = 10;
    public final static double HIGH_GOAL_INTAKE_TILT_BOUND = 80;
    public final static double HIGH_GOAL_INTAKE_TILT = 56;
    public final static double HIGH_GOAL_CAM_TILT = 138;
    private static final double HIGHGOAL_MIDPOINT = 97;
    private final static double TILT_THRESHOLD = 50;
    public final static double LOWER_TILT_COLLISION = 20;
    public final static double UPPER_TILT_COLLISION = 90;
    public static final double GEAR_RATIO_TWIST = 3.0 / 7;
    private static final double TWIST_LEVEL_THRESHOLD = 90; //tilt degree at which the bounds of the twist are changed
    private static final double TWIST_BOUND_HIGH_RIGHT = -37;
    private static final double TWIST_BOUND_HIGH_LEFT = 33;
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
    private boolean containerState;
    private boolean kickerState;
    private SensorInputControlSRX sensorControl;
    private double relativeTiltAngle;

    private double leftFlyWheelSpeedOffset;
    private double rightFlyWheelSpeedOffset;

    private boolean reseting;
    private boolean twistDone;
    
    private boolean isAutoTilt;
    private AutoShooterTilt autoShooterTilt;
    private AutoShooterTwist autoShooterTwist;
    
    private static final double TILT_P = 5;
    private static final double TILT_I = 0;
    private static final double TILT_D = 0;

    private static final double TWIST_P = 0.8;
    private static final double TWIST_I = 0.0002;
    private static final double TWIST_D = 0;

    private static final double SPIN_UP_P = 0.8;
    private static final double SPIN_UP_I = 0.0035;
    private static final double SPIN_UP_D = 0.5;
    
    private static final double SPIN_DOWN_P = 0.5;
    private static final double SPIN_DOWN_I = 0.001;
    private static final double SPIN_DOWN_D = 0.001;

    private boolean previousReset;
    private double tiltPosition;
    private double relativeTwistAngle;
    private double twistPosition;
    private boolean isAiming; //checks if we are using autonomous to aim - using camera vision
    private HighGoal hg;
    private Relay.Value tacticalLightState;
    private int shooterIndex;
    
    private long kickTime;
    private static final long KICK_TIME = 500;
    
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
        containerState = !OPEN;
        kickerState = containerState;
        shooterIndex = 0;
        shooterSpeed = shooterSpeedTable[shooterIndex];

        // Setting up Tilt Talon
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).changeControlMode(TalonControlMode.Position);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).setFeedbackDevice(FeedbackDevice.AnalogPot);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).setPID(TILT_P, TILT_I, TILT_D);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).reverseSensor(true);

        // Setting up Twist Talon
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).changeControlMode(TalonControlMode.Position);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).setFeedbackDevice(FeedbackDevice.QuadEncoder);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).setPID(TWIST_P, TWIST_I, TWIST_D);
        
        // Setting up Flywheel Left Talon
         RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).changeControlMode(TalonControlMode.Speed);
         RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).setFeedbackDevice(FeedbackDevice.QuadEncoder);
         RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).reverseSensor(true);
//        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).changeControlMode(TalonControlMode.PercentVbus);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).setPID(SPIN_UP_P, SPIN_UP_I, SPIN_UP_D);
//                 RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).configEncoderCodesPerRev(1024);

        // Setting up Flywheel Right Talon
         RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).changeControlMode(TalonControlMode.Speed);
         RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).setFeedbackDevice(FeedbackDevice.QuadEncoder);
//        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).changeControlMode(TalonControlMode.PercentVbus);
        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).setPID(SPIN_UP_P, SPIN_UP_I, SPIN_UP_D);
        lastLaunchCheck = System.currentTimeMillis();
        isAutoTilt = false;
        hg = ShooterDataClient.startShooterDataClient().getData();
        tacticalLightState = Relay.Value.kOff;
        kickTime = System.currentTimeMillis();
        // RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).configEncoderCodesPerRev(1024);
    }

    public void init() {
        TILT_LIMIT_LOWER = 0;
        TILT_LIMIT_UPPER = TILT_LIMIT_LOWER + STATIC_TILT_LIMIT_UPPER;
        autoShooterTilt = new AutoShooterTilt(LOW_GOAL_TILT);
        autoShooterTwist = new AutoShooterTwist(0);
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
 
        shooterSpeed = shooterSpeedTable[shooterIndex];
        tacticalLightState = Relay.Value.kOff;
        flywheelSpeedLeft = flywheelSpeedRight = 0;
        containerState = !OPEN;
        kickerState = !OPEN;

        if(driverInputControl.getButton(RobotButtonType.FLYWHEEL_OUT)){
            initiateLaunch();
        }
        if (driverInputControl.getButton(RobotButtonType.SHOOTER_LAUNCH)) {
            launchManualOverride();
        } //else{
//            lastLaunchCheck = System.currentTimeMillis();
//        }
        
        if(containerState != OPEN){
            kickTime = System.currentTimeMillis();
        }
        
        if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_IN) || driverInputControl.getButton(RobotButtonType.FLYWHEEL_INTAKE_IN)) {
            flywheelSpeedLeft = INTAKE_PROP;
            flywheelSpeedRight = INTAKE_PROP;
            containerState = OPEN;
            if(driverInputControl.getButton(RobotButtonType.FLYWHEEL_INTAKE_IN)){
                autoShooterTilt = new AutoShooterTilt(LOW_GOAL_TILT);
                isAutoTilt = true;
                isAiming = false;
            }
        }        
        
        if(driverInputControl.getButton(RobotButtonType.TACTICAL_LIGHT)){
//            DriverStation.reportError("\n TACTICAL LIGHT ON", false);
            tacticalLightState = Relay.Value.kForward;
        }else{
//            DriverStation.reportError("\n TACTICAL LIGHT OFF", false);
        }

//        DriverStation.reportError("\n Left Speed:: " + flywheelSpeedLeft + "Right Speed:: " + flywheelSpeedRight, false);
//        DriverStation.reportError("\n\n Left:: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).get() + " Right:: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).get(), false);
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
        flywheelSpeedLeft = -shooterSpeed;
        flywheelSpeedRight = -shooterSpeed;
    }
    /**
     * Locks the aim to the closes set position: low goal, high goal toward intake, or high goal toward the battery
     * @return the tilt angle being locked on to
     */
    public double lockAim(){
        autoShooterTilt = new AutoShooterTilt(getTiltAimLock());
        autoShooterTwist = new AutoShooterTwist(getTwistAimLock());
        return this.relativeTiltAngle;
    }
    /**
     * Releases the container after a delay to allow the motors to speed up
     * @return true if the container is opened
     */
    public boolean launch(){
//        if(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER) >= (FLYWHEEL_MIN_LAUNCH_SPEED * shooterSpeed)){ //adjusts min launch speed according to what we decide launching speed is, only waits for flywheel to rev up
        if (System.currentTimeMillis() - lastLaunchCheck > FIRE_DELAY) {
            return launchManualOverride();
            // DriverStation.reportError("\nFire", false);
        }
        return false;
    }
    public boolean launchManualOverride(){
//        DriverStation.reportError("\n Necessary Speed:: " + (FLYWHEEL_MAX_SPEED * shooterSpeed), false);
        if(Math.abs(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER)) >= (FLYWHEEL_MAX_SPEED * shooterSpeed)){ //adjusts min launch speed according to what we decide launching speed is, only waits for flywheel to rev up
            containerState = OPEN;
            if(System.currentTimeMillis() - kickTime > KICK_TIME){
                kickerState = OPEN;
                DriverStation.reportError("\nFIRE FIRE FIRE", false);
            }
        }
        return kickerState;
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
//        if(RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).isSensorPresent(FeedbackDevice.AnalogPot) == FeedbackDeviceStatus.FeedbackStatusPresent){
//            if(RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).getControlMode().equals(TalonControlMode.PercentVbus)){
//                RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).changeControlMode(TalonControlMode.Position);
//            }
            int userTiltDirection = driverInputControl.getShooterTilt();
//          DriverStation.reportError("\n" + Math.abs((((RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).get() - sensorControl.getInitialTiltPosition())/ (1024.0/360.0)) - this.relativeTiltAngle)), false);
//          DriverStation.reportError(" " + isAutoTilt, false);
//          double userTiltDirection = driverInputControl.getPressureButton(RobotButtonType.SHOOTER_TILT);
//          DriverStation.reportError("\n Theoretical Tilt: " + this.relativeTiltAngle + "  Actual Tilt: " + (RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).get() /*- sensorControl.getInitialTiltPosition()) * (360.0 / 1024*/), false);
            if(isAutoTilt) {
                isAutoTilt = userTiltDirection == 0;
            }
            if(isAiming) {
                isAiming = userTiltDirection == 0;
            }
            this.relativeTiltAngle += userTiltDirection * TILT_MOVEMENT_PROPORTION;
        
            return updateTiltPosition();
//        } 
//        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).changeControlMode(TalonControlMode.PercentVbus);
//        relativeTiltAngle = 0;
//        DriverStation.reportError("\nShooter Tilt Potentiometer Disconnected", false);
//        return false;
    }
    public boolean updateTiltPosition() {
        boolean isInPosition = true;
        int input = driverInputControl.getPOVButton(RobotButtonType.AIM);
        if(input >= 0){
            isAutoTilt = true;  
            switch(input){
                case 0: autoShooterTilt = new AutoShooterTilt(HIGH_GOAL_CAM_TILT); isAiming = true; 
                DriverStation.reportError("\nAIMING", false);break;
                case 90: autoShooterTilt = new AutoShooterTilt(HIGH_GOAL_CAM_TILT);isAiming = false; break;
                case 180: autoShooterTilt = new AutoShooterTilt(LOW_GOAL_TILT); isAiming = false; break;
                case 270: autoShooterTilt = new AutoShooterTilt(HIGH_GOAL_INTAKE_TILT); isAiming = false; break;
                default:
            }
            autoShooterTilt.init();
        }
        if(isAutoTilt && !isAiming){
            isAutoTilt = !autoShooterTilt.execute();
        }
        if(isAiming){
            autoShooterTilt = new AutoShooterTilt(getTiltAimLock());
            autoShooterTwist = new AutoShooterTwist(getTwistAimLock());
            autoShooterTilt.execute();
            autoShooterTwist.execute();
//            DriverStation.reportError("\nAuto aiming Tilt: " + getTiltAimLock() + " " + hg.isGoalFound(), false);
            DriverStation.reportError("\nGoal Found:" + hg.isGoalFound(), false);
        }
        
        double currentAngle = sensorControl.getAnalogGeneric(SensorType.SHOOTER_TILT_POTENTIOMETER);

        this.relativeTiltAngle = this.boundTilt(this.relativeTiltAngle);

        this.tiltPosition = (this.relativeTiltAngle * (1024 / 360.0))
                + sensorControl.getInitialTiltPosition();
//        DriverStation.reportError("\ntiltPosition: " + tiltPosition + "  relativeTiltAngle: " + relativeTiltAngle + "  Current Tilt:: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).get(), false);
       
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

    public double getRelativeTwist(){
        return relativeTwistAngle;
    }
    public void setToTwistValue(double angle) {
        relativeTwistAngle = angle;
    }
    public boolean updateTwist() {
//        if(RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).isSensorPresent(FeedbackDevice.QuadEncoder) == FeedbackDeviceStatus.FeedbackStatusPresent){
//            if(RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).getControlMode().equals(TalonControlMode.PercentVbus)){
//                RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).changeControlMode(TalonControlMode.Position);
//            }
            double userTwistDirection = driverInputControl.getShooterTwist();
        
            this.relativeTwistAngle += userTwistDirection * TWIST_MOVEMENT_PROPORTION;
//               DriverStation.reportError("\nUpdating Twist:: " + this.relativeTwistAngle, false);
            return updateTwistPosition();
//        }
//        RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).changeControlMode(TalonControlMode.PercentVbus);
//        relativeTwistAngle = 0;
//        DriverStation.reportError("\nShooter Pan Encoder Disconnected", false);
//        return false;
    }
    public boolean updateTwistPosition() {
        boolean isInPosition = true;
        double currentAngle = sensorControl.getZeroedEncoder(SensorType.SHOOTER_TWIST_ENCODER) / (4096 / 360.0);

        this.relativeTwistAngle = this.boundTwist(this.relativeTwistAngle);

        this.twistPosition = ((this.relativeTwistAngle * (4096 / 360.0)) + sensorControl.getInitialTwistPosition()) / GEAR_RATIO_TWIST;

        isInPosition = (currentAngle > relativeTwistAngle - ANGLE_ERROR) && (currentAngle < relativeTwistAngle + ANGLE_ERROR);
        
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
        if(this.relativeTiltAngle > TWIST_LEVEL_THRESHOLD && driverInputControl.getShooterTwist() != 0){
            twistInputAngle = twistInputAngle < TWIST_BOUND_HIGH_RIGHT ? TWIST_BOUND_HIGH_RIGHT : twistInputAngle;
            twistInputAngle = twistInputAngle > TWIST_BOUND_HIGH_LEFT ? TWIST_BOUND_HIGH_LEFT : twistInputAngle;
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
//         DriverStation.reportError("\n Tilt Position: " + tiltPosition, false);
//         DriverStation.reportError(" To Tilt Angle: " + relativeTiltAngle,false);
//         DriverStation.reportError(" Aiming:: " + isAiming, false);
        // DriverStation.reportError("\n Raw Pot: " + sensorControl.getAnalogGeneric(SensorType.SHOOTER_TILT_POTENTIOMETER), false);
        // DriverStation.reportError("\n", false);
//         DriverStation.reportError("\n Left Output:: " + flywheelSpeedLeft * FLYWHEEL_MIN_SPEED + " Right Output:: " + flywheelSpeedRight * FLYWHEEL_MIN_SPEED, false);
//        DriverStation.reportError("\nFlywheel Speed:: Right: " + flywheelSpeedRight * FLYWHEEL_MAX_SPEED + " Left: " + flywheelSpeedLeft * FLYWHEEL_MAX_SPEED, false);
//        DriverStation.reportError("\nSpeed Prop:: " + shooterSpeedTable[shooterIndex] + " Right Encoder:: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).getEncVelocity() + "Left Encoder:: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).getEncVelocity(),false);
        if(flywheelSpeedRight == 0 && flywheelSpeedLeft == 0){
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).changeControlMode(TalonControlMode.PercentVbus);
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).changeControlMode(TalonControlMode.PercentVbus);
        } else{
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_LEFT).changeControlMode(TalonControlMode.Speed);
            RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).changeControlMode(TalonControlMode.Speed);
        }
        
//        DriverStation.reportError("\n P:" + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).getP() + " I: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).getI() + " D: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.FLYWHEEL_RIGHT).getD(), false);
        RobotControlWithSRX.getInstance().updateFlywheelShooter(flywheelSpeedLeft * FLYWHEEL_MAX_SPEED, flywheelSpeedRight * FLYWHEEL_MAX_SPEED);
//        RobotControlWithSRX.getInstance().updateFlywheelShooter(flywheelSpeedLeft, flywheelSpeedRight);
        RobotControlWithSRX.getInstance().updateShooterTilt(tiltPosition);
        RobotControlWithSRX.getInstance().updateShooterTwist(twistPosition);
//        DriverStation.reportError("\nIntended Position:: " + this.relativeTwistAngle + " Actual Position" + (RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).getEncPosition() - sensorControl.getInitialTwistPosition()) * (360.0/4096) * (3.0 / 7) + " Ticks: " + RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).getEncPosition(), false);
        RobotControlWithSRX.getInstance().updateSingleSolenoid(RobotPneumaticType.SHOOTER_CONTAINER, containerState);
        RobotControlWithSRX.getInstance().updateSingleSolenoid(RobotPneumaticType.SHOOTER_KICKER, kickerState);
        RobotControlWithSRX.getInstance().getRelays().get(RelayType.TACTICAL_LIGHT).set(tacticalLightState);
        // DriverStation.reportError("\nContainer State:: " + isHeld, false);
    }
    
    public double getTwistAimLock(){
        double twistAngle = hg.getAzimuthX();
          
          if(twistAngle > 180){
              twistAngle -= 360;
          }
//          DriverStation.reportError("\n Twist:: " + twistAngle, false);
          return hg.isGoalFound() ? (RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TWIST).getEncPosition() - sensorControl.getInitialTwistPosition()) * (360.0/4096) * (3.0 / 7) - twistAngle : 0; 
    }
    public double getTiltAimLock(){
        double tiltAngle = hg.getAzimuthY();
        
        if(tiltAngle > 180){
            tiltAngle -= 360;
        }
        
        tiltAngle -= 10.0;
        
        double currentTilt = ((RobotControlWithSRX.getInstance().getTalons().get(RobotMotorType.SHOOTER_TILT).get() - sensorControl.getInitialTiltPosition()) * (360.0/1024));
        double goalTilt = currentTilt + tiltAngle;
        //        shooterSpeed *= BALL_FACTOR;
//        DriverStation.reportError("\n Camera Tilt Offset:: " + tiltAngle + " Current Tilt:: " + currentTilt + " Goal:: " + goalTilt, false);
        return hg.isGoalFound() ? goalTilt : this.relativeTiltAngle;
    }
    public boolean isGoalFound(){
        return hg.isGoalFound();
    }
    public double getDistanceToGoal(){
        return hg.getDistance();
    }
}