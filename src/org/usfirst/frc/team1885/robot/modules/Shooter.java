package org.usfirst.frc.team1885.robot.modules;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.RobotPneumaticType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class Shooter implements Module {

    private static final int FLYWHEEL_MIN_SPEED = 25000;
    private static Shooter instance;
    public static final double HIGH_GOAL_ANGLE = 130.0;
    public static final double ANGLE_ERROR = 1;
    public static final int TICK_ERROR = 10;
    public static final double SHOOTER_SPEED = 1;
    public static final double TWIST_SPEED = .3;
    public static final double TILT_SPEED = .2;
    private static final double TILT_BRAKE = .05;
    private static final double TILT_LIMIT_UPPER = 90;
    private static  final double TILT_LIMIT_LOWER = 5;
    public static final double GEAR_RATIO_TWIST = 3.0 / 7;
    private static final double TWIST_BOUND_RIGHT = (1024.0 / GEAR_RATIO_TWIST) / (360 / 45);
    private static final double TWIST_BOUND_LEFT = (1024.0 / GEAR_RATIO_TWIST) / (360 / 45);
    //public static final double GEAR_RATIO_TILT = 1.0 / 3;
    private double flywheelSpeedLeft;
    private MotorState leftState;
    private double flywheelSpeedRight;
    private MotorState rightState;
    private double twistSpeed;
    private MotorState twistState;
    private double tiltSpeed;
    private final double INITIAL_TWIST;
    private MotorState tiltState;
    private DriverInputControlSRX driverInputControl;
    private boolean isHeld, kickerState;
    private SensorInputControlSRX sensorControl;
    private double toTiltAngle;
    private double toTwistTicks;
    
    private double leftFlyWheelSpeedOffset;
    private double rightFlyWheelSpeedOffset;
    
    private boolean reseting;
    private boolean twistDone;
    
    private static final double TILT_P = 0.7;   
    private static final double TILT_I = 0.03;  
    private static final double TILT_D = 0.0;    
                                                 
    private static final double TWIST_P =0.7;   
    private static final double TWIST_I =0.005;
    private static final double TWIST_D =0.1;     
    
    /*We had to add secondary PID's because the rate at which the PID is 
    referenced in driver control is different than
    the rate in autonomous*/
    private static final double D_TILT_P  = 0.05;
    private static final double D_TILT_I  = 0.002;
    private static final double D_TILT_D  = 0.0;
    
    private static final double D_TWIST_P = 0.05;
    private static final double D_TWIST_I = 0.00015;
    private static final double D_TWIST_D = 0.01;   
    
    private static final double SPIN_P = 0.005;
    private static final double SPIN_I = 0.0;
    private static final double SPIN_D = 0.0;
    
    private PID tiltPID;
    private PID twistPID;
    private PID leftSpinnerPID;
    private PID rightSpinnerPID;
    private boolean previousReset;
    private boolean previousLow;
    private boolean previousHigh;
    private boolean previousFlywheel;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
    protected Shooter() {
        previousFlywheel = false;
        this.leftState = MotorState.OFF;
        this.rightState = MotorState.OFF;
        this.twistState = this.tiltState = MotorState.OFF;
        flywheelSpeedLeft = flywheelSpeedRight = twistSpeed = tiltSpeed = 0;
        driverInputControl = DriverInputControlSRX.getInstance();
        isHeld = kickerState = false;
        sensorControl = SensorInputControlSRX.getInstance();
        INITIAL_TWIST = sensorControl.getEncoderPos(
                SensorType.SHOOTER_TWIST_ENCODER) * GEAR_RATIO_TWIST;
        
        tiltPID = new PID(TILT_P, TILT_I, TILT_D);
        twistPID = new PID(TWIST_P, TWIST_I, TWIST_D);
        leftSpinnerPID = new PID(SPIN_P, SPIN_I, SPIN_D);
        rightSpinnerPID = new PID(SPIN_P, SPIN_I, SPIN_D);
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
        DriverStation.reportError("\n\nLeft Encoder:: " + sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_LEFT_ENCODER) + "\nRight Encoder:: " + sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER), false);

        if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_OUT)) {
            flywheelSpeedLeft = -SHOOTER_SPEED;
            flywheelSpeedRight = SHOOTER_SPEED;
            if(sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER) < 10){
                setToTiltValue(5);
                positionTilt();
            }
            if(Math.abs(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_LEFT_ENCODER)) >= FLYWHEEL_MIN_SPEED && Math.abs(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER)) >= FLYWHEEL_MIN_SPEED){
                isHeld = false;
            }
            
            leftFlyWheelSpeedOffset = -1;
            rightFlyWheelSpeedOffset = 1;
            
//            int vleft = Math.abs(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_RIGHT_ENCODER));
//            int vright = Math.abs(sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_LEFT_ENCODER));
//            
//            if(vleft > vright){
//                leftSpinnerPID.setScalingValue(vright);
//                double difference = leftSpinnerPID.getPID(vright, vleft)/2;
//                leftFlyWheelSpeedOffset -= difference;
//                rightFlyWheelSpeedOffset-= difference;
//            }else if(vright > vleft){
//                rightSpinnerPID.setScalingValue(vleft);
//                double difference = leftSpinnerPID.getPID(vleft, vright)/2;
//                leftFlyWheelSpeedOffset += difference;
//                rightFlyWheelSpeedOffset+= difference;
//            }
            
//            DriverStation.reportError("\nR flywheel velocity:" + vleft, false);
//            DriverStation.reportError("\nL flywheel velocity:" + vright, false);
//            
//            DriverStation.reportError("\nR flywheel power:" + (flywheelSpeedLeft + leftFlyWheelSpeedOffset), false);
//            DriverStation.reportError("\nL flywheel power:" + (flywheelSpeedRight + rightFlyWheelSpeedOffset), false);
        } else if (driverInputControl.getButton(RobotButtonType.FLYWHEEL_IN)) {
          if ( sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER) > 11 
                  && sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER) < 15) {
              flywheelSpeedLeft = SHOOTER_SPEED * .6;
              flywheelSpeedRight = -SHOOTER_SPEED * .6;
              leftFlyWheelSpeedOffset = 0;
              rightFlyWheelSpeedOffset = 0;
          }
          isHeld = false;
          setToTiltValue(13);
          positionTilt();
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
        RobotControlWithSRX.getInstance().updateFlywheelShooter(speedLeft, speedRight);
    }
    public void updateTilt() {     
        int direction = driverInputControl.getShooterTilt();
//        DriverStation.reportError("\nDirection:: " + direction, false);
        if(direction == 0){
            breakTilt();
        }
        else{
            updateTilt(tiltCheck(direction == -1? -0.3:0.3));
        }
    }
    public void updateTilt(double speed) {
        speed = tiltCheck(speed); 
        if (speed > 0) {
            tiltState = MotorState.REVERSE;
        } else if (speed < 0) {
            tiltState = MotorState.FORWARD;
        } else {
            tiltState = MotorState.OFF;
        }
//        DriverStation.reportError("\n Tilt Speed: " + speed, false);
        RobotControlWithSRX.getInstance().updateShooterTilt(speed<-0.2?-0.2:speed);
    }
    public void updateTwist() {
        twistSpeed = 0;
        double twistInput = driverInputControl.getShooterTwist();
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
        RobotControlWithSRX.getInstance().updateShooterTwist(Math.abs(speed) > 0.4? 0.4 * (speed/Math.abs(speed)) :speed);
    }
    public void reset() {
        tiltPID.reset();
        twistPID.reset();
        leftSpinnerPID.reset();
        rightSpinnerPID.reset();
        
        rightState = leftState = MotorState.OFF;
        flywheelSpeedRight = flywheelSpeedLeft = 0;
    }
    /**
     * true = container closed -- false = container open
     */
    public void updateContainer() {
//        if (DriverInputControlSRX.getInstance()
//                .getButton(RobotButtonType.FLYWHEEL_IN) && previousFlywheel) {
//            isHeld = false;
//            toTiltAngle = 13;
//            positionTilt();
//        } else if(DriverInputControlSRX.getInstance()
//                .getButton(RobotButtonType.FLYWHEEL_OUT) && sensorControl.getEncoderVelocity(SensorType.FLYWHEEL_LEFT_ENCODER) >= FLYWHEEL_MAX_SPEED && previousFlywheel){
//            isHeld = false;
//            toTiltAngle = 13;
//            positionTilt();
//        } else{
//            isHeld = true;
//        }
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
        RobotControlWithSRX.getInstance().updateShooterTilt(tiltSpeed);
        RobotControlWithSRX.getInstance().updateShooterTwist(twistSpeed);
         RobotControlWithSRX.getInstance()
         .updateSingleSolenoid(RobotPneumaticType.SHOOTER_CONTAINER,
         isHeld);
//         DriverStation.reportError("\nContainer State:: " + isHeld, false);
//        // RobotControlWithSRX.getInstance()
//        // .updateSingleSolenoid(RobotPneumaticType.SHOOTER_KICKER,
//        // kickerState);
    }
    public void listenHighGoal() {
        if(reseting){
            if(!twistDone && positionTwist()){
                twistDone = true;
            }
            if(twistDone){
                if(positionTilt()){
                    reseting = false;
                }
            }
        }
        else if(driverInputControl.getButton(RobotButtonType.READY_HIGH) && !previousHigh){
            setToTwistValue(0);
            setToTiltValue(HIGH_GOAL_ANGLE);
            twistPID = new PID(D_TWIST_P, D_TWIST_I, D_TWIST_D);
            tiltPID = new PID(D_TILT_P, D_TILT_I, D_TILT_D);
            reseting = true;
            twistDone = false;
        }
        else{
            updateTilt();
            updateTwist();
        }
        previousHigh = driverInputControl.getButton(RobotButtonType.READY_HIGH);
    }
    public void listenLowGoal() {
        if(reseting){
            if(!twistDone && positionTwist()){
                twistDone = true;
            }
            if(twistDone){
                if(positionTilt()){
                    reseting = false;
                }
            }
        }
        else if(driverInputControl.getButton(RobotButtonType.READY_LOW) && !previousLow ){
            setToTwistValue(0);
            setToTiltValue(5);
            twistPID = new PID(D_TWIST_P, D_TWIST_I, D_TWIST_D);
            tiltPID = new PID(D_TILT_P, D_TILT_I, D_TILT_D);
            reseting = true;
            twistDone = false;
        }
        else{
            updateTilt();
            updateTwist();
        }
        previousLow = driverInputControl.getButton(RobotButtonType.READY_LOW);
    }
    public void listenReset() {
        if(reseting){
            if(!twistDone && positionTwist()){
                twistDone = true;
            }
            if(twistDone){
                if(positionTilt()){
                    reseting = false;
                }
            }
        }
        else if(driverInputControl.getButton(RobotButtonType.SHOOTER_RESET) && !previousReset){
            setToTwistValue(0);
            setToTiltValue(13);
            twistPID = new PID(D_TWIST_P, D_TWIST_I, D_TWIST_D);
            tiltPID = new PID(D_TILT_P, D_TILT_I, D_TILT_D);
            reseting = true;
            twistDone = false;
        }
        else{
            updateTilt();
            updateTwist();
        }
        previousReset = driverInputControl.getButton(RobotButtonType.SHOOTER_RESET);
    }
    @Override
    public void update() {
        updateShooter();
        updateContainer();
        updateTilt();
        listenReset();
        listenLowGoal();
        listenHighGoal();
        updateOutputs();
    }
    
    public void setToTiltValue(double angle){
        tiltPID.setScalingValue(angle - sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER));
        toTiltAngle = angle;
    }
    public boolean positionTilt() {
        boolean isInPosition = true;
        tiltSpeed = TILT_BRAKE;
        double currentAngle = sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER);
        
        tiltSpeed = TILT_SPEED * tiltPID.getPID(toTiltAngle, currentAngle);
        
        isInPosition = (currentAngle > toTiltAngle - ANGLE_ERROR) && (currentAngle < toTiltAngle + ANGLE_ERROR);
        
        updateTilt(tiltSpeed);
        
        if (isInPosition) {
            breakTilt();
        }
        
        return isInPosition;
    }
    public double tiltCheck(double tiltInput) {
//        if ( sensorControl.getZeroedPotentiometer(SensorType.))
        double tiltSpeed = TILT_BRAKE;
        double totalTilt = sensorControl
                .getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER);
        double totalTwist = sensorControl
                .getZeroedEncoder(SensorType.SHOOTER_TWIST_ENCODER);
        if (tiltInput > 0) {
            if (totalTilt < TILT_LIMIT_UPPER) {
                tiltSpeed = tiltInput + TILT_BRAKE;
            }
        } else if (tiltInput < 0) {
            if (totalTilt > TILT_LIMIT_LOWER && Math.abs(totalTwist) <= 100) {
                tiltSpeed = tiltInput + TILT_BRAKE;
            }
        }
        return tiltSpeed;
    }
    public void breakTilt(){
//        RobotControlWithSRX.getInstance().updateShooterTilt(TILT_BRAKE * Math.cos(Math.toRadians(sensorControl.getZeroedPotentiometer(SensorType.SHOOTER_TILT_POTENTIOMETER))));
    }
    public void setToTwistValue(double angle){
        twistPID.setScalingValue(twistDegreesToTicks(angle) - getTwistEncoder());
//        DriverStation.reportError("\ngoing to " + twistDegreesToTicks(angle) + "\nwe are at " + getTwistEncoder(), false);
        toTwistTicks = twistDegreesToTicks(angle);
    }
    public boolean positionTwist() {
        twistSpeed = TWIST_SPEED * twistPID.getPID(toTwistTicks, getTwistEncoder());
        boolean isInPosition = (getTwistEncoder() > toTwistTicks - TICK_ERROR) && (getTwistEncoder() < toTwistTicks + TICK_ERROR);
        if(isInPosition){
            twistSpeed = 0;
            breakTwist();
        }
        updateTwist(twistSpeed);
        return isInPosition;
    }
    public void breakTwist(){
        RobotControlWithSRX.getInstance().updateShooterTwist(0);
    }
    public static double twistDegreesToTicks(double angle){
        return (1024.0 / GEAR_RATIO_TWIST) / (360 / angle);
    }
}
