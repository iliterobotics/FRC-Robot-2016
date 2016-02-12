package org.usfirst.frc.team1885.robot.manipulator;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * @author ILITE Robotics
 * @version 2/12/2016
 * 
 * UtilityArm is the code for the side arm of the 2016 Stronghold Robot
 * The arm features a double jointed system with a potentiometer and motor attached to each joint
 */
public class UtilityArm implements Module {

    private static UtilityArm instance;
    
    private static final double LENGTH_A = 17.5;
    private static final double LENGTH_B = 19.5;
    public static final double CONVERSION_FACTOR = 360.0 / 1024;
    
    private double jointASpeed; //speed that joint A will go at
    private double jointBSpeed; //speed that joint B will go at
    private double jointAAngle; //storage for updating the A angle
    private double jointBAngle; //storage for updating the B angle
    private double xDirection; //what direction the system needs to move in
    private double yDirection; //what direction the system needs to move in
    
    @SuppressWarnings("unused")
    private MotorState jointAState;
    @SuppressWarnings("unused")
    private MotorState jointBState;
    private SensorInputControlSRX sensorInputControl;
    private DriverInputControlSRX driverInputControl;
    
    protected UtilityArm() {
        this.jointAState = MotorState.OFF;
        this.jointBState = MotorState.OFF;
        jointAAngle = getAngleA();
        jointBAngle = getAngleB();
        xDirection = 0;
        yDirection = 0;
        sensorInputControl = SensorInputControlSRX.getInstance();
        driverInputControl = DriverInputControlSRX.getInstance();
    }
    
    //TODO singletons cause memory leaks
    public static UtilityArm getInstance() {
        if (instance == null) {
            instance = new UtilityArm();
        }
        return instance;
    }
    
    @Override
    public void update() {
        changeValues();
        updateOutputs();
    }

    @Override
    public void updateOutputs() {
        if(jointAAngle - getAngleA() > 5) {
            jointASpeed = .5;
        }
        else if(jointAAngle - getAngleA() < -5) {
            jointASpeed = .5;
        }
        else {
            jointASpeed = 0;
        }
        if(jointBAngle - getAngleB() > 5) {
            jointBSpeed = -.5;
        }
        else if(jointBAngle - getAngleB() < -5) {
            jointBSpeed = .5;
        }
        else {
            jointBSpeed = 0;
        }
        RobotControlWithSRX.getInstance().updateArmMotors(jointASpeed, jointBSpeed);
    }
    
    /*
     * Gets x position for the end-point of the utility arm with respect to the base joint in inches
     */
    public double getDistanceX() {
        double distanceB = (LENGTH_B * (Math.cos(Math.toRadians(getAngleB()))));
        double distanceA = (LENGTH_A * (Math.cos(Math.toRadians(getAngleA()))));
        return distanceA + distanceB;
    }
    
    /*
     * Gets y position for the end-point of the utility arm with respect to the base joint in inches
     */
    public double getDistanceY() {
        double distanceB = (LENGTH_B * (Math.sin(Math.toRadians(getAngleB()))));
        double distanceA = (LENGTH_A * (Math.sin(Math.toRadians(getAngleA()))));
        return distanceA + distanceB;
    }

    /*
     * Gets angle in degrees for the bottom joint going clockwise from 0
     */
    public double getAngleA() {
        sensorInputControl = SensorInputControlSRX.getInstance();
        double angleA = sensorInputControl.getAnalogGeneric(SensorType.JOINT_A_POTENTIOMETER) * CONVERSION_FACTOR;
        double zeroedA = angleA - sensorInputControl.getInitialPotAPostition();
        return zeroedA;
    }
    
    /*
     * Gets angle in degrees for the top joint going clockwise from 0
     */
    public double getAngleB() {
        double angleA = getAngleA();
        double angleB = angleA + 360 - ( sensorInputControl.getAnalogGeneric(SensorType.JOINT_B_POTENTIOMETER) 
                * CONVERSION_FACTOR - sensorInputControl.getInitialPotBPostition() + 190);
        return angleB;
    }
    
    /*
     * Finds the conversion from the joystick throttle value to x movement in inches
     */
    public double getJoystickXConversion() {
        double throttle = driverInputControl.getControllerThrottle();
        if (throttle > 0.1) {
            return xDirection = 0.1;
        }
        else if (throttle < -0.1) {
            return xDirection = -0.1;
        }
        else {
            return xDirection = 0.0;
        }
    }
    
    /*
     * Finds the conversion from the joystick twist value to y movement in inches
     */
    public double getJoystickYConversion() {
        double twist = driverInputControl.getControllerTwist();
        if (twist > 0.1) {
            return yDirection = -0.1;
        }
        else if (twist < -0.1) {
            return yDirection = 0.1;
        }
        else {
            return yDirection = 0.0;
        }
    }
    
    /*
     * Takes the change in x and y movement and converts them to angular change
     */
    public void changeValues() {
        //change in Y
        double origA = getAngleA();
        double sinY = Math.sin(Math.toRadians(getAngleB())) + getJoystickYConversion();
        if ( sinY <= 1 && sinY >= -1 ) {
            jointBAngle = 180 - Math.toDegrees(Math.asin(sinY));
            double cosX = Math.cos(Math.toRadians(getAngleA())) - 
                    (Math.cos(Math.toRadians(jointBAngle)) - Math.cos(Math.toRadians(getAngleB())));
            if ( cosX <= 0 && cosX >= -1 ) {
                //TODO set cosx to the limit that it is passing
                DriverStation.reportError("cos good", false);
                jointAAngle = Math.toDegrees(Math.acos(cosX));
            }
        }
        
        
        //change in X
        double cosX = Math.cos(Math.toRadians(jointAAngle)) + getJoystickXConversion();
        if ( cosX <= 0 && cosX >= -1 ) {
            jointAAngle = Math.toDegrees(Math.acos(cosX));
            sinY = Math.sin(Math.toRadians(jointBAngle)) -
                    (Math.sin(Math.toRadians(jointAAngle)) - Math.cos(Math.toRadians(origA)));
            if ( sinY <= 1 && sinY >= -1 ) {
                jointBAngle = 180 - Math.toDegrees(Math.asin(sinY));
            } 
        }       
        
        DriverStation.reportError("desired A angle:" + jointAAngle + "\n", false);
        DriverStation.reportError("desired B angle:" + jointBAngle + "\n", false);
    }
}
