package org.usfirst.frc.team1885.robot.manipulator;

import org.usfirst.frc.team1885.robot.common.PID;
import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class AuxArm implements Module {

    public static final double ERROR_DISTANCE = 0.3;
    public static final double P = .8, I = 0.02, D = 0;
    public static final double MAX_ARM_SPEED = 0.4;
    public static final double MIN_ARM_SPEED = 0.1;
    
    public static final double STOP_POWER = 0.05;
    public static final double CONVERSION_FACTOR = 360.0 / 1024;
    public static final double JOINT_A_CLOCK_BOUND = 155.0;
    public static final double JOINT_A_COUNTER_BOUND = 235.0;
    private static final double LENGTH_A = 17.5;
    private static final double LENGTH_B = 19.5;
    private static final double PERIMETER_LENGTH = 21.0;
    private static AuxArm instance;
    // joint A is the motor powering the joint directly connected to the base
    // joint B is the motor power the moving joint
    private final double FULL_SPEED = 0.3;
    private final double HALF_SPEED = FULL_SPEED / 2;
    private final double QUARTER_SPEED = HALF_SPEED / 2;
    private double armSpeed;
    private double jointASpeed;
    private double jointBSpeed;
    private MotorState jointAState;
    private MotorState jointBState;
    private PID pid;
    
    protected AuxArm() {
        this.jointAState = MotorState.OFF;
        this.jointBState = MotorState.OFF;
        jointASpeed = 0;
        jointBSpeed = 0;
        armSpeed = FULL_SPEED;
        
        pid = new PID(P, I, D);
    }
    public static AuxArm getInstance() {
        if (instance == null) {
            instance = new AuxArm();
        }
        return instance;
    }
    public double getJointASpeed() {
        return this.jointASpeed;
    }
    public double getJointBSpeed() {
        return this.jointBSpeed;
    }
    public void updateArm() {
        adjustPower();
        double aPosition = (SensorInputControlSRX.getInstance()
                .getAnalogGeneric(SensorType.JOINT_A_POTENTIOMETER)
                * CONVERSION_FACTOR);
        double bPosition = (SensorInputControlSRX.getInstance()
                .getAnalogGeneric(SensorType.JOINT_B_POTENTIOMETER)
                * CONVERSION_FACTOR);
        if (aPosition < JOINT_A_CLOCK_BOUND) {
            jointASpeed = STOP_POWER;
        } else if (aPosition > JOINT_A_COUNTER_BOUND) {
            jointASpeed = -STOP_POWER;
        } else {
            jointASpeed = 0;
        }
        jointBSpeed = 0;

        if ((DriverInputControlSRX.getInstance()
                .getButton(RobotButtonType.ARM_JOINT_A_EXTEND))) {
            jointAState = MotorState.FORWARD;
            jointASpeed = MAX_ARM_SPEED;
        }
        if ((DriverInputControlSRX.getInstance()
                .getButton(RobotButtonType.ARM_JOINT_A_RETRACT))) {
            jointAState = MotorState.REVERSE;
            jointASpeed = -MAX_ARM_SPEED;
        }
        if ((DriverInputControlSRX.getInstance()
                .getButton(RobotButtonType.ARM_JOINT_B_EXTEND))) {
            jointBState = MotorState.FORWARD;
            jointBSpeed = MAX_ARM_SPEED;
        }
        if ((DriverInputControlSRX.getInstance()
                .getButton(RobotButtonType.ARM_JOINT_B_RETRACT))) {
            jointBState = MotorState.REVERSE;
            jointBSpeed = -MAX_ARM_SPEED;
        }
        updateArm(jointASpeed, jointBSpeed);
    }
    public void updateArm(double aSpeed, double bSpeed) {
        jointASpeed = aSpeed;
        jointBSpeed = bSpeed;
        if (aSpeed > STOP_POWER) {
            jointAState = MotorState.REVERSE;
        } else if (aSpeed < STOP_POWER) {
            jointAState = MotorState.FORWARD;
        } else {
            jointAState = MotorState.OFF;
        }
        if (bSpeed > STOP_POWER) {
            jointBState = MotorState.REVERSE;
        } else if (bSpeed < STOP_POWER) {
            jointBState = MotorState.FORWARD;
        } else {
            jointBState = MotorState.OFF;
        }
    }
    public boolean isWithinPerimeterRange(double angleJointA, double angleJointB) {
        SensorInputControlSRX sensorInputControl = SensorInputControlSRX
                .getInstance();
        double zeroedA = angleJointA
                - sensorInputControl.getInitialPotAPostition();
        return (LENGTH_B * (Math.cos(Math.toRadians(
                angleJointB - sensorInputControl.getInitialPotBPostition()
                        - zeroedA))))
                - (LENGTH_A * (Math.cos(Math.toRadians(zeroedA)))) <= (PERIMETER_LENGTH - 1);
    }
    @Override
    public void update() {
        updateArm();
        updateOutputs();
    }

    @Override
    public void updateOutputs() {
        RobotControlWithSRX.getInstance().updateArmMotors(jointASpeed,
                jointBSpeed);
    }
    
    private void adjustPower(){
        SensorInputControlSRX sensorInputControl = SensorInputControlSRX
                .getInstance();
        double angleA = sensorInputControl.getAnalogGeneric(SensorType.JOINT_A_POTENTIOMETER) * CONVERSION_FACTOR;
        double zeroedA = angleA - sensorInputControl.getInitialPotAPostition();
        double angleB = zeroedA  + 360 - ( sensorInputControl.getAnalogGeneric(SensorType.JOINT_B_POTENTIOMETER) * CONVERSION_FACTOR - sensorInputControl.getInitialPotBPostition() + 190);
        double distanceB = //(LENGTH_B * (Math.cos(Math.toRadians(angleB))));
                            -LENGTH_B;
        double distanceA = (LENGTH_A * (Math.cos(Math.toRadians(zeroedA))));
        double distance = distanceA + distanceB;

        DriverStation.reportError("\nAngle A:: " + zeroedA, false);
        DriverStation.reportError("\nAngle B:: " + angleB, false);
        DriverStation.reportError("\nDistance A:: " + distanceA, false);
        DriverStation.reportError("\nDistance B:: " + distanceB, false);
        DriverStation.reportError("\nDistance:: " + distance + "\n", false);
        if(distance > -PERIMETER_LENGTH + ERROR_DISTANCE){
            if(distance < 0){
                armSpeed = MAX_ARM_SPEED * ( 1 - distance / -PERIMETER_LENGTH);
                if(armSpeed < MIN_ARM_SPEED){
                    armSpeed = MIN_ARM_SPEED;
                }
            }
            else armSpeed = MAX_ARM_SPEED;
            
            DriverStation.reportError("" + armSpeed, false);

        }
        else{
         armSpeed = 0;   
        }
    }

}
