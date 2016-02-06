package org.usfirst.frc.team1885.robot.manipulator;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AuxArm implements Module {

    public static final double ARM_SPEED = 0.3;
    public static final double STOP_POWER = 0.0;
    public static final double CONVERSION_FACTOR = 360.0 / 1024;
    public static final double JOINT_A_CLOCK_BOUND = 13.0;
    public static final double JOINT_A_COUNTER_BOUND = 20.0;
    private static final double JOINT_B_BOUND = 0;
    private static AuxArm instance;
    // joint A is the motor powering the joint directly connected to the base
    // joint B is the motor power the moving joint
    private double jointASpeed;
    private double jointBSpeed;
    private MotorState jointAState;
    private MotorState jointBState;

    protected AuxArm() {
        this.jointAState = MotorState.OFF;
        this.jointBState = MotorState.OFF;
        jointASpeed = 0;
        jointBSpeed = 0;
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
        double aPosition = (SensorInputControlSRX.getInstance().getAnalogInPosition(
                SensorType.JOINT_A_POTENTIOMETER) * CONVERSION_FACTOR);
        double bPosition = (SensorInputControlSRX.getInstance().getAnalogInPosition(
                SensorType.JOINT_B_POTENTIOMETER) * CONVERSION_FACTOR);
        if (aPosition < JOINT_A_CLOCK_BOUND) {
            jointASpeed = STOP_POWER;
        } else if(aPosition > JOINT_A_COUNTER_BOUND){
            jointASpeed = -STOP_POWER;
        } else{
            jointASpeed = 0;
        }
        jointBSpeed = 0;
            
        if ((DriverInputControlSRX.getInstance()
                .getButton(RobotButtonType.ARM_JOINT_A_CLOCK))) {
            jointAState = MotorState.FORWARD;
            jointASpeed = ARM_SPEED;
        }
        if ((DriverInputControlSRX.getInstance()
                .getButton(RobotButtonType.ARM_JOINT_A_COUNTER))) {
            jointAState = MotorState.REVERSE;
            jointASpeed = -ARM_SPEED;
        }
        if ((DriverInputControlSRX.getInstance()
                .getButton(RobotButtonType.ARM_JOINT_B_CLOCK))) {
            jointBState = MotorState.FORWARD;
            jointBSpeed = ARM_SPEED;
        }
        if ((DriverInputControlSRX.getInstance()
                .getButton(RobotButtonType.ARM_JOINT_B_COUNTER))) {
            jointBState = MotorState.REVERSE;
            jointBSpeed = -ARM_SPEED;
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

}
