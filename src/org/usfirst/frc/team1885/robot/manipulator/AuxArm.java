package org.usfirst.frc.team1885.robot.manipulator;

import org.usfirst.frc.team1885.robot.common.type.MotorState;
import org.usfirst.frc.team1885.robot.common.type.RobotButtonType;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.modules.Module;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AuxArm implements Module{

    public static final double ARM_SPEED = 0.5;
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
    
    public void updateArm() {
        jointASpeed = 0;
        jointBSpeed = 0;
        
        if ((DriverInputControlSRX.getInstance().getButton(
                RobotButtonType.INTAKE_IN))) {
                jointAState = MotorState.REVERSE;
                jointBState = MotorState.FORWARD;
                jointASpeed = ARM_SPEED;
                jointBSpeed = -ARM_SPEED;
            }
    
        if ((DriverInputControlSRX.getInstance().getButton(
                RobotButtonType.INTAKE_OUT))) {
                jointAState = MotorState.FORWARD;
                jointBState = MotorState.REVERSE;
                jointASpeed = -ARM_SPEED;
                jointBSpeed = ARM_SPEED;
            }
        updateArm(jointASpeed, jointBSpeed);
    }
    public void updateArm(double aSpeed, double bSpeed) {
        jointASpeed = aSpeed;
        jointBSpeed = bSpeed;
        if (aSpeed > 0) {
            jointAState = MotorState.REVERSE;
        } else if (aSpeed < 0) {
            jointAState = MotorState.FORWARD;
        } else {
            jointAState = MotorState.OFF;
        }
        if (bSpeed > 0) {
            jointBState = MotorState.REVERSE;
        } else if (bSpeed < 0) {
            jointBState = MotorState.FORWARD;
        } else {
            jointBState = MotorState.OFF;
        }
     }
    @Override
    public void update() {
        updateArm();        
    }

    @Override
    public void updateOutputs() {
        RobotControlWithSRX.getInstance().updateArmMotors(jointASpeed, jointBSpeed);
    }

}
