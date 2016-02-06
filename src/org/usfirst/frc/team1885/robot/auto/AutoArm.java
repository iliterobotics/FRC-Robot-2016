package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.manipulator.AuxArm;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoArm extends AutoCommand {
    
    private double jointSpeedA, jointSpeedB;
    private double potentiometerA, potentiometerB;
    public AutoArm( double speedA, double speedB, double potA, double potB ) {
        jointSpeedA = speedA;
        jointSpeedB = speedB;
        potentiometerA = potA;
        potentiometerB = potB + Robot.INITIAL_JOINT_B_POSITION;
    }


    @Override
    public boolean init() {
        reset();
        return true;
    }

    @Override
    public boolean execute() {
        DriverStation.reportError("\nAngle: " + SensorInputControlSRX.getInstance().getAnalogInPosition(SensorType.JOINT_B_POTENTIOMETER) / 1024.0 * 360 + "\nJointB Power: " + AuxArm.getInstance().getJointBSpeed(), false);
        boolean isJointAInPlace = true;
        boolean isJointBInPlace = false;
//        if ( SensorInputControlSRX.getInstance().getAnalogInPosition(SensorType.JOINT_A_POTENTIOMETER) / 1024.0 * 360 >= potentiometerA ) {
//            isJointAInPlace = true;
//            jointSpeedA = 0;
//        }
        if ( SensorInputControlSRX.getInstance().getAnalogInPosition(SensorType.JOINT_B_POTENTIOMETER) / 1024.0 * 360 >= potentiometerB ) {
            isJointBInPlace = true;
            jointSpeedB = 0;
        }
        if ( isJointAInPlace && isJointBInPlace ) {
            this.reset();
            return true;
        }
        AuxArm.getInstance().updateArm( jointSpeedA, jointSpeedB );
        return false;
    }

    @Override
    public boolean updateOutputs() {
        RobotControlWithSRX.getInstance().updateArmMotors(AuxArm.getInstance().getJointASpeed(), AuxArm.getInstance().getJointBSpeed());
        return false;
    }

    @Override
    public void reset() {
        AuxArm.getInstance().updateArm(0, 0);        
    }
}
