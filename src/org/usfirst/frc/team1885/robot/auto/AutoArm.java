package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.manipulator.AuxArm;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutoArm extends AutoCommand {

    private double jointSpeedA, jointSpeedB;
    private double potentiometerA, potentiometerB;

    public AutoArm(double speedA, double speedB, double potA, double potB) {
        jointSpeedA = speedA;
        jointSpeedB = speedB;
        potentiometerA = (potA + SensorInputControlSRX.getInstance()
                .getInitialPotAPostition());
        potentiometerB = (potB + SensorInputControlSRX.getInstance()
                .getInitialPotBPostition());
    }

    @Override
    public boolean init() {
        reset();
        return true;
    }

    @Override
    public boolean execute() {
        boolean isJointAInPlace = false;
        boolean isJointBInPlace = false;
        if (jointSpeedA >= 0) {
            if (SensorInputControlSRX.getInstance()
                    .getAnalogGeneric(SensorType.JOINT_A_CTRE_ABSOLUTE)
                    * AuxArm.CONVERSION_FACTOR >= potentiometerA) {
                isJointAInPlace = true;
                jointSpeedA = 0;
            }
        } else {
            if (SensorInputControlSRX.getInstance()
                    .getAnalogGeneric(SensorType.JOINT_A_CTRE_ABSOLUTE)
                    * AuxArm.CONVERSION_FACTOR <= potentiometerA) {
                isJointAInPlace = true;
                jointSpeedA = 0;
            }
        }

        if (jointSpeedB >= 0) {
            if (SensorInputControlSRX.getInstance()
                    .getAnalogGeneric(SensorType.JOINT_B_CTRE_ABSOLUTE)
                    * AuxArm.CONVERSION_FACTOR >= potentiometerB) {
                isJointBInPlace = true;
                jointSpeedB = 0;
            }
        } else {
            if (SensorInputControlSRX.getInstance()
                    .getAnalogGeneric(SensorType.JOINT_B_CTRE_ABSOLUTE)
                    * AuxArm.CONVERSION_FACTOR <= potentiometerB) {
                isJointBInPlace = true;
                jointSpeedB = 0;
            }
        }

        if (isJointAInPlace && isJointBInPlace) {
            this.reset();
            return true;
        }
        AuxArm.getInstance().updateArm(jointSpeedA, jointSpeedB);
        return false;
    }

    @Override
    public boolean updateOutputs() {
        RobotControlWithSRX.getInstance().updateArmMotors(
                (int) AuxArm.getInstance().getJointASpeed(),
                (int) AuxArm.getInstance().getJointBSpeed());
        return false;
    }

    @Override
    public void reset() {
        AuxArm.getInstance().updateArm(0, 0);
    }
}
