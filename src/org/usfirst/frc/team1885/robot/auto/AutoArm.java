package org.usfirst.frc.team1885.robot.auto;

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
        potentiometerA = (potA + SensorInputControlSRX.getInstance().getInitialPotAPostition());
        potentiometerB = (potB + SensorInputControlSRX.getInstance().getInitialPotBPostition());
        DriverStation.reportError("\n" + potentiometerB + " :: " + speedB, false);
    }


    @Override
    public boolean init() {
        reset();
        return true;
    }

    @Override
    public boolean execute() {
//        DriverStation.reportError("\nAngle: " + SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_B_POTENTIOMETER) * AuxArm.CONVERSION_FACTOR + "\nJointB Power: " + AuxArm.getInstance().getJointBSpeed(), false);
//        DriverStation.reportError("\nAngle: " + SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_A_POTENTIOMETER) * AuxArm.CONVERSION_FACTOR + "\nJointB Power: " + AuxArm.getInstance().getJointASpeed(), false);
        boolean isJointAInPlace = false;
        boolean isJointBInPlace = false;
        if( jointSpeedA >= 0 ){
            if ( SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_A_POTENTIOMETER) * AuxArm.CONVERSION_FACTOR >= potentiometerA ) {
                isJointAInPlace = true;
                jointSpeedA = 0;
            }
        } else{
            if ( SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_A_POTENTIOMETER) * AuxArm.CONVERSION_FACTOR <= potentiometerA ) {
                isJointAInPlace = true;
                jointSpeedA = 0;
            }
        }
        
        if( jointSpeedB >= 0 ){
            if ( SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_B_POTENTIOMETER) * AuxArm.CONVERSION_FACTOR >= potentiometerB ) {
                isJointBInPlace = true;
                jointSpeedB = 0;
            }
        } else{
            if ( SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_B_POTENTIOMETER) * AuxArm.CONVERSION_FACTOR <= potentiometerB ) {
                isJointBInPlace = true;
                jointSpeedB = 0;
            }
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
