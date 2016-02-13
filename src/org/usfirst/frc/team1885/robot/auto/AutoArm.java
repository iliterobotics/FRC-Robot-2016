package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoArm extends AutoCommand {
    
    private double jointASpeed, jointBSpeed;
    private double potentiometerA, potentiometerB;
    public AutoArm( double speedA, double speedB, double potA, double potB ) {
        jointASpeed = speedA;
        jointBSpeed = speedB;
        potentiometerA = (potA + SensorInputControlSRX.getInstance().getInitialPotAPostition());
        potentiometerB = (potB + SensorInputControlSRX.getInstance().getInitialPotBPostition());
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
        if( jointASpeed >= 0 ){
            if ( SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_A_POTENTIOMETER) * UtilityArm.CONVERSION_FACTOR >= potentiometerA ) {
                isJointAInPlace = true;
                jointASpeed = 0;
            }
        } else{
            if ( SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_A_POTENTIOMETER) * UtilityArm.CONVERSION_FACTOR <= potentiometerA ) {
                isJointAInPlace = true;
                jointASpeed = 0;
            }
        }
        
        if( jointBSpeed >= 0 ){
            if ( SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_B_POTENTIOMETER) * UtilityArm.CONVERSION_FACTOR >= potentiometerB ) {
                isJointBInPlace = true;
                jointBSpeed = 0;
            }
        } else{
            if ( SensorInputControlSRX.getInstance().getAnalogGeneric(SensorType.JOINT_B_POTENTIOMETER) * UtilityArm.CONVERSION_FACTOR <= potentiometerB ) {
                isJointBInPlace = true;
                jointBSpeed = 0;
            }
        }
        
        if ( isJointAInPlace && isJointBInPlace ) {
            this.reset();
            return true;
        }
        UtilityArm.getInstance().updateArm( jointASpeed, jointBSpeed );
        return false;
    }

    @Override
    public boolean updateOutputs() {
        RobotControlWithSRX.getInstance().updateArmMotors(UtilityArm.getInstance().getJointASpeed(), UtilityArm.getInstance().getJointBSpeed());
        return false;
    }

    @Override
    public void reset() {
        UtilityArm.getInstance().updateArm(0, 0);        
    }
}
