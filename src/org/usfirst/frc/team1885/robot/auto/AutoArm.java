package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

public class AutoArm extends AutoCommand {
    
    private double jointASpeed, jointBSpeed;
    private double potentiometerA, potentiometerB;
    private UtilityArm utilityArm;
    public AutoArm( double speedA, double speedB, double potA, double potB ) {
        utilityArm = UtilityArm.getInstance();
        jointASpeed = speedA;
        jointBSpeed = speedB;
        potentiometerA = potA;
        potentiometerB = potB;
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
            if (utilityArm.getAngleA() >= potentiometerA ) {
                isJointAInPlace = true;
                jointASpeed = 0;
            }
        } else{
            if ( utilityArm.getAngleA() <= potentiometerA ) {
                isJointAInPlace = true;
                jointASpeed = 0;
            }
        }
        
        if( jointBSpeed >= 0 ){
            if ( utilityArm.getAngleB() >= potentiometerB ) {
                isJointBInPlace = true;
                jointBSpeed = 0;
            }
        } else{
            if ( utilityArm.getAngleB() <= potentiometerB ) {
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
