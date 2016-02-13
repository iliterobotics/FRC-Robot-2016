package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoArm extends AutoCommand {
    private final double ERROR;

    private double potentiometerA, potentiometerB;
    private UtilityArm utilityArm;

    public AutoArm(double potA, double potB) {
        utilityArm = UtilityArm.getInstance();
        potentiometerA = potA;
        potentiometerB = potB;
        ERROR = 7;
    }

    @Override
    public boolean init() {
        reset();
        return true;
    }

    @Override
    public boolean execute() {
        boolean isAngleACorrect = potentiometerA
                - utilityArm.getAngleA() < ERROR
                && potentiometerA - utilityArm.getAngleA() > -ERROR;
        boolean isAngleBCorrect = potentiometerB
                - utilityArm.getAngleB() < ERROR
                && potentiometerB - utilityArm.getAngleB() > -ERROR;

        DriverStation.reportError("\nJoint A potentiometer: "
                + utilityArm.getAngleA() + " ::: Joint B potentiometer: "
                + utilityArm.getAngleB(), false);
        DriverStation.reportError("\nJoint A GOAL: " + potentiometerA
                + " ::: Joint B GOAL: " + potentiometerB
                + " ::: isAngleACorrect - " + isAngleACorrect
                + ", isAngleBCorrect - " + isAngleBCorrect, false);
        utilityArm.updateArm(potentiometerA, potentiometerB);

        if (isAngleACorrect && isAngleBCorrect) {
            DriverStation
                    .reportError("\n\nFinal position reacher: Joint A angle - "
                            + utilityArm.getAngleA() + ", Joint B angle - "
                            + utilityArm.getAngleB(), false);
            return true;
        }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        // methods in UtilityArm update outputs for us
        return false;
    }

    @Override
    public void reset() {
        UtilityArm.getInstance().updateArm(0, 0);
    }
}
