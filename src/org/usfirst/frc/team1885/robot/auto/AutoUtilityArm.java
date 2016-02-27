package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoUtilityArm extends AutoCommand {
    private UtilityArm uArm;
    private double xDistance, yDistance;
    private boolean reset = false;

    public AutoUtilityArm() {
        reset = true;
    }

    public AutoUtilityArm(double x, double y) {
        uArm = UtilityArm.getInstance();
        xDistance = x;
        yDistance = y;
    }

    @Override
    public boolean init() {
        if (!reset) {
            uArm.goTo(xDistance, yDistance);
        } else {
            uArm.resetPos();
        }
        return true;
    }

    @Override
    public boolean execute() {
        uArm.update();
        return uArm.isFinished();
    }

    @Override
    public boolean updateOutputs() {
        // Done for us in UtilityArm
        return false;
    }

    @Override
    public void reset() {
        uArm.resetPos();
    }

}
