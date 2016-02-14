package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoUtilityArm extends AutoCommand {

    private UtilityArm uArm;
    private double xDistance, yDistance;
    private boolean reset = false;

    public AutoUtilityArm(boolean b) {
        reset = b;
    }

    public AutoUtilityArm(double x, double y) {
        uArm = UtilityArm.getInstance();
        xDistance = x;
        yDistance = y;
    }

    @Override
    public boolean init() {
        if (reset) {
            xDistance = 0;
            yDistance = 4;
        } else {
            uArm.goTo(xDistance, yDistance);
        }
        return true;
    }

    @Override
    public boolean execute() {
        DriverStation.reportError("WOO", false);
        uArm.goTo(xDistance, yDistance);
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
