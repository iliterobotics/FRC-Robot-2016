package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;

public class AutoUtilityArm extends AutoCommand {

    private UtilityArm uArm;
    private double xDistance, yDistance;
    private boolean reset;

    public AutoUtilityArm(boolean b) {
        reset = b;
    }

    public AutoUtilityArm(int x, int y) {
        uArm = UtilityArm.getInstance();
        xDistance = x;
        yDistance = y;
    }

    @Override
    public boolean init() {
        if (!reset) {
            reset();
        } else {
            uArm.goTo(xDistance, yDistance);
        }
        return false;
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
