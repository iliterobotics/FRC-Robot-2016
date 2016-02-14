package org.usfirst.frc.team1885.robot.auto;

import org.usfirst.frc.team1885.robot.config2016.RobotConfiguration;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoDrawbridge extends AutoCommand {

    private final double DRAW_BRIDGE_HEIGHT = 37;
    private final double DIS_TO_ARM_Y = 9;
    private final double DIS_TO_ARM_X = 6;
    private final double ERROR_ZONE = 2;
    private final double ERROR_X = .5;
    private double armAngle;

    /**
     * @return Distance (in inches) the arm needs to go up to reach the top of
     *         the Drawbridge from the top of the ramp.
     */
    public double distanceY() {
        double disOffRamp = DIS_TO_ARM_Y / Math.cos(armAngle);
        DriverStation.reportError("\nDisOffRamp: " + disOffRamp, false);
        double dis = DRAW_BRIDGE_HEIGHT - disOffRamp;
        return dis;
    }

    /**
     * @return Distance (in inches) the arm needs to go forward to reach the top
     *         of the Drawbridge from the top of the ramp.
     */
    public double distanceX() {
        double disToDrawbridge = DIS_TO_ARM_X / Math.cos(armAngle);
        return disToDrawbridge;
    }

    @Override
    public boolean init() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public boolean execute() {
        armAngle = Math.toRadians(
                Math.abs(SensorInputControlSRX.getInstance().getRoll()));
        AutoUtilityArm reachTop = new AutoUtilityArm(distanceX() + ERROR_X,
                distanceY() + ERROR_ZONE);
        AutoUtilityArm reset = new AutoUtilityArm(true);
        DriverStation.reportError("Going to (" + distanceX() + ", "
                + distanceY() + ") ::: Angle is "
                + SensorInputControlSRX.getInstance().getRoll()
                + " ::: Abs angle is " + armAngle, false);
        if (reachTop.execute()) {
            return true;
        }

        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public boolean updateOutputs() {
        // Done for us by AutoUtilityArm class
        return false;
    }

    @Override
    public void reset() {
        new AutoUtilityArm(true);
    }

}
