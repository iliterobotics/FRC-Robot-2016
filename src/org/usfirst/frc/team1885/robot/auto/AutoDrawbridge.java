package org.usfirst.frc.team1885.robot.auto;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;

import edu.wpi.first.wpilibj.DriverStation;

public class AutoDrawbridge {

    private final double DRAW_BRIDGE_HEIGHT = 37;
    private final double DIS_TO_ARM_Y = 9;
    private final double DIS_TO_ARM_X = 6;
    private final double ERROR_ZONE = 2;
    private final double ERROR_X = .5;

    private AutoUtilityArm reachTop, grabBridge, reset;
    private AutoWait delay;
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

    public Collection<AutoCommand> execute() {
        DriverStation.reportError("\nAngle: " + armAngle, false);
        List<AutoCommand> list = new ArrayList<AutoCommand>();
        armAngle = Math.toRadians(
                Math.abs(SensorInputControlSRX.getInstance().getRoll()));
        reachTop = new AutoUtilityArm(distanceX(), distanceY());
        grabBridge = new AutoUtilityArm(distanceX(), distanceY() - 6);
        reset = new AutoUtilityArm();
        delay = new AutoWait(500);
        list.add(reachTop);
        list.add(delay);
        list.add(grabBridge);
        list.add(delay);
        list.add(reset);
        return list;
    }
}
