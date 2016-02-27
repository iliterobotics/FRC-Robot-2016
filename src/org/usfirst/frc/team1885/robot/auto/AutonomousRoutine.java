package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;
import org.usfirst.frc.team1885.robot.common.type.SensorType;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.manipulator.UtilityArm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutonomousRoutine {
    private Robot robot;
    private LinkedList<AutoCommand> commands;
    private static final double delay = 0.005;

    public AutonomousRoutine(Robot r) {
        commands = new LinkedList<AutoCommand>();
        robot = r;
        drawbridge();
    }
    public void execute() {
        int size = commands.size();
        while (!commands.isEmpty() && robot.isEnabled()
                && robot.isAutonomous()) {
            AutoCommand currCommand = commands.peek();
            if (currCommand.isInit()) {
                boolean commandState = currCommand.execute();
                currCommand.updateOutputs();
                if (commandState) {
                    commands.poll();
                    // DriverStation.reportError("\nJoint A Position: "
                    // + SensorInputControlSRX.getInstance()
                    // .getAnalogGeneric(
                    // SensorType.JOINT_A_POTENTIOMETER)
                    // + " --- Joint B Position: "
                    // + SensorInputControlSRX.getInstance()
                    // .getAnalogGeneric(
                    // SensorType.JOINT_B_POTENTIOMETER)
                    // + "\n Finished Command #"
                    // + (size - commands.size()), false);
                }
            } else {
                currCommand.setInit(currCommand.init());
            }
            Timer.delay(delay);
        }
    }

    private final double X_ERROR = 1;
    private final double Y_OVERSHOOT_DISTANCE = 2;

    public void drawbridge() {
        double disXInit = 0;
        double disYInit = 30 + Y_OVERSHOOT_DISTANCE; // Initialize arm position
        commands.add(new AutoUtilityArm(disXInit, disYInit));
        commands.add(new AutoWait(100));
        double disXHangOver = 6 + X_ERROR;
        double disYHangOver = 28.75 + Y_OVERSHOOT_DISTANCE; // Hang over the
        // drawbridge
        commands.add(new AutoUtilityArm(-disXHangOver, disYHangOver));
        commands.add(new AutoWait(300));
        double disXGrabOnto = disXHangOver;
        double disYGrabOnto = 26.25 + Y_OVERSHOOT_DISTANCE; // 'Grab' onto the
                                                            // bridge
        commands.add(new AutoUtilityArm(-disXGrabOnto, disYGrabOnto));
        commands.add(new AutoWait(100));
        // commands.add(new Auto); drive back SLOWLY
        double disXPushDown = 16 + X_ERROR;
        double disYPushDown = -2.3 - Y_OVERSHOOT_DISTANCE; // Push drawbridge
        // the rest of the way down
        commands.add(new AutoUtilityArm(-disXPushDown, disYPushDown));
        commands.add(new AutoWait(100));
        double disXCompactArm = 9 + X_ERROR;
        double disYCompactArm = -4.2 - Y_OVERSHOOT_DISTANCE; // Bring arm
        // closer to us, to make sure the wheels can go over the bridge
        commands.add(new AutoUtilityArm(-disXCompactArm, disYCompactArm));
        commands.add(new AutoWait(300));
        // commands.add(new Auto); drive forward onto ramp, somewhat slow
        commands.add(new AutoWait(1000));
    }
}
