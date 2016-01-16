package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;
import java.util.List;

import org.usfirst.frc.team1885.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class AutonomousRoutine
{
    private Robot robot;
    private LinkedList<AutoCommand> commands;
    private static final double delay = 0.05;
    public AutonomousRoutine(Robot r)
    {
        commands = new LinkedList<AutoCommand>();
        robot = r;
    }
    public void execute()
    {
        while(!commands.isEmpty() && robot.isEnabled() && robot.isAutonomous())
        {
            AutoCommand currCommand = commands.peek();
            if(currCommand.isInit())
            {
                boolean commandState = currCommand.execute();
                currCommand.updateOutputs();
                if(commandState)
                {
                    System.out.println("finished command " + commands.size());
                    commands.poll();
                }
            }
            else
            {
                currCommand.setInit(currCommand.init());
            }
            Timer.delay(delay);
        }
    }
    public void autoCanGrabber() {
        //Moves forward and grabs the cans moves backwards
        commands.add(new AutoCanGrabber(true));
        commands.add(new AutoDriveForward(.75*12, 1, 2));
        commands.add(new AutoWait(1000));
        commands.add(new AutoDriveForward(-6*12, 1, 2));
//      commands.add(new AutoCanGrabber(false));
        
    }
    public void autoOneTote() {
        // Picks up one tote, drives backwards to autozone, drops tote
        commands.add(new AutoClaw(false, false, true));
        commands.add(new AutoToteLift(1210, 10));
        commands.add(new AutoToteLift(1210, 10));
        commands.add(new AutoToteLift(1210, 10));
        commands.add(new AutoDriveForward(-86, 1, 2));
        // commands.add(new AutoToteLift(-1210, 10));
    }
    
    public void autoOneCan() {
        // Picks up one tote, drives backwards to autozone, drops tote
        commands.add(new AutoClaw(false, false, true));
        commands.add(new AutoDriveForward(3 * 12, 1, 2));
        // commands.add(new AutoToteLift(-1210, 10));
    }

    public void autoOneToteTwoBin() {
        // Picks up one tote, pivots and pinches the bin, drives forward and
        // lifts the bin, turn 90 degrees and drive to autozone
        commands.add(new AutoToteLift(1 * 1210, 10));
        commands.add(new AutoTurn(-45, 5));
        commands.add(new AutoDriveForward(1 * 12, 1, 2));
        commands.add(new AutoTurn(-45, 5));
        commands.add(new AutoClaw(false, false, true));
        commands.add(new AutoDriveForward(6 * 12, 1, 2));
        commands.add(new AutoToteLift(3 * 1210, 10));
        commands.add(new AutoTurn(90, 5));
        commands.add(new AutoDriveForward(7 * 12, 1, 2));

    }

    public void autoOneToteOneBin() {
        // Lifts up tote, pivot and lift up bin, drive backwards to autozone
        // commands.add(new AutoDriveForward(3, 1, 2));
        commands.add(new AutoToteLift(1 * 1190, 10));
        commands.add(new AutoToteLift(2 * 1210, 10));
        commands.add(new AutoTurn(48, 5));
        commands.add(new AutoDriveForward(10, 1, 2));
        commands.add(new AutoToteLift(1 * 1210, 10));

        // commands.add(new AutoToteLift(3*1210, 10));
        commands.add(new AutoTurn(-45, 5));
        commands.add(new AutoDriveForward(-7 * 12, 1, 2));
        // commands.add(new AutoToteLift(-3 * 1210, 10));
    }

    public void autoPushOneBinOneTote() {
        // Pushes the tote out of the way, lifts bin, lifts tote, drives
        // backwards to autozone, drops totes
        // Pushes one bin out of the way, pivots and lifts
        commands.add(new AutoDriveForward(30, 1, 2));
        commands.add(new AutoDriveForward(-6, 1, 2));
        commands.add(new AutoTurn(90, 5));

        commands.add(new AutoCommand() {

            AutoDriveForward driveFwd = new AutoDriveForward(10, 1, 2);
            AutoToteLift toteLift = new AutoToteLift(3 * 1210, 10);

            @Override
            public boolean init() {
                // TODO Auto-generated method stub
                return driveFwd.init() && toteLift.init();
            }

            @Override
            public boolean execute() {
                return driveFwd.execute() && toteLift.execute();
            }

            @Override
            public boolean updateOutputs() {
                return driveFwd.updateOutputs() && toteLift.updateOutputs();
            }

            @Override
            public void reset() {
                driveFwd.reset();
                toteLift.reset();
            }

        });
        commands.add(new AutoDriveForward(-5, 1, 2));
        commands.add(new AutoTurn(-90, 5));
        commands.add(new AutoDriveForward(18, 1, 2));
        commands.add(new AutoWait(250));
        commands.add(new AutoDriveForward(3, 1, 2));
        commands.add(new AutoToteLift(1 * 1210, 10));
        commands.add(new AutoDriveForward(-7 * 12, 1, 2));
        commands.add(new AutoToteLift(-1 * 1210, 10));

    }

    public void autoOneBinOneTote() {
        // Pinches the bin, lifts the tote
        commands.add(new AutoClaw(false, false, true)); // rotation, extension,
                                                        // pinch
        commands.add(new AutoDriveForward(.5 * 12, 1, 2));
        commands.add(new AutoTurn(90, 1));
        commands.add(new AutoDriveForward(2.0 * 12, 3, 2));
        commands.add(new AutoTurn(90, 1));
        commands.add(new AutoDriveForward(3.5 * 12, 1, 2));
        commands.add(new AutoToteLift(1210, 10));
        commands.add(new AutoDriveForward(-6 * 12, 1, 2));
        commands.add(new AutoToteLift(-1210, 10));
        // commands.add(new AutoTurn(180, 1));
    }

    public void autoOneBinThreeTotes() {
        //
        commands.add(new AutoToteLift(2 * 1210, 10));
        commands.add(new AutoDriveForward(1 * 12, 1, 2));
        commands.add(new AutoTurn(-60, 1));
        commands.add(new AutoDriveForward(.5 * 12, 1, 2));
        commands.add(new AutoToteLift(1 * 1210, 10));
        commands.add(new AutoDriveForward(6 * 12, 1, 2));
        commands.add(new AutoToteLift(1 * 1210, 10));
        commands.add(new AutoDriveForward(6 * 12, 1, 2));
        commands.add(new AutoToteLift(1 * 1210, 10));
        commands.add(new AutoTurn(-90, 1));
        commands.add(new AutoDriveForward(5 * 12, 1, 2));
    }

    public void autoSimple() {
        commands.add(new AutoToteLift(3 * 1210, 10));
        commands.add(new AutoDriveForward(1.5 * 12, 1, 2));
        commands.add(new AutoTurn(-55, 1));
        commands.add(new AutoDriveForward(.5 * 12, 1, 2));
        // commands.add(new AutoToteLift(-1 * 1210, 10));
        commands.add(new AutoToteLift(1 * 1210, 10));
        commands.add(new AutoTurn(-90, 1));
        commands.add(new AutoDriveForward(5 * 12, 1, 2));
        commands.add(new AutoToteLift(-1 * 1210, 10));
    }
}
