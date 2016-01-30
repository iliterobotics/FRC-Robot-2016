package org.usfirst.frc.team1885.graveyard;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
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
        drive();
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
//        commands.add(new AutoDriveForward(.75*12, 1, 2));;
//      commands.add(new AutoCanGrabber(false));
        
    }
    public void drive() {
        commands.add(new AutoDriveForward(4.0, .8));
    }
}
