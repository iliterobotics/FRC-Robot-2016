package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.Robot;

import edu.wpi.first.wpilibj.Timer;


public class AutonomousRoutine
{
    public static final double PITCH_CHANGE = 4.5;
    
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
    //STANDARD CONFIGURATION
    //startDrive
    //reachedDefense
    //in between
    //crossedDefense
    public void AutoMoatRoughTerrainRockWall() {
        commands.add(new AutoMoat());
    }
}
