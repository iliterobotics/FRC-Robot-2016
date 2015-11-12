package org.usfirst.frc.team1885.robot;

import org.usfirst.frc.team1885.robot.auto.AutoRoutine;
import org.usfirst.frc.team1885.robot.config2015.RobotConfigSRX;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.input.SensorInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends SampleRobot 
{
    private RobotControlWithSRX srx;
    private DriverInputControlSRX drx;
    private SensorInputControlSRX sensorrx;
    public Robot()
    {
        RobotConfigSRX.configureRobot();
        srx = RobotControlWithSRX.getInstance();
        drx = DriverInputControlSRX.getInstance();
        this.sensorrx = new SensorInputControlSRX();
    }
    public void operatorControl()
    {
        while(isOperatorControl() && isEnabled())
        {
            drx.update();
            sensorrx.update();
            Timer.delay(.005);
            //can't update faster than this for motors
        }
            
    }
    public void autonomous()
    {
        AutoRoutine ar = new AutoRoutine(this);
        ar.execute();
    }
}

