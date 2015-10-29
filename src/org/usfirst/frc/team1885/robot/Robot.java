package org.usfirst.frc.team1885.robot;

import org.usfirst.frc.team1885.robot.common.type.RobotJoystickType;
import org.usfirst.frc.team1885.robot.config2015.RobotConfigSRX;
import org.usfirst.frc.team1885.robot.input.DriverInputControl;
import org.usfirst.frc.team1885.robot.input.DriverInputControlSRX;
import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick.AxisType;

public class Robot extends SampleRobot 
{
    private RobotControlWithSRX srx;
    private DriverInputControlSRX drx;
    public Robot()
    {
        RobotConfigSRX.configureRobot();
        srx = RobotControlWithSRX.getInstance();
        drx = DriverInputControlSRX.getInstance();
    }
    public void operatorControl()
    {
        while(isOperatorControl() && isEnabled())
        {
            drx.update(drx.getJoystick(RobotJoystickType.LEFT_DRIVE).getAxis(AxisType.kY), drx.getJoystick(RobotJoystickType.RIGHT_DRIVE).getAxis(AxisType.kY));
            Timer.delay(.005);
        }
            
    }
}

