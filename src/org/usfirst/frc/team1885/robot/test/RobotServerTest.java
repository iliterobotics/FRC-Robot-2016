package org.usfirst.frc.team1885.robot.test;

import org.usfirst.frc.team1885.robot.comms.RobotServer;
import org.usfirst.frc.team1885.robot.comms.RobotServerEvent;
import org.usfirst.frc.team1885.robot.comms.RobotServerListener;

import edu.wpi.first.wpilibj.SampleRobot;

public class RobotServerTest extends SampleRobot implements RobotServerListener {
    private RobotServer robotServer;
    public RobotServerTest() {	
    	this.robotServer = RobotServer.getInstance();
    	this.robotServer.setup(4444);
    	this.robotServer.addListener( this );
    	if ( this.robotServer.startServer() ) {
    		System.out.println("Robot::Constructor - Successfully started data server!");
    	}
    }    
   
    public void receivedServerEvent( RobotServerEvent event ) {
    	System.out.println( event.getMessage() );
    }

}
