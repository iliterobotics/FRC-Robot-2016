package org.usfirst.frc.team1885.robot.auto;

import java.util.LinkedList;

import org.usfirst.frc.team1885.robot.output.RobotControlWithSRX;

import edu.wpi.first.wpilibj.Timer;

public class AutoTemplate extends AutoCommand{

	private LinkedList<AutoCommand> commands;
	
	public boolean init() {
		commands = new LinkedList<AutoCommand>();
		return false; 
	}

	public void add(AutoCommand command) {
		this.commands.add(command);
	}
	
	public boolean execute() {
		 AutoCommand currCommand = commands.peek();


         if(currCommand.isInit()) {
             boolean commandState = currCommand.execute();
             currCommand.updateOutputs();
             if(commandState) {
                 System.out.println("Finished command " + commands.size());
                 commands.poll();
             }
         } else {
             currCommand.setInit(currCommand.init());
         }

         Timer.delay(.005);
		return commands.isEmpty();
	}

	
	public boolean updateOutputs() {
	     
		return false;
	}

	
	public void reset() {
		commands.clear();
		RobotControlWithSRX.getInstance().updateDriveSpeed(0, 0);
	}

}
