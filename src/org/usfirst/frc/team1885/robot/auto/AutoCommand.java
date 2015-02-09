package org.usfirst.frc.team1885.robot.auto;


public interface AutoCommand {
	public void init();
	public boolean execute();
	public boolean updateOutputs();
	public void reset();
}
