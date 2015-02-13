package org.usfirst.frc.team1885.robot.auto;


public abstract class AutoCommand {
	
	protected boolean isInit = false;
	
	public abstract boolean init();
	public abstract boolean execute();
	public abstract boolean updateOutputs();
	public abstract void reset();
	
	public boolean isInit() {
		return isInit;
	}
	public void setInit(boolean isInit) {
		this.isInit = isInit;
	}
}
