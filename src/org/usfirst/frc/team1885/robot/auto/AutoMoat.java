package org.usfirst.frc.team1885.robot.auto;

public class AutoMoat extends AutoCommand{

    @Override
    public boolean init() {
        reset();
        return true;
    }

    @Override
    public boolean execute() {
        (new AutoStartDrive(4, .7)).execute();
        return true;
    }

    @Override
    public boolean updateOutputs() {
        return true;
    }

    @Override
    public void reset() {
    }

}
