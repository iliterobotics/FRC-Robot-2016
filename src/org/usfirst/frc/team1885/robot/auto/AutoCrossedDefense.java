package org.usfirst.frc.team1885.robot.auto;

public class AutoCrossedDefense extends AutoCommand {
    // autonomous checkpoint used to check anything after we cross a defense and
    // when to stop

    @Override
    public boolean init() {
        return true;
    }

    @Override
    public boolean execute() {
        // if(){
        // return true;
        // }
        return false;
    }

    @Override
    public boolean updateOutputs() {
        // no outputs to update, only a checkpoint
        return false;
    }

    @Override
    public void reset() {
        // no outputs changed, only a checkpoint
    }

}
