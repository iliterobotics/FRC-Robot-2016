package org.usfirst.frc.team1885.robot.auto;

public class AutoRamparts extends AutoCommand{

    @Override
    public boolean init() {
        reset();
        return true;
    }

    @Override
    public boolean execute() {
        // Drive forward and once BusVoltage dips drop right side power so that it goes over evenly
        // After we somehow figure out when we are done we stop the robot possibly on a time sensitive gyroscopic readings
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
