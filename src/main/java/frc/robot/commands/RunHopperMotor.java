package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class RunHopperMotor extends CommandBase {
    private final HopperSubsystem mHopperMotor;

    public RunHopperMotor(HopperSubsystem subsytem) {
        mHopperMotor = subsytem;
        addRequirements(subsytem);
    }

    @Override
    public void end(boolean useless) {
        mHopperMotor.disable();
    }

    @Override
    public void initialize() {
        mHopperMotor.enable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
