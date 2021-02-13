package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class RunHopperMotor extends CommandBase {
    private final HopperSubsystem mHopperMotor;
    private final IndexerSubsystem mIndexerMotor;

    public RunHopperMotor(HopperSubsystem subsytem, IndexerSubsystem subsystem2) {
        mHopperMotor = subsytem;
        mIndexerMotor = subsystem2;
        addRequirements(subsytem);
        addRequirements(subsystem2);
    }

    @Override
    public void end(boolean useless) {
        mHopperMotor.disable();
        mIndexerMotor.disable();
    }

    @Override
    public void initialize() {
        mHopperMotor.enable();
        mIndexerMotor.enable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
