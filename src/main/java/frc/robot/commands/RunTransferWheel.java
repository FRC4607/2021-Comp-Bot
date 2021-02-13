package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferWheelSubsystem;

public class RunTransferWheel extends CommandBase {
    private final TransferWheelSubsystem mTransferWheel;

    public RunTransferWheel(TransferWheelSubsystem subsytem) {
        mTransferWheel = subsytem;
        addRequirements(subsytem);
    }

    @Override
    public void end(boolean useless) {
        mTransferWheel.disable();
    }

    @Override
    public void initialize() {
        mTransferWheel.enable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
