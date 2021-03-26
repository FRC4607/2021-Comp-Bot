package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;

public class AutonFlywheel extends CommandBase {
    private final TransferWheelSubsystem mTransferWheel;
    private final HopperSubsystem mHopperMotor;
    private final IndexerSubsystem mIndexerMotor;

    public AutonFlywheel(TransferWheelSubsystem wheel, HopperSubsystem hopper, IndexerSubsystem index) {
        mTransferWheel = wheel;
        mHopperMotor = hopper;
        mIndexerMotor = index;
        addRequirements(mTransferWheel, mHopperMotor, mIndexerMotor);

    }

    @Override
    public void end(boolean useless) {
        mTransferWheel.disable();
        mHopperMotor.disable();
        mIndexerMotor.disable();
    }

    @Override
    public void initialize() {
        mTransferWheel.enable();
        mHopperMotor.enable();
        mIndexerMotor.enable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
