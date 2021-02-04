package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SwitchDriveMode extends CommandBase {
    private final DrivetrainSubsystem mDrivetrainSubsystem;

    public SwitchDriveMode(DrivetrainSubsystem subsytem) {
        mDrivetrainSubsystem = subsytem;
        addRequirements(subsytem);
    }

    @Override
    public void initialize() {
        mDrivetrainSubsystem.switchMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
