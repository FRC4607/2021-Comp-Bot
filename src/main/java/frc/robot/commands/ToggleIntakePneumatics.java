package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntakePneumatics extends CommandBase {
    private final IntakeSubsystem mPneumaticsSubsystem;

    public ToggleIntakePneumatics(IntakeSubsystem subsytem) {
        mPneumaticsSubsystem = subsytem;
        addRequirements(subsytem);
    }

    @Override
    public void initialize() {
        mPneumaticsSubsystem.toggleIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
