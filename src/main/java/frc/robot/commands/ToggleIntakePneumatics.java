package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakePneumaticsSubsystem;

public class ToggleIntakePneumatics extends CommandBase {
    private final IntakePneumaticsSubsystem mPneumaticsSubsystem;

    public ToggleIntakePneumatics(IntakePneumaticsSubsystem subsytem) {
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
