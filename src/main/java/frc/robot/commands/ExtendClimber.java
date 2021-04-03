package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ExtendClimber extends CommandBase{
    private final ClimberSubsystem mClimber;

    public ExtendClimber(ClimberSubsystem climber) {
        mClimber = climber;
        addRequirements(mClimber);
    }

    @Override
    public void initialize() {
        mClimber.toggle();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}