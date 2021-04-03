package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RetractClimber extends CommandBase{
    private final ClimberSubsystem mClimber;

    public RetractClimber(ClimberSubsystem climber) {
        mClimber = climber;
        addRequirements(mClimber);
    }

    @Override
    public void initialize() {
        mClimber.lock();
        mClimber.set(0.5);
    }

    @Override
    public void end(boolean a) {
        mClimber.set(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
