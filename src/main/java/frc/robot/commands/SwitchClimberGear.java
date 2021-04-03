package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class SwitchClimberGear extends CommandBase {
    private final ClimberSubsystem mClimber;

    public SwitchClimberGear(ClimberSubsystem climber) {
        mClimber = climber;
        addRequirements(mClimber);
    }

    @Override
    public void execute() {
        mClimber.toggle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
