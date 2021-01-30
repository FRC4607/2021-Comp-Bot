package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShifterSubsystem;

public class SwitchGears extends CommandBase {
    private final ShifterSubsystem mShifterSubsystem;

    public SwitchGears(ShifterSubsystem subsytem) {
        mShifterSubsystem = subsytem;
        addRequirements(subsytem);
    }

    @Override
    public void initialize() {
        mShifterSubsystem.switchGears();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
