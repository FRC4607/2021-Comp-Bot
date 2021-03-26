package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonIntake extends CommandBase {
    private final IntakeSubsystem mIntakeMotorSubsystem;

    public AutonIntake(IntakeSubsystem subsytem) {
        mIntakeMotorSubsystem = subsytem;
        mIntakeMotorSubsystem.setRoller(1);
        addRequirements(subsytem);
    }

    @Override
    public void end(boolean a) {
        mIntakeMotorSubsystem.setRoller(1 + (1/9));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}