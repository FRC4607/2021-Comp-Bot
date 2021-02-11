package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeMotor extends CommandBase {
    private final IntakeSubsystem mIntakeMotorSubsystem;
    private final XboxController mDriver;

    public SetIntakeMotor(IntakeSubsystem subsytem, XboxController driver) {
        mIntakeMotorSubsystem = subsytem;
        mDriver = driver;
        addRequirements(subsytem);
    }

    @Override
    public void execute() {
        mIntakeMotorSubsystem.setRoller(mDriver.getTriggerAxis(Hand.kRight) - mDriver.getTriggerAxis(Hand.kLeft));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}