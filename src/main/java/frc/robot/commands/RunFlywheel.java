package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelSubsystem;

public class RunFlywheel extends CommandBase {
    private final FlywheelSubsystem mFlywheel;
    private final XboxController mOperator;

    public RunFlywheel(FlywheelSubsystem flywheel, XboxController operator) {
        mFlywheel = flywheel;
        mOperator = operator;
        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        if (mOperator.getRawAxis ( 3 ) > Constants.FLYWHEEL.DEADBAND) {
            mFlywheel.setClosedLoop((mOperator.getRawAxis ( 3 ) - Constants.FLYWHEEL.DEADBAND) * 6000);
        }
        else {
            mFlywheel.setOpenLoop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

