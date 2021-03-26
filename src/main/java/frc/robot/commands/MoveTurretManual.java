package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretManual extends CommandBase {
    private final TurretSubsystem mTurret;
    private final XboxController mOperator;

    private double mTarget;

    public MoveTurretManual(TurretSubsystem turret, XboxController operator) {
        mTurret = turret;
        mOperator = operator;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (mOperator.getStickButton(Hand.kLeft)) {
            mTurret.enableTurret(mOperator.getX(Hand.kLeft) * 0.1);
        }
        else {
            mTurret.disableTurret();
        }
        if (Math.abs(mOperator.getY(Hand.kRight)) > Constants.HOOD.DEADBAND) {
            mTurret.enableHood(mOperator.getY(Hand.kRight) * -0.25);
        }
        else {
            mTurret.disableHood();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}