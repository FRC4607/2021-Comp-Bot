package frc.robot.commands;

import org.slf4j.LoggerFactory;

import org.slf4j.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.Vision.State;
import frc.robot.lib.controllers.Vision.Status;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAlignment extends CommandBase {
    private final TurretSubsystem mTurret;

    private final Logger mLogger = LoggerFactory.getLogger( TurretAlignment.class );

    private Status mStatus;

    private double mTarget;

    public TurretAlignment(TurretSubsystem turret) {
        mTurret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // Make sure the vision thread is processing the turning output
        if ( mTurret.mLimelight.getState() == State.kTurn ) {
            mStatus = mTurret.mLimelight.getStatus();
            // Check the status of the controller
            if ( mStatus == Status.kTargeting ) {
                mTurret.enableTurret( mTurret.mLimelight.getTurretOutput() );
                // mLogger.info( "Target at: [{}]", mLimelight.horizontalToTargetDeg()); 
            } else if ( mStatus == Status.kLostTarget ) {
                // mIsFinished = true;
                mLogger.info( "Lost target" );
            } else if ( mStatus == Status.kReachedTarget ) {
                // mIsFinished = true;
                mLogger.info( "Reached target" );
            } else {
                mTurret.disableTurret();
                mLogger.warn( "Unknown status: [{}]", mStatus );
            }
            SmartDashboard.putNumber("Limelight Measurement", mTurret.mLimelight.getVerticalDegrees());
            if (mTurret.mLimelight.getVerticalDegrees() != 0.0) {
                mTarget = 90 - mTurret.mLimelight.getVerticalDegrees() - 19;
                mTarget -= SmartDashboard.getNumber("Turret Trim", 0);
            }
            SmartDashboard.putNumber("Hood Target", mTarget);
            double error = mTarget - mTurret.mEncoder.getPosition();
            if (Math.abs(error) < 1) {
                error = 0;
            }
            SmartDashboard.putNumber("Hood Error", error);
            double speed = (Math.sqrt(Math.pow(error, 2)/8100) + 0.1) * Math.signum(error);
            mTurret.enableHood(speed);
            SmartDashboard.putNumber("Hood Speed", speed);
        }
    }

    @Override
    public void initialize() {
        mTurret.enableVision();
    }

    @Override
    public void end(boolean useless) {
        mTurret.disableTurret();
        mTurret.disableVision();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
