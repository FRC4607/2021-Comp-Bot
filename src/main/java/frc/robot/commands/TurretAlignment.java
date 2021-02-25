package frc.robot.commands;

import org.slf4j.LoggerFactory;

import org.slf4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.Vision.State;
import frc.robot.lib.controllers.Vision.Status;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAlignment extends CommandBase {
    private final TurretSubsystem mTurret;

    private final Logger mLogger = LoggerFactory.getLogger( TurretAlignment.class );

    private Status mStatus;
    
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
                mTurret.enableTurret( mTurret.mLimelight.getOutput() );
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
