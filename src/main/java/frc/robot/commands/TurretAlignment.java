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

    // Trim value needed for hood
    private double magicValue = 22.15;

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
                mTarget = 90 - mTurret.mLimelight.getVerticalDegrees() - magicValue;
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

    private double getTargetAngle(double vertDegrees) {
        // Quadratic of the form ax^2+bx+c calulated from data.

        //Limelight is angled 15 degrees up, and the distance between limelight and box it draws is 1.72085
        double x = 1.72085 / Math.tan(15 + vertDegrees);

        double a = -0.75910773186713193138660535049873;
        double b = 11.720642357721813698907471276834;
        double c = 24.760330578512396694214876033058;

        double y = a * Math.pow(x, 2) + b * x + c;
        //Clamp result between 0 and 90
        return Math.max(Math.min(y, 90), 0);
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
