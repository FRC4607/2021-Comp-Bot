package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.lib.controllers.Vision;
import frc.robot.lib.controllers.Vision.State;
import frc.robot.lib.drivers.SparkMax;

public class TurretSubsystem extends SubsystemBase {
    private final CANSparkMax mTurret;
    public final Vision mLimelight;
    private final CANSparkMax mHood;

    public TurretSubsystem(CANSparkMax turret, Vision limelight, CANSparkMax hood) {
        mTurret = turret;
        mLimelight = limelight;
        mHood = hood;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void disableVision() {
        mLimelight.setDriverMode();
        mLimelight.setLimelightLEDOff();
        mLimelight.setState(State.kPnP);
        mLimelight.mVisionThread.stop();
    }

    public void enableVision() {
        mLimelight.setVisionMode();
        mLimelight.setLimelightLEDOn();
        mLimelight.setState(State.kTurn);
        mLimelight.mVisionThread.startPeriodic(0.01);
    }

    public void enableTurret(double speed) {
        mTurret.set(speed);
    }

    public void disableTurret() {
        mTurret.set(0.0);
    }

    public void enableHood(double speed) {
        mHood.set(speed);
    }

    public void disableHood() {
        mHood.set(0.0);
    }

    public static TurretSubsystem create() {
        CANSparkMax turret = SparkMax.CreateSparkMax(new CANSparkMax(Constants.TURRET.MASTER_ID, MotorType.kBrushless));
        Vision limelight = Vision.create();
        CANSparkMax hood = SparkMax.CreateSparkMax(new CANSparkMax(Constants.HOOD.MASTER_ID, MotorType.kBrushless));
        return new TurretSubsystem(turret, limelight, hood);
    }
}