package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.lib.controllers.Vision;
import frc.robot.lib.controllers.Vision.State;
import frc.robot.lib.drivers.SparkMax;

public class TurretSubsystem extends SubsystemBase {
    private final CANSparkMax mTurret;
    public final Vision mLimelight;
    private final CANSparkMax mHood;

    public final CANEncoder mEncoder;

    public TurretSubsystem(CANSparkMax turret, Vision limelight, CANSparkMax hood) {
        mTurret = turret;
        mLimelight = limelight;
        mHood = hood;
        mEncoder = mHood.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
        mEncoder.setInverted(true);
        mEncoder.setPositionConversionFactor(33.6688282);
        mEncoder.setPosition(0);
        SmartDashboard.putNumber("Hood Target", mEncoder.getPosition());
        SmartDashboard.putData("Zero Hood", new InstantCommand(() -> { mEncoder.setPosition(0); }));
    }

    @Override
    public void periodic() {
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
        SmartDashboard.putNumber("Hood Encoder", mEncoder.getPosition());
    }

    public void disableHood() {
        mHood.set(0.0);
        SmartDashboard.putNumber("Hood Encoder", mEncoder.getPosition());
    }

    public static TurretSubsystem create() {
        CANSparkMax turret = SparkMax.CreateSparkMax(new CANSparkMax(Constants.TURRET.MASTER_ID, MotorType.kBrushless));
        Vision limelight = Vision.create();
        CANSparkMax hood = SparkMax.CreateSparkMax(new CANSparkMax(Constants.HOOD.MASTER_ID, MotorType.kBrushless));
        return new TurretSubsystem(turret, limelight, hood);
    }
}