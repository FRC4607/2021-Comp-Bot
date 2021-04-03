// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
 import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.drivers.SparkMax;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax mLeftMaster;
  private final CANSparkMax mLeftFollower;
  private final CANSparkMax mRightMaster;
  private final CANSparkMax mRightFollower;
  public final DifferentialDrive mDrive;

  /*
   * private final SpeedControllerGroup mLeft; private final SpeedControllerGroup
   * mRight;
   */

  private final CANEncoder mLeftEncoder;
  private final CANEncoder mRightEncoder;

  // private final PigeonIMU mGyro;
  private final Gyro mGyro;
  private final DifferentialDriveOdometry mOdometry;

  private boolean mDriveStraight;
  private static final AlternateEncoderType kAltEncType = AlternateEncoderType.kQuadrature;
  private static final int kCPR = 8192;

  private double scanCount = 0;
  private boolean captureHeading = false;
  private double holdHeading = 0;

  public DrivetrainSubsystem(CANSparkMax leftMaster, CANSparkMax leftFollower, CANSparkMax rightMaster,
      CANSparkMax rightFollower) {
    mLeftMaster = leftMaster;
    mLeftFollower = leftFollower;
    mRightMaster = rightMaster;
    mRightFollower = rightFollower;

    /*
     * mLeft = new SpeedControllerGroup(leftMaster, leftFollower); mRight = new
     * SpeedControllerGroup(rightMaster, rightFollower);
     */

    mDrive = new DifferentialDrive(mLeftMaster, mRightMaster);

    mLeftEncoder = mLeftMaster.getAlternateEncoder(kAltEncType, kCPR);
    mLeftEncoder.setInverted(true);
    mRightEncoder = mRightMaster.getAlternateEncoder(kAltEncType, kCPR);

    // mLeftEncoder = mLeftMaster.getEncoder();
    // mRightEncoder = mRightMaster.getEncoder();

    mGyro = new ADXRS450_Gyro();
    // mGyro = new PigeonIMU(new
    // WPI_TalonSRX(Constants.DRIVETRAIN.DRIVETRAIN_PIGEON));

    // Step 1: Multiply by gear ratio of output to get true RPM
    // Step 2: Multiply by wheel circumfrence in meters to get meters per minute
    // Step 3: Divide by 60 to get meters per second
    double velocityFactor = 0.47877872 / 60;
    mLeftEncoder.setVelocityConversionFactor(velocityFactor);
    mRightEncoder.setVelocityConversionFactor(velocityFactor);

    // Multiply by gear ratio wheel circumfrence to get total meters traveled.
    double positionFactor = 0.47877872;
    mLeftEncoder.setPositionConversionFactor(positionFactor);
    mRightEncoder.setPositionConversionFactor(positionFactor);

    mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d());

    mDriveStraight = false;

    mDrive.setDeadband(Constants.DRIVETRAIN.DEADBAND);

    mLeftMaster.setSmartCurrentLimit(Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT,
        Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT);
    mRightMaster.setSmartCurrentLimit(Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT,
        Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT);
    mLeftFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT,
        Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT);
    mRightFollower.setSmartCurrentLimit(Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT,
        Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT);
  }

  @Override
  public void periodic() {
    /* grab some input data from Pigeon and gamepad */
    SmartDashboard.putNumber("Left Dist", getLeftEncoderPos());
    SmartDashboard.putNumber("Left Dist Internal", mLeftMaster.getEncoder().getPosition());
    SmartDashboard.putNumber("Right Dist", getRightEncoderPos());
    SmartDashboard.putNumber("Right Dist Internal", mRightMaster.getEncoder().getPosition());
    SmartDashboard.putNumber("Left Vel", mLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Vel", mRightEncoder.getVelocity());
    SmartDashboard.putNumber("Gyro", mGyro.getRotation2d().getDegrees());

    Pose2d pose = getPose();
    SmartDashboard.putString("Pose", pose.toString());

    //SmartDashboard.putNumber("m_left_alternateEncoder", m_left_alternateEncoder.getPosition());
    //SmartDashboard.putNumber("m_right_alternateEncoder", m_right_alternateEncoder.getPosition());

    // This method will be called once per scheduler run
    mOdometry.update(mGyro.getRotation2d(), getLeftEncoderPos(), getRightEncoderPos());
  }

  public void setReverse() {
    mLeftMaster.setInverted(true);
    mRightMaster.setInverted(true);
  }

  public void setForward() {
    mLeftMaster.setInverted(false);
    mRightMaster.setInverted(false);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    mOdometry.resetPosition(pose, mGyro.getRotation2d());
  }

  public void resetEncoders() {
    mLeftEncoder.setPosition(0);
    mLeftMaster.getEncoder().setPosition(0);
    mRightEncoder.setPosition(0);
    mRightMaster.getEncoder().setPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(mLeftEncoder.getVelocity(), mRightEncoder.getVelocity());
  }

  public Rotation2d getFakeRotation2d() {
    return mGyro.getRotation2d().plus(new Rotation2d(Math.PI / 2));
  }

  public double getHeading() {
    return mGyro.getRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    mLeftMaster.setVoltage(leftVolts);
    mRightMaster.setVoltage(rightVolts);
    mDrive.feed();
  }

  public void feed() {
    mDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderPos() + getRightEncoderPos()) / 2.0;
  }

  public CANEncoder getLeftEncoder() {
    return mLeftEncoder;
  }

  public CANEncoder getRightEncoder() {
    return mRightEncoder;
  }
  public double getLeftEncoderPos() {
    return mLeftEncoder.getPosition();
    //return mLeftMaster.getEncoder().getPosition() * motor_kCPPR;
  }

  public double getRightEncoderPos() {
    //return mRightEncoder.getPosition();
    return -mRightMaster.getEncoder().getPosition()*.02;
  }

  public void setMaxOutput(double maxOutput) {
    mDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    mGyro.reset();
  }

  public void disableAssist() {
    mDriveStraight = false;
  }
  public void enableAssist() {
    mDriveStraight = true;
  }

  private void switchMode() {
    mDriveStraight = !mDriveStraight;
  }

  public void update(double stick1, double stick2, double stick3) {
    if (Math.abs(stick1) < 0.1) {
      if (scanCount >= 5) {
        if (!captureHeading) {
          holdHeading = getHeading();
          captureHeading = true;
        }
        if (mDriveStraight) {
          holdHeading = 0;
        }
        stick1 = (holdHeading - getHeading()) * -0.05;
      }
      else {
        scanCount += 1;
      }
    }
    else {
      scanCount = 0;
      captureHeading = false;
    }
    SmartDashboard.putNumber("stick1", stick1);
    SmartDashboard.putNumber("stick2", stick2);
    mDrive.arcadeDrive(stick2, stick1);
  }

  public static DrivetrainSubsystem create() {
    CANSparkMax leftMaster = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.LEFT_MASTER_ID, MotorType.kBrushless));
    CANSparkMax leftFollower = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.LEFT_FOLLOWER_ID, MotorType.kBrushless), leftMaster);
    CANSparkMax rightMaster = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.RIGHT_MASTER_ID, MotorType.kBrushless));
    CANSparkMax rightFollower = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.RIGHT_FOLLOWER_ID, MotorType.kBrushless), rightMaster);
    return new DrivetrainSubsystem(leftMaster, leftFollower, rightMaster, rightFollower);
  }
}
