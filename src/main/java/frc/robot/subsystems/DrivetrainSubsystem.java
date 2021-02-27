// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
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

  private final CANEncoder mLeftEncoder;
  private final CANEncoder mRightEncoder;

  //private final PigeonIMU mGyro;
  private final PigeonIMU _pidgey;      // Pigeon IMU used to enforce straight drive
  private final DifferentialDriveOdometry mOdometry;
  private double currentAngle;

  private boolean mUseTank;

  public DrivetrainSubsystem(CANSparkMax leftMaster, CANSparkMax leftFollower, CANSparkMax rightMaster, CANSparkMax rightFollower) {
    mLeftMaster = leftMaster;
    mLeftFollower = leftFollower;
    mRightMaster = rightMaster;
    mRightFollower= rightFollower;

    //mLeftMaster.setInverted(true);
    //mRightMaster.setInverted(true);

    mDrive = new DifferentialDrive(mLeftMaster, mRightMaster);
    
    mLeftEncoder = mLeftMaster.getEncoder();
    mRightEncoder = mRightMaster.getEncoder();

    boolean isPigeonOnCAN = false;
        if(isPigeonOnCAN){
            /* Pigeon is on CANBus (powered from ~12V, and has a device ID of zero) */
            _pidgey = new PigeonIMU(3);             // Change ID accordingly 
        }else{
            /* Pigeon is ribbon cabled to the specified CANTalon. */
            _pidgey = new PigeonIMU(new WPI_TalonSRX(Constants.DRIVETRAIN.DRIVETRAIN_PIGEON));   // Change Talon Accordingly
        }
        
    //mGyro = new PigeonIMU(new WPI_TalonSRX(Constants.DRIVETRAIN.DRIVETRAIN_PIGEON));

    // Step 1: Multiply by gear ratio of output to get true RPM
    // Step 2: Multiply by wheel circumfrence in meters to get meters per minute
    // Step 3: Divide by 60 to get meters per second
    double velocityFactor = 0.04291187739 * 0.47877872 / 60;
    mLeftEncoder.setVelocityConversionFactor(velocityFactor);
    mRightEncoder.setVelocityConversionFactor(velocityFactor);

    // Multiply by gear ratio wheel circumfrence to get total meters traveled.
    double positionFactor = 0.04291187739 * 0.47877872;
    mLeftEncoder.setPositionConversionFactor(positionFactor);
    mRightEncoder.setPositionConversionFactor(positionFactor);

    mOdometry = new DifferentialDriveOdometry(pigeonGetRotation2d());

    mUseTank = false;

    mDrive.setDeadband(Constants.DRIVETRAIN.DEADBAND);
    
    mLeftMaster.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
    mRightMaster.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
    mLeftFollower.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
    mRightFollower.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
  }

  private Rotation2d pigeonGetRotation2d() {
    return new Rotation2d( currentAngle * Math.PI/180);
  }

  @Override
  public void periodic() {

    /* get Pigeon status information from Pigeon API */
		PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
		PigeonIMU.FusionStatus fusionStatus = new PigeonIMU.FusionStatus();
		double [] xyz_dps = new double [3];
		/* grab some input data from Pigeon and gamepad*/
		_pidgey.getGeneralStatus(genStatus);
		_pidgey.getRawGyro(xyz_dps);
		_pidgey.getFusedHeading(fusionStatus);
    currentAngle = fusionStatus.heading;
		boolean angleIsGood = (_pidgey.getState() == PigeonIMU.PigeonState.Ready) ? true : false;
    double currentAngularRate = xyz_dps[2];
    
    SmartDashboard.putNumber("Gyro", currentAngle);
    SmartDashboard.putNumber("pigeonGetRotation2d", pigeonGetRotation2d().getRadians());
    SmartDashboard.putNumber("Left Dist", getLeftEncoderPos());
    SmartDashboard.putNumber("Right Dist", getRightEncoderPos());
    SmartDashboard.putNumber("Left Vel", mLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Vel", -mRightEncoder.getVelocity());
    // This method will be called once per scheduler run
    mOdometry.update(pigeonGetRotation2d(), getLeftEncoderPos(), getRightEncoderPos());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    mOdometry.resetPosition(pose, pigeonGetRotation2d());
  }

  public void resetEncoders() {
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(mLeftEncoder.getVelocity(), mRightEncoder.getVelocity());
  }

  public double getHeading() {
    return pigeonGetRotation2d().getDegrees();
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    mLeftMaster.setVoltage(leftVolts);
    mRightMaster.setVoltage(rightVolts);
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
  }

  public double getRightEncoderPos() {
    return mRightEncoder.getPosition();
  }

  public void setMaxOutput(double maxOutput) {
    mDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    _pidgey.setFusedHeading(0);
  }

  public void switchMode() {
    mUseTank = !mUseTank;
  }

  public void update(double stick1, double stick2, double stick3) {
    if (!mUseTank) {
      SmartDashboard.putNumber("stick1", stick1);
      SmartDashboard.putNumber("stick2", stick2);
      mDrive.arcadeDrive(stick1, stick2);
    }
    else {
      mDrive.tankDrive(stick1, stick3);
    }
  }

  public static DrivetrainSubsystem create() {
    CANSparkMax leftMaster = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.LEFT_MASTER_ID, MotorType.kBrushless));
    CANSparkMax leftFollower = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.LEFT_FOLLOWER_ID, MotorType.kBrushless), leftMaster);
    CANSparkMax rightMaster = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.RIGHT_MASTER_ID, MotorType.kBrushless));
    CANSparkMax rightFollower = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.RIGHT_FOLLOWER_ID, MotorType.kBrushless), rightMaster);
    return new DrivetrainSubsystem(leftMaster, leftFollower, rightMaster, rightFollower);
  }
}
