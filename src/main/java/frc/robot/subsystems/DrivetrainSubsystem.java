// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  public DrivetrainSubsystem(CANSparkMax leftMaster, CANSparkMax leftFollower, CANSparkMax rightMaster, CANSparkMax rightFollower, DifferentialDrive drive) {
    mLeftMaster = leftMaster;
    mLeftFollower = leftFollower;
    mRightMaster = rightMaster;
    mRightFollower= rightFollower;
    mDrive = drive;

    mLeftMaster.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
    mRightMaster.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
    mLeftFollower.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
    mRightFollower.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static DrivetrainSubsystem create() {
    CANSparkMax leftMaster = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.LEFT_MASTER_ID, MotorType.kBrushless));
    CANSparkMax leftFollower = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.LEFT_FOLLOWER_ID, MotorType.kBrushless), leftMaster);
    CANSparkMax rightMaster = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.RIGHT_MASTER_ID, MotorType.kBrushless));
    CANSparkMax rightFollower = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.RIGHT_FOLLOWER_ID, MotorType.kBrushless), rightMaster);
    DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);
    return new DrivetrainSubsystem(leftMaster, leftFollower, rightMaster, rightFollower, drive);
  }
}
