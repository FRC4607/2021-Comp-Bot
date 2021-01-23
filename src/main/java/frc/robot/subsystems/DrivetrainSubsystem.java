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
  private final CANSparkMax leftSide = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.LEFT_FOLLOWER_ID, MotorType.kBrushless), SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.LEFT_MASTER_ID, MotorType.kBrushless)));
  private final CANSparkMax rightSide = SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.RIGHT_FOLLOWER_ID, MotorType.kBrushless), SparkMax.CreateSparkMax(new CANSparkMax(Constants.DRIVETRAIN.RIGHT_MASTER_ID, MotorType.kBrushless)));
  public final DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);
  public DrivetrainSubsystem() {
    leftSide.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
    rightSide.setSmartCurrentLimit( Constants.CURRENT_LIMIT.SPARK_ZERO_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_FREE_RPM_LIMIT, Constants.CURRENT_LIMIT.SPARK_RPM_LIMIT );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
