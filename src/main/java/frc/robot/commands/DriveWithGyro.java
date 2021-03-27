/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveWithGyro extends CommandBase {

  private DrivetrainSubsystem mDrivetrain;
  private double mSetpoint;
  private double mSpeed;
  private double mHeading;

  /**
   * Creates a new DriveForDistance.
   */
  public DriveWithGyro(DrivetrainSubsystem drivetrain, double setpoint, double heading, double speed) {
    mDrivetrain = drivetrain;
    mSetpoint = setpoint;
    mHeading = heading;
    mSpeed = speed;
    addRequirements(mDrivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.resetEncoders();
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    // System.out.println("ini drivefordistance");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute () {
    double kP = -0.1;
    double turn = (mHeading - mDrivetrain.getHeading()) * kP;
    mDrivetrain.update( turn, mSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end( boolean interrupted ) {
    // System.out.println("end drivefordistance");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished () {
    // System.out.println("getLeftEoncdoer: " + mDrivetrain.getLeftEncoder());
    // System.out.println("mSetPoint: " + mSetpoint);
    return Math.abs( mDrivetrain.getLeftEncoderPos() ) > Math.abs( mSetpoint );
  }
}