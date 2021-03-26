/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForDistance extends CommandBase {

  private DrivetrainSubsystem mDrivetrain;
  private double mSetpoint;
  private double mSpeed;
  private double mTurn;
  /**
   * Creates a new DriveForDistance.
   */
  public DriveForDistance( DrivetrainSubsystem drivetrain, double setpoint, double turn, double speed ) {
    mDrivetrain = drivetrain;
    mSetpoint = setpoint;
    mTurn = turn;
    mSpeed = speed;
    addRequirements(mDrivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize () {
    mDrivetrain.resetEncoders();
    // System.out.println("ini drivefordistance");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute () {
    mDrivetrain.update( mTurn, mSpeed, 0);
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
