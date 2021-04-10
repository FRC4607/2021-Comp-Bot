/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShifterSubsystem;

public class DriveInCircle extends CommandBase {

  private DrivetrainSubsystem mDrivetrain;
  private ShifterSubsystem mShifter;
  private boolean mIsHighGear;
  private double mSetpoint;
  private double mDegrees;
  private double mSpeed;
  private boolean mClockwise;

  // Velocity of the robot in a straight line at full power.
  private double speedConstant = 4.788;
  private double startDegrees;

  /**
   * Creates a new DriveForDistance.
   */
  public DriveInCircle(DrivetrainSubsystem drivetrain, ShifterSubsystem shifter, double r, boolean cw, double degrees, double speed, boolean highGear) {
    mDrivetrain = drivetrain;
    mShifter = shifter;
    mIsHighGear = highGear;
    mSetpoint = r;
    mDegrees = degrees;
    mClockwise = cw;
    mSpeed = speed;

    startDegrees = mDrivetrain.getHeading();
    addRequirements(mDrivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mIsHighGear) {
      mShifter.highGear();
    }
    else {
      mShifter.lowGear();
    }
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
    // Positive values turn left, negative ones turn right.
    double turn = (mSetpoint - 0.3095625) / (mSetpoint + 0.3095625);

    // https://www.reddit.com/r/arduino/comments/2ypf1y/make_a_4wheel_robot_perform_a_circle_of_a/cpc5ejc?utm_source=share&utm_medium=web2x&context=3
    double target = Math.toDegrees(mSpeed * speedConstant / mSetpoint);

    if (mClockwise) {
      turn *= -1;
      target *= -1;
    }

    double turnCorrection = (target - mDrivetrain.getRate()) * kP;
    mDrivetrain.update( turn - turnCorrection, mSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end( boolean interrupted ) {
    // System.out.println("end drivefordistance");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("getLeftEoncdoer: " + mDrivetrain.getLeftEncoder());
    // System.out.println("mSetPoint: " + mSetpoint);
    if (mClockwise) {
      return mDrivetrain.getHeading() > startDegrees - mDegrees;
    }
    else {
      return mDrivetrain.getHeading() > startDegrees + mDegrees;
    }
  }
}