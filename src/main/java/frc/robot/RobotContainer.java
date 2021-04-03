// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auton_SixBall;
import frc.robot.commands.Auton_ThreeBall;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.MoveTurretManual;
import frc.robot.commands.RetractClimber;
import frc.robot.commands.RunFlywheel;
import frc.robot.commands.RunHopperMotor;
import frc.robot.commands.RunTransferWheel;
import frc.robot.commands.SetIntakeMotor;
import frc.robot.commands.SwitchClimberGear;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.commands.SwitchGears;
import frc.robot.commands.TeleOp;
import frc.robot.commands.ToggleIntakePneumatics;
import frc.robot.commands.TurretAlignment;
import frc.robot.commands.ZeroEncoders;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.ZeroHood;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.AndJoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..
  public static DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.create();
  public static ShifterSubsystem shifterSubsystem = ShifterSubsystem.create();
  public static IntakeSubsystem intakePneumaticsSubsystem = IntakeSubsystem.create();
  public static HopperSubsystem hopperSubsystem = HopperSubsystem.create();
  public static IndexerSubsystem indexerSubsystem = IndexerSubsystem.create();
  public static TransferWheelSubsystem transferWheelSubsystem = TransferWheelSubsystem.create();
  public static FlywheelSubsystem flywheelSubsystem = FlywheelSubsystem.create();
  public static TurretSubsystem turretSubsystem = TurretSubsystem.create();
  public static ClimberSubsystem climberSubsystem = ClimberSubsystem.create();

  // Joysticks
  public XboxController driver = new XboxController(Constants.CONTROLLER.DRIVER_XBOX);
  public XboxController operator = new XboxController(Constants.CONTROLLER.OPERATOR_XBOX);

  private final TeleOp teleOp = new TeleOp(drivetrainSubsystem, driver);
  private final SetIntakeMotor setIntakeMotor = new SetIntakeMotor(intakePneumaticsSubsystem, driver);
  private final RunFlywheel runFlywheel = new RunFlywheel(flywheelSubsystem, operator);
  private final MoveTurretManual moveTurretManual = new MoveTurretManual(turretSubsystem, operator);

  SendableChooser<Command> mChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //drivetrainSubsystem.setForward();
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Turret Trim", 0);
    turretSubsystem.mEncoder.setPosition(0);
    drivetrainSubsystem.setDefaultCommand(teleOp);
    intakePneumaticsSubsystem.setDefaultCommand(setIntakeMotor);
    flywheelSubsystem.setDefaultCommand(runFlywheel);
    turretSubsystem.setDefaultCommand(moveTurretManual);
    climberSubsystem.set(0);
    
    
    mChooser.addOption("Three Balls", new Auton_ThreeBall(drivetrainSubsystem, shifterSubsystem, turretSubsystem, flywheelSubsystem, transferWheelSubsystem, hopperSubsystem, indexerSubsystem, intakePneumaticsSubsystem));
    mChooser.setDefaultOption("Six Balls", new Auton_SixBall(drivetrainSubsystem, shifterSubsystem, turretSubsystem, flywheelSubsystem, transferWheelSubsystem, hopperSubsystem, indexerSubsystem, intakePneumaticsSubsystem));
    SmartDashboard.putData("Auto Mode", mChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton driver_aButton = new JoystickButton(driver, 1);
    JoystickButton driver_bButton = new JoystickButton(driver, 2);
    //JoystickButton driver_xButton = new JoystickButton(driver, 3);
    JoystickButton driver_yButton = new JoystickButton(driver, 4);
    JoystickButton driver_leftButton = new JoystickButton(driver, 5);
    //JoystickButton driver_rightButton = new JoystickButton(driver, 6);
    AndJoystickButton driver_rightButton = new AndJoystickButton(driver, 6, driver, 8);

    JoystickButton operator_aButton = new JoystickButton(operator, 1);
    JoystickButton operator_bButton = new JoystickButton(operator, 2);
    JoystickButton operator_xButton = new JoystickButton(operator, 3);
    JoystickButton operator_leftBumper = new JoystickButton(operator, 5);
    AndJoystickButton operator_ResetHood = new AndJoystickButton(operator, 7, operator, 8);

    driver_aButton.whenPressed(new SwitchGears(shifterSubsystem));
    driver_bButton.whenPressed(new ToggleIntakePneumatics(intakePneumaticsSubsystem));
    //driver_xButton.whenPressed(new SwitchClimberGear(climberSubsystem));
    driver_yButton.whileHeld(new SwitchDriveMode(drivetrainSubsystem));
    driver_leftButton.whileHeld(new RetractClimber(climberSubsystem));
    driver_rightButton.whileHeld(new ExtendClimber(climberSubsystem));
  

    operator_aButton.whileHeld(new RunHopperMotor(hopperSubsystem, indexerSubsystem));
    operator_bButton.whileHeld(new RunTransferWheel(transferWheelSubsystem));
    operator_xButton.whenPressed(new InstantCommand( () -> { hopperSubsystem.mBackwards = !hopperSubsystem.mBackwards;
      indexerSubsystem.mBackwards = !indexerSubsystem.mBackwards;
      transferWheelSubsystem.mBackwards = !transferWheelSubsystem.mBackwards; } ));
    operator_leftBumper.whileHeld(new TurretAlignment(turretSubsystem));

    operator_ResetHood.whenPressed(new ZeroHood(turretSubsystem));

  

    SmartDashboard.putData("Zero Heading", new ZeroHeading(drivetrainSubsystem));
    SmartDashboard.putData("Zero Encoders", new ZeroEncoders(drivetrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //drivetrainSubsystem.setReverse();
    /*String trajectoryJSON = "paths/Test.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }*/

    /*var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.DRIVETRAIN.kS,
                                       Constants.DRIVETRAIN.kV,
                                       Constants.DRIVETRAIN.kA),
            Constants.DRIVETRAIN.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.DRIVETRAIN.kMaxSpeedMetersPerSecond,
        Constants.DRIVETRAIN.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DRIVETRAIN.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0))),
        // Pass through these two interior waypoints, making an 's' curve path

        List.of(
            new Translation2d(1.0, 1.0),
            new Translation2d(2.0, -1.0),
            new Translation2d(3.0, 1.0),
            new Translation2d(4.0, -1.0),
            new Translation2d(5.0, 1.0),
            new Translation2d(6.0, -1.0)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(7.0, 0.0, new Rotation2d(Math.toRadians(0))),
        // Pass config
        config
    );

    shifterSubsystem.lowGear();
    drivetrainSubsystem.resetOdometry(trajectory.getInitialPose());

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
        drivetrainSubsystem::getPose,
        new RamseteController(Constants.DRIVETRAIN.kRamseteB, Constants.DRIVETRAIN.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.DRIVETRAIN.kS,
                                   Constants.DRIVETRAIN.kV,
                                   Constants.DRIVETRAIN.kA),
        Constants.DRIVETRAIN.kDriveKinematics,
        drivetrainSubsystem::getWheelSpeeds,
        new PIDController(Constants.DRIVETRAIN.kP, 0, 0),
        new PIDController(Constants.DRIVETRAIN.kP, 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrainSubsystem::tankDriveVolts,
        drivetrainSubsystem
    );

    
    return ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));*/
    
    drivetrainSubsystem.zeroHeading();
    turretSubsystem.mEncoder.setPosition(0);
    return mChooser.getSelected();
  }

  public Command getTestCommand() {
    return new InstantCommand(() -> { Gyro cali = new ADXRS450_Gyro();
      cali.calibrate();
      try {
        cali.close();
      }
      catch (Exception e) {
          int death = 1 / 0;
      }
    });
  }
}
