// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.commands.MoveTurretManual;
import frc.robot.commands.RunFlywheel;
import frc.robot.commands.RunHopperMotor;
import frc.robot.commands.RunTransferWheel;
import frc.robot.commands.SetIntakeMotor;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.commands.SwitchGears;
import frc.robot.commands.TeleOp;
import frc.robot.commands.ToggleIntakePneumatics;
import frc.robot.commands.TurretAlignment;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import frc.robot.subsystems.TransferWheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static DrivetrainSubsystem drivetrainSubsystem = DrivetrainSubsystem.create();
  public static ShifterSubsystem shifterSubsystem = ShifterSubsystem.create();
  public static IntakeSubsystem intakePneumaticsSubsystem = IntakeSubsystem.create();
  public static HopperSubsystem hopperSubsystem = HopperSubsystem.create();
  public static IndexerSubsystem indexerSubsystem = IndexerSubsystem.create();
  public static TransferWheelSubsystem transferWheelSubsystem = TransferWheelSubsystem.create();
  public static FlywheelSubsystem flywheelSubsystem = FlywheelSubsystem.create();
  public static TurretSubsystem turretSubsystem = TurretSubsystem.create();

  // Joysticks
  public XboxController driver = new XboxController(Constants.CONTROLLER.DRIVER_XBOX);
  public XboxController operator = new XboxController(Constants.CONTROLLER.OPERATOR_XBOX);

  private final TeleOp teleOp = new TeleOp(drivetrainSubsystem, driver);
  private final SetIntakeMotor setIntakeMotor = new SetIntakeMotor(intakePneumaticsSubsystem, driver);
  private final RunFlywheel runFlywheel = new RunFlywheel(flywheelSubsystem, operator);
  private final MoveTurretManual moveTurretManual = new MoveTurretManual(turretSubsystem, operator);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drivetrainSubsystem.setDefaultCommand(teleOp);
    intakePneumaticsSubsystem.setDefaultCommand(setIntakeMotor);
    flywheelSubsystem.setDefaultCommand(runFlywheel);
    turretSubsystem.setDefaultCommand(moveTurretManual);
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
    JoystickButton driver_yButton = new JoystickButton(driver, 4);

    JoystickButton operator_aButton = new JoystickButton(operator, 1);
    JoystickButton operator_bButton = new JoystickButton(operator, 2);
    JoystickButton operator_xButton = new JoystickButton(operator, 3);
    JoystickButton operator_leftBumper = new JoystickButton(operator, 5);

    driver_aButton.whenPressed(new SwitchGears(shifterSubsystem));
    driver_bButton.whenPressed(new ToggleIntakePneumatics(intakePneumaticsSubsystem));
    driver_yButton.whenPressed(new SwitchDriveMode(drivetrainSubsystem));

    operator_aButton.whileHeld(new RunHopperMotor(hopperSubsystem, indexerSubsystem));
    operator_bButton.whileHeld(new RunTransferWheel(transferWheelSubsystem));
    operator_xButton.whenPressed(new InstantCommand( () -> { hopperSubsystem.mBackwards = !hopperSubsystem.mBackwards;
      indexerSubsystem.mBackwards = !indexerSubsystem.mBackwards;
      transferWheelSubsystem.mBackwards = !transferWheelSubsystem.mBackwards; } ));
    operator_leftBumper.whileHeld(new TurretAlignment(turretSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    String trajectoryJSON = "paths/Test.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    drivetrainSubsystem.zeroHeading();
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
    
    return ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));
  }
}
