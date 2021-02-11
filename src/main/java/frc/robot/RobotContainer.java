// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.RunHopperMotor;
import frc.robot.commands.SetIntakeMotor;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.commands.SwitchGears;
import frc.robot.commands.TeleOp;
import frc.robot.commands.ToggleIntakePneumatics;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShifterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  // Joysticks
  public XboxController driver = new XboxController(Constants.CONTROLLER.DRIVER_XBOX);
  public XboxController operator = new XboxController(Constants.CONTROLLER.OPERATOR_XBOX);

  private final TeleOp teleOp = new TeleOp(drivetrainSubsystem, driver);
  private final SetIntakeMotor setIntakeMotor = new SetIntakeMotor(intakePneumaticsSubsystem, driver);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drivetrainSubsystem.setDefaultCommand(teleOp);
    intakePneumaticsSubsystem.setDefaultCommand(setIntakeMotor);
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
    JoystickButton operator_xButton = new JoystickButton(operator, 3);

    driver_aButton.whenPressed(new SwitchGears(shifterSubsystem));
    driver_bButton.whenPressed(new ToggleIntakePneumatics(intakePneumaticsSubsystem));
    driver_yButton.whenPressed(new SwitchDriveMode(drivetrainSubsystem));

    operator_aButton.whileHeld(new RunHopperMotor(hopperSubsystem));
    operator_xButton.whenPressed(new InstantCommand( () -> { hopperSubsystem.mBackwards = !hopperSubsystem.mBackwards; }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An TeleOp will run in autonomous
    return teleOp;
  }
}
