// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain drivetrain = new Drivetrain(
      new SwerveModule(SwerveModuleConstants.VELOCITY_MOTOR_ID_FL,
          SwerveModuleConstants.ANGULAR_MOTOR_ID_FL,
          SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FL,
          SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FL,
          new Translation2d(SwerveModuleConstants.MODULE_LOCATION_X,
              SwerveModuleConstants.MODULE_LOCATION_Y)),
      new SwerveModule(
          SwerveModuleConstants.VELOCITY_MOTOR_ID_FR,
          SwerveModuleConstants.ANGULAR_MOTOR_ID_FR,
          SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FR,
          SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FR,
          new Translation2d(SwerveModuleConstants.MODULE_LOCATION_X,
              -SwerveModuleConstants.MODULE_LOCATION_Y)),
      new SwerveModule(
          SwerveModuleConstants.VELOCITY_MOTOR_ID_BL,
          SwerveModuleConstants.ANGULAR_MOTOR_ID_BL,
          SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BL,
          SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BL,
          new Translation2d(-SwerveModuleConstants.MODULE_LOCATION_X,
              SwerveModuleConstants.MODULE_LOCATION_Y)),
      new SwerveModule(
          SwerveModuleConstants.VELOCITY_MOTOR_ID_BR,
          SwerveModuleConstants.ANGULAR_MOTOR_ID_BR,
          SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BR,
          SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BR,
          new Translation2d(-SwerveModuleConstants.MODULE_LOCATION_X,
              -SwerveModuleConstants.MODULE_LOCATION_Y)));
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(new TeleopDriveCommand(drivetrain, driverController, false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(exampleSubsystem);
  }
}
