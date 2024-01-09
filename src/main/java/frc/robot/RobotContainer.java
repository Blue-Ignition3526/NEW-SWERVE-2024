// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;


/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public class RobotContainer {
  /**
   * Main driver controller
   */
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  /**
   * Front Left Swerve Module
   */
  SwerveModule m_frontLeft;

  /**
   * Front Right Swerve Module
   */
  SwerveModule m_frontRight;

  /**
   * Back Left Swerve Module
   */
  SwerveModule m_backLeft;

  /**
   * Back Right Swerve Module
   */
  SwerveModule m_backRight;

  /**
   * Swerve Drivetrain
   */
  SwerveDrive m_swerveDrive;

  /**
   * Autonomous Command Chooser
   */
  SendableChooser<Command> m_autonomousChooser;

  public RobotContainer() {
    // Create all swerve modules and initialize
    this.m_frontLeft = new SwerveModule((Constants.Swerve.Motors.kFrontLeftVars));
    this.m_frontRight = new SwerveModule((Constants.Swerve.Motors.kFrontRightVars));
    this.m_backLeft = new SwerveModule((Constants.Swerve.Motors.kBackLeftVars));
    this.m_backRight = new SwerveModule((Constants.Swerve.Motors.kBackRightVars));
    
    // Create the swerve drive and initialize
    this.m_swerveDrive = new SwerveDrive(this.m_frontLeft, this.m_frontRight, this.m_backLeft, this.m_backRight);

    // Register all commands needed for Autonomous
    NamedCommands.registerCommand("IntakeIn", new IntakeIn());
    NamedCommands.registerCommand("IntakeOut", new IntakeOut());

    // Create a sendable chooser for the autonomous routines
    SendableChooser<Command> m_autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous Command", m_autonomousChooser);

    // Configure the controller bindings
    configureBindings();
  }

  
  private void configureBindings() {
    // By default drive the swerve drive forever and update with the joystick values
    m_swerveDrive.setDefaultCommand(
      new DriveSwerve(
        m_swerveDrive,
        () -> m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> !m_driverController.rightBumper().getAsBoolean()
      )
    );

    // When the right trigger is pressed, drive the swerve drive forward at 10% speed
    m_driverController.rightTrigger(0.1).whileTrue(new DriveSwerve(
      m_swerveDrive,
      () -> m_driverController.getRightTriggerAxis(),
      () -> 0.0,
      () -> 0.0,
      () -> !m_driverController.rightBumper().getAsBoolean()
    ));
  }

  /**
   * Returns the autonomous command to run. This will return null if no autonomous
   * @return Command
   */
  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}
