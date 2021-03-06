// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DrivetrainCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.WaterTurretCommand;
import frc.robot.commands.shootercommandments;
import frc.robot.commands.auto.TrajectoryFollower;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticWaterTurret;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.Controller;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static Controller controller = new Controller(0);

  private Drivetrain m_drive;
  private PneumaticWaterTurret m_waterTurret;
  private IntakeSubsystem intake;
  private IndexSubsystem index;
  private TurretSubsystem turret;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drive = new Drivetrain();
    // m_waterTurret = new PneumaticWaterTurret();
    intake = new IntakeSubsystem();
    index = new IndexSubsystem();
    turret = new TurretSubsystem();
    configureButtonBindings();
    
    m_drive.setDefaultCommand(new DrivetrainCommand(m_drive));
    intake.setDefaultCommand(new IntakeCommand(intake));
    index.setDefaultCommand(new IndexCommand(index));
    turret.setDefaultCommand(new shootercommandments(turret));
    // m_waterTurret.setDefaultCommand(new WaterTurretCommand());
    // m_waterTurret.setDefaultCommand(new WaterTurretCommand(1));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  public static Controller getController() {
    return controller;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new DrivetrainCommand(m_drive);
    m_drive.resetEncoders();
    m_drive.resetAngle();
    return TrajectoryFollower.getRamseteCommand(Constants.AutoConstants.SPLINE, m_drive);
  }
}
