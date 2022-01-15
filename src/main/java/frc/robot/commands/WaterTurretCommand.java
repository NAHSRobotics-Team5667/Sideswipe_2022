// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PneumaticWaterTurret;

public class WaterTurretCommand extends CommandBase {
  private PneumaticWaterTurret m_pneumaticWaterTurret;
  private double targetDistance = 0;
  /** 
   * Call this command if you do not have a specified distance for the wheel to go.
   */
  public WaterTurretCommand() {
    m_pneumaticWaterTurret = new PneumaticWaterTurret();
    addRequirements(m_pneumaticWaterTurret);
  }

  /**
   * Call this command if you want the wheel to go a specified distance.
   * 
   * @param distance target distance.
   */
  public WaterTurretCommand(double distance) {
    m_pneumaticWaterTurret = new PneumaticWaterTurret();
    targetDistance = distance;
    addRequirements(m_pneumaticWaterTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pneumaticWaterTurret.setPercentOutput(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (targetDistance == 0) { // if we do not get a target distance
      if (RobotContainer.getController().getRightTrigger() > 0) {
        m_pneumaticWaterTurret.setPercentOutput(RobotContainer.getController().getRightTrigger());
      } else if (RobotContainer.getController().getLeftTrigger() > 0) {
        m_pneumaticWaterTurret.setPercentOutput(-RobotContainer.getController().getLeftTrigger());
      } else {
        m_pneumaticWaterTurret.setPercentOutput(0);
      }
    } else { // we DO get a target distance
      if (m_pneumaticWaterTurret.getWheelDistance() < targetDistance) {
        m_pneumaticWaterTurret.setPercentOutput(0.1);
      } else if (m_pneumaticWaterTurret.getWheelDistance() > targetDistance) {
        m_pneumaticWaterTurret.setPercentOutput(-0.1);
      } else {
        m_pneumaticWaterTurret.setPercentOutput(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pneumaticWaterTurret.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
