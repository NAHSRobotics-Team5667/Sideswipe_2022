// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  TalonFX m_fL, m_fR, m_rL, m_rR;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
