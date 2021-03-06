// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private WPI_TalonFX m_intakeMotor;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor = new WPI_TalonFX(Constants.IntakeConstants.kIntakeId);

    m_intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void set(double percentOutput) {
    m_intakeMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
