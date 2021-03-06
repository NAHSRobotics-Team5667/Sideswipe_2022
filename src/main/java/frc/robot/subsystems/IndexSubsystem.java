// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {
  WPI_TalonFX m_IndexMotor;

  /** Creates a new IndexSubsystem. */
  public IndexSubsystem() {
    m_IndexMotor = new WPI_TalonFX(Constants.IndexConstants.kIndexId);

    m_IndexMotor.setNeutralMode(NeutralMode.Brake);
  }
  
  public void set(double percentOutput){
    m_IndexMotor.set(ControlMode.PercentOutput, percentOutput);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
