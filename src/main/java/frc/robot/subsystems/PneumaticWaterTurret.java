// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticWaterTurret extends SubsystemBase {
  private WPI_TalonFX m_waterWheelMotor;
  private double wheelDistance = 0;

  /** Creates a new PneumaticWaterTurret. */
  public PneumaticWaterTurret() {
    m_waterWheelMotor = new WPI_TalonFX(Constants.kWaterWheelId);
    m_waterWheelMotor.setInverted(true);

    m_waterWheelMotor.setNeutralMode(NeutralMode.Brake);
  }

  public double getWheelDistance() {
    return wheelDistance;
  }

  public void updateWheelDistance() {
    wheelDistance = ((m_waterWheelMotor.getSelectedSensorPosition() / 2048) / 10) * 8 * Math.PI;
  }

  public void setPercentOutput(double percentOutput) {
    m_waterWheelMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void periodic() {
    updateWheelDistance();
  }
}
