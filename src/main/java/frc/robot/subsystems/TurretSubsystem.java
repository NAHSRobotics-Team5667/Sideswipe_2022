// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
  private WPI_TalonFX m_turretmotor;
  private double angleOfTurret;
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    m_turretmotor = new WPI_TalonFX(Constants.ShooterConstants.kTurretId);
    m_turretmotor.setNeutralMode(NeutralMode.Brake);
  }
  public double turretAngle (){
    angleOfTurret = ((m_turretmotor.getSelectedSensorPosition() * Constants.ShooterConstants.TURRET_GEAR_RATIO*360)/2048);
    return angleOfTurret;
  }
  public void set(double percentOutput){
    m_turretmotor.set(ControlMode.PercentOutput, percentOutput);
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("PLEASEWORK", angleOfTurret);
    // This method will be called once per scheduler run

  }
}
