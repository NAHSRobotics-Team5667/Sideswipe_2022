// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX m_leftMaster, m_rightMaster, m_leftSlave, m_rightSlave;
  private AHRS m_gyro;

  private DifferentialDrive m_drive;
  private SpeedControllerGroup m_leftDrive, m_rightDrive;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_leftMaster = new WPI_TalonFX(DriveConstants.kLeftMaster);
    m_rightMaster = new WPI_TalonFX(DriveConstants.kRightMaster);
    m_leftSlave = new WPI_TalonFX(DriveConstants.kLeftSlave);
    m_rightSlave = new WPI_TalonFX(DriveConstants.kRightSlave);

    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    m_rightMaster.setInverted(InvertType.InvertMotorOutput);
    m_rightSlave.setInverted(InvertType.FollowMaster);

    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_leftSlave.setNeutralMode(NeutralMode.Brake);
    m_rightSlave.setNeutralMode(NeutralMode.Brake);

    m_gyro = new AHRS();
  }

  public void setSpeed(double leftOutput, double rightOutput) {
    m_leftMaster.setVoltage(leftOutput);
    m_rightMaster.setVoltage(rightOutput);
  }

  public void stopMotors() {
    m_leftMaster.set(0);
    m_rightMaster.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Master", m_leftMaster.getBusVoltage());
  }
}
