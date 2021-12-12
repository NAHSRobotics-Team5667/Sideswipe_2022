// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private AHRS m_gyro;

  private DifferentialDriveOdometry m_odometry;
  private DifferentialDrive m_drive;

  private DifferentialDrivetrainSim m_driveSim;

  Supplier<Pose2d> m_robotPose;
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_frontLeft = new WPI_TalonFX(DriveConstants.kFrontLeftId);
    m_frontRight = new WPI_TalonFX(DriveConstants.kFrontRightId);
    m_backLeft = new WPI_TalonFX(DriveConstants.kBackLeftId);
    m_backRight = new WPI_TalonFX(DriveConstants.kBackRightId);

    // m_backLeft.follow(m_frontLeft);
    // m_backRight.follow(m_frontRight);

    SpeedControllerGroup left = new SpeedControllerGroup(m_frontLeft, m_backLeft);
    SpeedControllerGroup right = new SpeedControllerGroup(m_frontRight, m_backRight);

    m_frontLeft.setNeutralMode(NeutralMode.Brake);
    m_frontRight.setNeutralMode(NeutralMode.Brake);
    m_backLeft.setNeutralMode(NeutralMode.Brake);
    m_backRight.setNeutralMode(NeutralMode.Brake);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
    m_robotPose = () -> m_odometry.getPoseMeters();

    m_drive = new DifferentialDrive(left, right);

    m_gyro = new AHRS(SPI.Port.kMXP);
  }

  public Pose2d getPose() {
    return m_robotPose.get();
  }

  public void tankDriveVoltage(double leftVolts, double rightVolts) {
    m_frontLeft.setVoltage(leftVolts);
    m_frontRight.setVoltage(-rightVolts);
    m_drive.feed();
  }

  public void stopMotors() {
    m_frontLeft.set(0);
    m_frontRight.set(0);
    m_backLeft.set(0);
    m_backRight.set(0);
  }

  public void drive(double throttle, double angle) {
    m_drive.arcadeDrive(throttle, angle);
  }

  public double getAngle() {
    return -m_gyro.getAngle();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      (m_frontLeft.getSelectedSensorVelocity() + m_backLeft.getSelectedSensorVelocity()) / 2,
      (m_frontRight.getSelectedSensorVelocity() + m_backRight.getSelectedSensorVelocity()) / 2);
  }

  private double falconTicksToMeters(double frontTicks, double backTicks) {
    return (((frontTicks + backTicks) / 2) / 2048) * 0.4788;
  }

  @Override
  public void periodic() {
    m_odometry.update(
      Rotation2d.fromDegrees(getAngle()), 
      falconTicksToMeters(m_frontLeft.getSelectedSensorPosition(), m_backLeft.getSelectedSensorPosition()), 
      falconTicksToMeters(m_frontRight.getSelectedSensorPosition(), m_backRight.getSelectedSensorPosition()));

    SmartDashboard.putNumber("FL Volts", m_frontLeft.getBusVoltage());
    SmartDashboard.putNumber("FR Volts", m_frontRight.getBusVoltage());
    SmartDashboard.putNumber("BL Volts", m_backLeft.getBusVoltage());
    SmartDashboard.putNumber("BR Volts", m_backRight.getBusVoltage());

  }
}
