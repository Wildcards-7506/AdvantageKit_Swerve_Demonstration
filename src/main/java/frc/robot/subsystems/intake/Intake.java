// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax m_intake;
  private final SparkMaxConfig c_intake;
  private final SparkMax m_rotator;
  private final SparkMaxConfig c_rotator;
  private final SparkClosedLoopController pid_rotator;

  /** Creates a new Intake. */
  public Intake() {
    m_intake = new SparkMax(10, MotorType.kBrushless);
    c_intake = new SparkMaxConfig();
    m_rotator = new SparkMax(11, MotorType.kBrushless);
    c_rotator = new SparkMaxConfig();
    pid_rotator = m_rotator.getClosedLoopController();

    c_intake.idleMode(IdleMode.kCoast).smartCurrentLimit(40);

    c_rotator.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    c_rotator
        .softLimit
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(230.0)
        .reverseSoftLimit(0.0);
    c_rotator.encoder.positionConversionFactor(360.0 / 5.0);
    c_rotator.closedLoop.pid(0.005, 0.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Position", getIntakePosition());
  }

  public void setIntakePosition(double setpoint) {
    pid_rotator.setReference(setpoint, ControlType.kPosition);
  }

  public void setIntakeVoltage(double setpoint) {
    m_intake.setVoltage(setpoint);
  }

  public double getIntakePosition() {
    return m_rotator.getEncoder().getPosition();
  }

  public double getIntakeVelocity() {
    return m_intake.getEncoder().getVelocity();
  }
}
