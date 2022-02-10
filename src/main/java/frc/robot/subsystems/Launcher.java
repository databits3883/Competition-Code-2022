// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.LauncherContants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  private final CANSparkMax m_launchMotor;
  private final SparkMaxPIDController m_controller;
  
  private final RelativeEncoder m_encoder;
  /** Creates a new Launcher. */
  public Launcher() {
    m_launchMotor = new CANSparkMax(LEADER_CHANNEL, MotorType.kBrushless);
    m_controller = m_launchMotor.getPIDController();
    m_encoder = m_launchMotor.getEncoder();

    m_encoder.setPositionConversionFactor(ENCODER_POSITIONAL_CONVERSION);
  }

  public void SetShooterSpeed(double rpm){
    //m_controller.setReference(rpm, ControlType.kVelocity);
    m_controller.setReference(rpm, ControlType.kVelocity);

  }

  public void setDutyCycle(double dutyCycle){
    m_launchMotor.set(dutyCycle);
  }
}
