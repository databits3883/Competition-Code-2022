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

  /*public double setpointCopy = 0;
  double pcopy=0;
  double icopy=0;
  double dcopy=0;
  double fcopy=0;*/

  /** Creates a new Launcher. */
  public Launcher() {
    m_launchMotor = new CANSparkMax(LEADER_CHANNEL, MotorType.kBrushless);
    m_controller = m_launchMotor.getPIDController();
    m_encoder = m_launchMotor.getEncoder();


    m_encoder.setVelocityConversionFactor(ENCODER_POSITIONAL_CONVERSION);

    m_controller.setFF(0.0003);
    m_controller.setP(0.001);
    m_controller.setI(0);
    m_controller.setD(0);
  }

  public void SetShooterSpeed(double rpm){
    m_controller.setReference(rpm, ControlType.kVelocity);
    //setpointCopy = rpm;
  }

  public void setDutyCycle(double dutyCycle){

    m_launchMotor.set(dutyCycle);
  }

  /*@Override 
  public void initSendable(SendableBuilder builder){
    super.initSendable(builder);
    builder.addDoubleProperty("kp", ()->pcopy,(p)->{ m_controller.setP(p); pcopy=p;});
    builder.addDoubleProperty("ki",()->icopy, (i)->{ m_controller.setI(i); icopy=i;});
    builder.addDoubleProperty("kd", ()->dcopy, (d)->{ m_controller.setD(d); dcopy=d;});
    builder.addDoubleProperty("kf", ()->fcopy, (f)->{ m_controller.setFF(f); fcopy=f;});

    builder.addDoubleProperty("setpoint",()->this.setpointCopy, this::SetShooterSpeed);
    builder.addDoubleProperty("speed", m_encoder::getVelocity, null);

  }*/

  
}
