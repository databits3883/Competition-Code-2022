// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.StagingConstants.*;

public class CargoStaging extends SubsystemBase {

  private final CANSparkMax m_motor;
  private final DigitalInput m_inSensor = new DigitalInput(SENSOR_CHANNEL);




  /** Creates a new CargoStaging. */
  public CargoStaging() {
    m_motor  = new CANSparkMax(MOTOR_CHANNEL, MotorType.kBrushless);
    m_motor.setInverted(true);
  }

  public void runIn(){
    m_motor.set(IN_SPEED);
  }
  public void runOut(){
    m_motor.set(OUT_SPEED);
  }
  public void stop(){
    m_motor.set(0);
  }

  public boolean cargoAtEntrance(){
    return !m_inSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}