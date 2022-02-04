// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CargoStaging extends SubsystemBase {

  private final PWMSparkMax motor = new PWMSparkMax(Constants.StagingConstants.motorChannel);




  /** Creates a new CargoStaging. */
  public CargoStaging() {
    
  }

  public void SetMotorSpeed(int direction){
    motor.set(Constants.StagingConstants.motorSpeed * direction);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}