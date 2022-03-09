// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  static NetworkTable table;
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    
  }
  public double getHorizontalOffset(){
    return table.getEntry("tx").getDouble(0);
  }

  public boolean isTargetValid(){
    if (table.getEntry("tv").getNumber(0).intValue() == 1){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
