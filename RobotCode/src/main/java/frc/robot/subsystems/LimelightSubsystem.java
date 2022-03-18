// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.cameraserver.CameraServer;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */

  public LimelightSubsystem() {
    //CameraServer.startAutomaticCapture();
  }

  public double getTX() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    return table.getEntry("tx").getDouble(0.0);
  }

  public double getTY() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    return table.getEntry("ty").getDouble(0.0);
  }

  public double getArea() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    return table.getEntry("ta").getDouble(0.0);
  }

public boolean isValidTarget() {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  double check = table.getEntry("tv").getDouble(0.0);
  SmartDashboard.putNumber("Valid Target", check);
  if (check == 1.0) {
    return true;
  }
  else if (check == 0.0) {
    return false;
  }
  else {
    return false;
  }
}

public boolean isOnTarget() {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  return this.isValidTarget() && Math.abs(this.getTX()) <=1;
}

public void setCamMode (double value) {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  table.getEntry("camMode").setDouble(value);
}

public void setStreamMode (double value) {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  table.getEntry("streamMode").setDouble(value);
}

public void setLED (double value) {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  table.getEntry("LEDMode").setDouble(value);
}

public void setPipeline (double value) {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  table.getEntry("pipeline").setDouble(value);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("isOnTarget", isOnTarget());
  }
}
