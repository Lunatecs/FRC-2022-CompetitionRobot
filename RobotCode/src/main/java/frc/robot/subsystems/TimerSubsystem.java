// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TimerSubsystem extends SubsystemBase {

  /** Creates a new TimerSubsystem. */
  public TimerSubsystem() {
    
  }

  @Override
  public void periodic() {
    LEDSubsystem led = LEDSubsystem.instanceOf();
    
    if(led!=null && DriverStation.isTeleop()) {
      double time = DriverStation.getMatchTime();
      //SmartDashboard.putNumber("time", time);
      //SmartDashboard.putBoolean("25", time <= 25.0);
      //SmartDashboard.putBoolean("35", time <= 35.0);
      if(time <= 25.0) {
        led.addColor(led.SECOND_ALARM);
      } else if (time <= 35.0) {
        led.addColor(led.FIRST_ALARM);
      } else {
        led.removeColor(led.SECOND_ALARM);
        led.removeColor(led.FIRST_ALARM);
      }
    }
  }
}
