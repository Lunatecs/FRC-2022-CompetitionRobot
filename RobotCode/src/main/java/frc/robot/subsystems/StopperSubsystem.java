// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StopperConstants;

public class StopperSubsystem extends SubsystemBase {

  private final DoubleSolenoid stopperSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.REVPH,
    StopperConstants.FORWARD_CHANNEL, 
    StopperConstants.REVERSE_CHANNEL);

  /** Creates a new StopperSubsystem. */
  public StopperSubsystem() {

    this.stopperSolenoid.set(DoubleSolenoid.Value.kForward);

  }

  public void toggleStopper() {
    this.stopperSolenoid.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
