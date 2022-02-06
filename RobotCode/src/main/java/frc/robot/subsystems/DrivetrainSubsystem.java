// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */


  WPI_TalonFX leftFront = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_FRONT);
  WPI_TalonFX rightFront = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_FRONT);
  WPI_TalonFX leftBack = new WPI_TalonFX(Constants.DrivetrainConstants.LEFT_BACK);
  WPI_TalonFX rightBack = new WPI_TalonFX(Constants.DrivetrainConstants.RIGHT_BACK);

  DifferentialDrive drive = new DifferentialDrive(leftFront, rightFront);


  public DrivetrainSubsystem() {
    leftFront.configFactoryDefault();
    leftBack.configFactoryDefault();
    rightFront.configFactoryDefault();
    rightBack.configFactoryDefault();

    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

    rightFront.setInverted(false);
    leftFront.setInverted(true);

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);

    
    drive = new DifferentialDrive(leftFront, rightFront);
    
    
  }
  public void arcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
