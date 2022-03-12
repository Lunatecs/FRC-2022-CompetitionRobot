// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  private final TalonSRX turret = new TalonSRX(TurretConstants.Turret_ID);
  private NeutralMode TURRET_NEUTRALMODE = NeutralMode.Brake;
  private PIDController pidControllerFwd;
  private PIDController pidControllerBck;
  private PIDController pidControllerLock;
  private boolean lock = false;
  
  public TurretSubsystem() {
    turret.configFactoryDefault();
    turret.setNeutralMode(TURRET_NEUTRALMODE);
    //resetPosition();
    pidControllerFwd = new PIDController(TurretConstants.FwdKp,TurretConstants.FwdKi,TurretConstants.FwdKd);
    pidControllerFwd.setSetpoint(TurretConstants.FwdMaxSensorPostion);
    pidControllerBck = new PIDController(TurretConstants.BckKp,TurretConstants.BckKi,TurretConstants.BckKd);
    pidControllerBck.setSetpoint(TurretConstants.BckMaxSensorPostion);
    turret.setSensorPhase(true);
    this.pidControllerLock = new PIDController(0.0, 0.0, .9);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Encoder", getPosition());

    if(lock) {
      this.setTurretSpeed(pidControllerLock.calculate(this.getPosition()), true);
    }

    super.periodic();
  }

  public void turn(double speed, boolean ignoreMin) {
    double position = getPosition(); 
    double speedLimitFwd = pidControllerFwd.calculate(position);
    double speedLimitBck = pidControllerBck.calculate(position);
    //SmartDashboard.putNumber("speedLimitFwd", speedLimitFwd);
    //SmartDashboard.putNumber("speedLimitBck", speedLimitBck);
    
    if(speed > speedLimitFwd && speed > 0) {
      speed = speedLimitFwd;
    } else if(speed < speedLimitBck && speed <= 0) {
      speed = speedLimitBck;
    }
    if (!(speed == 0) && Math.abs(speed) < TurretConstants.MinSpeed && !ignoreMin) {
      if(speed < 0) {
        speed = -TurretConstants.MinSpeed;
      } else if(speed > 0) {
        speed = TurretConstants.MinSpeed;
      }
    }
  }

  public void setTurretSpeed(double speed, boolean ignoreMin) {
    double position = getPosition(); 
    double speedLimitFwd = pidControllerFwd.calculate(position);
    double speedLimitBck = pidControllerBck.calculate(position);
    //SmartDashboard.putNumber("speedLimitFwd", speedLimitFwd);
    //SmartDashboard.putNumber("speedLimitBck", speedLimitBck);
    
    if(speed > speedLimitFwd && speed > 0) {
      speed = speedLimitFwd;
    } else if(speed < speedLimitBck && speed <= 0) {
      speed = speedLimitBck;
    }
    if (!(speed == 0) && Math.abs(speed) < TurretConstants.MinSpeed && !ignoreMin) {
      if(speed < 0) {
        speed = -TurretConstants.MinSpeed;
      } else if(speed > 0) {
        speed = TurretConstants.MinSpeed;
      }
    }
    //SmartDashboard.putNumber("actualSpeed", speed);
    turret.set(ControlMode.PercentOutput, speed);
  }


  public double getPosition() {
    return this.turret.getSelectedSensorPosition(0);
  }

  public void resetPosition() {
    turret.setSelectedSensorPosition(0, 0, 10);
  }

  public boolean isFwdLimit() {
    return turret.getSensorCollection().isFwdLimitSwitchClosed();
    // if(getPosition() >= TurretConstants.FwdMaxSensorPostion) {
    //   return true;
    // } else {
    //   return false;
    // }
  }

  public boolean isRevLimit() {
    return turret.getSensorCollection().isRevLimitSwitchClosed();
    // if(getPosition() <= TurretConstants.BckMaxSensorPostion) {
    //   return true;
    // } else {
    //   return false;
    // }
  }

}
