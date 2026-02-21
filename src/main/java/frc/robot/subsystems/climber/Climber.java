// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {
    mClimber.setNeutralMode(NeutralMode.Brake);
    SmartDashboard.putNumber("climber setter", 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public WPI_TalonSRX mClimber = new WPI_TalonSRX(ClimberConstants.kClimbMotorId);

  public void climbUp() {
    mClimber.set(SmartDashboard.getNumber("climber setter", 0.0));
  }

  public void climbZero() {
    mClimber.set(0);
  }

  public void climbDown() {
    mClimber.set(-1 * SmartDashboard.getNumber("climber setter", 0.0));
  }
}
