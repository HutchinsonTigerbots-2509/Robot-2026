// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SubsystemConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  static public TalonFX mShooter = new TalonFX(SubsystemConstants.kShootMotorId);

  public static void shootpositive() {
    mShooter.set(1);
  }

  public static void shootzero() {
    mShooter.set(0);
  }

  public static void shootnegative() {
    mShooter.set(-1);
  }

  public static void shootvariable(Double num) {
    mShooter.set(num);
  }
}
