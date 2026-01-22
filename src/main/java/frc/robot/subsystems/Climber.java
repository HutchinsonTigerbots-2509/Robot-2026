// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SubsystemConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  static public TalonFX mClimber = new TalonFX(SubsystemConstants.kClimbMotorId);

  public static void climbpositive() {
    mClimber.set(1);
  }

  public static void climbzero() {
    mClimber.set(0);
  }

  public static void climbnegative() {
    mClimber.set(-1);
  }
}
