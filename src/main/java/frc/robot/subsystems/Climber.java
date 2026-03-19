// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {
    mClimber.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public DigitalInput wClimbMax = new DigitalInput(5);
  public WPI_TalonSRX mClimber = new WPI_TalonSRX(9);

  public void climbUp() {
    mClimber.set(1.0);
  }

  public void climbZero() {
    mClimber.set(0);
  }

  public void climbDown() {
    mClimber.set(-1.0);
  }

  public void climb1() {
    if (!wClimbMax.get()) {
      mClimber.set(0);
    } else {
      mClimber.set(-1.0);
    }
  }

  public void climb2() {
    if (!wClimbMax.get()) {
      mClimber.set(0);
    } else {
      mClimber.set(1.0);
    }
  }
}
