// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */

  private double kLiftTorqueMax = 200;
  private double kLiftOutSpeed;
  private double kLiftOutFastSpeed;
  private double kLiftInSpeed;
  private double kLiftInFastSpeed;
  private boolean kTorqueLimit;

  public Lift() {
    mLiftA.setNeutralMode(NeutralModeValue.Brake);
    mLiftB.setNeutralMode(NeutralModeValue.Brake);
    liftSpeedSetter();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("LiftSwitch", !wLiftMax.get());
    SmartDashboard.putBoolean("LiftCycle", liftCycle);
    SmartDashboard.putNumber("LiftPos", eLift.get());
    if (!wLiftMax.get()) {
      eLift.reset();
    }
    SmartDashboard.putNumber("mLiftATorque", mLiftA.getTorqueCurrent().getValueAsDouble()); 
    SmartDashboard.putNumber("mLiftBTorque", mLiftB.getTorqueCurrent().getValueAsDouble());
    liftTorqueChecker();
  }

  private TalonFX mLiftA = new TalonFX(15);
  private TalonFX mLiftB = new TalonFX(16);
  public DigitalInput wLiftMax = new DigitalInput(4);
  public Encoder eLift = new Encoder(2, 3);

  private boolean liftCycle = false;

  public void liftZero() {
    mLiftA.set(0);
    mLiftB.set(0);
  }

  public void liftOut() {
    mLiftA.set(kLiftOutSpeed);
    mLiftB.set(-kLiftOutSpeed);
  }

  public void liftOutFast() {
    mLiftA.set(kLiftOutFastSpeed);
    mLiftB.set(-kLiftOutFastSpeed);
  }

  public void liftIn() {
    if(liftCycle) {
      mLiftA.set(-kLiftInSpeed);
      mLiftB.set(kLiftInSpeed);
    }
  }

  public void liftInFast() {
    if(liftCycle) {
      mLiftA.set(-kLiftInFastSpeed);
      mLiftB.set(kLiftInFastSpeed);
    }
  }

  public void liftInEmergency() {
    mLiftA.set(-0.1);
    mLiftB.set(0.1);
  }

  public void modLiftCycle() {
    liftCycle = true;
  }

  public void liftTorqueChecker() {
    if (Math.abs(mLiftA.getTorqueCurrent().getValueAsDouble()) > kLiftTorqueMax || Math.abs(mLiftB.getTorqueCurrent().getValueAsDouble()) > kLiftTorqueMax) {
      kLiftOutSpeed = 0;
      kLiftOutFastSpeed = 0;
      kLiftInSpeed = 0;
      kLiftInFastSpeed = 0;
      kTorqueLimit = true;
    }
    torqueOutputMessage();
  }

  public void liftSpeedSetter() {
    kLiftOutSpeed = 0.25;
    kLiftOutFastSpeed = 0.8;
    kLiftInSpeed = 0.25;
    kLiftInFastSpeed = 0.4;
    kTorqueLimit = false;
    System.out.println("Lift motors' output values have been set to defaults");
  }

  public void torqueOutputMessage() {
    if (kTorqueLimit) {
      System.out.println(" ");
      System.out.println("Torque produced by Lift motors exceeded the max torque limit of: " + kLiftTorqueMax);
      System.out.println(" ");
      System.out.println("To reset the Lift motors' output values, please press 'y' on your controller");
      System.out.println(" ");
    }
  }
}
