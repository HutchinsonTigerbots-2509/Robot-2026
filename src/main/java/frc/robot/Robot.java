// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.HootReplay;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();
    public Robot() {
        m_robotContainer = new RobotContainer();
        RobotController.setBrownoutVoltage(6.0);
    }

    @Override
    public void robotPeriodic() {
        SignalLogger.enableAutoLogging(false);
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        //SignalLogger.stop();
        SmartDashboard.putNumber("pigeon", m_robotContainer.getDrivetrain().getPigeon2().getRotation2d().getRadians());
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new InstantCommand(() -> m_robotContainer.getFeeder().feedZero()), new InstantCommand(() -> m_robotContainer.getIntake().intakeZero()), new InstantCommand(() -> m_robotContainer.getLift().liftZero()), new InstantCommand(() -> m_robotContainer.getLighting().lightOff()), new InstantCommand(() -> m_robotContainer.getShooter().shootZero())));
    }

    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new InstantCommand(() -> m_robotContainer.getFeeder().feedZero()), new InstantCommand(() -> m_robotContainer.getIntake().intakeZero()), new InstantCommand(() -> m_robotContainer.getLift().liftZero()), new InstantCommand(() -> m_robotContainer.getLighting().lightOff()), new InstantCommand(() -> m_robotContainer.getShooter().shootZero())));
    }

    @Override
    public void disabledExit() {
        CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new InstantCommand(() -> m_robotContainer.getFeeder().feedZero()), new InstantCommand(() -> m_robotContainer.getIntake().intakeZero()), new InstantCommand(() -> m_robotContainer.getLift().liftZero()), new InstantCommand(() -> m_robotContainer.getLighting().lightOff()), new InstantCommand(() -> m_robotContainer.getShooter().shootZero())));
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.ApplyStart();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
