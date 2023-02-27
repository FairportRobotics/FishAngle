// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.GamePiece;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    public static ShuffleboardTab TESTING_TAB = Shuffleboard.getTab("Testing");
    public static ShuffleboardTab MAIN_TAB = Shuffleboard.getTab("Main");
    public static ShuffleboardTab DEBUG_TAB = Shuffleboard.getTab("Debug");

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        Shuffleboard.selectTab("Main");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.lightingSubsystem.rainbow();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        if (!m_robotContainer.getTeleopDriveCommand().isScheduled()) {
            m_robotContainer.getTeleopDriveCommand().schedule();
        }

        if (m_robotContainer.requestedGamePiece == GamePiece.CONE) {
            m_robotContainer.lightingSubsystem.setConeColor();
        } else if (m_robotContainer.requestedGamePiece == GamePiece.CUBE) {
            m_robotContainer.lightingSubsystem.setCubeColor();
        }

    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        Shuffleboard.selectTab("Testing");
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
