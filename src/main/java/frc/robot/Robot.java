// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.GamePiece;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    public static ShuffleboardTab MAIN_TAB = Shuffleboard.getTab("Main");

    @Override
    public void robotInit() {

        Logger logger = Logger.getInstance();

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            // Running on a real robot, log to a USB stick
            case REAL:
                logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
                logger.addDataReceiver(new NT4Publisher());
                break;

            // Running a physics simulator, log to local folder
            case SIM:
                logger.addDataReceiver(new WPILOGWriter(""));
                logger.addDataReceiver(new NT4Publisher());
                break;

            // Replaying a log, set up replay source
            case REPLAY:
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                logger.setReplaySource(new WPILOGReader(logPath));
                logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.getInstance().disableDeterministicTimestamps()

        // Start AdvantageKit logger
        logger.start();

        m_robotContainer = new RobotContainer();
        Shuffleboard.selectTab("Main");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        RobotContainer.lightingSubsystem.rainbow();
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

        RobotContainer.swerveDriveSubsystem.zeroGyroscope();

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
        RobotContainer.swerveDriveSubsystem.zeroGyroscope();
    }

    @Override
    public void teleopPeriodic() {
        if (!m_robotContainer.getTeleopDriveCommand().isScheduled()) {
            m_robotContainer.getTeleopDriveCommand().schedule();
        }

        if (RobotContainer.requestedGamePiece == GamePiece.CONE) {
            RobotContainer.lightingSubsystem.setConeColor();
        } else if (RobotContainer.requestedGamePiece == GamePiece.CUBE) {
            RobotContainer.lightingSubsystem.setCubeColor();
        }

        Logger.getInstance().recordOutput("Requested GamePiece", RobotContainer.requestedGamePiece.toString());
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        if (!m_robotContainer.getTeleopDriveCommand().isScheduled()) {
            m_robotContainer.getTeleopDriveCommand().schedule();
        }
        Shuffleboard.selectTab("Testing");
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
