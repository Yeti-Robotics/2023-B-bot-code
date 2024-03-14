// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LimelightHelpers;
import org.opencv.video.Video;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    AutoBuilder autoBuilder;
    private AutoConstants.AutoModes previousSelectedAuto;

    private static SendableChooser<AutoConstants.AutoModes> autoChooser;


    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        autoChooser = new SendableChooser<>();
        new Thread(() -> {
            UsbCamera camera = CameraServer.startAutomaticCapture();

            // 176 x 144 - 30 fps
            // 320 x 240 - 12 - 13 fps
            // 352 x 288 - 10 fps
            // anything higher :vomit_emoji:

            camera.setVideoMode(PixelFormat.kMJPEG, 1280, 720, 30);
            VideoSink sink = CameraServer.getServer(camera.getName());
            MjpegServer server = (MjpegServer) sink;
            server.setCompression(100);
            server.setResolution(176, 144);
            server.setDefaultCompression(100);
        }).start();
        // camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
        // camera.setVideoMode(PixelFormat.kMJPEG, 176, 144, 30);
        //camera.setVideoMode(PixelFormat.kMJPEG, 176, 144, 30);
        autoChooser.setDefaultOption(AutoConstants.AutoModes.MID_3_THREE_PIECE.name, AutoConstants.AutoModes.MID_3_THREE_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.TESTING.name, AutoConstants.AutoModes.TESTING);
        autoChooser.addOption(AutoConstants.AutoModes.AMP_5_THREE_PIECE.name, AutoConstants.AutoModes.AMP_5_THREE_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.SOURCE_4_THREE_PIECE.name, AutoConstants.AutoModes.SOURCE_4_THREE_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.AMP_4_THREE_PIECE.name, AutoConstants.AutoModes.AMP_4_THREE_PIECE);
        autoChooser.addOption(AutoConstants.AutoModes.NEW_AUTO.name, AutoConstants.AutoModes.NEW_AUTO);
        autoChooser.addOption(AutoConstants.AutoModes.MID_3_THREE_PIECE.name, AutoConstants.AutoModes.MID_3_THREE_PIECE);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        previousSelectedAuto = autoChooser.getSelected();
        autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == (DriverStation.Alliance.Red)) {
                Pose2d llPose = lastResult.getBotPose2d_wpiRed();
                if (lastResult.valid) {
                    robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
                }
            } else {
                Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
                if (lastResult.valid) {
                    robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
                }

            }
        }
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        if (previousSelectedAuto != autoChooser.getSelected()) {
            previousSelectedAuto = autoChooser.getSelected();
        }

        autonomousCommand = AutoBuilder.buildAuto(previousSelectedAuto.name);

    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {

        autonomousCommand.schedule();

    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
