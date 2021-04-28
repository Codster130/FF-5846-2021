// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
//import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  Trajectory bounce1Trajectory = new Trajectory();
  Trajectory bounce2Trajectory = new Trajectory();
  Trajectory bounce3Trajectory = new Trajectory();
  Trajectory bounce4Trajectory = new Trajectory();

  Arm arm = new Arm();
  
  // Assumes a gamepad plugged into channnel 0
  private final Joystick m_controller = new Joystick(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Generate a trajectory following Ramsete command
   * 
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were 
   * found empirically by using the frc-characterization tool.
   * 
   * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
   */
  private Command generateRamseteCommand() {
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                       DriveConstants.kvVoltSecondsPerMeter, 
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);

    // This trajectory can be modified to suit your purposes
    // Note that all coordinates are in meters, and follow NWU conventions.
    // If you would like to specify coordinates in inches (which might be easier
    // to deal with for the Romi), you can use the Units.inchesToMeters() method
    
    /***********\
     Bounce1 Code
    \***********/
    
    String trajectory1JSON = "paths/Bounce1_Final.wpilib.json"; //change the ".json" to get a different path to run as long as file is in src/main/deploy/paths
      try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory1JSON);
    bounce1Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
     DriverStation.reportError("Unable to open trajectory: " + trajectory1JSON, ex.getStackTrace());
}
    RamseteCommand Bounce1 = new RamseteCommand(
        bounce1Trajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        m_drivetrain::tankDriveVolts,
        m_drivetrain);
        // takes json file from src/main/deploy/paths in this case cc.wpilib.json and reads the path for the robot to execute
    m_drivetrain.resetOdometry(bounce1Trajectory.getInitialPose());


    /***********\
     Bounce2 Code
    \***********/
    
    String trajectory2JSON = "paths/Bounce2_Final.wpilib.json"; //change the ".json" to get a different path to run as long as file is in src/main/deploy/paths
    try {
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory2JSON);
  bounce2Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException ex) {
   DriverStation.reportError("Unable to open trajectory: " + trajectory2JSON, ex.getStackTrace());
}
  RamseteCommand Bounce2 = new RamseteCommand(
      bounce2Trajectory,
      m_drivetrain::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      m_drivetrain::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      m_drivetrain::tankDriveVolts,
      m_drivetrain);
      // takes json file from src/main/deploy/paths in this case cc.wpilib.json and reads the path for the robot to execute
  m_drivetrain.resetOdometry(bounce2Trajectory.getInitialPose());



    /***********\
     Bounce3 Code
    \***********/
    
    String trajectory3JSON = "paths/Bounce3_Final.wpilib.json"; //change the ".json" to get a different path to run as long as file is in src/main/deploy/paths
      try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory3JSON);
    bounce3Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
     DriverStation.reportError("Unable to open trajectory: " + trajectory3JSON, ex.getStackTrace());
}
    RamseteCommand Bounce3 = new RamseteCommand(
        bounce3Trajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        m_drivetrain::tankDriveVolts,
        m_drivetrain);
        // takes json file from src/main/deploy/paths in this case cc.wpilib.json and reads the path for the robot to execute
    m_drivetrain.resetOdometry(bounce3Trajectory.getInitialPose());



    /***********\
     Bounce4 Code
    \***********/
    
    String trajectory4JSON = "paths/Bounce4_Final.wpilib.json"; //change the ".json" to get a different path to run as long as file is in src/main/deploy/paths
      try {
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory4JSON);
    bounce4Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
     DriverStation.reportError("Unable to open trajectory: " + trajectory4JSON, ex.getStackTrace());
}
    RamseteCommand Bounce4 = new RamseteCommand(
        bounce4Trajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        m_drivetrain::tankDriveVolts,
        m_drivetrain);
        // takes json file from src/main/deploy/paths in this case cc.wpilib.json and reads the path for the robot to execute
    m_drivetrain.resetOdometry(bounce4Trajectory.getInitialPose());




    
    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> m_drivetrain.resetOdometry(bounce1Trajectory.getInitialPose()), m_drivetrain)
        // next, we run the actual ramsete command
        .andThen(Bounce1)
        .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain))
        //.andThen(new InstantCommand(() -> m_drivetrain.resetOdometry(bounce2Trajectory.getInitialPose()), m_drivetrain))
        //.andThen(new InstantCommand(() -> arm.incrementTilt(.5), arm))
        .andThen(Bounce2)
        .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain))
        //.andThen(new InstantCommand(() -> m_drivetrain.resetOdometry(bounce3Trajectory.getInitialPose()), m_drivetrain))
        .andThen(Bounce3)
        .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain))
        //.andThen(new InstantCommand(() -> m_drivetrain.resetOdometry(bounce4Trajectory.getInitialPose()), m_drivetrain))
        .andThen(Bounce4)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain));
  } 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenInactive(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Ramsete Trajectory", generateRamseteCommand());
    m_chooser.addOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    

  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> m_controller.getRawAxis(4));
  }//axis 4 divided by 1.25 for easier circle turning, downside of sloing robot
}
