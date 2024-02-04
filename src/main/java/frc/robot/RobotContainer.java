// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.constants.Ports;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  
  /* Controllers */
  private final XboxController DRIVER = new XboxController(Ports.Gamepad.DRIVER);
  private final XboxController OPERATOR = new XboxController(Ports.Gamepad.OPERATOR);

  /* Driver Buttons */
 private final Trigger driver_A_zeroGyro = new JoystickButton(DRIVER, XboxController.Button.kA.value);

 private final Trigger driver_xlock = new JoystickButton(DRIVER, XboxController.Button.kX.value);

 private final Trigger driver_slowSpeed_rightBumper = new JoystickButton(DRIVER,
     XboxController.Button.kRightBumper.value);

 // Additional buttons
 private final Trigger driver_leftBumper = new JoystickButton(DRIVER, XboxController.Button.kLeftBumper.value);

 private final Trigger driver_B = new JoystickButton(DRIVER, XboxController.Button.kB.value);

 private final Trigger driver_Y = new JoystickButton(DRIVER, XboxController.Button.kY.value);

 private final Trigger driver_BottomRightRearButton = new JoystickButton(DRIVER,
     XboxController.Button.kStart.value);
 private final Trigger driver_BottomLeftRearButton = new JoystickButton(DRIVER,
     XboxController.Button.kBack.value);

 private final Trigger driver_TopRightRearButton = new JoystickButton(DRIVER,
     XboxController.Button.kRightStick.value);

 private final Trigger driver_START = new JoystickButton(DRIVER, XboxController.Button.kStart.value);
 private final Trigger driver_BACK = new JoystickButton(DRIVER, XboxController.Button.kBack.value);

 private final Trigger dpadUpButtonDrive = new Trigger(() -> DRIVER.getPOV() == 0);
 private final Trigger dpadRightButtonDrive = new Trigger(() -> DRIVER.getPOV() == 90);
 private final Trigger dpadDownButtonDrive = new Trigger(() -> DRIVER.getPOV() == 180);
 private final Trigger dpadLeftButtonDrive = new Trigger(() -> DRIVER.getPOV() == 270);

private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);//Might need to switch placement

    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(swerveDrive,
                                                                   () -> MathUtil.applyDeadband(DRIVER.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(DRIVER.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(DRIVER.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   DRIVER::getYButtonPressed,
                                                                   DRIVER::getAButtonPressed,
                                                                   DRIVER::getXButtonPressed,
                                                                   DRIVER::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = swerveDrive.driveCommand(
        ( ) -> MathUtil.applyDeadband(DRIVER.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(DRIVER.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> DRIVER.getRightX(),
        () -> DRIVER.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = swerveDrive.driveCommand(
        () -> MathUtil.applyDeadband(DRIVER.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(DRIVER.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> DRIVER.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = swerveDrive.simDriveCommand(
        () -> MathUtil.applyDeadband(DRIVER.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(DRIVER.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> DRIVER.getRawAxis(2));

    swerveDrive.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // new JoystickButton(DRIVER, 1).onTrue((new InstantCommand(swerveDrive::zeroGyro)));//TODO try this soon!!!
    driver_A_zeroGyro.onTrue(new InstantCommand(() -> swerveDrive.zeroGyro()));
    driver_xlock.onTrue(new InstantCommand(()-> swerveDrive.lock()));
    driver_B.whileTrue( Commands.deferredProxy(() -> swerveDrive.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    // new JoystickButton(DRIVER,
    //                    2).whileTrue(
    //     Commands.deferredProxy(() -> swerveDrive.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
//    new JoystickButton(DRIVER, 3).whileTrue(new RepeatCommand(new InstantCommand(swerveDrive::lock, swerveDrive)));//TODO try this soon!!!
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    // return swerveDrive.getAutonomousCommand("testPath", true);
    PathPlannerPath test = PathPlannerPath.fromPathFile("testPath");
    return autoChooser.getSelected();

  }

  public void setDriveMode()
  {
    //swerveDrive.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorBrake(brake);
  }
}
