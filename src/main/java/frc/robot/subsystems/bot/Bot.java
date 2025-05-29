// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.bot.arm.Arm;
import frc.robot.subsystems.bot.arm.ArmIOSIM;
import frc.robot.subsystems.bot.drive.Drive;
import frc.robot.subsystems.bot.drive.DriveIOCTRE;
import frc.robot.subsystems.bot.elevator.Elevator;
import frc.robot.subsystems.bot.elevator.ElevatorIOSIM;
import frc.robot.subsystems.bot.intake.Intake;
import frc.robot.subsystems.bot.intake.IntakeIOSIM;
import frc.robot.subsystems.bot.vision.Vision;
import frc.robot.subsystems.bot.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;

public class Bot extends SubsystemBase {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final Drive drivetrain;
  private final Vision vision;
  private final Arm arm;
  private final Intake intake;
  private final Elevator elevator;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed.times(0.1))
      .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final TunableController joystick;
  private final LoggedDashboardChooser<Command> autoChooser;
  private boolean tempDissabled = false;
  private final BotType botType;
  private final BotDificulty botDificulty;
  private States state;

  public Bot(boolean testingBot, BotType type, BotDificulty dificulty) {
    botType = type;
    botDificulty = dificulty;
    joystick = new TunableController(testingBot ? 0 : 1)
    .withControllerType(TunableControllerType.QUADRATIC);
    autoChooser = new LoggedDashboardChooser<>("Bot Auto Choices", AutoBuilder.buildAutoChooser());
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    drivetrain = new Drive(currentDriveTrain);
    ArmIOSIM temp = new ArmIOSIM();
    elevator = new Elevator(new ElevatorIOSIM(temp.getMechanismRoot()));
    arm = new Arm(temp);
    intake = new Intake(new IntakeIOSIM());

    vision = new Vision(
        drivetrain::addVisionData,
        new VisionIOPhotonVisionSIM(
            "FrontLeft",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(10.066),
                    Units.inchesToMeters(11.959),
                    Units.inchesToMeters(8.55647482)), // IN
                // METERS
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-25.16683805),
                    Units.degreesToRadians(330)) // IN
            // RADIANS
            ),
            drivetrain::getVisionParameters),
        new VisionIOPhotonVisionSIM(
            "FrontRight",
            new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(10.066),
                    -Units.inchesToMeters(11.959),
                    Units.inchesToMeters(8.55647482)), // IN
                // METERS
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-25.16683805),
                    Units.degreesToRadians(30)) // IN
            // RADIANS
            ),
            drivetrain::getVisionParameters),
        new VisionIOPhotonVisionSIM(
            "BackLeft",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(9.896),
                    Units.inchesToMeters(10.938),
                    Units.inchesToMeters(8.55647482)), // IN
                // METERS
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-25.16683805),
                    Units.degreesToRadians(150)) // IN
            // RADIANS
            ),
            drivetrain::getVisionParameters),
        new VisionIOPhotonVisionSIM(
            "BackRight",
            new Transform3d(
                new Translation3d(
                    -Units.inchesToMeters(9.896),
                    -Units.inchesToMeters(10.938),
                    Units.inchesToMeters(8.55647482)), // IN
                // METERS
                new Rotation3d(
                    0,
                    Units.degreesToRadians(-25.16683805),
                    Units.degreesToRadians(210)) // IN
            // RADIANS
            ),
            drivetrain::getVisionParameters));
  }

  @Override
  public void periodic() {
    if (!tempDissabled) {

    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void dissableBotTemp() {
    tempDissabled = true;
  }

  public void undissableBotTemp() {
    tempDissabled = false;
  }

  public enum BotType {
    CYCLES(),
    POINTS(),
    RP()
  }

  public enum BotDificulty {
    LIGHT(),
    FRIGHTENED(),
    AGRESSIVE()
  }

  private enum States {
    INTAKE(),
    TOREEF(),
    ALIGN(),
    PLACE(),
    TOINTAKE(),
    AUTOINTAKE()
  }
}
