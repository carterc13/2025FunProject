// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.botCommands.Align;
import frc.robot.commands.botCommands.Place;
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
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.SimCoral;
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
  private final Memory memory;
  private States state;
  private Command currentCommand;

  public Bot(boolean testingBot, BotType type) {
    botType = type;
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

    memory = new Memory(botType);
    memory.updateNextBest(drivetrain.getPose());
    elevator.IDLE().schedule();
    intake.STOW().schedule();
    arm.CORALIDLE().schedule();
    drivetrain.setDefaultCommand(drivetrain
        .applyRequest(
            () -> drive
                .withVelocityX(MaxSpeed.times(joystick.getLeftY()))
                .withVelocityY(MaxSpeed.times(joystick.getLeftX()))
                .withRotationalRate(Constants.MaxAngularRate.times(joystick.getRightX()))));
  }

  @Override
  public void periodic() {
    if (currentCommand.isFinished()) {
      switch (state) {
        case TOREEF:
          state = States.ALIGN;
          currentCommand = new Align(drivetrain, arm, elevator, intake, memory);
          break;
        case ALIGN:
        state = States.PLACE;
          currentCommand = new Place(drivetrain, arm, elevator, intake, memory);
          break;
          case PLACE:
          state = States.TOINTAKE;
            currentCommand = newAutoToIntake();
            break;
            case TOINTAKE:
            state = States.AUTOINTAKE;
              //currentCommand = new AutoIntake(drivetrain, arm, elevator, intake, memory);///////////////////////////////////////////////
              break;
              case AUTOINTAKE:
        state = States.TOREEF;
          currentCommand = newAutoToReef();
          break;
        default:
          break;
      }
      currentCommand.schedule();
    } else {
      switch (state) {
        case TOREEF:
          if (Pathfinding.isNewPathAvailable()) {
            currentCommand.cancel();
            currentCommand = newAutoToReef();
            currentCommand.schedule();
          }
          break;
        default:
          break;
      }
    }
  }

  private Command newAutoToIntake() {
    Pathfinding.setStartPosition(
        AllianceFlipUtil.apply(AutoBuilder.getCurrentPose().getTranslation()));
    Pathfinding.setGoalPosition(SimCoral.getLeftPose().getTranslation().getDistance(AutoBuilder.getCurrentPose().getTranslation()) > SimCoral.getRightPose().getTranslation().getDistance(AutoBuilder.getCurrentPose().getTranslation()) ? SimCoral.getRightPose().getTranslation().plus(new Translation2d(Units.inchesToMeters(5), Units.inchesToMeters(0))) : SimCoral.getLeftPose().getTranslation().plus(new Translation2d(Units.inchesToMeters(5), Units.inchesToMeters(0))));
    return 
        AutoBuilder.followPath(
            Pathfinding.getCurrentPath(
                new PathConstraints(5.72, 14.7, 4.634, Units.degreesToRadians(1136)),
                new GoalEndState(MetersPerSecond.of(0.5), drivetrain.getReefPosition().getAngle())));
  }

  private Command newAutoToReef() {
    Pathfinding.setStartPosition(
        AllianceFlipUtil.apply(AutoBuilder.getCurrentPose().getTranslation()));
    Pathfinding.setGoalPosition(drivetrain.getReefPosition().getPoseOffset());
    return 
        AutoBuilder.followPath(
            Pathfinding.getCurrentPath(
                new PathConstraints(5.72, 14.7, 4.634, Units.degreesToRadians(1136)),
                new GoalEndState(MetersPerSecond.of(0.5), drivetrain.getReefPosition().getAngle())));
  }

  public enum BotType {
    CYCLES(),
    POINTS(),
    RP()
  }

  private enum States {
    TOREEF(),
    ALIGN(),
    PLACE(),
    TOINTAKE(),
    AUTOINTAKE()
  }
}
