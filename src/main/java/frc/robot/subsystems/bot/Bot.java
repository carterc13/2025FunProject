// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.bot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.botCommands.Align;
import frc.robot.commands.botCommands.Place;
import frc.robot.commands.botCommands.ReturnTrue;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.bot.arm.Arm;
import frc.robot.subsystems.bot.arm.Arm.ArmPosition;
import frc.robot.subsystems.bot.arm.ArmIOSIM;
import frc.robot.subsystems.bot.drive.Drive;
import frc.robot.subsystems.bot.drive.DriveIOCTRE;
import frc.robot.subsystems.bot.elevator.Elevator;
import frc.robot.subsystems.bot.elevator.ElevatorIOSIM;
import frc.robot.subsystems.bot.intake.Intake;
import frc.robot.subsystems.bot.intake.Intake.IntakePosition;
import frc.robot.subsystems.bot.intake.IntakeIOSIM;
import frc.robot.subsystems.bot.vision.Vision;
import frc.robot.subsystems.bot.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.SimCoral;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.Logger;

public class Bot extends SubsystemBase {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final Drive drivetrain;
  private final Vision vision;
  private final Arm arm;
  private final Intake intake;
  private final Elevator elevator;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final TunableController joystick;
  private final BotType botType;
  private final Memory memory;
  private States state = States.AUTOINTAKE;
  private Command currentCommand = new ReturnTrue();
  private Pose2d currentTarget = new Pose2d();
  private PathConstraints constraints =
      new PathConstraints(5.72, 14.7, 4.634, Units.degreesToRadians(1136));
  PIDController translation = new PIDController(5, 0, 0.25);
  PIDController translationy = new PIDController(5, 0, 0.25);
  PIDController rotation = new PIDController(0.1, 0, 0.003);
  private boolean hasCoral = false;
  private boolean firstRun = true;

  public Bot(boolean testingBot, BotType type) {
    SimCoral.start();
    // rotation.enableContinuousInput(-Math.PI, Math.PI);
    botType = type;
    joystick =
        new TunableController(testingBot ? 0 : 1)
            .withControllerType(TunableControllerType.QUADRATIC);
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    drivetrain = new Drive(currentDriveTrain);
    ArmIOSIM temp = new ArmIOSIM();
    elevator = new Elevator(new ElevatorIOSIM(temp.getMechanismRoot()));
    arm = new Arm(temp);
    intake = new Intake(new IntakeIOSIM());

    Pathfinding.setGoalPosition(
        drivetrain.getReefPosition().getPose().getTranslation().toTranslation2d());

    vision =
        new Vision(
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
                        0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(330)) // IN
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
                        0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(30)) // IN
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
                        0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(150)) // IN
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
                        0, Units.degreesToRadians(-25.16683805), Units.degreesToRadians(210)) // IN
                    // RADIANS
                    ),
                drivetrain::getVisionParameters));

    memory = new Memory(botType);
    memory.updateNextBest(drivetrain.getPose());
    elevator.IDLE().schedule();
    intake.STOW().schedule();
    arm.CORALIDLE().schedule();
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(MaxSpeed.times(joystick.getLeftY()))
                    .withVelocityY(MaxSpeed.times(joystick.getLeftX()))
                    .withRotationalRate(Constants.MaxAngularRate.times(joystick.getRightX()))));
    currentTarget =
        new Pose2d(
            drivetrain.getReefPosition().getPose().getTranslation().toTranslation2d().getX(),
            drivetrain.getReefPosition().getPose().getTranslation().toTranslation2d().getY(),
            drivetrain.getReefPosition().getPose().getTranslation().toTranslation2d().getAngle());
    Pathfinding.setStartPosition(drivetrain.getPose().getTranslation());
    currentCommand = new ReturnTrue();
    currentCommand.schedule();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Bot/State", state);
    Logger.recordOutput("States/ROBOT", state);
    SimCoral.loggingPeriodic(drivetrain.getPose());
    if (DriverStation.isEnabled()) {
      if (currentCommand.isFinished()) {
        switch (state) {
          case TOREEF:
            state = States.ALIGN;
            translation.reset();
            translationy.reset();
            rotation.reset();
            currentCommand = new Align(arm, elevator, intake, memory);
            Pathfinding.setGoalPosition(getSimCoralPose().getTranslation());
            Logger.recordOutput("Coral Pose", getSimCoralPose());
            break;
          case ALIGN:
            if (drivetrain.isAtTarget() && arm.isAtTarget() && elevator.isAtTarget()) {
              state = States.PLACE;
              currentCommand = new Place(drivetrain, arm, elevator, intake, memory);
            } else {
              drivetrain
                  .applyRequest(
                      () ->
                          drive
                              .withVelocityX(
                                  MaxSpeed.times(
                                      translation.calculate(
                                          drivetrain.getPose().getX(),
                                          drivetrain.getReefPosition().getPose().getX())))
                              .withVelocityY(
                                  MaxSpeed.times(
                                      translationy.calculate(
                                          drivetrain.getPose().getY(),
                                          drivetrain.getReefPosition().getPose().getY())))
                              .withRotationalRate(
                                  Constants.MaxAngularRate.times(
                                      rotation.calculate(
                                          drivetrain
                                              .getPose()
                                              .getRotation()
                                              .minus(
                                                  drivetrain
                                                      .getReefPosition()
                                                      .getPose()
                                                      .getRotation()
                                                      .toRotation2d())
                                              .minus(new Rotation2d(Degrees.of(180)))
                                              .getDegrees(),
                                          0))))
                  .schedule();
            }
            break;
          case PLACE:
            state = States.TOINTAKE;
            currentCommand = newAutoToIntake();
            memory.updateNextBest(getSimCoralPose().getTranslation());
            drivetrain.setReefPosition(memory.getCurrentTarget().getReefPosition());
            Pathfinding.setGoalPosition(
                drivetrain.getReefPosition().getPose().getTranslation().toTranslation2d());
            break;
          case TOINTAKE:
            state = States.AUTOINTAKE;
            hasCoral = false;
            arm.PREPINTAKE().schedule();
            intake.INTAKE().schedule();
            currentCommand = new ReturnTrue();
            break;
          case AUTOINTAKE:
            if (intakeIsReady() || firstRun) {
              state = States.TOREEF;
              arm.CORALIDLE().schedule();
              if (!firstRun) {
                if (drivetrain.isRightSource()) {
                  SimCoral.DropR().schedule();
                } else {
                  SimCoral.DropL().schedule();
                }
              } else {
                SimCoral.DropR().schedule();
                SimCoral.DropL().schedule();
              }
              firstRun = false;
              currentCommand = newAutoToReef();
            } else {
              runIntake();
            }
            break;
          default:
            break;
        }
        currentCommand.schedule();
      } else {
        switch (state) {
          case ALIGN:
            break;
          default:
            break;
        }
      }
    }
  }

  private boolean intakeIsReady() {
    return arm.isAtTarget()
        && intake.isAtTarget()
        && arm.getMode() == ArmPosition.PREPINTAKE
        && intake.getMode() == IntakePosition.STOW
        && hasCoral;
  }

  private void runIntake() {
    Pose2d coralPose = getSimCoralPose();
    if (drivetrain.isReadyForIntake(coralPose)) {
      if (arm.isAtTarget()
          && arm.getMode() == ArmPosition.PREPINTAKE
          && intake.isAtTarget()
          && intake.getMode() == IntakePosition.STOW
          && !hasCoral) {
        intake.INTAKE().schedule();
        Logger.recordOutput("Intake", 1);
      } else if (arm.isAtTarget()
          && intake.isAtTarget()
          && arm.getMode() == ArmPosition.PREPINTAKE
          && intake.getMode() == IntakePosition.INTAKE
          && !hasCoral) {
        intake.HANDOFF().schedule();
        Logger.recordOutput("Intake", 2);
      } else if (arm.isAtTarget()
          && intake.isAtTarget()
          && arm.getMode() == ArmPosition.PREPINTAKE
          && intake.getMode() == IntakePosition.HANDOFF
          && !hasCoral) {
        arm.INTAKE().schedule();
        hasCoral = true;
        Logger.recordOutput("Intake", 3);
      } else if (arm.isAtTarget()
          && intake.isAtTarget()
          && arm.getMode() == ArmPosition.INTAKE
          && intake.getMode() == IntakePosition.HANDOFF
          && hasCoral) {
        arm.PREPINTAKE().schedule();
        Logger.recordOutput("Intake", 4);
      } else if (arm.isAtTarget()
          && intake.isAtTarget()
          && arm.getMode() == ArmPosition.PREPINTAKE
          && intake.getMode() == IntakePosition.HANDOFF
          && hasCoral) {
        intake.STOW().schedule();
        Logger.recordOutput("Intake", 5);
      }
    }
    drivetrain
        .applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            translation.calculate(drivetrain.getPose().getX(), coralPose.getX())))
                    .withVelocityY(
                        MaxSpeed.times(
                            translationy.calculate(drivetrain.getPose().getY(), coralPose.getY())))
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            rotation.calculate(drivetrain.angleFromIntake(coralPose), 0))))
        .schedule();
  }

  private Command newAutoToIntake() {
    Logger.recordOutput(
        "Path",
        Pathfinding.getCurrentPath(
                constraints,
                new GoalEndState(
                    MetersPerSecond.of(0.0),
                    getSimCoralPose()
                        .getRotation()
                        .minus(new Rotation2d(Units.degreesToRadians(180)))))
            .getPathPoses()
            .toArray(
                new Pose2d
                    [Pathfinding.getCurrentPath(
                            constraints,
                            new GoalEndState(
                                MetersPerSecond.of(0.0),
                                getSimCoralPose()
                                    .getRotation()
                                    .minus(new Rotation2d(Units.degreesToRadians(180)))))
                        .getPathPoses()
                        .size()]));
    return AutoBuilder.followPath(
        Pathfinding.getCurrentPath(
            new PathConstraints(5.72, 14.7, 4.634, Units.degreesToRadians(1136)),
            new GoalEndState(
                MetersPerSecond.of(0.0),
                getSimCoralPose()
                    .getRotation()
                    .minus(new Rotation2d(Units.degreesToRadians(180))))));
  }

  private Command newAutoToReef() {
    Logger.recordOutput(
        "Path",
        Pathfinding.getCurrentPath(
                constraints,
                new GoalEndState(
                    MetersPerSecond.of(0.0),
                    drivetrain
                        .getReefPosition()
                        .getAngle()
                        .minus(new Rotation2d(Units.degreesToRadians(0)))))
            .getPathPoses()
            .toArray(
                new Pose2d
                    [Pathfinding.getCurrentPath(
                            constraints,
                            new GoalEndState(
                                MetersPerSecond.of(0.0),
                                drivetrain
                                    .getReefPosition()
                                    .getAngle()
                                    .minus(new Rotation2d(Units.degreesToRadians(0)))))
                        .getPathPoses()
                        .size()]));
    return AutoBuilder.followPath(
        Pathfinding.getCurrentPath(
            new PathConstraints(5.72, 14.7, 4.634, Units.degreesToRadians(1136)),
            new GoalEndState(
                MetersPerSecond.of(0),
                drivetrain
                    .getReefPosition()
                    .getAngle()
                    .plus(new Rotation2d(Units.degreesToRadians(180))))));
    // currentTarget =
    // new Pose2d(
    // drivetrain.getReefPosition().getPose().getX(),
    // drivetrain.getReefPosition().getPose().getY(),
    // drivetrain.getReefPosition().getPose().getAngle());
    // return AutoBuilder.pathfindToPose(currentTarget, constraints);
  }

  private Pose2d getSimCoralPose() {
    return SimCoral.getLeftPose()
                .getTranslation()
                .getDistance(drivetrain.getPose().getTranslation())
            > SimCoral.getRightPose()
                .getTranslation()
                .getDistance(AutoBuilder.getCurrentPose().getTranslation())
        ? SimCoral.getRightPose()
                    .plus(
                        new Transform2d(
                            0,
                            Units.inchesToMeters(32),
                            new Rotation2d(Units.degreesToRadians(90))))
                    .getTranslation()
                    .getDistance(drivetrain.getPose().getTranslation())
                > SimCoral.getRightPose()
                    .plus(
                        new Transform2d(
                            0,
                            Units.inchesToMeters(-32),
                            new Rotation2d(Units.degreesToRadians(-90))))
                    .getTranslation()
                    .getDistance(drivetrain.getPose().getTranslation())
            ? SimCoral.getRightPose()
                .plus(
                    new Transform2d(
                        0, Units.inchesToMeters(-32), new Rotation2d(Units.degreesToRadians(-90))))
            : SimCoral.getRightPose()
                .plus(
                    new Transform2d(
                        0, Units.inchesToMeters(32), new Rotation2d(Units.degreesToRadians(90))))
        : SimCoral.getLeftPose()
                    .plus(
                        new Transform2d(
                            0,
                            Units.inchesToMeters(32),
                            new Rotation2d(Units.degreesToRadians(90))))
                    .getTranslation()
                    .getDistance(drivetrain.getPose().getTranslation())
                > SimCoral.getLeftPose()
                    .plus(
                        new Transform2d(
                            0,
                            Units.inchesToMeters(-32),
                            new Rotation2d(Units.degreesToRadians(-90))))
                    .getTranslation()
                    .getDistance(drivetrain.getPose().getTranslation())
            ? SimCoral.getLeftPose()
                .plus(
                    new Transform2d(
                        0, Units.inchesToMeters(-32), new Rotation2d(Units.degreesToRadians(-90))))
            : SimCoral.getLeftPose()
                .plus(
                    new Transform2d(
                        0, Units.inchesToMeters(32), new Rotation2d(Units.degreesToRadians(90))));
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
