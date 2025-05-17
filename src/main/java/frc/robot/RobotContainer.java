package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.State.DriveStates;
import frc.robot.State.GamePieceStates;
import frc.robot.State.GameStates;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOCTRE;
import frc.robot.subsystems.arm.ArmIOSIM;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOCTRE;
import frc.robot.subsystems.intake.IntakeIOSIM;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSIM;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts;
  private final TunableController joystick =
      new TunableController(0).withControllerType(TunableControllerType.QUADRATIC);

  private final LoggedDashboardChooser<Command> autoChooser;

  public final Drive drivetrain;
  // CTRE Default Drive Request
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final Vision vision;
  private final Arm arm;
  private final Intake intake;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public RobotContainer() {
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);
        arm = new Arm(new ArmIOCTRE());
        intake = new Intake(new IntakeIOCTRE());

        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIOPhotonVision(
                    "FrontLeft",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(330)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "FrontRight",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            -Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(30)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "BackLeft",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(150)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVision(
                    "BackRight",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            -Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(210)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);
        arm = new Arm(new ArmIOSIM());
        intake = new Intake(new IntakeIOSIM());

        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIOPhotonVisionSIM(
                    "FrontLeft",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(330)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVisionSIM(
                    "FrontRight",
                    new Transform3d(
                        new Translation3d(
                            Units.inchesToMeters(10.066),
                            -Units.inchesToMeters(11.959),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(30)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVisionSIM(
                    "BackLeft",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(150)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters),
                new VisionIOPhotonVisionSIM(
                    "BackRight",
                    new Transform3d(
                        new Translation3d(
                            -Units.inchesToMeters(9.896),
                            -Units.inchesToMeters(10.938),
                            Units.inchesToMeters(8.55647482)), // IN METERS
                        new Rotation3d(
                            0,
                            Units.degreesToRadians(-25.16683805),
                            Units.degreesToRadians(210)) // IN RADIANS
                        ),
                    drivetrain::getVisionParameters));

        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain = new Drive(new DriveIO() {});
        arm = new Arm(new ArmIO() {});
        intake = new Intake(new IntakeIO() {});

        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        MaxSpeed.times(
                            -joystick
                                .customLeft()
                                .getY())) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        MaxSpeed.times(
                            -joystick.customLeft().getX())) // Drive left with negative X (left)
                    .withRotationalRate(
                        Constants.MaxAngularRate.times(
                            -joystick
                                .customRight()
                                .getX())))); // Drive counterclockwise with negative X (left)

    joystick.back().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));

    Trigger swichToAuto = new Trigger(() -> DriverStation.isDisabled() && (DriverStation.isAutonomous() || DriverStation.isTest()));
    Trigger swichToTeleop = new Trigger(() -> DriverStation.isDisabled() && DriverStation.isTeleop());
    Trigger endgame = new Trigger(() -> DriverStation.getMatchTime() < 20);

    swichToAuto.onTrue(new InstantCommand(() -> State.setDriveState(DriveStates.AUTOTOREEF)).alongWith(new InstantCommand(() -> State.setGamePieceState(GamePieceStates.CORAL)).alongWith(new InstantCommand(() -> State.setGameState(GameStates.AUTO)))));
    swichToTeleop.onTrue(new InstantCommand(() -> State.setDriveState(DriveStates.IDLE)).alongWith(new InstantCommand(() -> State.setGamePieceState(GamePieceStates.NONE)).alongWith(new InstantCommand(() -> State.setGameState(GameStates.TELEOP)))));
    endgame.onTrue(new InstantCommand(() -> State.setGameState(GameStates.ENDGAME)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
