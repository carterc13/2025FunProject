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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveHandler;
import frc.robot.commands.PathFind;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOCTRE;
import frc.robot.subsystems.arm.ArmIOSIM;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOCTRE;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOCTRE;
import frc.robot.subsystems.elevator.ElevatorIOSIM;
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

  private final Vision vision;
  private final Arm arm;
  private final Intake intake;
  private final Elevator elevator;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed.times(0.1))
          .withRotationalDeadband(Constants.MaxAngularRate.times(0.1)) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public RobotContainer() {
    DriveIOCTRE currentDriveTrain = TunerConstants.createDrivetrain();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drivetrain = new Drive(currentDriveTrain);
        arm = new Arm(new ArmIOCTRE());
        intake = new Intake(new IntakeIOCTRE());
        elevator = new Elevator(new ElevatorIOCTRE());

        vision =
            new Vision(
                drivetrain::addVisionData,
                new VisionIOPhotonVision(
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
                new VisionIOPhotonVision(
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
                new VisionIOPhotonVision(
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
                new VisionIOPhotonVision(
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
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drivetrain = new Drive(currentDriveTrain);
        ArmIOSIM temp = new ArmIOSIM();
        elevator = new Elevator(new ElevatorIOSIM(temp.getMechanismRoot()));
        arm = new Arm(temp);
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

        break;

      default:
        // Replayed robot, disable IO implementations
        drivetrain = new Drive(new DriveIO() {});
        arm = new Arm(new ArmIO() {});
        intake = new Intake(new IntakeIO() {});
        elevator = new Elevator(new ElevatorIO() {});

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
    drivetrain.setDefaultCommand(
        new DriveHandler(
            drivetrain,
            () -> -joystick.customLeft().getY(),
            () -> -joystick.customLeft().getX(),
            () -> -joystick.customRight().getX()));

    joystick.back().onTrue(Commands.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)));

    joystick.pov(0).onTrue(drivetrain.setDriveToALIGN());
    joystick.pov(0).onFalse(drivetrain.setDriveToIDLE());

    // joystick.a().whileTrue(new PathFind(drivetrain));

    // joystick
    //         .a()
    //         .onTrue(
    //                 AutoBuilder.pathfindThenFollowPath(
    //                         new PathPlannerPath(
    //                                 PathPlannerPath.waypointsFromPoses(
    //                                         new Pose2d[] {
    //                                                 drivetrain.getPose(),
    //                                                 new Pose2d(10, 5,
    // drivetrain.getPose().getRotation())
    //                                         }),
    //                                 constraints,
    //                                 null,
    //                                 new GoalEndState(0, Rotation2d.fromDegrees(0))),
    //                         constraints));

    // Pose2d targetPose = new Pose2d(10, 5, new Rotation2d(180));

    // // Create the constraints to use while pathfinding
    // PathConstraints constraints =
    //     new PathConstraints(5.72, 14.7, 4.634, Units.degreesToRadians(1136));

    // // Since AutoBuilder is configured, we can use it to build pathfinding commands
    // Command pathfindingCommand =
    //     AutoBuilder.pathfindToPose(
    //         targetPose, constraints, 0.0 // Goal end velocity in meters/sec
    //         );

    joystick.a().onTrue(new PathFind(drivetrain));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
