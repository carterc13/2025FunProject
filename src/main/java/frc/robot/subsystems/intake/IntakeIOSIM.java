package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeIOSIM extends IntakeIOCTRE {
  private final DCMotor motor = DCMotor.getKrakenX60(1);
  private final PWMTalonFX talonfx = new PWMTalonFX(1);
  private final PIDController controller = new PIDController(1, 0, 0.075);

  private double setpoint = 210;

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          motor,
          5,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(12), Units.lbsToKilograms(20)),
          Units.inchesToMeters(12),
          Units.degreesToRadians(-45),
          Units.degreesToRadians(210),
          true,
          Units.degreesToRadians(210));

  private final LoggedMechanism2d m_mech2d = new LoggedMechanism2d(59.375, 60);
  private final LoggedMechanismRoot2d m_armPivot =
      m_mech2d.getRoot("ArmPivot", 30, Units.inchesToMeters(12));
  private final LoggedMechanismLigament2d m_armTower =
      m_armPivot.append(
          new LoggedMechanismLigament2d(
              "ArmTower", Units.inchesToMeters(10), -90, 4, new Color8Bit(Color.kRed)));
  private final LoggedMechanismLigament2d m_arm =
      m_armPivot.append(
          new LoggedMechanismLigament2d(
              "Arm",
              Units.inchesToMeters(15),
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              4,
              new Color8Bit(Color.kRed)));

  public IntakeIOSIM() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    talonfx.setVoltage(
        controller.calculate(Units.radiansToDegrees(m_armSim.getAngleRads()), setpoint));

    m_armSim.setInput(talonfx.get() * RobotController.getBatteryVoltage());

    m_armSim.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

    inputs.angle = Radians.of(m_armSim.getAngleRads());

    inputs.setpoint = Degrees.of(setpoint);

    Logger.recordOutput("Arm2", m_mech2d);
  }

  @Override
  public void setAngle(Angle angle) {
    setpoint = angle.in(Degrees);
  }
}
