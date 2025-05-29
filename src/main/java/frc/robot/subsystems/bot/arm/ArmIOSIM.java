package frc.robot.subsystems.bot.arm;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmIOSIM extends ArmIOCTRE {
  private final DCMotor motor = DCMotor.getKrakenX60(2);
  private final PWMTalonFX talonfx = new PWMTalonFX(0);
  private final PIDController controller = new PIDController(0.5, 0, 0.01);
  private double setpoint = 50;

  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          motor,
          75,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(27), Units.lbsToKilograms(45)),
          Units.inchesToMeters(27),
          Units.degreesToRadians(10),
          Units.degreesToRadians(165),
          true,
          Units.degreesToRadians(50));

  private final LoggedMechanism2d m_mech2d = new LoggedMechanism2d(60.5, 6);
  private final LoggedMechanismRoot2d m_armPivot =
      m_mech2d.getRoot("ArmPivot", 30, Units.inchesToMeters(8));
  private final LoggedMechanismLigament2d m_armTower =
      m_armPivot.append(
          new LoggedMechanismLigament2d(
              "ArmTower", Units.inchesToMeters(6), -90, 12, new Color8Bit(Color.kBlueViolet)));
  private final LoggedMechanismLigament2d m_arm =
      m_armPivot.append(
          new LoggedMechanismLigament2d(
              "Arm",
              Units.inchesToMeters(27),
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              8,
              new Color8Bit(Color.kBlueViolet)));

  public ArmIOSIM() {}

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    talonfx.setVoltage(
        controller.calculate(Units.radiansToDegrees(m_armSim.getAngleRads()), setpoint));

    m_armSim.setInput(talonfx.get() * RobotController.getBatteryVoltage());

    m_armSim.update(0.020);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

    inputs.angle = Radians.of(m_armSim.getAngleRads());

    inputs.setpoint = Degrees.of(setpoint);

    SmartDashboard.putNumber("Arm Angle", inputs.angle.in(Degrees));

    Logger.recordOutput("Arm3", m_mech2d);
  }

  @Override
  public void setAngle(Angle angle) {
    setpoint = angle.in(Degrees);
  }

  public LoggedMechanismRoot2d getMechanismRoot() {
    return m_armPivot;
  }
}
