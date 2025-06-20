package frc.robot;

import frc.robot.subsystems.bot.Bot;
import frc.robot.subsystems.bot.Bot.BotType;

public class RobotContainer {
  private final boolean testingBot = false;

  private final BotType botType = BotType.CYCLES;

  @SuppressWarnings("unused")
  private final Bot bot;
  // private final RealDriver driver;

  public RobotContainer() {
    bot = new Bot(testingBot, botType);
    // driver = new RealDriver(testingBot);

    configureBindings();
  }

  private void configureBindings() {}
}
