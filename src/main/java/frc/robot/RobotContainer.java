// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.commands.Autos;
//import frc.robot.commands.ExampleCommand;
import frc.robot.util.DriverOI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // Define robot subsystems here!
  private final Drivetrain m_Drivetrain;
  private final DriverOI m_DriverOI;
  private final Autonomous m_Autonomous;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_Drivetrain = Drivetrain.getInstance();
    m_Drivetrain.setDefaultCommand(new SwerveDriveCommand());
    m_Autonomous = Autonomous.getInstance();
    m_DriverOI = DriverOI.getInstance();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_Autonomous.returnAutonomousCommand();
  }
}
