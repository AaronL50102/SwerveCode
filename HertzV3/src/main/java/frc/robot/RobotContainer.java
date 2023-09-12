
package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveJoystickCMD;
import frc.robot.commands.SwerveZeroHeading;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Joystick driverJoystick = new Joystick(OperatorConstants.DriveControllerPort);


  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(
        swerveSubsystem,
        () -> -driverJoystick.getRawAxis(OperatorConstants.DriverYAxis),
        () -> driverJoystick.getRawAxis(OperatorConstants.DriverXAxis),
        () -> driverJoystick.getRawAxis(OperatorConstants.kDriverRotAxis),
        () -> !driverJoystick.getRawButton(OperatorConstants.DriverFieldOrientedButton)
    ));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2).onTrue(new SwerveZeroHeading(swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
