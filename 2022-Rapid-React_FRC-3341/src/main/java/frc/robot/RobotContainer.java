// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ManualClimb;
import frc.robot.subsystems.ArmSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private ArmSubsystem _lfArm, _rfArm, _lrArm, _rrArm;
  private ManualClimb _manualClimb;

  private Joystick _joystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    _lfArm = new ArmSubsystem(Constants.CANID.Climber.LeftFrontArmPivot, Constants.CANID.Climber.LeftFrontArmWinch, 
                    Constants.AnalogPort.Zero, true, true, false, false, false, true, 0);
    _rfArm = new ArmSubsystem(Constants.CANID.Climber.RightFrontArmPivot, Constants.CANID.Climber.RightFrontArmWinch, 
                    Constants.AnalogPort.One, false, false, false, false, false, false, 0);
    _lrArm = new ArmSubsystem(Constants.CANID.Climber.LeftRearArmPivot, Constants.CANID.Climber.LeftRearArmWinch, 
                    Constants.AnalogPort.Two, true, true, false, false, true, true, 0);
    _rrArm = new ArmSubsystem(Constants.CANID.Climber.RightRearArmPivot, Constants.CANID.Climber.RightRearArmWinch, 
                    Constants.AnalogPort.Three, false, false, false, false, true, false, 0);

    _joystick = new Joystick(Constants.USBOrder.Two);
    _manualClimb = new ManualClimb(_lfArm, _rfArm, _lrArm, _rrArm, _joystick);

    _lfArm.setDefaultCommand(_manualClimb);
    _rfArm.setDefaultCommand(_manualClimb);
    _lrArm.setDefaultCommand(_manualClimb);
    _rrArm.setDefaultCommand(_manualClimb);
      
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
