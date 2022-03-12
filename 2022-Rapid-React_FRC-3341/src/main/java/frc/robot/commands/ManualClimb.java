// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util;
import frc.robot.subsystems.ArmSubsystem;

public class ManualClimb extends CommandBase {
  /** Creates a new ManualClimb. */
  private ArmSubsystem _lfArm;
  private ArmSubsystem _rfArm;
  private ArmSubsystem _lrArm;
  private ArmSubsystem _rrArm;
  private ArmSubsystem _activeArm;

  private Joystick _joystick;

  private boolean _balancingOn;
  
  private double _powerMultiplier;
  private int _position;

  public ManualClimb(ArmSubsystem lfArm, ArmSubsystem rfArm, ArmSubsystem lrArm, ArmSubsystem rrArm, Joystick joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    _lfArm = lfArm;
    _rfArm = rfArm;
    _lrArm = lrArm;
    _rrArm = rrArm;
    _activeArm = _lfArm;

    _joystick = joystick;

    _balancingOn = false;

    _powerMultiplier = 0.4;
    _position = 3;

    addRequirements(_lfArm, _rfArm, _lrArm, _rrArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_joystick.getRawButtonPressed(5)) {
      _activeArm = _lfArm;
    }
    else if(_joystick.getRawButtonPressed(6)) {
      _activeArm = _rfArm;
    }
    else if(_joystick.getRawButtonPressed(3)) {
      _activeArm = _lrArm;
    }
    if(_joystick.getRawButtonPressed(4)) {
      _activeArm = _rrArm;
    }
    
    if(_joystick.getRawButtonPressed(1)) {
      _balancingOn = true;
    }
    else if(_joystick.getRawButtonPressed(2)) {   
      _balancingOn = false;
    }

    updateStabilization();

    if(_joystick.getRawButton(7)) {
      _powerMultiplier = 0.4;
    }
    else if(_joystick.getRawButton(8)) {
      _powerMultiplier = 0.6;
    }
    else if(_joystick.getRawButton(9)) {
      _powerMultiplier = 0.8;
    }
    else if(_joystick.getRawButton(10)) {
      _powerMultiplier = 1.0;
    }
    else {
      _powerMultiplier = 0.3;
    }

    if(_joystick.getRawButtonPressed(11)) {
      _activeArm.setReflectiveMarkerPos(_position);
    }

    if(_joystick.getRawButton(12)) {
      _activeArm.resetReflectiveMarkerStatus();
    }

    if(_activeArm.isAHRSStabilizing() == false) {
      double pivotPower = Util.deadband(-1 * 0.3 * _joystick.getY(), -0.07, 0.07);
      double winchPower = Util.deadband(_powerMultiplier * -1 * _joystick.getX(), -0.07, 0.07);
    
      _activeArm.pivot(pivotPower);
      _activeArm.extend(winchPower);
    }
  }

  public void updateStabilization() {
    _activeArm.enableAHRSStabilization(false);

    if (_lfArm != _activeArm) _lfArm.enableAHRSStabilization(_balancingOn);
    if (_rfArm != _activeArm) _rfArm.enableAHRSStabilization(_balancingOn);
    if (_lrArm != _activeArm) _lrArm.enableAHRSStabilization(_balancingOn);
    if (_rrArm != _activeArm) _rrArm.enableAHRSStabilization(_balancingOn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
