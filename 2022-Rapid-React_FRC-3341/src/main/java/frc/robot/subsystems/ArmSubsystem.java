// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util;

public class ArmSubsystem extends SubsystemBase {
  private WPI_TalonSRX _pivotTalon;
  private WPI_TalonSRX _winchTalon;
  private boolean _isLeft;
  private boolean _isFront;
  private double _pitchMultiplier;
  private double _rollMultiplier;
  private double _gravityComp;

  private AnalogInput _reflectiveSensor;
  private final AHRS _ahrs;

  private boolean _toStabilize;

  private double _wPower;
  private double _pPower;

  private boolean _isDetected;
  private double _threshold; // analog reading threshold
  private int _count;
  private int _targetPos;
  
  /** Creates a new Arm. */
  public ArmSubsystem(int pivotCANID, int winchCANID, int analogPort, 
              boolean invertPivotMC, boolean invertWinchMC, 
              boolean invertedPivotEncoder, boolean invertedWinchEncoder,
              boolean isLeft, boolean isFront, double gravityComp) {

    _pivotTalon = new WPI_TalonSRX(pivotCANID);
    _winchTalon = new WPI_TalonSRX(winchCANID);

    _pivotTalon.setInverted(invertPivotMC);
    _winchTalon.setInverted(invertWinchMC);

    _isFront = isFront;
    _isLeft = isLeft;

    _pitchMultiplier = isFront() ? -1 : 1;
    _rollMultiplier = isLeft() ? 1 : -1;

    _pivotTalon.setNeutralMode(NeutralMode.Brake); // could be changed to coast
    _winchTalon.setNeutralMode(NeutralMode.Brake);

    // encoders
    _pivotTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _winchTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    _pivotTalon.setSensorPhase(invertedPivotEncoder);
    _winchTalon.setSensorPhase(invertedWinchEncoder);

    // limit switches
    _pivotTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed); //figure out why pivot worked on normally open
    _pivotTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    
    _reflectiveSensor = new AnalogInput(analogPort);
    _ahrs = new AHRS(SPI.Port.kMXP);

    _gravityComp = gravityComp;
    _toStabilize = false;

    _pPower = _wPower = 0;
    pivot(_pPower);
    extend(_wPower);

    _isDetected = false;
    _count = 0;
    _threshold = 800;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    stabilizeWithAHRS();

    moveToReflectiveMarkerPos();
    countReflectiveMarkers();
  }

  public void pivot(double power) {
    _pPower = power;
    _pivotTalon.set(ControlMode.PercentOutput, Math.signum(_pPower) 
                                                * MathUtil.clamp(Math.abs(_pPower), 
                                                              Constants.Climber.MinPivotPower, 
                                                              Constants.Climber.MaxPivotPower));
  }

  public void extend(double power) {
    _wPower = power + _gravityComp;
    
    _winchTalon.set(ControlMode.PercentOutput, Math.signum(_wPower) 
                                                * MathUtil.clamp(Math.abs(_wPower), 
                                                              Constants.Climber.MinExtendPower, 
                                                              Constants.Climber.MaxExtendPower));
  }

  public void setGravityComp(double gravityComp) {
      _gravityComp = gravityComp;
  }

  public double getPivotEncoderPosition() {
      return _pivotTalon.getSelectedSensorPosition();
  }

  public double getWinchEncoderPosition() {
      return _winchTalon.getSelectedSensorPosition();
  }
  
  public double getPivotEncoderVelocity() {
    return _pivotTalon.getSelectedSensorVelocity();
  }

  public double getWinchEncoderVelociy() {
      return _winchTalon.getSelectedSensorVelocity();
  }
  

  public boolean isLeft() {
      return _isLeft;
  }

  public boolean isFront() {
      return _isFront;
  }

  public double pitchMultiplier() {
      return _pitchMultiplier;
  }

  public double rollMultiplier() {
      return _rollMultiplier;
  }

  public double getWinchDirection() {
      return Math.signum(_winchTalon.getSelectedSensorVelocity());
  }

  public double getReflectiveSensorVal() {
      return _reflectiveSensor.getValue();
  }

  public void enableAHRSStabilization(boolean toStabilize) {
    if(_toStabilize == true && toStabilize == false) {
      this.extend(0);
    }
    _toStabilize = toStabilize;
  }

  public boolean isAHRSStabilizing() {
    return _toStabilize;
  }

  private void stabilizeWithAHRS() {
    if(!_toStabilize) {
      return;
    }

    //balancing code
    double _pitch = Util.deadband(_ahrs.getPitch(), Constants.AHRS.MinDeadband, Constants.AHRS.MaxDeadband);
    double _roll = Util.deadband(_ahrs.getRoll(), Constants.AHRS.MinDeadband, Constants.AHRS.MaxDeadband);
    
    double _normalizedPitch = _pitch / (Math.abs(_pitch) + Math.abs(_roll));
    double _normalizedRoll = _roll / (Math.abs(_pitch) + Math.abs(_roll));

    double _impact = (this.pitchMultiplier() * _normalizedPitch) + (this.rollMultiplier() * _normalizedRoll);

    this.extend(_impact);
  }

  private void countReflectiveMarkers() {
    double _input = getReflectiveSensorVal();

    if(_input > _threshold) {
      if(_wPower > 0 && _isDetected == false) {
       _count++;
      }
      _isDetected = true;
    }
    else {
      if(_wPower < 0 && _isDetected == true) {
        _count--;
      }
      _isDetected = false;
    }
  }

  public int getRelfectiveMarkerCount() {
    return _count;
  }

  public void setReflectiveMarkerPos(int pos) {
    _targetPos = pos;
  }

  private void moveToReflectiveMarkerPos() {
    if(_toStabilize) return;

    if(_count < _targetPos) {
      _wPower = 0.5;
    }
    else if(_count > _targetPos) {
      _wPower = -0.5;
    }    
  }
  
  public void resetReflectiveMarkerStatus() {
    _count = 0;
  }  
}
