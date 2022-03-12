// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Util {
    public static double deadband(double val, double min, double max) {
        if(val >= min && val <= max) {
            return 0.0;
        }
        else {
            return val;
        }
    }
}
