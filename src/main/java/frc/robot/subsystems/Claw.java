// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  Compressor compressor = new Compressor(Constants.OperatorConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH);
  DoubleSolenoid solenoid = new DoubleSolenoid(Constants.OperatorConstants.COMPRESSOR_ID, PneumaticsModuleType.REVPH, Constants.OperatorConstants.SOLENOID_ID_IN, Constants.OperatorConstants.SOLENOID_ID_OUT);
  Boolean extended = false;
  /** Creates a new Claw. */
  public Claw() {
    compressor.enableDigital();
  }
/**Toggles the solenoid.
 * 
 */
  public void solenoidToggle(){
    if(extended)
      solenoid.set(Value.kForward);
    else
      solenoid.set(Value.kReverse);
    extended = !extended;
  }  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
