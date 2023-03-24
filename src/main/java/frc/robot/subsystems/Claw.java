// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  Relay spike = new Relay(Constants.OperatorConstants.spikeRelayID);
  Compressor compressor = new Compressor(Constants.OperatorConstants.COMPRESSOR_ID, PneumaticsModuleType.CTREPCM);
  DoubleSolenoid solenoid = new DoubleSolenoid(Constants.OperatorConstants.COMPRESSOR_ID, PneumaticsModuleType.CTREPCM, Constants.OperatorConstants.SOLENOID_ID_IN, Constants.OperatorConstants.SOLENOID_ID_OUT);
  Boolean extended = true;
  Boolean sucky = false;
  /** Creates a new Claw. */
  public Claw() {
    
    compressor.enableDigital();
    spike.set(Relay.Value.kOff);
  }
/**Toggles the solenoid.
 * 
 */
  public void solenoidToggle(){
    if(extended)
      solenoid.set(DoubleSolenoid.Value.kForward);
    else
      solenoid.set(DoubleSolenoid.Value.kReverse);
    extended = !extended;
  }  

  public void plantSpike(){
    if(sucky){
      spike.set(Relay.Value.kOn);
      spike.set(Relay.Value.kForward);
    }
    else
      spike.set(Relay.Value.kOff);
    sucky = !sucky;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
