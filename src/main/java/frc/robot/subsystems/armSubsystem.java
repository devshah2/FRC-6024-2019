/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.sensors;
import frc.robot.commands.*;
import frc.robot.*;
/**
 * Add your docs here.
 */
public class armSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX arm=new TalonSRX(RobotMap.arm);
  TalonSRX intakeB=new TalonSRX(RobotMap.intakeBot);
  TalonSRX intakeT=new TalonSRX(RobotMap.intakeTop);


  public void setup(){
    sensors.navx.reset();
    arm.set(ControlMode.PercentOutput,0);
    arm.setNeutralMode(NeutralMode.Brake);

    intakeB.set(ControlMode.PercentOutput,0);
    intakeB.setNeutralMode(NeutralMode.Brake);

    intakeT.set(ControlMode.PercentOutput,0);
    intakeT.setNeutralMode(NeutralMode.Brake);

    // intakeT.setInverted(true);
  }

  public void moveArm(double forward){
    arm.set(ControlMode.PercentOutput, forward);
  }

  public void intake(double speed){
    intakeB.set(ControlMode.PercentOutput, speed);
    intakeT.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new armCommand());
  }

  public void teleop(){
    double multiplier=OI.other.getRawButton(1)?1.5:1;
    if(OI.other.getY()>0.3) moveArm(0.2*multiplier);
    else if(OI.other.getY()<-0.3) moveArm(-0.2*multiplier);
    else moveArm(0);
    if(OI.other.getRawButton(5)) intake(-0.6*multiplier);
    else if(OI.other.getRawButton(6)) intake(0.6*multiplier);
    else intake(0);
  }

}
