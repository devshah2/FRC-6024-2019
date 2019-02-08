/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.sensors;
import frc.robot.commands.driveCommand;

/**
 * Add your docs here.
 */
public class driveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX lf=new TalonSRX(RobotMap.lf);
  TalonSRX rf=new TalonSRX(RobotMap.rf);
  TalonSRX rb=new TalonSRX(RobotMap.rb);
  TalonSRX lb=new TalonSRX(RobotMap.lb);

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("SmartDashboard");
  NetworkTableEntry xEntry = table.getEntry("x");


  public void setup(){
    sensors.navx.reset();
    lb.set(ControlMode.PercentOutput,0);
    lf.set(ControlMode.PercentOutput,0);
    rb.set(ControlMode.PercentOutput,0);
    rf.set(ControlMode.PercentOutput,0);

    lb.setNeutralMode(NeutralMode.Brake);
    lf.set(ControlMode.PercentOutput,0);
    rb.set(ControlMode.PercentOutput,0);
    rf.set(ControlMode.PercentOutput,0);

    lb.setInverted(true);
    lf.setInverted(true);
  }
  public void move(double forward,double turn){
    forward=forward*-1;
    turn=turn*-1;
    lf.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
    lb.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
    rf.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
    rb.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
  }

  public void move(double forwardL,double forwardR,double turn){
    lf.set(ControlMode.PercentOutput, forwardL, DemandType.ArbitraryFeedForward, turn);
    lb.set(ControlMode.PercentOutput, forwardL, DemandType.ArbitraryFeedForward, turn);
    rf.set(ControlMode.PercentOutput, forwardR, DemandType.ArbitraryFeedForward, -turn);
    rb.set(ControlMode.PercentOutput, forwardR, DemandType.ArbitraryFeedForward, -turn);
  }
  public void turnToAngle(double target) {
    double error = 100;
    while (Math.abs(error) > 3 && OI.driveStick.getPOV() != -1) {
      double x = sensors.navx.getFusedHeading();
      error = x > target + 180 ? (target + 360) - x : target - x;
      double one = 1;
      move(0, Math.max(-one, Math.min(error * 0.005, one)));
    }
    move(0, 0);
  }


  double Deadband(double value) {
    if (value >= +0.2)
      return value;
    if (value <= -0.2)
      return value;
    return 0;
  }

  public void teleop(){
    while (OI.driveStick.getRawButton(1)) {
      move(0,0);
    }

    while (OI.driveStick.getRawButton(3)) {
      double x = xEntry.getDouble(0);
      double error = x - 80;
      double proportional = 0.002 * error;
      if (x == -1)
        x = 80;
      move((double) -0.3, (double) proportional);
    }

    while (OI.driveStick.getRawButton(5)) {
      move(0.3,0);
    }

    double angle = OI.driveStick.getPOV();
    if (angle != -1) {
      turnToAngle(angle);
    }

    double forward = -Deadband(OI.driveStick.getY() * 0.5);
    double turn = Deadband(OI.driveStick.getTwist() * 0.5);
    move(forward, turn);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new driveCommand());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
