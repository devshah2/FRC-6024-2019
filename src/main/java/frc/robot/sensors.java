/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
/**
 * Add your docs here.
 */
public class sensors {
    public static AHRS navx = new AHRS(I2C.Port.kOnboard);
    public static int[] encoder=RobotMap.encoder;
		public static Encoder leftE = new Encoder(0,1, false);
		public static Encoder rightE = new Encoder(2,3, true);
		public static Encoder leftLE = new Encoder(0,1, false);
		public static Encoder rightLE = new Encoder(2,3, true);
    public static void init() {
		navx.reset();
		leftE.reset(); rightE.reset();
		leftLE.reset(); rightLE.reset();
    }
    public static void dashboardUpdate() {
		SmartDashboard.putNumber("encoder left", leftE.get());
		SmartDashboard.putNumber("encoder right", rightE.get());
		SmartDashboard.putNumber("navx angle", navx.getFusedHeading());
		SmartDashboard.putNumber("left speed", leftE.getRate());
		SmartDashboard.putNumber("right speed", rightE.getRate());
		SmartDashboard.putNumber("left lift speed", leftLE.getRate());
		SmartDashboard.putNumber("right lift speed", rightLE.getRate());
		SmartDashboard.putNumber("left lift", leftLE.get());
		SmartDashboard.putNumber("right lift", rightLE.get());
		SmartDashboard.putBoolean("navx connected", navx.isConnected());
		
    }
    public static double findHeading(double old, double cur) {
		double heading = (old-cur+360)%360;
		heading = heading > 180 ? heading - 360 : heading;
		return heading;
	}
}
