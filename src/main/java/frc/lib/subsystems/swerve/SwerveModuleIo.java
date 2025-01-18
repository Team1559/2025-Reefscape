package frc.lib.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIo {
    /**
     * Sets the speed of the wheel.
     * 
     * @param speed the desired wheel speed in meters per second
     */
    void setSpeed(double speed);

    /**
     * Sets the target direction for the wheel 
     * @param angle the disired angle of the wheel 
     */
    void setAngle(Rotation2d angle);
/**
 * @return The location of the wheel relative to the origin of the robot in meters
 */
    Translation2d getLocation();
/**
 * 
 * @return The current wheel speed in meters per second 
 */
    double getSpeed();
/**
 * The current heading of the wheel 
 * @return THe current heading of the wheel 
 */
    Rotation2d getAngle();
/**
 * The distance a wheel has spun 
 * @return The current distance of the wheel in meters 
 */
    double getDistanceTraveled();
/**
 * The heat of the steer motor 
 * @return The heat of the steer motor in Celsius 
 */
    double getSteerMotorTemperature();
/**
 * The heat of the drive motor 
 * @return The heat of the drive motor in Celsius 
 */
    double getDriveMotorTemperature();
/**
 *  returns the steer motor current  
 * @return The steer motor current amps 
 */
    double getSteerMotorCurrent();
/**
 * returns the drive motor current 
 * @return returns the drive motor current in amps 
 */
    double getDriveMotorCurrent();
/**
 * returns the name of the module
 * @return returns the name of the module
 */
    String getName();
/**
 * 
 * @return gets the setpoint, with both speed and angle
 */
    SwerveModuleState getSetpoint();

/**
 * 
 * @return gets the current state, with speed and angle
 */
    default SwerveModuleState getCurrentState(){
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    /**
     * it returns the swerve module
     * @return
     */
    default SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistanceTraveled(), getAngle());
    }

    default void setState(SwerveModuleState state) {
        setAngle(state.angle);
        setSpeed(state.speedMetersPerSecond);
    }
}