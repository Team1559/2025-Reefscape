package frc.lib.commands;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.subsystems.DriverAssist;
import frc.lib.subsystems.swerve.SwerveDrive;
import frc.lib.subsystems.swerve.SwerveDrive.SwerveConstraints;

public class AutoSwerveAlignmentCommand extends DeferredCommand{
    public AutoSwerveAlignmentCommand(SwerveDrive drivetrain, SwerveConstraints swerveConstraints, DriverAssist driverAssist, Pose2d endPoint, Translation2d approach){
        super(() -> getCommand(drivetrain, swerveConstraints, driverAssist, endPoint, approach), Set.of(driverAssist, drivetrain));
        
        //System.out.println("Whoopee");
    }

    private static Command getCommand(SwerveDrive drivetrain, SwerveConstraints swerveConstraints, DriverAssist driverAssist, Pose2d endPoint, Translation2d approach){
        Pose2d midPoint = new Pose2d(endPoint.getTranslation().minus(approach), endPoint.getRotation());
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivetrain.getPosition(), midPoint, endPoint);
        PathConstraints constraints = new PathConstraints(swerveConstraints.getMaxLinearVelocity(), swerveConstraints.getMaxLinerAccel(), swerveConstraints.getMaxAngularVelocity(), swerveConstraints.getMaxAngularAccel());
        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0,  Rotation2d.fromDegrees(-180)));
        Command command = AutoBuilder.followPath(path);
        System.out.println("Double Whoopeeeeeeeee");
        return driverAssist.configure(command);
    }

}