// package frc.robot.commands;

// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Meters;

// import java.util.Optional;
// import java.util.function.DoubleSupplier;

// import org.photonvision.PhotonUtils;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.PathConstraints;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.Vision;
// import edu.wpi.first.units.DistanceUnit;
// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.math.util.Units;


// public class AutoAlignL1 extends Command{
//     SwerveSubsystem swerve;
//     public Command pathfindingCommand;

//     public AutoAlignL1(SwerveSubsystem swerve) {
//         this.swerve = swerve;
//         addRequirements(swerve);
//     }

//     @Override
//     public void initialize() {
//         // PPHolonomicDriveController.overrideXFeedback() -> {
//             // Pose2d pose = swerve.getPose();
//             // double xVel = alignToPoseCommand.pidControllerX.calculate(pose.getX());

//             // return xVel;
        
//         pathfindingCommand = testAutoAnd();
//         if (pathfindingCommand != null) {
//             pathfindingCommand.schedule();
//         }
//     }

// public Command testAutoAnd(){
//   PathConstraints constraints = new PathConstraints(
//         5.2155, 6.0,
//         Units.degreesToRadians(540), Units.degreesToRadians(720));
//         SmartDashboard.putNumber("closest id", RobotContainer.m_vision.closestTag());

// // Since AutoBuilder is configured, we can use it to build pathfinding commands
// // PathPlannerPath path = PathPlannerPath.
// // return AutoBuilder.

// return AutoBuilder.pathfindToPose(
        
//   getReefPose(() -> RobotContainer.m_vision.closestTag()),
//         constraints);
//   // return null;
// }


// // Pose at midpoint between tags 18 and 21 (which are opposite on blue reef)
// private static final Translation2d REEF_CENTER_BLUE = Vision.fieldLayout.getTagPose(18).get().toPose2d().getTranslation()
//     .plus(Vision.fieldLayout.getTagPose(21).get().toPose2d().getTranslation()).div(2);

// // Pose at midpoint between tags 10 and 7 (which are opposite on red reef)
// private static final Translation2d REEF_CENTER_RED = Vision.fieldLayout.getTagPose(10).get().toPose2d().getTranslation()
//     .plus(Vision.fieldLayout.getTagPose(7).get().toPose2d().getTranslation()).div(2);

// private static boolean flipToRed; // whether to use red reef (otherwise blue)

// // Distance from center of robot to center of reef



// public static final Distance DISTANCE_TO_REEF = Inches.of(27 / 2).plus(Inches.of(3.5)); // 4 is bumper thickness


// // Found by taking distance from tag 18 to center and adding offset from reef
// private static final Distance REEF_APOTHEM = Meters.of(
//         Vision.fieldLayout.getTagPose(18).get().toPose2d().getTranslation().getDistance(REEF_CENTER_BLUE))
//         .plus(DISTANCE_TO_REEF);

// // translation to move from centered on a side to scoring position for the left branch
// private static final Translation2d CENTERED_TO_LEFT_BRANCH = new Translation2d(Meters.of(0),
//         Inches.of(12.94 / 2));

// /**
//  * Calculates the pose of the robot for scoring on a branch or trough.
//  *
//  * @param side The side of the reef (0 for left, increases clockwise).
//  * @param relativePos The relative position on the reef (-1 for right branch, 0 for center, 1 for left branch).
//  * @return The calculated Pose2d for scoring.
//  */
// public static Pose2d getReefPose(DoubleSupplier side) {
//     // determine whether to use red or blue reef position
//     flipToRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

//     // initially do all calculations from blue, then flip later
//     Translation2d reefCenter = REEF_CENTER_BLUE;

//     // robot position centered on close reef side
//     Translation2d translation = reefCenter.plus(new Translation2d(REEF_APOTHEM.unaryMinus(), Meters.zero()));
//     // translate to correct branch (left, right, center)
//     translation = translation.plus(CENTERED_TO_LEFT_BRANCH.times(0));
//     // rotate to correct side
//     translation = translation.rotateAround(reefCenter, Rotation2d.fromDegrees(-60 * side.getAsDouble()));

//     // make pose from translation and correct rotation
//     Pose2d reefPose = new Pose2d(translation,
//             Rotation2d.fromDegrees(-60 * side.getAsDouble()+90));

//     if (flipToRed) {
//         reefPose = flipPose(reefPose);
//     }

//     return reefPose;
// }

// private static Pose2d flipPose(Pose2d pose) {
//     Translation2d center = REEF_CENTER_BLUE.interpolate(REEF_CENTER_RED, 0.5);
//     Translation2d poseTranslation = pose.getTranslation();
//     poseTranslation = poseTranslation.rotateAround(center, Rotation2d.k180deg);
//     return new Pose2d(poseTranslation, pose.getRotation().rotateBy(Rotation2d.k180deg));
// }

















//     @Override
//     public void execute() {
//     }

//     @Override
//     public void end(boolean interrupted) {

//     }

//     public void encc() {
//         // pathfindingCommand.cancel();
//     }

//     @Override
//     public boolean isFinished() {
//         if (!RobotContainer.m_driverController.povLeft().getAsBoolean() && !RobotContainer.m_driverController.povRight().getAsBoolean()) {
//             swerve.stopDrive();
//             return true;
//         }
//         return false;
//     }
// }
