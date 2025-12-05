package org.firstinspires.ftc.teamcode.opmodes.poses

import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.opmodes.commands.releasePattern
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import org.firstinspires.ftc.teamcode.subsystems.huskylens.HuskyLens
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter
import org.firstinspires.ftc.teamcode.util.IndexTracker
import kotlin.math.PI

const val robotWidth = 18.0
const val robotLength = 13.38
val scorePosition = Vector.fromCartesian(144.0, 72.0)

fun getScoreDistance(position: Vector, isRed: Boolean = false) =
    (scorePosition.mirroredIf(isRed) - position).length


fun getScoreAngle(position: Vector, isRed: Boolean = false) =
    (scorePosition.mirroredIf(isRed) - position).angle


fun getScorePose(position: Vector, isRed: Boolean = false) =
    Pose(position.x, position.y, (scorePosition.mirroredIf(isRed) - position).angle)

val farPose = getScorePose(Vector.fromCartesian(12.0, 12.0))
val closePose = getScorePose(Vector.fromCartesian(84.0, 12.0))
val closeDistance = getScoreDistance(Vector.fromPose(closePose))

val farDistance = getScoreDistance(Vector.fromPose(farPose))
val startPose = Pose(robotLength/2.0, robotWidth/2.0, 0.0)
val parkPose = Pose(24.0+(robotLength/2.0), -24.0-(robotWidth/2.0), PI)


fun inLaunchZone(pose: Pose) = (pose.x - 72.0 >= pose.y && pose.x - 72.0 >= -pose.y)
                            || (pose.x <= pose.y + 24.0 && pose.x <= -pose.y + 24.0)