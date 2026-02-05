package org.firstinspires.ftc.teamcode.opmodes.poses

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.math.PI

const val robotWidth = 18.0
const val robotLength = 13.38
val scorePosition = Vector.fromCartesian(144.0, 72.0)

fun getScoreDistance(position: Vector, isRed: Boolean = false) =
    (scorePosition.mirroredIf(isRed) - position).length


fun getScoreAngle(position: Vector, isRed: Boolean = false) =
    (scorePosition.plus(Vector.fromCartesian(0.0, -6.0/72.0 * clamp((72.0 - position.x), 0.0, 72.0))).mirroredIf(isRed) - position).angle


fun getScorePose(position: Vector, isRed: Boolean = false) =
    Pose(position.x, position.y, getScoreAngle(position, isRed))

val farPose = getScorePose(Vector.fromCartesian(14.0, 14.0))
val closePose = getScorePose(Vector.fromCartesian(79.5, 12.0))
val closeDistance = getScoreDistance(Vector.fromPose(closePose))

val farDistance = getScoreDistance(Vector.fromPose(farPose))
val farStartPose = Pose(robotLength/2.0, robotWidth/2.0, 0.0)
val closeStartPose = Pose(122.45460330031989, 47.57810577632874, Math.PI/4.0)
val parkPose = Pose(24.0+(robotLength/2.0), -24.0-(robotWidth/2.0), 0.0)
fun inLaunchZone(pose: Pose, threshold: Double = 0.0) =
    inLaunchZone(
        pose.vector() +
        Vector.fromPolar(pose.heading + PI/2, robotWidth/2.0) +
        Vector.fromPolar(pose.heading, robotLength/2.0),
        threshold
    ) ||
    inLaunchZone(
        pose.vector() +
        Vector.fromPolar(pose.heading + PI/2, robotWidth/2.0) -
        Vector.fromPolar(pose.heading, robotLength/2.0),
        threshold
    ) ||
    inLaunchZone(
        pose.vector() -
        Vector.fromPolar(pose.heading + PI/2, robotWidth/2.0) +
        Vector.fromPolar(pose.heading, robotLength/2.0),
        threshold
    ) ||
    inLaunchZone(
        pose.vector() -
        Vector.fromPolar(pose.heading + PI/2, robotWidth/2.0) -
        Vector.fromPolar(pose.heading, robotLength/2.0),
        threshold
    )

private fun inLaunchZone(point: Vector, threshold: Double = 1.5) = (point.x - 72.0 - threshold >= point.y && point.x - 72.0 - threshold >= -point.y)
                            || (point.x <= point.y + 24.0 - threshold && point.x <= -point.y + 24.0 - threshold)