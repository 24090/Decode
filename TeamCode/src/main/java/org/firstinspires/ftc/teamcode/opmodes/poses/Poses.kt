package org.firstinspires.ftc.teamcode.opmodes.poses

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
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
val closePose = getScorePose(Vector.fromCartesian(76.0, 12.0))
val closeDistance = getScoreDistance(Vector.fromPose(closePose))

val farDistance = getScoreDistance(Vector.fromPose(farPose))
val startPose = Pose(robotLength/2.0, robotWidth/2.0, 0.0)
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