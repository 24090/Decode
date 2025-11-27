package org.firstinspires.ftc.teamcode.opmodes.poses

import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
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
val closeDistance = (scorePosition - Vector.fromPose(closePose)).length

val startPose = Pose(robotLength/2, robotWidth/2, 0.0)

var storedPose: Pose? = null