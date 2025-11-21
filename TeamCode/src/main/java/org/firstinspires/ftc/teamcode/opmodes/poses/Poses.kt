package org.firstinspires.ftc.teamcode.opmodes.poses

import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import kotlin.math.PI

const val robotWidth = 16.0
const val robotLength = 18.0
val scorePosition = Vector.fromCartesian(144.0, 72.0)
val farPose = Pose(12.0, 12.0, 0.3805063771)
val farDistance = (scorePosition - Vector.fromPose(farPose)).length
val closePose = Pose(84.0, 12.0, PI / 4)
val closeDistance = (scorePosition - Vector.fromPose(closePose)).length

val startPose = Pose(robotLength/2, robotWidth/2, 0.0)

var storedPose: Pose? = null