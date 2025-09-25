package org.firstinspires.ftc.teamcode.opmodes.poses

import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.drivetrain.Vector
import kotlin.math.PI

val scorePosition = Vector.fromCartesian(132.0, 60.0)
val farPose = Pose(12.0, 12.0, -3 * PI / 4)
val farDistance = (scorePosition - Vector.fromPose(farPose)).length
val closePose = Pose(84.0, 12.0, 0.3805063771 - PI)
val closeDistance = (scorePosition - Vector.fromPose(closePose)).length