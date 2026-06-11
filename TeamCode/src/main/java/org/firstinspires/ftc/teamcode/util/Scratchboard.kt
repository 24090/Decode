package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.opmodes.poses.closeStartPose
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid.SysIDRoutine
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import kotlin.math.PI

fun main(){
    val shootPose = ShootPose.Close
    val path = PurePursuitPath(
        listOf(
            shootPose,
            Pose(
                59.4 + 1.0,
                47.0,
                1.05
            ),
            Pose(
                59.4 + 1.0,
                56.12 + 1.3,
                1.05
            ),
        ),
        listOf(
            HeadingBehaviour.Interpolate,
            HeadingBehaviour.Interpolate,
        ),
        listOf(
            15.0,
            15.0
        ),
    )
    for (xIndex in -5..< 20) {
        for (yIndex in -5..< 20) {
            val x = -xIndex * 5.0 + shootPose.x
            val y = yIndex * 5.0 + shootPose.y
            val position = Vector.fromCartesian(x,y)
            path.t = path.getTValue(position, 0.0, 1000.0)
            val followPosition = path.getFollowPoint(position, path.getTValue(position, 0.0, 1000.0), 48.0).second
            print("${position.x}\t${position.y}\t")
            println("${followPosition.x}\t${followPosition.y}")

        }
    }
}