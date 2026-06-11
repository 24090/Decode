package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.opmodes.poses.ShootPose
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.sysid.SysIDRoutine
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import kotlin.math.PI

fun main(){
    val path = PurePursuitPath(
        listOf(Pose(0.0, 0.0, 0.0), Pose(50.0, 0.0, 0.0)),
        listOf(HeadingBehaviour.Snap),
        listOf(5.0)
    )

    println(path.getFollowPoint(Vector.fromCartesian(0.0, 0.0), 0.0, 48.0))
}