package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.PurePursuitPath
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers.HeadingBehaviour
import kotlin.math.PI

fun main(){
    val path = PurePursuitPath(
        listOf(
            closePose,
            Pose(60.0, 20.0, PI),
            Pose(60.0, 60.0, -PI/2),
        ),
        listOf(
            HeadingBehaviour.Interpolate,
            HeadingBehaviour.Snap,
            HeadingBehaviour.Tangent(0.0),
            HeadingBehaviour.Interpolate,
        ),
        listOf(
            10.0,
            10.0,
            10.0,
            10.0
        )
    )
    println(path.lines.map { "{${it.start}, ${it.end}}" })
    var v = Pose(0.0, 0.0, 0.0).vector()
    var position = closePose.vector()
    for (n in (1..1000)){
        println(position.let { Pose(it.x, it.y, 0.0) })
        val followPoint = path.getFollowPoint(position, 40.0, true)
        println("follow: $followPoint")
        v -= v.normalized() * (0.1) + v * 0.08
        v += (followPoint.vector() - position).norm() * (0.3)
        position += v
    }
    println(path.lines[1])
    println(path.lines[1].getTvalue(Vector.fromCartesian(50.0, -8.717797887081352)))
}