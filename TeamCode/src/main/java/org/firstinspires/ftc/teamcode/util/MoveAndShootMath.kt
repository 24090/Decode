package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan

fun main(){
    for (n in 3 .. 10){
        val results = moveShootKinematics(Vector.fromCartesian(12.0 * n * sqrt(2.0), 0.001), Pose(0.0, 0.0, 0.0).vector())
        println("${n * 12}")
        println(results?.first)
        println(results?.second)
    }

}