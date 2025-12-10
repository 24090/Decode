package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose

fun main(){
    for (n in 3..11) {
        val results = moveShootKinematics(Pose(12.0, 12.0, 0.0).vector() * n, Pose(0.0, 0.0, 0.0).vector())
        println("${n*12} root 2")
        println(results?.first)
        println(results?.second)
    }


}