package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.subsystems.drive.followCurve
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.CubicHermiteSpline
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import kotlin.math.PI

fun main(){
    for (i in (3..10)){
        moveShootKinematics(Vector.fromCartesian(i * 12.0, i * 12.0), Vector.fromCartesian(0.0, 0.0)). also {
            println("${i*12}√2 -> ${it.first}")
        }
    }
}