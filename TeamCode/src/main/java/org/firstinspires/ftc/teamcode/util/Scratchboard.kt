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
    Vector.fromPolar(0.0, 1.0).scalarInverseProjection(Vector.fromPolar(0.0, 100.0))
        .also { println(it) }
}