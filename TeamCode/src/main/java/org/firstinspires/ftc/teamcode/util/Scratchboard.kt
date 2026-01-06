package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector

fun main(){
    val driveVectors = DriveVectors.fromTranslation(Vector.fromCartesian(-100.0, 0.0)).trimmed(0.8)
//        .apply { if (false) {
//            tipCorrected(
//                tipAccelBackward,
//                tipAccelForward,
//                kS,
//                kV,
//                kA,
//                0.0
//            )
//        }}
    val leftPowers = driveVectors.getLeftPowers()
    val rightPowers = driveVectors.getRightPowers()

    println("fl ${leftPowers.first}, fr ${leftPowers.second}, bl ${rightPowers.first}, br ${rightPowers.second}")
}