package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.lateralFactor
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.getWheelVector
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import kotlin.math.sqrt

fun main(){
    val driveVectors = DriveVectors.fromRotation(0.0)
        .addWithoutPriority(
            DriveVectors.fromTranslation(Vector.fromCartesian(-1.0, 0.0))
        ).trimmed(1.0)
        .tipCorrected(
            -150.0,
            150.0,
            kS,
            kV,
            kA,
            40.0
        )
    val leftPowers = driveVectors.getLeftPowers()
    val rightPowers = driveVectors.getRightPowers()

    println("fl ${leftPowers.first}, fr ${leftPowers.second}, bl ${rightPowers.first}, br ${rightPowers.second}")
}