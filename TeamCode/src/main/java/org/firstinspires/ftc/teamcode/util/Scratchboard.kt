package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.drive.minStopDistance
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose

fun main(){
    println(
        minStopDistance(0.0, 0.0, Pose(10.0, 0.0, 0.0), tipAccelBackward, tipAccelForward)
    )
}