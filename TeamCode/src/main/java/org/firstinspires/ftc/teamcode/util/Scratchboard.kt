package org.firstinspires.ftc.teamcode.util

/*
 SCRATCHBOARD:
    for running random tests and stuff
 */

import org.firstinspires.ftc.teamcode.opmodes.poses.closeDistance
import org.firstinspires.ftc.teamcode.opmodes.poses.closePose
import org.firstinspires.ftc.teamcode.opmodes.poses.inLaunchZone
import org.firstinspires.ftc.teamcode.opmodes.poses.robotLength
import org.firstinspires.ftc.teamcode.opmodes.poses.robotWidth
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hD
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hP
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelBackward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.tipAccelForward
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyD
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyP
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyT
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.followCurve
import org.firstinspires.ftc.teamcode.subsystems.drive.minStopDistance
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.CubicHermiteSpline
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.strafeAccelCorrected
import kotlin.math.PI

fun main(){
    println(
        minStopDistance(0.0, 0.0, Pose(10.0, 0.0, 0.0), tipAccelBackward, tipAccelForward)
    )
}