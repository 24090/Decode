package org.firstinspires.ftc.teamcode.subsystems.drive.pathing.followers

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.subsystems.controlsystems.PDLT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hD
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hP
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.hT
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.kS
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyD
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyP
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive.DriveConstants.xyT
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveVectors.Companion.processTurnDriveStrafe
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativePose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.getRelativeVelocity
import org.firstinspires.ftc.teamcode.util.Reference


fun getPointToPoint(targetPose: Reference<Pose>, localizer: Localizer) = { pointToPoint(localizer.pose, localizer.poseVel, targetPose.get()) }
fun pointToPoint(pose: Pose, velocity: Pose, targetPose: Pose, full: Boolean = true): DriveVectors {
    val error = getRelativePose(pose, targetPose)
    val dError = -getRelativeVelocity(pose, velocity)
    val turn = PDLT(AngleUnit.normalizeRadians(error.heading), dError.heading, hP, hD, kS, hT)
    val translational =
        PDLT(Vector.fromPose(error), Vector.fromPose(dError), xyP, xyD, kS, xyT)
    val drive = if (translational.x.isNaN()) 0.0 else translational.x
    val strafe = if (translational.y.isNaN()) 0.0 else translational.y
    return if (!full) {
        DriveVectors.fromRotation(turn) + DriveVectors.fromTranslation(drive, strafe)
    } else {
        processTurnDriveStrafe(turn, drive, strafe, pose, velocity)
    }
}


fun getMoveShootPointToPoint(targetPose: Pose, localizer: Localizer, getAngle: () -> Double?) = { moveShootPointToPoint(localizer.pose, localizer.poseVel, targetPose, getAngle()) }
fun moveShootPointToPoint(pose: Pose, velocity: Pose, targetPose: Pose, angle: Double?): DriveVectors {

    val targetPose = Pose(targetPose.x, targetPose.y, angle ?: targetPose.heading)
    val error = getRelativePose(pose, targetPose)
    val dError = -getRelativeVelocity(pose, velocity)
    val turn = PDLT(AngleUnit.normalizeRadians(error.heading), dError.heading, hP, hD, kS, hT)
    val translational =
        PDLT(Vector.fromPose(error), Vector.fromPose(dError), xyP, xyD, kS, xyT)
    val drive = if (translational.x.isNaN()) 0.0 else translational.x
    val strafe = if (translational.y.isNaN()) 0.0 else translational.y

    return processTurnDriveStrafe(turn, drive, strafe, pose, velocity)
}