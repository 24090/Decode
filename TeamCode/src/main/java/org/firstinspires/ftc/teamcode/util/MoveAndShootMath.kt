package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.opmodes.poses.startPose
import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter.Params.shooterAngle
import kotlin.math.pow
import kotlin.math.tan

fun main(){
    val results = moveShootKinematics(Vector.fromCartesian(330.0, 0.0), Pose(0.0, 0.0, 0.0).vector())
    println(solvePolynomialIA(
        interval = Interval(0.0, 10.0),
        iterations = 30,
        -57455.78327361352, 0.0, -15920.458119590417, -0.0, 107797.14763120271
    ).last())
    println(solveQuarticNewton(
        -57455.78327361352, 0.0, -15920.458119590417, -0.0, 107797.14763120271,
        guess = 3.0
    ))
    println(results?.first)
    println(results?.second)
}