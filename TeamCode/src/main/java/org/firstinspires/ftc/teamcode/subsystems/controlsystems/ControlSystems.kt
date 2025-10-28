package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import org.firstinspires.ftc.teamcode.drivetrain.Vector
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.math.sqrt

fun PD(error: Double, dError: Double, kP: Double, kD: Double) = error * kP + dError * kD
fun PD(error: Vector, dError: Vector, kP: Double, kD: Double) = error * kP + dError * kD

fun PDL(error: Double, dError: Double, kP: Double, kD: Double, kL: Double) = PD(error, dError, kP, kD) + BangBang(error, kL)
fun PDL(error: Vector, dError: Vector, kP: Double, kD: Double, kL: Double) = PD(error, dError, kP, kD) + BangBang(error, kL)

fun PDLT(error: Vector, dError: Vector, kP: Double, kD: Double, kL: Double, kThresh: Double) =
    if (error.length < kThresh)
        PD(error, dError, kP, kD) 
    else PDL(error, dError, kP, kD, kL)

fun PDLT(error: Double, dError: Double, kP: Double, kD: Double, kL: Double, kThresh: Double) =
    if (error.absoluteValue < kThresh)
        PD(error, dError, kP, kD)
    else PDL(error, dError, kP, kD, kL)

fun SquID(error: Vector, kSq: Double) = error.norm() * sqrt(error.length * kSq)
fun SquID(error: Double, kSq: Double) = error.sign * sqrt(error.absoluteValue * kSq)

fun BangBang(error: Vector, k: Double) = error.norm() * k
fun BangBang(error: Double, k: Double) = k * error.sign