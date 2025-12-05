package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.subsystems.drive.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.Vector
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter.Params.shooterAngle
import kotlin.math.acos
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

fun Int.factorial() : Int = (1..this).reduce(Int::times)
/**
 * @return The number of ways to choose `r` unique elements from a set of `n` without replacement, ignoring order
 */
fun nCr(n: Int, r: Int) = n.factorial()/(r.factorial() * (n-r).factorial())
/**
 * Infix version of `nCr`
 * @see nCr
 */
infix fun Int.choose(r: Int) = nCr(this, r)

fun clamp(x: Double, min: Double, max: Double): Double {
    assert(min <= max)
    return min(max(x, min), max)
}

fun newtonQuartic(a: Double,b: Double,c: Double,d: Double,e: Double, guess: Double): Double{
    val derivA: Double = 4*a
    val derivB: Double = 3*b
    val derivC: Double = 2*c
    val derivD: Double = 1*d
    var initGuess: Double = guess
    var i = 0
    while (i<100){
        val slope: Double = evalCubic(derivA,derivB,derivC, derivD, initGuess)
        val newGuess: Double = initGuess -(evalQuartic(a,b,c,d,e,initGuess))/slope
        initGuess = newGuess
        i++
    }
    return initGuess
}
fun evalCubic(a: Double, b: Double,c: Double,d: Double, value: Double): Double{
    return a*(value.pow(3))+b*(value.pow(2))+c*(value.pow(1))+d
}
fun evalQuartic(a: Double, b: Double,c: Double,d: Double, e: Double, value: Double): Double{
    return a*(value.pow(4))+b*(value.pow(3))+c*(value.pow(2))+d*(value.pow(1))+e
}

fun findLineIntersection(lineA: Pair<Vector, Vector>, lineB: Pair<Vector, Vector>): Vector? {
    if (lineA.first.x == lineA.second.x) {
        return null
    } else if (lineB.first.x == lineB.second.x) {
        return null
    }
    val slopeA = (lineA.first.y - lineA.second.y)/(lineA.first.x - lineA.second.x)
    val slopeB = (lineB.first.y - lineB.second.y)/(lineB.first.x - lineB.second.x)
    val interceptA = lineA.first.x - lineA.first.y * slopeA
    val interceptB = lineB.first.x - lineB.first.y * slopeA
    val pointInRange = { x: Double, a: Double, b: Double -> x>= min(a, b)&& x<= max(a, b)}
    if (slopeA == slopeB) {
        return (
            if (interceptB != interceptA) null
            else if (pointInRange(lineB.first.x, lineA.first.x, lineA.second.x)) lineB.first
            else if (pointInRange(lineB.second.x, lineA.first.x, lineA.second.x)) lineB.second
            else null
        )
    }

    val x =  (interceptB - interceptA) / (slopeA - slopeB)

    if (pointInRange(x, lineA.first.x, lineA.second.x) && pointInRange(x, lineB.first.x, lineB.second.x)) {
        return Vector.fromCartesian(x, x * slopeA + interceptA)
    }
    return null
}
