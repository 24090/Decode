package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.opencv.core.Core.sqrt
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

fun Int.factorial() : Int = (1..this).fold(1, Int::times)
/**
 * @return The number of ways to choose `r` unique elements from a set of `n` without replacement, ignoring order
 */
fun nCr(n: Int, r: Int) = n.factorial()/(r.factorial() * (n-r).factorial())
/**
 * Infix version of `nCr`
 * @see nCr
 */
infix fun Int.choose(r: Int) = nCr(this, r)

fun clamp(x: Int, min: Int, max: Int): Int {
    assert(min <= max)
    return min(max(x, min), max)
}
fun clamp(x: Double, min: Double, max: Double): Double {
    assert(min <= max)
    return min(max(x, min), max)
}

fun solveQuarticNewton(a: Double, b: Double, c: Double, d: Double, e: Double, initialGuess: Double): Double{
    val polynomial = Polynomial(e, d, c, b, a)
    val derivative = polynomial.derivative()
    var guess: Double = initialGuess
    var i = 0
    while (i<100){
        val slope: Double = polynomial.evaluate(guess)
        val newGuess: Double = guess -(derivative.evaluate(guess))/slope
        guess = newGuess
        i++
    }
    return guess
}

fun findLineIntersection(lineA: Pair<Vector, Vector>, lineB: Pair<Vector, Vector>): Vector? {

    // i'm to lazy to handle vertical lines so adding 0.001 is the solution
    val lineA = if (lineA.first.x == lineA.second.x) {
        Pair(Vector.fromCartesian(lineA.first.x + 0.001, lineA.first.y), lineA.second)
    } else lineA

    val lineB = if (lineB.first.x == lineB.second.x) {
        Pair(Vector.fromCartesian(lineB.first.x + 0.001, lineB.first.y), lineB.second)
    } else lineB

    val slopeA = (lineA.first.y - lineA.second.y)/(lineA.first.x - lineA.second.x)
    val slopeB = (lineB.first.y - lineB.second.y)/(lineB.first.x - lineB.second.x)
    val interceptA = lineA.first.y - lineA.first.x * slopeA
    val interceptB = lineB.first.y - lineB.first.x * slopeB
    val pointInRange = { x: Double, a: Double, b: Double -> x>= min(a, b)&& x<= max(a, b)}
    val x = (interceptA - interceptB) / (slopeB - slopeA)
    if (pointInRange(x, lineA.first.x, lineA.second.x) && pointInRange(x, lineB.first.x, lineB.second.x)) {
        return Vector.fromCartesian(x, x * slopeA + interceptA)
    }

    if (slopeA == slopeB) {
        return (
            if (interceptB != interceptA) null
            else if (pointInRange(lineB.first.x, lineA.first.x, lineA.second.x)) lineB.first
            else if (pointInRange(lineB.second.x, lineA.first.x, lineA.second.x)) lineB.second
            else null
        )
    }
    return null

}
