package org.firstinspires.ftc.teamcode.util

import kotlin.math.max
import kotlin.math.min

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
