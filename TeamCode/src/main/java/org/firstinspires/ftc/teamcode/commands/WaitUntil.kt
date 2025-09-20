package org.firstinspires.ftc.teamcode.commands

fun WaitUntil(f: () -> Boolean, name: String) = RepeatUntil({Instant{}}, f, name)
fun WaitUntil(f: () -> Boolean) = WaitUntil(f, "WaitUntil")