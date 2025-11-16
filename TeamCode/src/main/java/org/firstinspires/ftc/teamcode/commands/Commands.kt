package org.firstinspires.ftc.teamcode.commands

import com.rathippo.commandviewer.CommandMessage
import com.rathippo.commandviewer.CommandViewer
import com.rathippo.commandviewer.CommandViewer.registerFunction

var currentCommandID = 0
fun registerCommandID(): Int{
    currentCommandID += 1
    return currentCommandID
}

/**
 * Signifies if the command should continue, and, if not, why
 */
sealed class CommandResult {
    /**
     * Signifies that the command should not continue running
     */
    object Continue : CommandResult()

    /**
     * Signifies that the command should continue running, either due to error (`Result.failure()`) or success (`Result.success()`)
     * @see Result
     */
    open class End(val result: Result<String>): CommandResult()
}

abstract class Command(
    protected val name: String = "Unknown Name",
    protected val selfCondense: Boolean = false,
    protected val uid: Int = registerCommandID(),
    protected val cssClass: String = "",
){
    var commandOverride: CommandOverride = CommandOverride.None

    open fun getButtons(): ArrayList<Pair<Int, String>>{
        return arrayListOf()
    }

    /**
     * Updates the command with logging
     * To control command behaviour, do not change this method. Instead, implement the desired behaviour in `run()`
     * @return should be used to decide whether or not continue running the command or turn it into a dead one
     * @see Command.run
     */
    fun update(): CommandResult{
        CommandMessage.Begin(uid, name, getButtons(), selfCondense, cssClass).send()

        val result = when (commandOverride) {
            CommandOverride.Pause -> CommandResult.Continue
            CommandOverride.Skip -> CommandResult.End(Result.success("Skipped"))
            CommandOverride.None -> try {
                run()
            } catch (e: Throwable){
                CommandMessage.Error(e).send()
                CommandResult.End(Result.failure(e))
            }
        }

        CommandMessage.End.send()
        return result
    }

    /**
     * Gets the dead version of the command. Should be overridden by commands with subcommands
     * @param reason why the command is dead
     * @return the dead version of the command
     */
    open fun getDead(reason: CommandResult.End): DeadCommand{
        return DeadCommand(reason, ArrayList(), name, selfCondense, uid, cssClass)
    }

    open fun getFuture(): FutureCommand {
        return FutureCommand(ArrayList(), name, selfCondense, uid, cssClass)
    }

    /**
     * This method determines the behaviour of the command and is run by `update()`.
     * @see Command.update
     */
    protected abstract fun run(): CommandResult
}

sealed interface CommandOverride{
    object None: CommandOverride
    object Pause: CommandOverride
    object Skip: CommandOverride
}

class DeadCommand(
    private val reason: CommandResult.End,
    private val subcommands: List<DeadCommand> = ArrayList(),
    name: String = "Unknown Name",
    selfCondense: Boolean = false,
    uid: Int = registerCommandID(),
    cssClass: String = ""
): Command(name, selfCondense, uid, "$cssClass dead") {
    override fun run(): CommandResult {
        reason.result.fold(
            {v -> CommandMessage.Log(v).send()},
            {e -> CommandMessage.Error(e).send()}
        )
        subcommands.forEach {c -> c.update()}
        return CommandResult.Continue
    }

    override fun getDead(reason: CommandResult.End): DeadCommand {
        return this
    }
}

class FutureCommand(
    private val subcommands: List<Command> = ArrayList(),
    name: String = "Unknown Name",
    selfCondense: Boolean = false,
    uid: Int = registerCommandID(),
    cssClass: String = ""
): Command(name, selfCondense, uid, "$cssClass future") {
    override fun run(): CommandResult {
        subcommands.forEach {c -> c.update()}
        return CommandResult.Continue
    }

    override fun getFuture(): FutureCommand {
        return this
    }
}

abstract class OverrideButtonCommand(
    name: String = "Unknown Name",
    selfCondense: Boolean = false,
    uid: Int = registerCommandID(),
    cssClass: String = "",
): Command(name, selfCondense, uid, cssClass){
    open val pause = registerFunction { commandOverride = CommandOverride.Pause }
    open val skip = registerFunction { commandOverride = CommandOverride.Skip }
    open val none = registerFunction { commandOverride = CommandOverride.None }
    override fun getButtons(): ArrayList<Pair<Int, String>> {
        val buttons = ArrayList<Pair<Int, String>>()
        if (commandOverride != CommandOverride.None)  buttons.add(Pair(none, "▶"))
        if (commandOverride != CommandOverride.Pause)  buttons.add(Pair(pause, "⏸"))
        if (commandOverride != CommandOverride.Skip)  buttons.add(Pair(skip, "▶▶"))
        return buttons
    }
}



fun runBlocking(command: Command){
    var c = command
    while (!Thread.currentThread().isInterrupted){
        val result = c.update()
        CommandViewer.update()
        when (result) {
            is CommandResult.Continue -> continue
            is CommandResult.End -> {
                if (result.result.isFailure){
                    throw result.result.exceptionOrNull()!!
                }
                c = c.getDead(result)
                break
            }
        }
    }
    // run once in order to display dead commands
    c.update()
    CommandViewer.update()
}
