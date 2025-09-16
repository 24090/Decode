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


sealed interface CommandOverride{
    object None: CommandOverride
    object Pause: CommandOverride
    object Skip: CommandOverride
}

class DeadCommand(
    private val reason: CommandResult.End,
    private val subcommands: ArrayList<DeadCommand> = ArrayList(),
    name: String = "Unknown Name",
    selfCondense: Boolean = false,
    uid: Int = registerCommandID(),
    cssClass: String = ""
): Command(name, selfCondense, uid, cssClass + " dead") {
    override fun run(): CommandResult {
        reason.result.fold(
            {v -> CommandMessage.Log(v).send()},
            {e -> CommandMessage.Error(e).send()}
        )
        subcommands.forEach {c -> c.update()}
        return CommandResult.Continue
    }
}

class FutureCommand(
    private val subcommands: ArrayList<Command> = ArrayList(),
    name: String = "Unknown Name",
    selfCondense: Boolean = false,
    uid: Int = registerCommandID(),
    cssClass: String = ""
): Command(name, selfCondense, uid, cssClass + " future") {
    override fun run(): CommandResult {
        subcommands.forEach {c -> c.update()}
        return CommandResult.Continue
    }
}

abstract class OverrideButtonCommand(
    name: String = "Unknown Name",
    selfCondense: Boolean = false,
    uid: Int = registerCommandID(),
    cssClass: String = "",
): Command(name, selfCondense, uid, cssClass){
    open val pause = registerFunction({commandOverride = CommandOverride.Pause})
    open val skip = registerFunction({commandOverride = CommandOverride.Skip})
    open val none = registerFunction({commandOverride = CommandOverride.None})
    override fun getButtons(): ArrayList<Pair<Int, String>> {
        val buttons = ArrayList<Pair<Int, String>>()
        if (commandOverride != CommandOverride.None)  buttons.add(Pair(none, "▶"))
        if (commandOverride != CommandOverride.Pause)  buttons.add(Pair(pause, "⏸"))
        if (commandOverride != CommandOverride.Skip)  buttons.add(Pair(skip, "▶▶"))
        return buttons;
    }
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

class Sequence(vararg commands: Command): OverrideButtonCommand("Sequence", true){
    private val pastCommands: ArrayList<DeadCommand> = ArrayList()
    private val futureCommands:  ArrayList<Command> = ArrayList()
    private var currentCommand: Command?
    init {
        commands.toCollection(futureCommands)
        currentCommand = futureCommands.removeAt(0)
    }

    override fun getFuture(): FutureCommand{
        val commands: ArrayList<Command> = pastCommands.clone() as ArrayList<Command>
        currentCommand?.let { commands.add(it.getFuture()) }
        for (command in futureCommands) {
            commands.add(command.getFuture())
        }
        return FutureCommand(commands, name, selfCondense, uid)
    }

    override fun getDead(reason: CommandResult.End): DeadCommand{
        val parentEndedEarly = CommandResult.End(Result.failure(Error("Parent ended early")))
        currentCommand?.let { pastCommands.add(it.getDead(parentEndedEarly)) }
        for (command in futureCommands) {
            pastCommands.add(command.getDead(parentEndedEarly))
        }
        return DeadCommand(reason, pastCommands, name, selfCondense, uid)
    }

    override fun run(): CommandResult {
        // uses local val so only one null-check necessary per loop
        val currentCommand = currentCommand
            ?: return CommandResult.End(Result.success("No Commands Left"))

        // show dead past commands
        for (command in pastCommands) {
            command.update()
        }
        val result = currentCommand.update()
        // show future commands
        for (command in futureCommands) {
            command.getFuture().update()
        }
        when (result) {
            CommandResult.Continue -> return CommandResult.Continue
            is CommandResult.End -> {
                pastCommands.add(currentCommand.getDead(result))
                if (result.result.isFailure){
                    return CommandResult.End(Result.failure(Error("Child error")))
                }
                this.currentCommand = try {
                    futureCommands.removeAt(0)
                } catch (e: IndexOutOfBoundsException) {
                    null
                }
                return CommandResult.Continue
            }
        }
    }
}

class WaitUntil(val f: () -> Boolean, name: String = "WaitUntil"): OverrideButtonCommand(name) {
    override fun run(): CommandResult {
        if (!f()){
            return CommandResult.Continue
        }
        return CommandResult.End(Result.success("WaitUntil ended"))
    }
}

class Future(val f: () -> Command, name: String = "Future"): OverrideButtonCommand(name) {
    private var command: Command? = null
    override fun run(): CommandResult {
        command = command ?: f()
        val result = command!!.update()
        when (result){
            CommandResult.Continue -> {}
            is CommandResult.End -> command = command!!.getDead(result)
        }
        return result

    }

    override fun getDead(reason: CommandResult.End): DeadCommand {
        val command = command
        return DeadCommand(
            reason,
            when (command) {
                null -> ArrayList()
                is DeadCommand -> arrayListOf(command)
                else -> arrayListOf(command.getDead(
                    CommandResult.End(Result.failure(Error("Parent ended early")))
                ))
            },
            name, selfCondense, uid
        )
    }
}

class Forever(val f: () -> Command, name: String = "Forever"): OverrideButtonCommand(name) {
    val pastCommands: ArrayList<DeadCommand> = arrayListOf()
    val clearDead = registerFunction { pastCommands.clear() }
    var currentCommand: Command? = f.invoke()
    override fun getButtons(): ArrayList<Pair<Int, String>> {
        val buttons = super.getButtons()
        buttons.add(Pair(clearDead, "Clear Dead Commands"))
        return buttons
    }
    override fun run(): CommandResult {
        for (command in pastCommands){
            command.update()
        }
        when (val result = currentCommand!!.update()) {
            CommandResult.Continue -> {}
            is CommandResult.End -> {
                pastCommands.add(currentCommand!!.getDead(result))
                if (result.result.isFailure) {
                    currentCommand = null
                    return result
                }
                currentCommand = f.invoke()
            }
        }
        return CommandResult.Continue
    }

    override fun getDead(reason: CommandResult.End): DeadCommand {
        currentCommand?.let {
            pastCommands.add(it.getDead(CommandResult.End(Result.failure(Error("Parent ended early")))))
        }
        return DeadCommand(reason, pastCommands, name, selfCondense, uid)
    }

    override fun getFuture(): FutureCommand {
        val commands = pastCommands.clone() as ArrayList<Command>
        currentCommand?.let {
            commands.add(it.getFuture())
        }
        return FutureCommand(commands, name, selfCondense, uid)
    }
}
fun Sleep(timeSeconds: Double, name: String = "Sleep"): Command{
    return Future({
        val targetTime = System.currentTimeMillis() + timeSeconds * 1000
        WaitUntil({System.currentTimeMillis() >= targetTime})
    }, name)
}
class Instant(val f: () -> Unit, name: String = "Instant"): Command(name){
    override fun run(): CommandResult {
        f.invoke()
        return CommandResult.End(Result.success("Instant function ran"))
    }
}