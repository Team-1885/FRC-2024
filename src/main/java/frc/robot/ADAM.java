// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Map;
import java.util.TimeZone;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * The ADAM class represents an error handling and logging utility for the
 * robot.
 */
@SuppressWarnings("PMD.CommentSize") public class ADAM implements Thread.UncaughtExceptionHandler {

  /**
   * ...
   */
  private @Getter String errorMessage;
  /**
   * ...
   */
  private @Getter ADAMErrorCodes errorCode;
  /**
   * ...
   */
  private @Getter Logger logger = Logger.getLogger(ADAM.class.getName());

  /**
   * ...
   */
  private @Getter final Map<ADAMErrorCodes, Integer> errorCounts = new ConcurrentHashMap<>();
  /**
   * ...
   */
  private @Getter static final int MAX_ERROR_COUNT = 5; // Maximum allowed errors within the time window
  /**
   * ...
   */
  private @Getter static final long TIME_WINDOW_MS = 10_000; // Time window in milliseconds (e.g., 10 seconds)
  /**
   * ...
   */
  private @Getter long lastErrorTime = -1;
  /**
   * ...
   */
  private @Getter final long robotStartTime;

  /**
   * ...
   */
  public enum ADAMErrorCodes {
    OFF(0, "No logging"),
    FATAL(100, "The application is unusable. Action needs to be taken immediately."),
    ERROR(200, "An error occurred in the application."),
    WARN(300, "Something unexpected happened and needs to be watched."),
    INFO(400, "A normal, expected, relevant event happened."),
    DEBUG(500, "Used for debugging purposes"),
    TRACE(600, "Used for debugging purposes, includes the most detailed information");
    // Add more error codes as needed

    /**
     * ...
     */
    private @Getter final int code;
    /**
     * ...
     */
    private @Getter final String message;

    ADAMErrorCodes(final int code, final String message) {
      this.code = code;
      this.message = message;
    }
  }

  /**
   * Adam Severity.
   */
  public enum ADAMSeverity {
    ALL(Level.ALL),
    CONFIG(Level.CONFIG),
    FINE(Level.FINE),
    FINER(Level.FINER),
    FINEST(Level.FINEST),
    INFO(Level.INFO),
    OFF(Level.OFF),
    SEVERE(Level.SEVERE),
    WARNING(Level.WARNING);

    /**
     * ...
     */
    private @Getter final Level level;

    ADAMSeverity(final Level level) {
      this.level = level;
    }
  }

  private static ADAMSeverity getSeverityForError(final Throwable throwable) {
    ADAMSeverity severity = ADAMSeverity.WARNING;

    if (throwable instanceof Exception) {
      severity = ADAMSeverity.SEVERE;
    } else if (throwable instanceof Exception) {
      severity = ADAMSeverity.SEVERE;
    }

    return severity;
  }

  /**
   * Constructs an ADAM instance to handle uncaught exceptions and log errors.
   *
   * @param e The uncaught exception.
   */
  public ADAM(final Throwable throwable) {
    this.errorMessage = determineErrorCode(throwable).getMessage();
    this.errorCode = determineErrorCode(throwable);
    logger.setLevel(getSeverityForError(throwable).getLevel());

    // Set the time zone to Eastern Standard Time (EST)
    final TimeZone estTimeZone = TimeZone.getTimeZone("EST");
    TimeZone.setDefault(estTimeZone);

    // Record the start time of the robot
    robotStartTime = System.currentTimeMillis();
  }

  private ADAMErrorCodes determineErrorCode(final Throwable throwable) {
    ADAMErrorCodes errorCode = ADAMErrorCodes.WARN;

    if (throwable instanceof Exception) {
      errorCode = ADAMErrorCodes.ERROR;
    } else if (throwable instanceof NullPointerException) {
      errorCode = ADAMErrorCodes.FATAL;
    }

    return errorCode;
  }

  @SuppressWarnings("PMD.DoNotUseThreads")
  @Override
  public void uncaughtException(final Thread thread, final Throwable throwable) {
    // Implement rate limiting for logging
    if (shouldLogError()) {
      logError(throwable);
    }
  }

  // Method to check if the error should be logged based on rate limiting
  private boolean shouldLogError() {
    final long currentTime = System.currentTimeMillis();

    boolean shouldLog = false; // Default value

    if (currentTime - lastErrorTime > TIME_WINDOW_MS) {
      // Reset the error counts if the time window has passed
      errorCounts.clear();
      lastErrorTime = currentTime;
      shouldLog = true;
    } else {
      final int count = errorCounts.getOrDefault(errorCode, 0);
      if (count < MAX_ERROR_COUNT) {
        // Log the error if the error count is within the limit
        errorCounts.put(errorCode, count + 1);
        shouldLog = true;
      }
      // Don't log the error if the error count exceeds the limit within the time
      // window
    }

    return shouldLog;
  }

  // Method to format elapsed time in a human-readable format
  private String formatElapsedTime(final long elapsedTimeMillis) {
    final long seconds = (elapsedTimeMillis / 1000) % 60;
    final long minutes = (elapsedTimeMillis / (1000 * 60)) % 60;
    final long hours = (elapsedTimeMillis / (1000 * 60 * 60)) % 24;
    return String.format("%02d:%02d:%02d", hours, minutes, seconds);
  }

  // Method to log the error and related information
  private void logError(final Throwable throwable) {
    // ADAM ASCII art title
    final String adamTitle = "       /)\n"
        + "  .-\"\".L,\"\"-.\n"
        + " ;       :.  :\n"
        + " (       7:  )\n"
        + "  :         ;\n"
        + "   \"..-\"-..\"";

    // Set the time zone to Eastern Standard Time (EST)
    final TimeZone estTimeZone = TimeZone.getTimeZone("America/New_York");

    // Format date in 12-hour time with AM/PM and Eastern Standard Time (EST)
    final SimpleDateFormat sdf = new SimpleDateFormat("hh:mm:ss a", Locale.US);
    sdf.setTimeZone(estTimeZone);
    final String formattedTime = sdf.format(new Date());

    // Create a separator line
    final String separatorLine = "==================================================";

    // Log error with the specified logging level
    final StringBuilder logMessageBuilder = new StringBuilder(200);

    logMessageBuilder.append(separatorLine)
        .append('\n')
        .append(adamTitle)
        .append(" ADAM v1.0: Michael has nothing on me\n")
        .append(separatorLine)
        .append("\n[")
        .append(formattedTime)
        .append("] Uncaught exception (Error Code ")
        .append(errorCode.getCode())
        .append("): ")
        .append(errorMessage);

    // Log the elapsed time since the robot started
    final long elapsedTime = System.currentTimeMillis() - robotStartTime;
    logMessageBuilder.append("\nElapsed Time Since Robot Started: ").append(formatElapsedTime(elapsedTime));

    // Log class, method, and line number where the error occurred
    final StackTraceElement[] stackTrace = throwable.getStackTrace();
    if (stackTrace.length > 0) {
      final StackTraceElement topFrame = stackTrace[0];
      final String className = topFrame.getClassName();
      final String methodName = topFrame.getMethodName();
      final int lineNumber = topFrame.getLineNumber();

      logMessageBuilder
          .append("\nError occurred in class: ").append(className)
          .append(", method: ").append(methodName)
          .append(", line: ").append(lineNumber);
    }

    // Log the original exception message and stack trace
    logMessageBuilder.append("\n\nOriginal Exception Message: ").append(throwable.getMessage())
        .append("\n\nOriginal Stack Trace:");
    for (final StackTraceElement traceElement : stackTrace) {
      logMessageBuilder.append("\n  ").append(traceElement.toString());
    }

    logMessageBuilder.append('\n').append(separatorLine).append('\n');

    final String logMessage = logMessageBuilder.toString();
    if (logger.isLoggable(logger.getLevel())) {
      logger.log(logger.getLevel(), logMessage);
    }

    // Log error to driver station with custom message and error code
    DriverStation.reportError(logMessage, false);

    // Log error to console with custom message and error code
    logger.info(logMessage);

  }
}