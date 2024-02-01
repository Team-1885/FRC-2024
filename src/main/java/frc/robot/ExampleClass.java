// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import lombok.Getter;

/**
 * This class serves as a placeholder for reference purposes.
 * It provides a template for creating and testing runnable methods.
 */
@SuppressWarnings("PMD.CommentSize") public class ExampleClass {

  /**
   * ...
   */
  private @Getter static ADAM adam = new ADAM(null);

  /**
   * ...
   */
  public ExampleClass() {
    // ...
  }

  /**
   * The main method serves as the entry point for the class.
   *
   * @param args Command-line arguments provided when running the program.
   */
  public static void main(final String[] args) {
    runTest(() -> {
      // Code
    });
  }

  /**
   * ...
   */
  public static void myMethod() {
    runTest(() -> {
      // Code
    });
  }

  /**
   * ...
   */
  public void debugClass() {
    runTest(() -> main(null));
    runTest(() -> myMethod());
  }

  /**
   * Runs the provided code as a runnable task. If the code throws an exception,
   * it is caught, and an uncaught exception is passed to the default uncaught
   * exception handler for the current thread.
   *
   * @param code The runnable task to be executed.
   */
  public static void runTest(final Runnable code) {
    try {
      code.run();
    } catch (Exception e) {
      adam.uncaughtException(Thread.currentThread(), e);
    }
  }
}
