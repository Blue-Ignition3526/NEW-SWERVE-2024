package org.team3526.lib.util;

public class Conversions {

  /**
   * Converts meters to inches
   * @param meters to convert to inches
   * @return inches
   */
  public static double mToInch(double meters) {
    return meters*39.3701;
  }

  /**
   * Converts inches to meters
   * @param inches to convert to meters
   * @return meters
   */
  public static double inchToM(double inches) {
    return inches/39.3701;
  }

  /**
   * Converts centimeters to inches
   * @param centimeters to convert to inches
   * @return inches
   */
  public static double cmToInch(double centimeters) {
    return centimeters/2.54;
  }

  /**
   * Converts inches to centimeters
   * @param inches to convert to centimeters
   * @return centimeters
   */
  public static double inchToCm(double inches) {
    return inches*2.54;
  }

  /**
   * Converts milimeters to meters
   * @param milimeters to convert to meters
   * @return meters
   */
  public static double mmToM(double mm) {
    return mm/1000;
  } 

  /**
   * Converts meters to milimeters
   * @param meters to convert to milimeters
   * @return milimeters
   */
  public static double mToMm(double m) {
    return m*1000;
  }
}
