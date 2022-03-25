// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

  Spark blinkin;

  //public static double rainbow_rainbow_palette = -0.99;
  //public static double rainbow_party_palette = -0.97;
  //public static double rainbow_ocean_palette = -0.95;
  //public static double rainbow_lava_palette = -0.93;
  //public static double rainbow_forest_palette = -0.91;
  //public static double rainbow_with_glitter = -0.89;
  //public static double confetti = -0.87;
  //public static double shot_red = -0.85;
  //public static double shot_blue = -0.83;
  //public static double shot_white = -0.81;
  //public static double sinelon_rainbow_palette = -0.79;
  //public static double sinelon_party_palette = -0.77;
  //public static double sinelon_ocean_palette = -0.75;
  //public static double sinelon_lava_palette = -0.73;
  //public static double sinelon_forest_palette = -0.71;
  //public static double beats_per_minute_rainbow_palette = -0.69;
  //public static double beats_per_minute_party_palette = -0.67;
  //public static double beats_per_minute_ocean_palette = -0.65;
  //public static double beats_per_minute_lava_palette = -0.63;
  //public static double beats_per_minute_forest_palette = -0.61;
  //public static double fire_medium = -0.59;
  //public static double fire_large = -0.57;
  //public static double twinkles_rainbow_palette = -0.55;
  //public static double twinkles_party_palette = -0.53;
  //public static double twinkles_ocean_palette = -0.51;
  //public static double twinkles_lava_palette = -0.49;
  //public static double twinkles_forest_palette = -0.47;
  //public static double color_waves_rainbow_palette = -0.45;
  //public static double color_waves_party_palette = -0.43;
  //public static double color_waves_ocean_palette = -0.41;
  //public static double color_waves_lava_palette = -0.39;
  //public static double color_waves_forest_palette = -0.37;
  //public static double larson_scanner_red = -0.35;
  //public static double larson_scanner_gray = -0.33;
  //public static double light_chase_red = -0.31;
  //public static double light_chase_blue = -0.29;
  //public static double light_chase_gray = -0.27;
  //public static double heartbeat_red = -0.25;
  //public static double heartbeat_blue = -0.23;
  //public static double heartbeat_white = -0.21;
  //public static double heartbeat_gray = -0.19;
  //public static double breath_red = -0.17;
  //public static double breath_blue = -0.15;
  //public static double breath_gray = -0.13;
  //public static double strobe_red = -0.11;
  //public static double strobe_blue = -0.09;
  //public static double strobe_gold = -0.07;
  //public static double strobe_white = -0.05;
  //public static double color_1_end_to_end_blend_to_black = -0.03;
  //public static double color_1_larson_scanner = -0.01;
  //public static double color_1_light_chase = 0.01;
  //public static double color_1_heartbeat_slow = 0.03;
  //public static double color_1_heartbeat_medium = 0.05;
  //public static double color_1_heartbeat_fast = 0.07;
  //public static double color_1_breath_slow = 0.09;
  //public static double color_1_breath_fast = 0.11;
  //public static double color_1_shot = 0.13;
  //public static double color_1_strobe = 0.15;
  //public static double color_2_end_to_end_blend_to_black = 0.17;
  //public static double color_2_larson_scanner = 0.19;
  //public static double color_2_light_chase = 0.21;
  //public static double color_2_heartbeat_slow = 0.23;
  //public static double color_2_heartbeat_medium = 0.25;
  //public static double color_2_heartbeat_fast = 0.27;
  //public static double color_2_breath_slow = 0.29;
  //public static double color_2_breath_fast = 0.31;
  //public static double color_2_shot = 0.33;
  //public static double color_2_strobe = 0.35;
  //public static double both_colors_sparkle_1_on_2 = 0.37;
  //public static double both_colors_sparkle_2_on_1 = 0.39;
  //public static double both_colors_Color_Gradient = 0.41;
  //public static double both_colors_beats_per_minute = 0.43;
  //public static double both_colors_end_to_end_blend_1_to_2 = 0.45;
  //public static double both_colors_end_to_end_blend = 0.47;
  //public static double both_colors_no_blend = 0.49;
  //public static double both_colors_twinkles = 0.51;
  //public static double both_colors_color_waves = 0.53;
  //public static double both_colors_sinelon = 0.55;
  //public static double hot_pink = 0.57;
  //public static double dark_red = 0.59;
  public static double red = 0.61;
  //public static double red_orange = 0.63;
  //public static double orange = 0.65;
  public static double gold = 0.65;
  //public static double yellow = 0.69;
  //public static double lawn_green = 0.71;
  //public static double lime = 0.73;
  //public static double dark_green = 0.75;
  public static double green = 0.77;
  //public static double blue_green = 0.79;
  //public static double aqua = 0.81;
  //public static double sky_blue = 0.83;
  //public static double dark_blue = 0.85;
  //public static double blue = 0.87;
  //public static double blue_violet = 0.89;
  //public static double violet = 0.91;
  //public static double white = 0.93;
  //public static double gray = 0.95;
  //public static double dark_gray = 0.97;
  //public static double black = 0.99;

  public static double[] STATES = {red, gold, green};

  /** Creates a new LEDs. */
  public LEDs() {
    blinkin = new Spark(Constants.LEDs.leds);
  }

  public void set(double color){
    blinkin.set(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
