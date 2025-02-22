#include "robot.h"

float textY = 5;
float textX = 3;

void brain_screen()
{
  Brain.Screen.setFont(mono12);
  
  Brain.Screen.setCursor(textX, textY);
  Brain.Screen.newLine();

  if (left_motor_front.installed())
  {
    Brain.Screen.setCursor(textX, textY);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Left Motor Front: ");
    Brain.Screen.print(left_motor_front.temperature(percent));
    Brain.Screen.print("% Temperature");
    
  }
  else
  {
    Brain.Screen.setCursor(textX, textY);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Left Motor Front: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (left_motor_middle.installed())
  {
    Brain.Screen.setCursor(textX + 2, textY);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Left Motor Middle: ");
    Brain.Screen.print(left_motor_front.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setCursor(textX + 2, textY);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Left Motor Middle: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (left_motor_back.installed())
  {
    Brain.Screen.setCursor(textX + 4, textY);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Left Motor Back: ");
    Brain.Screen.print(left_motor_front.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setCursor(textX + 4, textY);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Left Motor Back: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (right_motor_front.installed())
  {
    Brain.Screen.setCursor(textX + 6, textY);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Right Motor Front: ");
    Brain.Screen.print(right_motor_front.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setCursor(textX + 6, textY);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Right Motor Front: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (right_motor_middle.installed())
  {
    Brain.Screen.setCursor(textX + 8, textY);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Right Motor Middle: ");
    Brain.Screen.print(right_motor_middle.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setCursor(textX + 8, textY);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Right Motor Middle: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (right_motor_back.installed())
  {
    Brain.Screen.setCursor(textX + 10, textY);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Right Motor Back: ");
    Brain.Screen.print(right_motor_back.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setCursor(textX + 10, textY);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Right Motor Back: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (hook.installed())
  {
    Brain.Screen.setCursor(textX + 12, textY);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Hook: ");
    Brain.Screen.print(hook.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setCursor(textX + 12, textY);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Hook: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  if (LB.installed())
  {
    Brain.Screen.setCursor(textX + 14, textY);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("Lady B: ");
    Brain.Screen.print(LB.temperature(percent));
    Brain.Screen.print("% Temperature");
    Brain.Screen.newLine();
  }
  else
  {
    Brain.Screen.setCursor(textX + 14, textY);
    Brain.Screen.setPenColor(red);
    Brain.Screen.print("Lady Brown: PROBLEM DETECTED!");
    Brain.Screen.newLine();
  }
  /**/
  
}

void controllerScreen() {
  Controller.Screen.clearLine();
  Controller.Screen.print("LB Position: ");
  Controller.Screen.print(LBRotation.angle());
}

void drawLogo() {
    
}
