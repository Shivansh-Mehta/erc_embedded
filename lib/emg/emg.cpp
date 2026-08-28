// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#include "emg.h"

StackLight::StackLight(uint8_t pin_green, uint8_t pin_yellow, uint8_t pin_red)
    : m_pin_green(pin_green),
      m_pin_yellow(pin_yellow),
      m_pin_red(pin_red) {}

void StackLight::init_light()
{
  pinMode(m_pin_green, OUTPUT);
  pinMode(m_pin_yellow, OUTPUT);
  pinMode(m_pin_red, OUTPUT);
  state(255);
}

void StackLight::state(uint8_t state)
{
  if (state == 1)
  {
    digitalWrite(m_pin_green, LOW);
    digitalWrite(m_pin_yellow, HIGH);
    digitalWrite(m_pin_red, HIGH);
  }
  else if (state == 2)
  {
    digitalWrite(m_pin_green, HIGH);
    digitalWrite(m_pin_yellow, LOW);
    digitalWrite(m_pin_red, HIGH);
  }
  else if (state == 3)
  {
    digitalWrite(m_pin_green, HIGH);
    digitalWrite(m_pin_yellow, HIGH);
    digitalWrite(m_pin_red, LOW);
  }
  else
  {
    digitalWrite(m_pin_green, HIGH);
    digitalWrite(m_pin_yellow, HIGH);
    digitalWrite(m_pin_red, HIGH);
  }
}
