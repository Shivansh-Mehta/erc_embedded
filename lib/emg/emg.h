// Developed as a part of HSM Aries team,
// by Shivansh Mehta (https://github.com/Shivansh-Mehta),
// for the European Rover Challenge

#ifndef EMG_H
#define EMG_H

#pragma once
#include <Arduino.h>

class StackLight
{
public:
  StackLight(uint8_t pin_green, uint8_t pin_yellow, uint8_t pin_red);
  void init_light();
  void state(uint8_t state);

private:
  uint8_t m_pin_green;
  uint8_t m_pin_yellow;
  uint8_t m_pin_red;
  uint8_t m_state;
};

#endif
