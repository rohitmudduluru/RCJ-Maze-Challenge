#pragma once
#include <math.h>
#include <Arduino.h>

class Vector2D {
private:
  static constexpr double _deadzone = 2;
public:
  double x, y;
  Vector2D(double xval = 0, double yval = 0);
  Vector2D operator+(Vector2D const& obj) const;
  Vector2D operator-(Vector2D const& obj) const;
  Vector2D operator*(double const& d) const;    //scalar multiplication
  double operator*(Vector2D const& obj) const;  //dot product
  bool operator==(Vector2D const& obj) const;
  void print() const;
  double mag() const;
  Vector2D normalized() const;
  Vector2D proj(const Vector2D& v2D) const; // this projected onto v2D
};