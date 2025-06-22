#include "vector2D.h"

Vector2D::Vector2D(double xval, double yval) {
  x = xval;
  y = yval;
}

Vector2D Vector2D::operator+(Vector2D const& obj) const {
  return Vector2D(x + obj.x, y + obj.y);
}
Vector2D Vector2D::operator-(Vector2D const& obj) const {
  return Vector2D(x - obj.x, y - obj.y);
}
Vector2D Vector2D::operator*(double const& d) const {  //scalar multiplication
  return Vector2D(x * d, y * d);
}
double Vector2D::operator*(Vector2D const& obj) const {  //dot product
  return x * obj.x + y * obj.y;
}
bool Vector2D::operator==(Vector2D const& v2D) const {
  return Vector2D(x - v2D.x, y - v2D.y).mag() < _deadzone;
}
void Vector2D::print() const {
  Serial.print("<");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(">");
}
double Vector2D::mag() const {
  return sqrt(x * x + y * y);
}
Vector2D Vector2D::normalized() const {
  double m = mag();
  if (fabs(m) < 1e-10) return Vector2D();
  return Vector2D(x / m, y / m);
}
Vector2D Vector2D::proj(const Vector2D& v2D) const {
  if (fabs(v2D.mag()) < 1e-10) return Vector2D();
  return v2D * ((*this * v2D) / (v2D.y * v2D.y + v2D.x * v2D.x));
}