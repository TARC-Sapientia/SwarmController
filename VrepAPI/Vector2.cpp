/* Teleoperation System for Mobile Manipulators Framework
 *
 * Copyright (C) 2015 
 * RECONFIGURABLE CONTROL OF ROBOTIC SYSTEMS OVER NETWORKS Project
 * Robotics and Control Systems Laboratory
 * Department of Electrical Engineering
 * Sapientia Hungarian University of Transylvania
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * For more details see the project homepage:
 * <http://www.ms.sapientia.ro/~martonl/MartonL_Research_TE.htm>
 */


#include "Vector2.h"
#include <cmath>
#include <sstream>
#include "Utilities.h"

namespace VrepAPI
{
	Vector2::Vector2()
		: x(0), y(0)
	{
	}

	Vector2::Vector2(double x, double y)
		: x(x), y(y)
	{
	}

	Vector2::Vector2(const Vector2& vector)
		: x(vector.x), y(vector.y)
	{
	}

	Vector2::~Vector2()
	{
	}

	bool Vector2::operator!=(const Vector2& vector) const
	{
		return x != vector.x || y != vector.y;
	}

	bool Vector2::operator==(const Vector2& vector) const
	{
		return (*this != vector) == false;
	}

	Vector2& Vector2::operator+=(const Vector2& vector)
	{
		x += vector.x;
		y += vector.y;
		return *this;
	}

	Vector2& Vector2::operator-=(const Vector2& vector)
	{
		x -= vector.x;
		y -= vector.y;
		return *this;
	}

	Vector2& Vector2::operator*=(double value)
	{
		x *= value;
		y *= value;
		return *this;
	}

	Vector2& Vector2::operator/=(double value)
	{
		x /= value;
		y /= value;
		return *this;
	}

	Vector2 operator+(const Vector2& vector1, const Vector2& vector2)
	{
		return Vector2(vector1.x + vector2.x, vector1.y + vector2.y);
	}

	Vector2 operator-(const Vector2& vector1, const Vector2& vector2)
	{
		return Vector2(vector1.x - vector2.x, vector1.y - vector2.y);
	}

	Vector2 operator*(const Vector2& vector1, double value)
	{
		return Vector2(vector1.x * value, vector1.y * value);
	}

	Vector2 operator*(double value, const Vector2& vector1)
	{
		return Vector2(vector1.x * value, vector1.y * value);
	}

	Vector2 operator/(const Vector2& vector1, double value)
	{
		return Vector2(vector1.x / value, vector1.y / value);
	}

	Vector2 operator*(const Vector2& vector1, const Vector2& vector2)
	{
		return Vector2(vector1.x * vector2.x, vector1.y * vector2.y);
	}

	Vector2 operator/(const Vector2& vector1, const Vector2& vector2)
	{
		return Vector2(vector1.x / vector2.x, vector1.y / vector2.y);
	}

	double Vector2::Magnitude()
	{
		return sqrt(x * x + y * y);
	}

	double Vector2::MagnitudeSqr()
	{
		return x * x + y * y;
	}

	Vector2 Vector2::Perpendicular()
	{
		return Vector2(-y, x);
	}

	void Vector2::Normalize()
	{
		double magnitude = Magnitude();

		x /= magnitude;
		y /= magnitude;
	}

	Vector2 Vector2::Invert()
	{
		return Vector2(-x, -y);
	}

	void Vector2::SetMagnitude(double magnitude)
	{
		Normalize();

		x *= magnitude;
		y *= magnitude;
	}

	void Vector2::Clamp(double max)
	{
		if (Magnitude() > max)
		{
			SetMagnitude(max);
		}
	}

	// Vector2 cartesian(double radius, double angle)
	//{
	// return Vector2(radius * cos(angle), radius * sin(angle));
	//}

	// Vector2 polar(double x, double y)
	//{
	// return Vector2(atan2(y, x), sqrt(x * x + y * y));
	//}

	double Vector2::DotProduct(const Vector2& vector)
	{
		return x * vector.x + y * vector.y;
	}

	double Vector2::CrossProduct(const Vector2& vector)
	{
		return x * vector.y - y * vector.x;
	}

	Vector2& Vector2::Rotate(double angle)
	{
		angle = Utils::DegreeToRadian(angle);

		double cosA = cos(angle);
		double sinA = sin(angle);

		//Essentially, apply a 2x2 rotation matrix to the vector
		double xt = x * cosA - y * sinA;
		double yt = x * sinA + y * cosA;

		x = xt;
		y = yt;
		return (*this);
	}

	double Vector2::Angle()
	{
		return -Utils::RadianToDegree(atan2(y, x));
	}

	double Vector2::Angle(const Vector2& vector)
	{
		return -Utils::RadianToDegree(atan2(CrossProduct(vector), DotProduct(vector)));
	}

	std::string Vector2::ToString()
	{
		std::stringstream ss;
		ss << "(" << x << ", " << y << ")";
		return ss.str();
	}
}