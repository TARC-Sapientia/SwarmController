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


#ifndef VECTOR2_H
#define VECTOR2_H

#include <string>

namespace VrepAPI
{
	class Vector2
	{
	public:
		Vector2();
		Vector2(double x, double y);
		Vector2(const Vector2& vector);
		~Vector2();

		Vector2& operator+=(const Vector2& vector);
		Vector2& operator-=(const Vector2& vector);
		Vector2& operator*=(double num);
		Vector2& operator/=(double num);

		bool operator!=(const Vector2& vector) const;
		bool operator==(const Vector2& vector) const;

		friend Vector2 operator+(const Vector2& vector1, const Vector2& vector2);
		friend Vector2 operator-(const Vector2& vector1, const Vector2& vector2);
		friend Vector2 operator*(const Vector2& vector1, double value);
		friend Vector2 operator*(double value, const Vector2& vector1);
		friend Vector2 operator/(const Vector2& vector1, double value);
		friend Vector2 operator*(const Vector2& vector1, const Vector2& vector2);
		friend Vector2 operator/(const Vector2& vector1, const Vector2& vector2);

		double Magnitude();
		double MagnitudeSqr();
		Vector2 Perpendicular();
		void Normalize();
		Vector2 Invert();

		void SetMagnitude(double magnitude);
		void Clamp(double max);

		// TODO:
		//Vector2 polar(double x, double y);
		//Vector2 cartesian(double radius, double angle);

		double DotProduct(const Vector2& vector);
		double CrossProduct(const Vector2& vector);

		// Angle in degrees.
		Vector2& Rotate(double angle);

		// Angle in degrees.
		double Angle();
		double Angle(const Vector2& vector);

		std::string ToString();

		double x;
		double y;
	};
}
#endif // VECTOR2_H