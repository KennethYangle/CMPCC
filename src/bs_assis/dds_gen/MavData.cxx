// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file MavData.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace {
char dummy;
}  // namespace
#endif  // _WIN32

#include "MavData.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

#define MavData_max_cdr_typesize 68ULL;
#define MavData_max_key_cdr_typesize 0ULL;

MavData::MavData()
{
    // short m_system_ID
    m_system_ID = 0;
    // float m_pose_x
    m_pose_x = 0.0;
    // float m_pose_y
    m_pose_y = 0.0;
    // float m_pose_z
    m_pose_z = 0.0;
    // float m_quat_w
    m_quat_w = 0.0;
    // float m_quat_x
    m_quat_x = 0.0;
    // float m_quat_y
    m_quat_y = 0.0;
    // float m_quat_z
    m_quat_z = 0.0;
    // float m_latitude
    m_latitude = 0.0;
    // float m_longitude
    m_longitude = 0.0;
    // float m_altitude
    m_altitude = 0.0;
    // float m_vel_x
    m_vel_x = 0.0;
    // float m_vel_y
    m_vel_y = 0.0;
    // float m_vel_z
    m_vel_z = 0.0;
    // float m_ang_x
    m_ang_x = 0.0;
    // float m_ang_y
    m_ang_y = 0.0;
    // float m_ang_z
    m_ang_z = 0.0;

}

MavData::~MavData()
{

















}

MavData::MavData(
        const MavData& x)
{
    m_system_ID = x.m_system_ID;
    m_pose_x = x.m_pose_x;
    m_pose_y = x.m_pose_y;
    m_pose_z = x.m_pose_z;
    m_quat_w = x.m_quat_w;
    m_quat_x = x.m_quat_x;
    m_quat_y = x.m_quat_y;
    m_quat_z = x.m_quat_z;
    m_latitude = x.m_latitude;
    m_longitude = x.m_longitude;
    m_altitude = x.m_altitude;
    m_vel_x = x.m_vel_x;
    m_vel_y = x.m_vel_y;
    m_vel_z = x.m_vel_z;
    m_ang_x = x.m_ang_x;
    m_ang_y = x.m_ang_y;
    m_ang_z = x.m_ang_z;
}

MavData::MavData(
        MavData&& x) noexcept 
{
    m_system_ID = x.m_system_ID;
    m_pose_x = x.m_pose_x;
    m_pose_y = x.m_pose_y;
    m_pose_z = x.m_pose_z;
    m_quat_w = x.m_quat_w;
    m_quat_x = x.m_quat_x;
    m_quat_y = x.m_quat_y;
    m_quat_z = x.m_quat_z;
    m_latitude = x.m_latitude;
    m_longitude = x.m_longitude;
    m_altitude = x.m_altitude;
    m_vel_x = x.m_vel_x;
    m_vel_y = x.m_vel_y;
    m_vel_z = x.m_vel_z;
    m_ang_x = x.m_ang_x;
    m_ang_y = x.m_ang_y;
    m_ang_z = x.m_ang_z;
}

MavData& MavData::operator =(
        const MavData& x)
{

    m_system_ID = x.m_system_ID;
    m_pose_x = x.m_pose_x;
    m_pose_y = x.m_pose_y;
    m_pose_z = x.m_pose_z;
    m_quat_w = x.m_quat_w;
    m_quat_x = x.m_quat_x;
    m_quat_y = x.m_quat_y;
    m_quat_z = x.m_quat_z;
    m_latitude = x.m_latitude;
    m_longitude = x.m_longitude;
    m_altitude = x.m_altitude;
    m_vel_x = x.m_vel_x;
    m_vel_y = x.m_vel_y;
    m_vel_z = x.m_vel_z;
    m_ang_x = x.m_ang_x;
    m_ang_y = x.m_ang_y;
    m_ang_z = x.m_ang_z;

    return *this;
}

MavData& MavData::operator =(
        MavData&& x) noexcept
{

    m_system_ID = x.m_system_ID;
    m_pose_x = x.m_pose_x;
    m_pose_y = x.m_pose_y;
    m_pose_z = x.m_pose_z;
    m_quat_w = x.m_quat_w;
    m_quat_x = x.m_quat_x;
    m_quat_y = x.m_quat_y;
    m_quat_z = x.m_quat_z;
    m_latitude = x.m_latitude;
    m_longitude = x.m_longitude;
    m_altitude = x.m_altitude;
    m_vel_x = x.m_vel_x;
    m_vel_y = x.m_vel_y;
    m_vel_z = x.m_vel_z;
    m_ang_x = x.m_ang_x;
    m_ang_y = x.m_ang_y;
    m_ang_z = x.m_ang_z;

    return *this;
}

bool MavData::operator ==(
        const MavData& x) const
{

    return (m_system_ID == x.m_system_ID && m_pose_x == x.m_pose_x && m_pose_y == x.m_pose_y && m_pose_z == x.m_pose_z && m_quat_w == x.m_quat_w && m_quat_x == x.m_quat_x && m_quat_y == x.m_quat_y && m_quat_z == x.m_quat_z && m_latitude == x.m_latitude && m_longitude == x.m_longitude && m_altitude == x.m_altitude && m_vel_x == x.m_vel_x && m_vel_y == x.m_vel_y && m_vel_z == x.m_vel_z && m_ang_x == x.m_ang_x && m_ang_y == x.m_ang_y && m_ang_z == x.m_ang_z);
}

bool MavData::operator !=(
        const MavData& x) const
{
    return !(*this == x);
}

size_t MavData::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    static_cast<void>(current_alignment);
    return MavData_max_cdr_typesize;
}

size_t MavData::getCdrSerializedSize(
        const MavData& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

void MavData::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_system_ID;
    scdr << m_pose_x;
    scdr << m_pose_y;
    scdr << m_pose_z;
    scdr << m_quat_w;
    scdr << m_quat_x;
    scdr << m_quat_y;
    scdr << m_quat_z;
    scdr << m_latitude;
    scdr << m_longitude;
    scdr << m_altitude;
    scdr << m_vel_x;
    scdr << m_vel_y;
    scdr << m_vel_z;
    scdr << m_ang_x;
    scdr << m_ang_y;
    scdr << m_ang_z;

}

void MavData::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_system_ID;
    dcdr >> m_pose_x;
    dcdr >> m_pose_y;
    dcdr >> m_pose_z;
    dcdr >> m_quat_w;
    dcdr >> m_quat_x;
    dcdr >> m_quat_y;
    dcdr >> m_quat_z;
    dcdr >> m_latitude;
    dcdr >> m_longitude;
    dcdr >> m_altitude;
    dcdr >> m_vel_x;
    dcdr >> m_vel_y;
    dcdr >> m_vel_z;
    dcdr >> m_ang_x;
    dcdr >> m_ang_y;
    dcdr >> m_ang_z;
}

/*!
 * @brief This function sets a value in member system_ID
 * @param _system_ID New value for member system_ID
 */
void MavData::system_ID(
        int16_t _system_ID)
{
    m_system_ID = _system_ID;
}

/*!
 * @brief This function returns the value of member system_ID
 * @return Value of member system_ID
 */
int16_t MavData::system_ID() const
{
    return m_system_ID;
}

/*!
 * @brief This function returns a reference to member system_ID
 * @return Reference to member system_ID
 */
int16_t& MavData::system_ID()
{
    return m_system_ID;
}

/*!
 * @brief This function sets a value in member pose_x
 * @param _pose_x New value for member pose_x
 */
void MavData::pose_x(
        float _pose_x)
{
    m_pose_x = _pose_x;
}

/*!
 * @brief This function returns the value of member pose_x
 * @return Value of member pose_x
 */
float MavData::pose_x() const
{
    return m_pose_x;
}

/*!
 * @brief This function returns a reference to member pose_x
 * @return Reference to member pose_x
 */
float& MavData::pose_x()
{
    return m_pose_x;
}

/*!
 * @brief This function sets a value in member pose_y
 * @param _pose_y New value for member pose_y
 */
void MavData::pose_y(
        float _pose_y)
{
    m_pose_y = _pose_y;
}

/*!
 * @brief This function returns the value of member pose_y
 * @return Value of member pose_y
 */
float MavData::pose_y() const
{
    return m_pose_y;
}

/*!
 * @brief This function returns a reference to member pose_y
 * @return Reference to member pose_y
 */
float& MavData::pose_y()
{
    return m_pose_y;
}

/*!
 * @brief This function sets a value in member pose_z
 * @param _pose_z New value for member pose_z
 */
void MavData::pose_z(
        float _pose_z)
{
    m_pose_z = _pose_z;
}

/*!
 * @brief This function returns the value of member pose_z
 * @return Value of member pose_z
 */
float MavData::pose_z() const
{
    return m_pose_z;
}

/*!
 * @brief This function returns a reference to member pose_z
 * @return Reference to member pose_z
 */
float& MavData::pose_z()
{
    return m_pose_z;
}

/*!
 * @brief This function sets a value in member quat_w
 * @param _quat_w New value for member quat_w
 */
void MavData::quat_w(
        float _quat_w)
{
    m_quat_w = _quat_w;
}

/*!
 * @brief This function returns the value of member quat_w
 * @return Value of member quat_w
 */
float MavData::quat_w() const
{
    return m_quat_w;
}

/*!
 * @brief This function returns a reference to member quat_w
 * @return Reference to member quat_w
 */
float& MavData::quat_w()
{
    return m_quat_w;
}

/*!
 * @brief This function sets a value in member quat_x
 * @param _quat_x New value for member quat_x
 */
void MavData::quat_x(
        float _quat_x)
{
    m_quat_x = _quat_x;
}

/*!
 * @brief This function returns the value of member quat_x
 * @return Value of member quat_x
 */
float MavData::quat_x() const
{
    return m_quat_x;
}

/*!
 * @brief This function returns a reference to member quat_x
 * @return Reference to member quat_x
 */
float& MavData::quat_x()
{
    return m_quat_x;
}

/*!
 * @brief This function sets a value in member quat_y
 * @param _quat_y New value for member quat_y
 */
void MavData::quat_y(
        float _quat_y)
{
    m_quat_y = _quat_y;
}

/*!
 * @brief This function returns the value of member quat_y
 * @return Value of member quat_y
 */
float MavData::quat_y() const
{
    return m_quat_y;
}

/*!
 * @brief This function returns a reference to member quat_y
 * @return Reference to member quat_y
 */
float& MavData::quat_y()
{
    return m_quat_y;
}

/*!
 * @brief This function sets a value in member quat_z
 * @param _quat_z New value for member quat_z
 */
void MavData::quat_z(
        float _quat_z)
{
    m_quat_z = _quat_z;
}

/*!
 * @brief This function returns the value of member quat_z
 * @return Value of member quat_z
 */
float MavData::quat_z() const
{
    return m_quat_z;
}

/*!
 * @brief This function returns a reference to member quat_z
 * @return Reference to member quat_z
 */
float& MavData::quat_z()
{
    return m_quat_z;
}

/*!
 * @brief This function sets a value in member latitude
 * @param _latitude New value for member latitude
 */
void MavData::latitude(
        float _latitude)
{
    m_latitude = _latitude;
}

/*!
 * @brief This function returns the value of member latitude
 * @return Value of member latitude
 */
float MavData::latitude() const
{
    return m_latitude;
}

/*!
 * @brief This function returns a reference to member latitude
 * @return Reference to member latitude
 */
float& MavData::latitude()
{
    return m_latitude;
}

/*!
 * @brief This function sets a value in member longitude
 * @param _longitude New value for member longitude
 */
void MavData::longitude(
        float _longitude)
{
    m_longitude = _longitude;
}

/*!
 * @brief This function returns the value of member longitude
 * @return Value of member longitude
 */
float MavData::longitude() const
{
    return m_longitude;
}

/*!
 * @brief This function returns a reference to member longitude
 * @return Reference to member longitude
 */
float& MavData::longitude()
{
    return m_longitude;
}

/*!
 * @brief This function sets a value in member altitude
 * @param _altitude New value for member altitude
 */
void MavData::altitude(
        float _altitude)
{
    m_altitude = _altitude;
}

/*!
 * @brief This function returns the value of member altitude
 * @return Value of member altitude
 */
float MavData::altitude() const
{
    return m_altitude;
}

/*!
 * @brief This function returns a reference to member altitude
 * @return Reference to member altitude
 */
float& MavData::altitude()
{
    return m_altitude;
}

/*!
 * @brief This function sets a value in member vel_x
 * @param _vel_x New value for member vel_x
 */
void MavData::vel_x(
        float _vel_x)
{
    m_vel_x = _vel_x;
}

/*!
 * @brief This function returns the value of member vel_x
 * @return Value of member vel_x
 */
float MavData::vel_x() const
{
    return m_vel_x;
}

/*!
 * @brief This function returns a reference to member vel_x
 * @return Reference to member vel_x
 */
float& MavData::vel_x()
{
    return m_vel_x;
}

/*!
 * @brief This function sets a value in member vel_y
 * @param _vel_y New value for member vel_y
 */
void MavData::vel_y(
        float _vel_y)
{
    m_vel_y = _vel_y;
}

/*!
 * @brief This function returns the value of member vel_y
 * @return Value of member vel_y
 */
float MavData::vel_y() const
{
    return m_vel_y;
}

/*!
 * @brief This function returns a reference to member vel_y
 * @return Reference to member vel_y
 */
float& MavData::vel_y()
{
    return m_vel_y;
}

/*!
 * @brief This function sets a value in member vel_z
 * @param _vel_z New value for member vel_z
 */
void MavData::vel_z(
        float _vel_z)
{
    m_vel_z = _vel_z;
}

/*!
 * @brief This function returns the value of member vel_z
 * @return Value of member vel_z
 */
float MavData::vel_z() const
{
    return m_vel_z;
}

/*!
 * @brief This function returns a reference to member vel_z
 * @return Reference to member vel_z
 */
float& MavData::vel_z()
{
    return m_vel_z;
}

/*!
 * @brief This function sets a value in member ang_x
 * @param _ang_x New value for member ang_x
 */
void MavData::ang_x(
        float _ang_x)
{
    m_ang_x = _ang_x;
}

/*!
 * @brief This function returns the value of member ang_x
 * @return Value of member ang_x
 */
float MavData::ang_x() const
{
    return m_ang_x;
}

/*!
 * @brief This function returns a reference to member ang_x
 * @return Reference to member ang_x
 */
float& MavData::ang_x()
{
    return m_ang_x;
}

/*!
 * @brief This function sets a value in member ang_y
 * @param _ang_y New value for member ang_y
 */
void MavData::ang_y(
        float _ang_y)
{
    m_ang_y = _ang_y;
}

/*!
 * @brief This function returns the value of member ang_y
 * @return Value of member ang_y
 */
float MavData::ang_y() const
{
    return m_ang_y;
}

/*!
 * @brief This function returns a reference to member ang_y
 * @return Reference to member ang_y
 */
float& MavData::ang_y()
{
    return m_ang_y;
}

/*!
 * @brief This function sets a value in member ang_z
 * @param _ang_z New value for member ang_z
 */
void MavData::ang_z(
        float _ang_z)
{
    m_ang_z = _ang_z;
}

/*!
 * @brief This function returns the value of member ang_z
 * @return Value of member ang_z
 */
float MavData::ang_z() const
{
    return m_ang_z;
}

/*!
 * @brief This function returns a reference to member ang_z
 * @return Reference to member ang_z
 */
float& MavData::ang_z()
{
    return m_ang_z;
}



size_t MavData::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    static_cast<void>(current_alignment);
    return MavData_max_key_cdr_typesize;
}

bool MavData::isKeyDefined()
{
    return false;
}

void MavData::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
}

