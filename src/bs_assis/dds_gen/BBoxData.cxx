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
 * @file BBoxData.cpp
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

#include "BBoxData.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

#define BBoxData_max_cdr_typesize 312ULL;
#define BBoxData_max_key_cdr_typesize 0ULL;

BBoxData::BBoxData()
{
    // short m_id
    m_id = 0;
    // string m_obj_class
    m_obj_class ="";
    // long long m_xmin
    m_xmin = 0;
    // long long m_ymin
    m_ymin = 0;
    // long long m_xmax
    m_xmax = 0;
    // long long m_ymax
    m_ymax = 0;
    // float m_probability
    m_probability = 0.0;
    // float m_pose_x
    m_pose_x = 0.0;
    // float m_pose_y
    m_pose_y = 0.0;
    // float m_pose_z
    m_pose_z = 0.0;

}

BBoxData::~BBoxData()
{










}

BBoxData::BBoxData(
        const BBoxData& x)
{
    m_id = x.m_id;
    m_obj_class = x.m_obj_class;
    m_xmin = x.m_xmin;
    m_ymin = x.m_ymin;
    m_xmax = x.m_xmax;
    m_ymax = x.m_ymax;
    m_probability = x.m_probability;
    m_pose_x = x.m_pose_x;
    m_pose_y = x.m_pose_y;
    m_pose_z = x.m_pose_z;
}

BBoxData::BBoxData(
        BBoxData&& x) noexcept 
{
    m_id = x.m_id;
    m_obj_class = std::move(x.m_obj_class);
    m_xmin = x.m_xmin;
    m_ymin = x.m_ymin;
    m_xmax = x.m_xmax;
    m_ymax = x.m_ymax;
    m_probability = x.m_probability;
    m_pose_x = x.m_pose_x;
    m_pose_y = x.m_pose_y;
    m_pose_z = x.m_pose_z;
}

BBoxData& BBoxData::operator =(
        const BBoxData& x)
{

    m_id = x.m_id;
    m_obj_class = x.m_obj_class;
    m_xmin = x.m_xmin;
    m_ymin = x.m_ymin;
    m_xmax = x.m_xmax;
    m_ymax = x.m_ymax;
    m_probability = x.m_probability;
    m_pose_x = x.m_pose_x;
    m_pose_y = x.m_pose_y;
    m_pose_z = x.m_pose_z;

    return *this;
}

BBoxData& BBoxData::operator =(
        BBoxData&& x) noexcept
{

    m_id = x.m_id;
    m_obj_class = std::move(x.m_obj_class);
    m_xmin = x.m_xmin;
    m_ymin = x.m_ymin;
    m_xmax = x.m_xmax;
    m_ymax = x.m_ymax;
    m_probability = x.m_probability;
    m_pose_x = x.m_pose_x;
    m_pose_y = x.m_pose_y;
    m_pose_z = x.m_pose_z;

    return *this;
}

bool BBoxData::operator ==(
        const BBoxData& x) const
{

    return (m_id == x.m_id && m_obj_class == x.m_obj_class && m_xmin == x.m_xmin && m_ymin == x.m_ymin && m_xmax == x.m_xmax && m_ymax == x.m_ymax && m_probability == x.m_probability && m_pose_x == x.m_pose_x && m_pose_y == x.m_pose_y && m_pose_z == x.m_pose_z);
}

bool BBoxData::operator !=(
        const BBoxData& x) const
{
    return !(*this == x);
}

size_t BBoxData::getMaxCdrSerializedSize(
        size_t current_alignment)
{
    static_cast<void>(current_alignment);
    return BBoxData_max_cdr_typesize;
}

size_t BBoxData::getCdrSerializedSize(
        const BBoxData& data,
        size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;


    current_alignment += 2 + eprosima::fastcdr::Cdr::alignment(current_alignment, 2);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + data.obj_class().size() + 1;

    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 8 + eprosima::fastcdr::Cdr::alignment(current_alignment, 8);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

void BBoxData::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{

    scdr << m_id;
    scdr << m_obj_class.c_str();
    scdr << m_xmin;
    scdr << m_ymin;
    scdr << m_xmax;
    scdr << m_ymax;
    scdr << m_probability;
    scdr << m_pose_x;
    scdr << m_pose_y;
    scdr << m_pose_z;

}

void BBoxData::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{

    dcdr >> m_id;
    dcdr >> m_obj_class;
    dcdr >> m_xmin;
    dcdr >> m_ymin;
    dcdr >> m_xmax;
    dcdr >> m_ymax;
    dcdr >> m_probability;
    dcdr >> m_pose_x;
    dcdr >> m_pose_y;
    dcdr >> m_pose_z;
}

/*!
 * @brief This function sets a value in member id
 * @param _id New value for member id
 */
void BBoxData::id(
        int16_t _id)
{
    m_id = _id;
}

/*!
 * @brief This function returns the value of member id
 * @return Value of member id
 */
int16_t BBoxData::id() const
{
    return m_id;
}

/*!
 * @brief This function returns a reference to member id
 * @return Reference to member id
 */
int16_t& BBoxData::id()
{
    return m_id;
}

/*!
 * @brief This function copies the value in member obj_class
 * @param _obj_class New value to be copied in member obj_class
 */
void BBoxData::obj_class(
        const std::string& _obj_class)
{
    m_obj_class = _obj_class;
}

/*!
 * @brief This function moves the value in member obj_class
 * @param _obj_class New value to be moved in member obj_class
 */
void BBoxData::obj_class(
        std::string&& _obj_class)
{
    m_obj_class = std::move(_obj_class);
}

/*!
 * @brief This function returns a constant reference to member obj_class
 * @return Constant reference to member obj_class
 */
const std::string& BBoxData::obj_class() const
{
    return m_obj_class;
}

/*!
 * @brief This function returns a reference to member obj_class
 * @return Reference to member obj_class
 */
std::string& BBoxData::obj_class()
{
    return m_obj_class;
}
/*!
 * @brief This function sets a value in member xmin
 * @param _xmin New value for member xmin
 */
void BBoxData::xmin(
        int64_t _xmin)
{
    m_xmin = _xmin;
}

/*!
 * @brief This function returns the value of member xmin
 * @return Value of member xmin
 */
int64_t BBoxData::xmin() const
{
    return m_xmin;
}

/*!
 * @brief This function returns a reference to member xmin
 * @return Reference to member xmin
 */
int64_t& BBoxData::xmin()
{
    return m_xmin;
}

/*!
 * @brief This function sets a value in member ymin
 * @param _ymin New value for member ymin
 */
void BBoxData::ymin(
        int64_t _ymin)
{
    m_ymin = _ymin;
}

/*!
 * @brief This function returns the value of member ymin
 * @return Value of member ymin
 */
int64_t BBoxData::ymin() const
{
    return m_ymin;
}

/*!
 * @brief This function returns a reference to member ymin
 * @return Reference to member ymin
 */
int64_t& BBoxData::ymin()
{
    return m_ymin;
}

/*!
 * @brief This function sets a value in member xmax
 * @param _xmax New value for member xmax
 */
void BBoxData::xmax(
        int64_t _xmax)
{
    m_xmax = _xmax;
}

/*!
 * @brief This function returns the value of member xmax
 * @return Value of member xmax
 */
int64_t BBoxData::xmax() const
{
    return m_xmax;
}

/*!
 * @brief This function returns a reference to member xmax
 * @return Reference to member xmax
 */
int64_t& BBoxData::xmax()
{
    return m_xmax;
}

/*!
 * @brief This function sets a value in member ymax
 * @param _ymax New value for member ymax
 */
void BBoxData::ymax(
        int64_t _ymax)
{
    m_ymax = _ymax;
}

/*!
 * @brief This function returns the value of member ymax
 * @return Value of member ymax
 */
int64_t BBoxData::ymax() const
{
    return m_ymax;
}

/*!
 * @brief This function returns a reference to member ymax
 * @return Reference to member ymax
 */
int64_t& BBoxData::ymax()
{
    return m_ymax;
}

/*!
 * @brief This function sets a value in member probability
 * @param _probability New value for member probability
 */
void BBoxData::probability(
        float _probability)
{
    m_probability = _probability;
}

/*!
 * @brief This function returns the value of member probability
 * @return Value of member probability
 */
float BBoxData::probability() const
{
    return m_probability;
}

/*!
 * @brief This function returns a reference to member probability
 * @return Reference to member probability
 */
float& BBoxData::probability()
{
    return m_probability;
}

/*!
 * @brief This function sets a value in member pose_x
 * @param _pose_x New value for member pose_x
 */
void BBoxData::pose_x(
        float _pose_x)
{
    m_pose_x = _pose_x;
}

/*!
 * @brief This function returns the value of member pose_x
 * @return Value of member pose_x
 */
float BBoxData::pose_x() const
{
    return m_pose_x;
}

/*!
 * @brief This function returns a reference to member pose_x
 * @return Reference to member pose_x
 */
float& BBoxData::pose_x()
{
    return m_pose_x;
}

/*!
 * @brief This function sets a value in member pose_y
 * @param _pose_y New value for member pose_y
 */
void BBoxData::pose_y(
        float _pose_y)
{
    m_pose_y = _pose_y;
}

/*!
 * @brief This function returns the value of member pose_y
 * @return Value of member pose_y
 */
float BBoxData::pose_y() const
{
    return m_pose_y;
}

/*!
 * @brief This function returns a reference to member pose_y
 * @return Reference to member pose_y
 */
float& BBoxData::pose_y()
{
    return m_pose_y;
}

/*!
 * @brief This function sets a value in member pose_z
 * @param _pose_z New value for member pose_z
 */
void BBoxData::pose_z(
        float _pose_z)
{
    m_pose_z = _pose_z;
}

/*!
 * @brief This function returns the value of member pose_z
 * @return Value of member pose_z
 */
float BBoxData::pose_z() const
{
    return m_pose_z;
}

/*!
 * @brief This function returns a reference to member pose_z
 * @return Reference to member pose_z
 */
float& BBoxData::pose_z()
{
    return m_pose_z;
}



size_t BBoxData::getKeyMaxCdrSerializedSize(
        size_t current_alignment)
{
    static_cast<void>(current_alignment);
    return BBoxData_max_key_cdr_typesize;
}

bool BBoxData::isKeyDefined()
{
    return false;
}

void BBoxData::serializeKey(
        eprosima::fastcdr::Cdr& scdr) const
{
    (void) scdr;
}
