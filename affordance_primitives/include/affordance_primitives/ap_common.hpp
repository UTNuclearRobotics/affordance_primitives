///////////////////////////////////////////////////////////////////////////////
//      Title     : ap_common.hpp
//      Project   : affordance_primitives
//      Created   : 02/09/2021
//      Author    : Adam Pettinger
//      Copyright : Copyright© The University of Texas at Austin, 2014-2022. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <unordered_map>

namespace affordance_primitives
{
const std::unordered_map<int, std::string> EXECUTION_RESULT_MAP(
  {{0, "INVALID RESULT"},
   {1, "SUCCESS"},
   {2, "STOP REQUESTED"},
   {3, "PARAMETER FAILURE"},
   {4, "FORCE TORQUE VIOLATION"},
   {5, "KINEMATIC VIOLATION"},
   {6, "TIMED OUT"}});

inline std::string to_string(const int input) { return EXECUTION_RESULT_MAP.at(input); }
}  // namespace affordance_primitives
