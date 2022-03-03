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
enum ExecutionResult
{
  INVALID_RESULT,
  SUCCESS,
  STOP_REQUESTED,
  PARAM_FAILURE,
  FT_VIOLATION,
  KIN_VIOLATION,
  TIME_OUT
};

const std::unordered_map<ExecutionResult, std::string>
    EXECUTION_RESULT_MAP({ { ExecutionResult::INVALID_RESULT, "INVALID RESULT" },
                           { ExecutionResult::SUCCESS, "SUCCESS" },
                           { ExecutionResult::STOP_REQUESTED, "STOP REQUESTED" },
                           { ExecutionResult::PARAM_FAILURE, "PARAMETER FAILURE" },
                           { ExecutionResult::FT_VIOLATION, "FORCE TORQUE VIOLATION" },
                           { ExecutionResult::KIN_VIOLATION, "KINEMATIC VIOLATION" },
                           { ExecutionResult::TIME_OUT, "TIMED OUT" } });

inline std::string to_string(const ExecutionResult input)
{
  return EXECUTION_RESULT_MAP.at(input);
}
}  // namespace affordance_primitives
