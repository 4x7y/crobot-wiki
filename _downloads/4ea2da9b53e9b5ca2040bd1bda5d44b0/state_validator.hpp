#pragma once

#include <memory>

namespace crobot::planning {

class StateValidator {
public:
};

using StateValidatorPtr      = std::shared_ptr<StateValidator>;
using ConstStateValidatorPtr = std::shared_ptr<const StateValidator>;

} // namespace crobot::planning

