#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "../riscv.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MultiBranch : public Component {
public:
  MultiBranch(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    // clang-format off
        res << [=] {
            switch(comp_op.uValue()){
                case CompOp::NOP: return false;
                case CompOp::EQ: 
                    return zero.uValue() == 0x1;
                case CompOp::NE: 
                    return zero.uValue() == 0x0;
                case CompOp::LT: 
                    return res_sign.uValue() == 0x1;
                //case CompOp::LTU: 

                case CompOp::GE: 
                    return zero.uValue() == 0x1 || res_sign.uValue() == 0x0; 
                //case CompOp::GEU: 

                default: assert("Comparator: Unknown comparison operator"); return false;
            }
        };
    // clang-format on
  }

  INPUTPORT_ENUM(comp_op, CompOp);
  INPUTPORT(zero, 1);
  INPUTPORT(res_sign, 1);
  OUTPUTPORT(res, 1);
};

} // namespace core
} // namespace vsrtl
