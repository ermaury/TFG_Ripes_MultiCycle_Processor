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
        /*res << [=] {
            switch(comp_op.uValue()){
                case CompOp::NOP: return false;
                case CompOp::EQ: 
                    return (bool)zero.uValue();
                case CompOp::NE: 
                    return (bool)(!zero.uValue());
                case CompOp::LT: 
                     // XOR
                     return (bool)(res_sign.uValue() ^ v_flag.uValue());
                case CompOp::LTU:
                    return (bool)(!c_flag.uValue());
                 
                case CompOp::GE: 
                    // XNOR
                    return (bool)(!(res_sign.uValue() ^ v_flag.uValue()));
                case CompOp::GEU:
                    return (bool)(c_flag.uValue()); 

                default: assert("Comparator: Unknown comparison operator"); return false;
            }
        };*/
        // NOP
        0 >> multiplexer->get(CompOp::NOP);
        // BEQ
        zero >> multiplexer->get(CompOp::EQ);
        // BNE
        zero >> *not_bne->in[0];
        not_bne->out >> multiplexer->get(CompOp::NE);
        // BLT
        res_sign >> *xor_blt->in[0];
        v_flag >> *xor_blt->in[1];
        xor_blt->out >> multiplexer->get(CompOp::LT);
        // BGE
        xor_blt->out >> *not_bge->in[0];
        not_bge->out >> multiplexer->get(CompOp::GE);
        // BLTU
        c_flag >> *not_bltu->in[0];
        not_bltu->out >> multiplexer->get(CompOp::LTU);
        // BGEU
        c_flag >> multiplexer->get(CompOp::GEU);

        comp_op >> multiplexer->select;
        multiplexer->out >> res;

    // clang-format on
  }

  INPUTPORT_ENUM(comp_op, CompOp);
  INPUTPORT(zero, 1);
  INPUTPORT(res_sign, 1);
  INPUTPORT(v_flag, 1);
  INPUTPORT(c_flag, 1);
  OUTPUTPORT(res, 1);

  SUBCOMPONENT(multiplexer, TYPE(EnumMultiplexer<CompOp, 1>));
  SUBCOMPONENT(xor_blt, TYPE(Xor<1, 2>));
  SUBCOMPONENT(not_bge, TYPE(Not<1, 1>));
  SUBCOMPONENT(not_bne, TYPE(Not<1, 1>));
  SUBCOMPONENT(not_bltu, TYPE(Not<1, 1>));
};

} // namespace core
} // namespace vsrtl
