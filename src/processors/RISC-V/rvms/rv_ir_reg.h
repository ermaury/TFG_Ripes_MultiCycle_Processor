#pragma once

#include "../riscv.h"
#include "../rv_decode.h"
#include "../rv_uncompress.h"
#include "VSRTL/core/vsrtl_component.h"
#include "rv_write_control_register.h"


namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class IR_REG : public Component {
public:
  void setISA(const std::shared_ptr<ISAInfoBase> &isa) {
    decode->setISA(isa);
  }

  IR_REG(std::string name, SimComponent *parent) : Component(name, parent) {
    instr >> reg->in;

    wr_en >> reg->wr_en;
    reg->out >> decode->instr;
    reg->out >> exp_instr;

    decode->opcode >> opcode;
    decode->wr_reg_idx >> wr_reg_idx;
    decode->r1_reg_idx >> r1_reg_idx;
    decode->r2_reg_idx >> r2_reg_idx;
  }

  SUBCOMPONENT(decode, TYPE(Decode<XLEN>));
  SUBCOMPONENT(reg, WriteControlRegister<c_RVInstrWidth>);

  INPUTPORT(instr, c_RVInstrWidth);
  INPUTPORT(wr_en, 1);
  OUTPUTPORT_ENUM(opcode, RVInstr);
  OUTPUTPORT(wr_reg_idx, c_RVRegsBits);
  OUTPUTPORT(r1_reg_idx, c_RVRegsBits);
  OUTPUTPORT(r2_reg_idx, c_RVRegsBits);
  OUTPUTPORT(exp_instr, c_RVInstrWidth);
};

} // namespace core
} // namespace vsrtl
