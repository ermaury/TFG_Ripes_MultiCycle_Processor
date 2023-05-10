#pragma once

#include "../riscv.h"
#include "VSRTL/core/vsrtl_memory.h"
#include "VSRTL/core/vsrtl_wire.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned int addrWidth, unsigned int dataWidth,
          bool byteIndexed = true>
class ReadControlROM : public Component, public BaseMemory<byteIndexed> {
  template <unsigned int, unsigned int>
  friend class Memory;

public:
  SetGraphicsType(ClockedComponent);
  ReadControlROM(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    data_out << [=] {
      if (rd_en) {
        auto _addr = addr.uValue();
        auto val = this->read(
            _addr, dataWidth / CHAR_BIT,
            ceillog2((byteIndexed ? addrWidth : dataWidth) / CHAR_BIT));
        return val;
      } else {
        return reg->out.uValue();
      }
    };

    data_out >> reg->in;
    rd_en >> reg->wr_en;
  }

  AddressSpace::RegionType accessRegion() const override {
    return this->memory()->regionType(addr.uValue());
  }

  virtual VSRTL_VT_U addressSig() const override { return addr.uValue(); };
  virtual VSRTL_VT_U wrEnSig() const override { return 0; };

  INPUTPORT(addr, addrWidth);
  INPUTPORT(rd_en, 1);
  OUTPUTPORT(data_out, dataWidth);

  SUBCOMPONENT(reg, WriteControlRegister<dataWidth>);
};

} // namespace core
} // namespace vsrtl
