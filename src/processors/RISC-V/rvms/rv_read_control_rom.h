#pragma once

#include "VSRTL/core/vsrtl_memory.h"
#include "VSRTL/core/vsrtl_wire.h"
#include "../riscv.h"

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
        lastSavedAddr = addr.uValue();
        auto val = this->read(
            _addr, dataWidth / CHAR_BIT,
            ceillog2((byteIndexed ? addrWidth : dataWidth) / CHAR_BIT));
        return val;
      } else {
        auto _addr = lastSavedAddr;
        auto val = this->read(
            _addr, dataWidth / CHAR_BIT,
            ceillog2((byteIndexed ? addrWidth : dataWidth) / CHAR_BIT));
        return val;
      }
    };
  }

  AddressSpace::RegionType accessRegion() const override {
    return this->memory()->regionType(addr.uValue());
  }

  virtual VSRTL_VT_U addressSig() const override { return lastSavedAddr; };
  virtual VSRTL_VT_U wrEnSig() const override { return 0; };

  VSRTL_VT_U lastSavedAddr = 0x00000000;

  INPUTPORT(addr, addrWidth);
  INPUTPORT(rd_en, 1);

  OUTPUTPORT(data_out, dataWidth);
};

} // namespace core
} // namespace vsrtl
