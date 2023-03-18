#pragma once

#include "VSRTL/core/vsrtl_memory.h"
#include "VSRTL/core/vsrtl_wire.h"
#include "../riscv.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned int addrWidth, unsigned int dataWidth>
class RVREMemory : public Component, public BaseMemory<true> {
public:
  SetGraphicsType(ClockedComponent);
  RVREMemory(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    addr >> mem->addr;
    wr_en >> mem->wr_en;
    data_in >> mem->data_in;

    wr_width->setSensitiveTo(&op);
    wr_width->out << [=] {
      switch (op.uValue()) {
      case MemOp::SB:
        return 1;
      case MemOp::SH:
        return 2;
      case MemOp::SW:
        return 4;
      case MemOp::SD:
        return 8;
      default:
        return 0;
      }
    };
    wr_width->out >> mem->wr_width;

    data_out << [=] { 

        if (rd_en) {
          const auto &value = mem->data_out.uValue();
          switch (op.uValue()) {
              case MemOp::LB:
                currentData = VT_U(signextend<8>(value & 0xFFUL));
              case MemOp::LBU:
                currentData = value & 0xFFUL;
              case MemOp::LH:
                currentData = VT_U(signextend<16>(value & 0xFFFFUL));
              case MemOp::LHU:
                currentData = value & 0xFFFFUL;
              case MemOp::LWU:
                currentData = value & 0xFFFFFFFFUL;
              case MemOp::LW:
                currentData = VT_U(signextend<32>(value));
              case MemOp::LD:
                currentData = value;
              default:
                currentData = value;
          }
        } 
        return currentData;
    };
  }

  void setMemory(AddressSpace *addressSpace) {
    setMemory(addressSpace);
    mem->setMemory(addressSpace);
  }

  /* VSRTL_VT_U getCurrentData() {
    if (rd_en) {
      const auto &value = mem->data_out.uValue();
      switch (op.uValue()) {
      case MemOp::LB:
        currentData = VT_U(signextend<8>(value & 0xFFUL));
        return currentData
      case MemOp::LBU:
        currentData = value & 0xFFUL;
        return currentData
      case MemOp::LH:
        currentData = VT_U(signextend<16>(value & 0xFFFFUL));
        return currentData
      case MemOp::LHU:
        currentData = value & 0xFFFFUL;
        return currentData
      case MemOp::LWU:
        currentData = value & 0xFFFFFFFFUL;
        return currentData
      case MemOp::LW:
        currentData = VT_U(signextend<32>(value));
        return currentData
      case MemOp::LD:
        currentData = value;
        return currentData
      default:
        currentData = value;
        return currentData
      }

    } else {
      return currentData;
    }
  }*/

  // RVMemory is also a BaseMemory... A bit redundant, but RVMemory has a notion
  // of the memory operation that is happening, while the underlying
  // MemoryAsyncRd does not.
  VSRTL_VT_U addressSig() const override { return addr.uValue(); };
  VSRTL_VT_U wrEnSig() const override { return wr_en.uValue(); };
  VSRTL_VT_U opSig() const override { return op.uValue(); };
  AddressSpace::RegionType accessRegion() const override {
    return mem->accessRegion();
  }

  VSRTL_VT_U currentData = 0x0000000000000000;

  SUBCOMPONENT(mem, TYPE(MemoryAsyncRd<addrWidth, dataWidth>));

  WIRE(wr_width, ceillog2(dataWidth / 8 + 1)); // Write width, in bytes

  INPUTPORT(addr, addrWidth);
  INPUTPORT(data_in, dataWidth);
  INPUTPORT(wr_en, 1);
  INPUTPORT(rd_en, 1);
  INPUTPORT_ENUM(op, MemOp);
  OUTPUTPORT(data_out, dataWidth);
};

} // namespace core
} // namespace vsrtl
