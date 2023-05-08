#pragma once

#include "../riscv.h"
#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;


template <unsigned int W>
class WriteControlRegister : public RegisterBase {
public:
  SetGraphicsType(Register);

  WriteControlRegister(const std::string &name, SimComponent *parent)
      : RegisterBase(name, parent) {
    // Calling out.propagate() will clock the register the register
    out << ([=] { return m_savedValue; });
  }

  void setInitValue(VSRTL_VT_U value) { m_initvalue = value; }

  void reset() override {
    m_savedValue = m_initvalue;
    m_reverseStack.clear();
  }

  void save() override {
    saveToStack();
    if (wr_en.uValue()) {
      m_savedValue = in.uValue();
    }
  }

  void forceValue(VSRTL_VT_U /* addr */, VSRTL_VT_U value) override {
    // Sign-extension with unsigned type forces width truncation to m_width bits
    m_savedValue = signextend<W>(value);
    // Forced values are a modification of the current state and thus not pushed
    // onto the reverse stack
  }

  void reverse() override {
    if (m_reverseStack.size() > 0) {
      m_savedValue = m_reverseStack.front();
      m_reverseStack.pop_front();
    }
  }

  PortBase *getIn() override { return &in; }
  PortBase *getOut() override { return &out; }

  INPUTPORT(in, W);
  INPUTPORT(wr_en, 1);
  OUTPUTPORT(out, W);

  void reverseStackSizeChanged() override {
    if (reverseStackSize() < m_reverseStack.size()) {
      m_reverseStack.resize(m_reverseStack.size());
    }
  }

protected:
  void saveToStack() {
    m_reverseStack.push_front(m_savedValue);
    if (m_reverseStack.size() > reverseStackSize()) {
      m_reverseStack.pop_back();
    }
  }

  VSRTL_VT_U m_savedValue = 0;
  VSRTL_VT_U m_initvalue = 0;
  std::deque<VSRTL_VT_U> m_reverseStack;
};

} // namespace core
} // namespace vsrtl
