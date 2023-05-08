#pragma once

#include "VSRTL/core/vsrtl_adder.h"
#include "VSRTL/core/vsrtl_constant.h"
#include "VSRTL/core/vsrtl_design.h"
#include "VSRTL/core/vsrtl_logicgate.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "../../ripesvsrtlprocessor.h"

#include "../riscv.h"
#include "../rv_ecallchecker.h"
#include "../rv_immediate.h"
#include "../rv_decode.h"
#include "../rv_registerfile.h"

#include "rv_alu_zero.h"
#include "rv_multi_branch.h"
#include "rv_multi_control.h"
#include "rv_ir_reg.h"
#include "rv_read_control_rom.h"
#include "rv_write_control_register.h"
#include "rv_memory_read_enable.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <typename XLEN_T>
class RVMS : public RipesVSRTLProcessor {
  static_assert(std::is_same<uint32_t, XLEN_T>::value ||
                    std::is_same<uint64_t, XLEN_T>::value,
                "Only supports 32- and 64-bit variants");
  static constexpr unsigned XLEN = sizeof(XLEN_T) * CHAR_BIT;

public:
  RVMS(const QStringList &extensions)
      : RipesVSRTLProcessor("Multi Cycle RISC-V Processor") {
    m_enabledISA = std::make_shared<ISAInfo<XLenToRVISA<XLEN>()>>(extensions);
    ir_reg->setISA(m_enabledISA);
    uncompress->setISA(m_enabledISA);
    // -----------------------------------------------------------------------
    // Program counter
    pc_src->out >> pc_reg->in;
    pc_reg->out >> pc_cur->in;
    2 >> pc_inc->get(PcInc::INC2);
    4 >> pc_inc->get(PcInc::INC4);

    branch->res >> *pc_wr_and->in[0];
    control->PCWriteCond >> *pc_wr_and->in[1];

    pc_wr_and->out >> *pc_wr_or->in[0];
    control->PCWrite >> *pc_wr_or->in[1];

    pc_wr_or->out >> pc_reg->wr_en;
    pc_wr_or->out >> pc_cur->wr_en;

    control->PCSource >> pc_src->select;

    // -----------------------------------------------------------------------
    // Instruction memory
    pc_reg->out >> instr_mem->addr;
    instr_mem->setMemory(m_memory);
    control->InstrRead >> instr_mem->rd_en;

    // -----------------------------------------------------------------------
    // Instruction Register
    uncompress->exp_instr >> ir_reg->instr;
    control->IRWrite >> ir_reg->wr_en;

    // -----------------------------------------------------------------------
    // Control signals
    ir_reg->opcode >> control->opcode;

    // -----------------------------------------------------------------------
    // Immediate
    ir_reg->opcode >> immediate->opcode;
    ir_reg->exp_instr >> immediate->instr;

    // -----------------------------------------------------------------------
    // Registers

    // IR -> Register File
    ir_reg->wr_reg_idx >> registerFile->wr_addr;
    ir_reg->r1_reg_idx >> registerFile->r1_addr;
    ir_reg->r2_reg_idx >> registerFile->r2_addr;

    // Register File -> ALU'S aux registers (A & B)
    registerFile->r1_out >> a_reg->in;
    registerFile->r2_out >> b_reg->in;

    // Register file write control signal
    control->RegWrite >> registerFile->wr_en;

    // Connection between multiplexer "reg_wr_src" and Register File "Data In" port.
    reg_wr_src->out >> registerFile->data_in;

    // Memory data register
    data_mem->data_out >> mem_reg->in;

    // Data to be selected in "reg_wr_src" multiplexer
    mem_reg->out >> reg_wr_src->get(RegDstEnum::MemReg);
    alu_reg->out >> reg_wr_src->get(RegDstEnum::AluReg);
    pc_reg->out >> reg_wr_src->get(RegDstEnum::PC);
    immediate->imm >> reg_wr_src->get(RegDstEnum::IMM);

    // Multiplexer "reg_wr_src" control
    control->RegDst >> reg_wr_src->select;

    registerFile->setMemory(m_regMem);

    // Data to be selected in "pc_src" multiplexer
    alu->res >> pc_src->get(PCSourceEnum::AluOut);
    alu_reg->out >> pc_src->get(PCSourceEnum::AluReg);

    // -----------------------------------------------------------------------
    // ALU

    // Data to be selected in "alu_op1_src" multiplexer
    pc_cur->out >> alu_op1_src->get(SelAluAEnum::PC_MINUS4);
    a_reg->out >> alu_op1_src->get(SelAluAEnum::RegA);
    pc_reg->out >> alu_op1_src->get(SelAluAEnum::PC);
    control->SelAluA >> alu_op1_src->select;
    alu_op1_src->out >> alu->op1;

    // Data to be selected in "alu_op2_src" multiplexer
    b_reg->out >> alu_op2_src->get(SelAluBEnum::RegB);
    pc_inc->out >> alu_op2_src->get(SelAluBEnum::PCINC);
    immediate->imm >> alu_op2_src->get(SelAluBEnum::IMM);
    control->SelAluB >> alu_op2_src->select;
    alu_op2_src->out >> alu->op2;
    control->AluCtrl >> alu->ctrl;

    // ALU output -> ALU register
    alu->res >> alu_reg->in;

    // -----------------------------------------------------------------------
    // Data memory
    alu_reg->out >> data_mem->addr;
    control->MemRead >> data_mem->rd_en;

    control->MemWrite >> data_mem->wr_en;
    b_reg->out >> data_mem->data_in;
    control->MemCtrl >> data_mem->op;
    data_mem->mem->setMemory(m_memory);

    // -----------------------------------------------------------------------
    // Branch Detector
    alu->zero >> branch->zero;
    alu->res_sign >> branch->res_sign;
    alu->c_flag >> branch->c_flag;
    alu->v_flag >> branch->v_flag;
    control->BranchCtrl >> branch->comp_op;

    // -----------------------------------------------------------------------
    // Uncompress

    instr_mem->data_out >> uncompress->instr;
    uncompress->Pc_Inc >> pc_inc->select;

    // -----------------------------------------------------------------------
    // Ecall checker
    control->ecallOpcode >> ecallChecker->opcode;
    ecallChecker->setSyscallCallback(&trapHandler);
    0 >> ecallChecker->stallEcallHandling;
  }

  // Design subcomponents
  SUBCOMPONENT(registerFile, TYPE(RegisterFile<XLEN, false>));
  SUBCOMPONENT(alu, TYPE(ALUZero<XLEN>));
  SUBCOMPONENT(control, MultiControl);
  SUBCOMPONENT(immediate, TYPE(Immediate<XLEN>));
  SUBCOMPONENT(ir_reg, TYPE(IR_REG<XLEN>));
  SUBCOMPONENT(uncompress, TYPE(Uncompress<XLEN>));
  SUBCOMPONENT(branch, TYPE(MultiBranch<XLEN>));

  // Registers
  SUBCOMPONENT(pc_reg, WriteControlRegister<XLEN>);
  SUBCOMPONENT(pc_cur, WriteControlRegister<XLEN>);
  SUBCOMPONENT(a_reg, Register<XLEN>);
  SUBCOMPONENT(b_reg, Register<XLEN>);
  SUBCOMPONENT(mem_reg, Register<XLEN>);
  SUBCOMPONENT(alu_reg, Register<XLEN>);

  // Multiplexers
  SUBCOMPONENT(reg_wr_src, TYPE(EnumMultiplexer<RegDstEnum, XLEN>));
  SUBCOMPONENT(pc_src, TYPE(EnumMultiplexer<PCSourceEnum, XLEN>));
  SUBCOMPONENT(alu_op1_src, TYPE(EnumMultiplexer<SelAluAEnum, XLEN>));
  SUBCOMPONENT(alu_op2_src, TYPE(EnumMultiplexer<SelAluBEnum, XLEN>));
  SUBCOMPONENT(pc_inc, TYPE(EnumMultiplexer<PcInc, XLEN>));

  // Memories
  SUBCOMPONENT(instr_mem, TYPE(ReadControlROM<XLEN, c_RVInstrWidth>));
  SUBCOMPONENT(data_mem, TYPE(RVREMemory<XLEN, XLEN>));

  // Gates
  SUBCOMPONENT(pc_wr_and, TYPE(And<1, 2>));
  SUBCOMPONENT(pc_wr_or, TYPE(Or<1, 2>));

  // Address spaces
  ADDRESSSPACEMM(m_memory);
  ADDRESSSPACE(m_regMem);

  SUBCOMPONENT(ecallChecker, EcallChecker);

  // Ripes interface compliance
  const ProcessorStructure &structure() const override { return m_structure; }

  unsigned int getPcForStage(StageIndex idx) const override {
    return pc_cur->out.uValue();
  }

  AInt nextFetchedAddress() const override { return pc_src->out.uValue(); }

  QString stageName(StageIndex) const override { return "•"; }

  StageInfo stageInfo(StageIndex) const override {

    bool valid = true;
    // We need to check if the current state of the processor 
    // indicates that an instruction is going to be read (FETCH state).
    if (control->InstrRead) {
      // In that case, if the address of the instruction stored in the PC register is invalid
      // the isExecutableAddress() function will return false.
      valid = isExecutableAddress(pc_reg->out.uValue());
      return StageInfo({pc_reg->out.uValue(), valid, StageInfo::State::None});
    } else {
      // If the current state corresponds to an intermediate state of an instruction
      // we will return the content of Current PC register.
      valid = isExecutableAddress(pc_cur->out.uValue());
      return StageInfo({pc_cur->out.uValue(), valid, StageInfo::State::None});
    }
  }
  void setProgramCounter(AInt address) override {
    pc_reg->forceValue(0, address);
    pc_cur->forceValue(0, address);
    propagateDesign();
  }
  void setPCInitialValue(AInt address) override {
    pc_reg->setInitValue(address);
    pc_cur->setInitValue(address);
  }
  AddressSpaceMM &getMemory() override { return *m_memory; }
  VInt getRegister(RegisterFileType, unsigned i) const override {
    return registerFile->getRegister(i);
  }
  void finalize(FinalizeReason fr) override {
    if (fr == FinalizeReason::exitSyscall) {
      // Allow one additional clock cycle to clear the current instruction
      m_finishInNextCycle = true;
    }
  }
  bool finished() const override {
    return m_finished || !stageInfo({0, 0}).stage_valid;
  }
  const std::vector<StageIndex> breakpointTriggeringStages() const override {
    return {{0, 0}};
  }

  MemoryAccess dataMemAccess() const override {

    // If the read or write control signal is set, we will read the next memory address
    // (Data cache sim)
    if (control->MemRead || control->MemWrite) {
      return memToAccessInfo(data_mem);
    } else {
      MemoryAccess noneAccess;
      noneAccess.type = MemoryAccess::None;
      return noneAccess;
    }
  }
  
  MemoryAccess instrMemAccess() const override {
    // If the read control signal is set, we will read the next instruction
    // (Instruction cache sim)
    if (control->InstrRead) {
      MemoryAccess instrAccess = memToAccessInfo(instr_mem);
      instrAccess.type = MemoryAccess::Read;
      return instrAccess;
    } else {
      // En caso negativo, no leemos ninguna instrucción.
      MemoryAccess noneAccess;
      noneAccess.type = MemoryAccess::None;
      return noneAccess;
    }
  }

  void setRegister(RegisterFileType, unsigned i, VInt v) override {
    setSynchronousValue(registerFile->_wr_mem, i, v);
  }

  void clockProcessor() override {

    int instructionFinished = control->clock();
    if (instructionFinished) {
      m_instructionsRetired++;
    }

    // m_finishInNextCycle may be set during Design::clock(). Store the value
    // before clocking the processor, and emit finished if this was the final
    // clock cycle.
    const bool finishInThisCycle = m_finishInNextCycle;

    Design::clock();
    if (finishInThisCycle) {
      m_finished = true;
    }
  }

  void reverse() override {
    if (control->currentState == State::FETCH) {
      m_instructionsRetired--;
    }
      control->unclock();
      Design::reverse();
 
    // Ensure that reverses performed when we expected to finish in the
    // following cycle, clears this expectation.
    m_finishInNextCycle = false;
    m_finished = false;
  }

  void reset() override {
    Design::reset();
    control->reset();
    m_finishInNextCycle = false;
    m_finished = false;
  }

  static ProcessorISAInfo supportsISA() {
    return ProcessorISAInfo{
        std::make_shared<ISAInfo<XLenToRVISA<XLEN>()>>(QStringList()),
        {"M", "C"},
        {"M"}};
  }
  const ISAInfoBase *implementsISA() const override {
    return m_enabledISA.get();
  }
  const std::set<RegisterFileType> registerFiles() const override {
    std::set<RegisterFileType> rfs;
    rfs.insert(RegisterFileType::GPR);

    // @TODO: uncomment when enabling floating-point support
    // if (implementsISA()->extensionEnabled("F")) {
    //     rfs.insert(RegisterFileType::Float);
    // }
    return rfs;
  }

private:
  bool m_finishInNextCycle = false;
  bool m_finished = false;
  std::shared_ptr<ISAInfoBase> m_enabledISA;
  ProcessorStructure m_structure = {{0, 1}};
};

} // namespace core
} // namespace vsrtl*/