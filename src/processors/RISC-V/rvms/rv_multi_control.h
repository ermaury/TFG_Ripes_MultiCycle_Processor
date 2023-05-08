#pragma once

#include "../riscv.h"
#include "VSRTL/core/vsrtl_component.h"

enum State {
  FETCH,
  DECODE,
  EX_typeR,
  EX_typeRImm,
  EX_LUI,
  MEM_typeR,
  EX_typeMem,
  MEM_Load,
  WB_Load,
  MEM_Store,
  EX_branch,
  EX_jal,
  EX_jalR,
  ECALL
};

enum Signal {
  IRWrite,
  InstrRead,
  MemRead,
  RegDst,
  RegWrite,
  PCWrite,
  PCWriteCond,
  PCSource,
  SelAluA,
  SelAluB,
  MemWrite,
  AluCtrl,
  EcallOpcode,
};

enum InstrType {
  RType,
  IType,
  LoadType,
  StoreType,
  BranchType,
  AUIPC,
  LUI,
  JalType,
  JalRType,
  Syscall,
  None
};

namespace vsrtl {
namespace core {
using namespace Ripes;

class MultiControl : public Component {
public:
  /* clang-format off */

    static InstrType get_instr_type(const VSRTL_VT_U& opc) {
        switch (opc) {
        // R-TYPE
        // 
          // RV32I
          case RVInstr::ADD: case RVInstr::SUB:
          case RVInstr::AND: case RVInstr::OR:
          case RVInstr::SLL: case RVInstr::SLT:
          case RVInstr::SLTU: case RVInstr::SRL:
          case RVInstr::SRA: case RVInstr::XOR:
          // RV32M
          case RVInstr::MUL: case RVInstr::MULH:
          case RVInstr::MULHU: case RVInstr::MULHSU:
          case RVInstr::DIV: case RVInstr::DIVU:
          case RVInstr::REM: case RVInstr::REMU:
          // RV64I
          case RVInstr::ADDW: case RVInstr::SUBW:
          case RVInstr::SLLW: case RVInstr::SRLW: 
          case RVInstr::SRAW: 
          // RV64M
          case RVInstr::MULW: case RVInstr::DIVW: 
          case RVInstr::DIVUW: case RVInstr::REMW: 
          case RVInstr::REMUW:
              return InstrType::RType;
        // I-TYPE
        // 
          // RV32I
          case RVInstr::ADDI: case RVInstr::SLTI:
          case RVInstr::ANDI: case RVInstr::SLTIU:
          case RVInstr::XORI: case RVInstr::SLLI:
          case RVInstr::ORI: case RVInstr::SRLI:
          case RVInstr::SRAI: 
          // RV64I  
          case RVInstr::ADDIW:
          case RVInstr::SLLIW: case RVInstr::SRLIW: 
          case RVInstr::SRAIW: 
              return InstrType::IType;
        
        // RV32I
        // 
        // B-TYPE (branches)
        //
          case RVInstr::BEQ: case RVInstr::BNE:
          case RVInstr::BLT: case RVInstr::BGE:
          case RVInstr::BGEU: case RVInstr::BLTU:
            return InstrType::BranchType;
        // JAL-TYPE
          case RVInstr::JAL: 
              return InstrType::JalType;
        // JALR-TYPE
          case RVInstr::JALR: 
              return InstrType::JalRType;
        // LOAD-TYPE
          case RVInstr::LB: case RVInstr::LH:
          case RVInstr::LW: case RVInstr::LBU:
          case RVInstr::LHU: case RVInstr::LWU:
          case RVInstr::LD:
              return InstrType::LoadType;
        // STORE-TYPE
          case RVInstr::SB: case RVInstr::SH:
          case RVInstr::SW: case RVInstr::SD:
             return InstrType::StoreType;
         // U-TYPE
          case RVInstr::AUIPC:
              return InstrType::AUIPC;
          case RVInstr::LUI:
              return InstrType::LUI;
         // ECALL
          case RVInstr::ECALL:
              return InstrType::Syscall;
        }
       return InstrType::None;
    }

    static CompOp do_comp_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            case RVInstr::BEQ: return CompOp::EQ;
            case RVInstr::BNE: return CompOp::NE;
            case RVInstr::BLT: return CompOp::LT;
            case RVInstr::BGE: return CompOp::GE;
            case RVInstr::BLTU: return CompOp::LTU;
            case RVInstr::BGEU: return CompOp::GEU;
            default: return CompOp::NOP;
        }
    }

    static MemOp do_mem_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            case RVInstr::SB: return MemOp::SB;
            case RVInstr::SH: return MemOp::SH;
            case RVInstr::SW: return MemOp::SW;
            case RVInstr::SD: return MemOp::SD;
            case RVInstr::LB: return MemOp::LB;
            case RVInstr::LH: return MemOp::LH;
            case RVInstr::LW: return MemOp::LW;
            case RVInstr::LD: return MemOp::LD;
            case RVInstr::LBU: return MemOp::LBU;
            case RVInstr::LHU: return MemOp::LHU;
            case RVInstr::LWU: return MemOp::LWU;
            default:
                return MemOp::NOP;
        }
    }


    static ALUOp do_alu_ctrl(const VSRTL_VT_U& opc) {
        switch(opc) {
            case RVInstr::LB: case RVInstr::LH: case RVInstr::LW: case RVInstr::LBU: case RVInstr::LHU:
            case RVInstr::SB: case RVInstr::SH: case RVInstr::SW: case RVInstr::LWU: case RVInstr::LD:
            case RVInstr::SD:
                return ALUOp::ADD;
            case RVInstr::LUI:
                return ALUOp::LUI;
            case RVInstr::JAL: case RVInstr::JALR: case RVInstr::AUIPC:
            case RVInstr::ADD: case RVInstr::ADDI:
            case RVInstr::BEQ: case RVInstr::BNE: case RVInstr::BLT:
            case RVInstr::BGE: case RVInstr::BLTU: case RVInstr::BGEU:
                return ALUOp::ADD;
            case RVInstr::SUB: return ALUOp::SUB;
            case RVInstr::SLT: case RVInstr::SLTI:
                return ALUOp::LT;
            case RVInstr::SLTU: case RVInstr::SLTIU:
                return ALUOp::LTU;
            case RVInstr::XOR: case RVInstr::XORI:
                return ALUOp::XOR;
            case RVInstr::OR: case RVInstr::ORI:
                return ALUOp::OR;
            case RVInstr::AND: case RVInstr::ANDI:
                return ALUOp::AND;
            case RVInstr::SLL: case RVInstr::SLLI:
                return ALUOp::SL;
            case RVInstr::SRL: case RVInstr::SRLI:
                return ALUOp::SRL;
            case RVInstr::SRA: case RVInstr::SRAI:
                return ALUOp::SRA;
            case RVInstr::MUL   : return ALUOp::MUL;
            case RVInstr::MULH  : return ALUOp::MULH;
            case RVInstr::MULHU : return ALUOp::MULHU;
            case RVInstr::MULHSU: return ALUOp::MULHSU;
            case RVInstr::DIV   : return ALUOp::DIV;
            case RVInstr::DIVU  : return ALUOp::DIVU;
            case RVInstr::REM   : return ALUOp::REM;
            case RVInstr::REMU  : return ALUOp::REMU;
            case RVInstr::ADDIW : return ALUOp::ADDW;
            case RVInstr::SLLIW : return ALUOp::SLW;
            case RVInstr::SRLIW : return ALUOp::SRLW;
            case RVInstr::SRAIW : return ALUOp::SRAW;
            case RVInstr::ADDW  : return ALUOp::ADDW ;
            case RVInstr::SUBW  : return ALUOp::SUBW ;
            case RVInstr::SLLW  : return ALUOp::SLW ;
            case RVInstr::SRLW  : return ALUOp::SRLW ;
            case RVInstr::SRAW  : return ALUOp::SRAW ;
            case RVInstr::MULW  : return ALUOp::MULW ;
            case RVInstr::DIVW  : return ALUOp::DIVW ;
            case RVInstr::DIVUW : return ALUOp::DIVUW;
            case RVInstr::REMW  : return ALUOp::REMW ;
            case RVInstr::REMUW : return ALUOp::REMUW;

            default: return ALUOp::NOP;
        }
    }

  /* clang-format on */

public:
  MultiControl(const std::string &name, SimComponent *parent)
      : Component(name, parent) {

    IRWrite << [=] { return signalTable[currentState][Signal::IRWrite]; };
    InstrRead << [=] { return signalTable[currentState][Signal::InstrRead]; };
    MemRead << [=] { return signalTable[currentState][Signal::MemRead]; };
    RegDst << [=] { return signalTable[currentState][Signal::RegDst]; };
    RegWrite << [=] { return signalTable[currentState][Signal::RegWrite]; };
    PCWrite << [=] { return signalTable[currentState][Signal::PCWrite]; };
    PCWriteCond << [=] { return signalTable[currentState][Signal::PCWriteCond]; };
    PCSource << [=] { return signalTable[currentState][Signal::PCSource]; };
    SelAluA << [=] { return signalTable[currentState][Signal::SelAluA]; };
    SelAluB << [=] { return signalTable[currentState][Signal::SelAluB]; };
    MemWrite << [=] { return signalTable[currentState][Signal::MemWrite]; };
    AluCtrl << [=] { return signalTable[currentState][Signal::AluCtrl]; };
    MemCtrl << [=] { return do_mem_ctrl(opcode.uValue()); };
    state << [=] { return currentState; };
    BranchCtrl << [=] { return do_comp_ctrl(opcode.uValue()); };
    ecallOpcode << [=] { return signalTable[currentState][Signal::EcallOpcode]; };
  }

  State currentState = State::FETCH;
  std::list<State> listStates;
  long long m_cycles_count = 0;

  INPUTPORT_ENUM(opcode, RVInstr);

  OUTPUTPORT_ENUM(ecallOpcode, RVInstr);
  OUTPUTPORT(PCWriteCond, 1);
  OUTPUTPORT(PCWrite, 1);
  OUTPUTPORT_ENUM(RegDst, RegDstEnum);
  OUTPUTPORT(IRWrite, 1);
  OUTPUTPORT(InstrRead, 1);
  OUTPUTPORT(MemRead, 1);
  OUTPUTPORT_ENUM(BranchCtrl, CompOp);
  OUTPUTPORT_ENUM(PCSource, PCSourceEnum);
  OUTPUTPORT_ENUM(SelAluA, SelAluAEnum);
  OUTPUTPORT_ENUM(SelAluB, SelAluBEnum);
  OUTPUTPORT(RegWrite, 1);
  OUTPUTPORT_ENUM(MemCtrl, MemOp);
  OUTPUTPORT_ENUM(state, StateEnum);
  OUTPUTPORT(MemWrite, 1);
  OUTPUTPORT_ENUM(AluCtrl, ALUOp);

  int clock() {
    listStates.push_back(currentState);
    m_cycles_count++;
    currentState = getNextState(currentState, get_instr_type(opcode.uValue()));
    refreshTable();
    return currentState == State::FETCH;
  }

  int unclock() {
    currentState = listStates.back();
    m_cycles_count--;
    listStates.pop_back();
  }

  int reset() {
    currentState = State::FETCH;
    m_cycles_count = 0;
    listStates.clear();
  }

  State getNextState(State currentState, InstrType instrType) {
    if (currentState == State::DECODE || currentState == State::EX_typeMem) {
      return stateMachine[std::pair{currentState, instrType}];
    } else
      return stateMachine[std::pair{currentState, _}];
  }

  // MAQUINA DE ESTADOS
  using StateKey = std::pair<State, InstrType>;
  InstrType _ = InstrType::None;
  std::map<StateKey, State> stateMachine = {
      // CICLO 1
      {{State::FETCH, _}, State::DECODE},
      // CICLO 2
      {{State::DECODE, InstrType::RType}, State::EX_typeR},
      {{State::DECODE, InstrType::IType}, State::EX_typeRImm},
      {{State::DECODE, InstrType::AUIPC}, State::MEM_typeR},
      {{State::DECODE, InstrType::LUI}, State::EX_LUI},
      {{State::DECODE, InstrType::BranchType}, State::EX_branch},
      {{State::DECODE, InstrType::JalType}, State::EX_jal},
      {{State::DECODE, InstrType::JalRType}, State::EX_jalR},
      {{State::DECODE, InstrType::Syscall}, State::ECALL},
      {{State::DECODE, InstrType::StoreType}, State::EX_typeMem},
      {{State::DECODE, InstrType::LoadType}, State::EX_typeMem},
      // CICLO 3
      {{State::EX_typeR, _}, State::MEM_typeR},
      {{State::EX_typeRImm, _}, State::MEM_typeR},
      {{State::EX_LUI, _}, State::FETCH},
      {{State::EX_branch, _}, State::FETCH},
      {{State::EX_jal, _}, State::FETCH},
      {{State::EX_jalR, _}, State::FETCH},
      {{State::EX_typeMem, InstrType::LoadType}, State::MEM_Load},
      {{State::EX_typeMem, InstrType::StoreType}, State::MEM_Store},
      // CICLO 4
      {{State::MEM_Load, _}, State::WB_Load},
      {{State::MEM_Store, _}, State::FETCH},
      {{State::MEM_typeR, _}, State::FETCH},
      // CICLO 5
      {{State::WB_Load, _}, State::FETCH}};

  // CURRENT STATE CONTROL SIGNALS

  using ControlSignals = std::map<Signal, int>;
  using SignalTable = std::unordered_map<State, ControlSignals>;

  void refreshTable() {
    if (currentState == State::EX_typeR || currentState == State::EX_typeRImm) {
      signalTable[currentState][Signal::AluCtrl] = do_alu_ctrl(opcode.uValue());
    }
  }

  SignalTable signalTable = {
      {State::FETCH,
       {{Signal::IRWrite, true},
        {Signal::InstrRead, true},
        {Signal::MemRead, false},
        {Signal::RegDst, false},
        {Signal::RegWrite, false},
        {Signal::PCWrite, true},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, PCSourceEnum::AluOut},
        {Signal::SelAluA, SelAluAEnum::PC},
        {Signal::SelAluB, SelAluBEnum::PCINC},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::ADD}}},

      {State::DECODE,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, false},
        {Signal::RegWrite, false},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, SelAluAEnum::PC_MINUS4},
        {Signal::SelAluB, SelAluBEnum::IMM},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::ADD}}},

      {State::EX_typeR,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, false},
        {Signal::RegWrite, false},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, SelAluAEnum::RegA},
        {Signal::SelAluB, SelAluBEnum::RegB},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, do_alu_ctrl(opcode.uValue())}}},

      {State::EX_typeRImm,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, false},
        {Signal::RegWrite, false},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, SelAluAEnum::RegA},
        {Signal::SelAluB, SelAluBEnum::IMM},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, do_alu_ctrl(opcode.uValue())}}},

      {State::EX_LUI,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, RegDstEnum::IMM},
        {Signal::RegWrite, true},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, false},
        {Signal::SelAluB, false},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::NOP}}},

      {State::MEM_typeR,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, RegDstEnum::AluReg},
        {Signal::RegWrite, true},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, false},
        {Signal::SelAluB, false},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::NOP}}},

      {State::EX_typeMem,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, false},
        {Signal::RegWrite, false},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, SelAluAEnum::RegA},
        {Signal::SelAluB, SelAluBEnum::IMM},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::ADD}}},

      {State::MEM_Load,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, true},
        {Signal::RegDst, false},
        {Signal::RegWrite, false},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, false},
        {Signal::SelAluB, false},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::NOP}}},

      {State::WB_Load,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, RegDstEnum::MemReg},
        {Signal::RegWrite, true},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, false},
        {Signal::SelAluB, false},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::NOP}}},

      {State::MEM_Store,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, false},
        {Signal::RegWrite, false},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, false},
        {Signal::SelAluB, false},
        {Signal::MemWrite, true},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::NOP}}},

      {State::EX_branch,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, false},
        {Signal::RegWrite, false},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, true},
        {Signal::PCSource, PCSourceEnum::AluReg},
        {Signal::SelAluA, SelAluAEnum::RegA},
        {Signal::SelAluB, SelAluBEnum::RegB},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::SUB}}},

      {State::EX_jal,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, RegDstEnum::PC},
        {Signal::RegWrite, true},
        {Signal::PCWrite, true},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, PCSourceEnum::AluReg},
        {Signal::SelAluA, false},
        {Signal::SelAluB, false},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::NOP}}},

      {State::EX_jalR,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, RegDstEnum::PC},
        {Signal::RegWrite, true},
        {Signal::PCWrite, true},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, PCSourceEnum::AluOut},
        {Signal::SelAluA, SelAluAEnum::RegA},
        {Signal::SelAluB, SelAluBEnum::IMM},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::NOP},
        {Signal::AluCtrl, ALUOp::ADD}}},

      {State::ECALL,
       {{Signal::IRWrite, false},
        {Signal::InstrRead, false},
        {Signal::MemRead, false},
        {Signal::RegDst, false},
        {Signal::RegWrite, false},
        {Signal::PCWrite, false},
        {Signal::PCWriteCond, false},
        {Signal::PCSource, false},
        {Signal::SelAluA, false},
        {Signal::SelAluB, false},
        {Signal::MemWrite, false},
        {Signal::EcallOpcode, RVInstr::ECALL},
        {Signal::AluCtrl, ALUOp::NOP}}},

  };
};
} // namespace core
} // namespace vsrtl