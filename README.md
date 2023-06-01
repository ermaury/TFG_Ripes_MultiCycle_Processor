# Modified Version of Ripes simulator to include RISC-V multicycle processor.

This project is a modified version of the original repository [Ripes](https://github.com/mortbopet/Ripes), which implements a RISC-V multicycle processor.

## Description

The goal of this project is to extend the original repository to add additional features and specific enhancements related to the implementation of a RISC-V multicycle processor.

## Modified Features

- New VSRTL components such as write control registers, read control memory (instrucction and data), ALU flags to check conditional branches.

- New multicycle processor model added ([state diagram](./src/processors/RISC-V/rvms/StateDiagram/RVI32_StateMachine.drawio.pdf)).

## Compilation

This project follows the same compilation guidelines as the original repository [Ripes](https://github.com/mortbopet/Ripes).


Ripes is a visual computer architecture simulator and assembly code editor built for the [RISC-V instruction set architecture](https://content.riscv.org/wp-content/uploads/2017/05/riscv-spec-v2.2.pdf).


## Usage
Ripes may be used to explore concepts such as:
- How machine code is executed on a variety of microarchitectures (RV32IMC/RV64IMC based)
- How different cache designs influence performance
- How C and assembly code is compiled and assembled to executable machine code
- How a processor interacts with memory-mapped I/O

If this is your first time using Ripes, please refer to the [introduction/tutorial](docs/introduction.md).  
For further information, please refer to the [Ripes documentation](docs/README.md).

## Downloading & Installation
Prebuilt binaries are available for Linux, Windows & Mac through the [Releases page](https://github.com/mortbopet/Ripes/releases).  

### Linux
Releases for Linux are distributed in the AppImage format. To run an AppImage:
* Run `chmod a+x` on the AppImage file
* Run the file!
The AppImage for Linux should be compatible with most Linux distributions.

### Windows
For Windows, the C++ runtime library must be available (if not, a msvcp140.dll error will be produced). You most likely already have this installed, but if this is not the case, you download it [here](https://www.microsoft.com/en-us/download/details.aspx?id=48145).

## Building
Initially, the following dependencies must be made available:
- A recent (>=5.15) version of [Qt](https://www.qt.io/download) + Qt Charts (**not** bundled with Qt by default, but can be selected during Qt installation)
- [CMake](https://cmake.org/)

Then, Ripes can be checked out and built as a standard CMake project:
```
git clone --recursive https://github.com/ermaury/TFG_Ripes_MultiCycle_Processor.git
cd Ripes/
cmake .
Unix:               Windows:
make                jom.exe / nmake.exe / ...
```
Note, that you must have Qt available in your `CMAKE_PREFIX_PATH`. For further information on building Qt projects with CMake, refer to [Qt: Build with CMake](https://doc.qt.io/qt-5/cmake-manual.html).