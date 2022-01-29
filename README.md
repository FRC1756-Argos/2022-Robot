# 2022-Robot

[![CI](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/ci.yml/badge.svg)](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/ci.yml) [![Format](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/format.yml/badge.svg)](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/format.yml) [![Documentation](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/doxygen.yml/badge.svg)](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/doxygen.yml)

Robot code for 2022 FRC Season

## Project Setup

### Pre-Commit

This project uses [pre-commit](https://pre-commit.com/) to check code formatting before accepting commits.

First install the prerequisites:

* python3 (with pip) - [instructions](https://realpython.com/installing-python/)
  * Python 3.9.x from the [Python website](https://www.python.org/downloads/) works well.  Make sure to check the add to path option in the installer.
* pip packages:
  * You may need to add the pip install path to your shell's path if you're using Git Bash.  In git bash:
    1. Open (or create) new file `~/.bashrc` by running `vim ~/.bashrc`
    2. Add this to the end: `PATH=$PATH:$LOCALAPPDATA/Programs/Python/Python39/Scripts/` (change `Python39` to match your python version)
       * **Note**: The actual path you need to add (`$LOCALAPPDATA/Programs/Python/Python39/Scripts/` in the above example) depends on your Python installation.  If y ou do the `pip install` steps first, pip will print the path you need to add.
       * To type in Vim, type Mkbd>i</kbd> and you should see `INSERT` at the bottom of the window to indicate you're editing in insert mode
    3. Exit by pressing <kbd>Esc</kbd> then type `:wq` and press <kbd>Enter</kbd>
    4. Run `source ~/.bashrc` to update your session
  * wpiformat - `pip install wpiformat`
  * clang-format - `pip install clang-format`
  * pre-commit - `pip install pre-commit`

Then initialize:

```
pre-commit install
pre-commit run
```

The first run may take a moment, but subsequent automatic runs are very fast.

You'll now have the linter run before each commit!  For compatibility with Windows, we recommend the pip version of clang-format, but wpi-format will find any installed `clang-format` binary in the system path.

## controls

**Driver:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | drive |
| Left JS Y       | drive |
| Right JS X      | turn |
| Right JS Y      | unused |
| DPad Up         | initiates climb|
| DPad Right      | unused |
| DPad Down       | redo climb |
| DPad Left       | retract hook |
| A               | Unused |
| B               | Unused |
| X               | Unused |
| Y               | Unused |
| LB              | Unused |
| RB              | reverse intake |
| LT              | shooting |
| RT              | intake |
| Back            | Unused |
| Start           | Unused |
| Left JS Button  | Unused |
| Right JS Button | Unused |

**operator:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | manual aiming |
| Left JS Y       | manual hood |
| Right JS X      | climbing hooks |
| Right JS Y      | climbing arm |
| DPad Up         | raises arms |
| DPad Right      | extend hook |
| DPad Down       | lowers arms |
| DPad Left       | retract hook |
| A               | Unused |
| B               | Unused |
| X               | Unused |
| Y               | raises frame |
| LB              | unused |
| RB              | Unused |
| LT              | Unused |
| RT              | vision |
| Back            | Unused |
| Start           | Unused |
| Left JS Button  | Unused |
| Right JS Button | Unused |
