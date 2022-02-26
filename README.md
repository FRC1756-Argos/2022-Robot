# 2022-Robot

[![CI](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/ci.yml/badge.svg)](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/ci.yml) [![Format](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/format.yml/badge.svg)](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/format.yml) [![Documentation](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/doxygen.yml/badge.svg)](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/doxygen.yml)

Robot code for 2022 FRC Season

## Commissioning A Robot

When commissioning a new robot, you should set the instance type to either "Competition" or "Practice" by creating a text file readable by `lvuser` on the RoboRIO at path `/home/lvuser/robotInstance`.  The content of this file should be just the text `Competition` or `Practice` with no whitespace preceding.  If no valid instance is found at runtime, competition instance will be used and an error will be generated.

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
       * To type in Vim, type <kbd>i</kbd> and you should see `INSERT` at the bottom of the window to indicate you're editing in insert mode
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

## Controls

**Driver:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | Drive |
| Left JS Y       | Drive |
| Right JS X      | Turn |
| Right JS Y      | Unused |
| DPad Up         | Initiates Climb|
| DPad Right      | Unused |
| DPad Down       | Redo Climb |
| DPad Left       | Retract Hook |
| A               | Home Swerve (hold with <kbd>B</kbd> and <kbd>X</kbd>) |
| B               | Home Swerve (hold with <kbd>A</kbd> and <kbd>X</kbd>) |
| X               | Home Swerve (hold with <kbd>A</kbd> and <kbd>B</kbd>) |
| Y               | Field Home (hold) |
| LB              | Field- vs. Robot-Centric Drive |
| RB              | Reverse Intake |
| LT              | Shoot |
| RT              | Intake |
| Back            | Swap (hold with <kbd>Start</kbd>) |
| Start           | Swap (hold with <kbd>Back</kbd>) |
| Left JS Button  | Unused |
| Right JS Button | Unused |

**Operator:**
| Button          | Function |
| --------------- | -------- |
| Left JS X       | Manual Aim |
| Left JS Y       | Manual Hood |
| Right JS X      | Climbing Hooks |
| Right JS Y      | Climbing Arm |
| A               | Home Turret (hold with <kbd>X</kbd> and <kbd>B</kbd>) + Lower Arm |
| B               | Home Turret (hold with <kbd>X</kbd> and <kbd>A</kbd>) + Extend Hook |
| X               | Home Turret (hold with <kbd>A</kbd> and <kbd>B</kbd>) + Retract Hook |
| Y               | Raise Arm |
| DPad Up         | Front Close Up Shot |
| DPad Right      | Right Close Up Shot |
| DPad Down       | Back Close Up Shot |
| DPad Left       | Left Close Up Shot |
| LB              | Unused |
| RB              | Unused |
| LT              | Unused |
| RT              | Vision Target |
| Back            | Swap (hold with <kbd>Start</kbd>) |
| Start           | Swap (hold with <kbd>Back</kbd>) |
| Left JS Button  | Unused |
| Right JS Button | Unused |
