# 2022-Robot

[![CI](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/ci.yml/badge.svg)](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/ci.yml) [![Format](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/format.yml/badge.svg)](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/format.yml) [![Documentation](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/doxygen.yml/badge.svg)](https://github.com/FRC1756-Argos/2022-Robot/actions/workflows/doxygen.yml)

Robot code for 2022 FRC Season

## Key Features

 * Automated intake system with ball indexing [`IntakeSubsystem`](src/main/include/subsystems/intake_subsystem.h)
   * Vibration feedback on cargo acquisition
 * Semi-autonomous high-rung climb with configurable positions [`ClimberSubsystem`](src/main/include/subsystems/climber_subsystem.h)
 * Modular swerve drive system built on [SDS Mk4](https://www.swervedrivespecialties.com/products/mk4-swerve-module) hardware and Falcon 500 motors [`SwerveDriveSubsystem`](src/main/include/subsystems/swerve_drive_subsystem.h)
   * Custom module state optimizer with support for continuous module rotation [`argos_lib::swerve::Optimize`](src/argos_lib/general/swerve_utils.h)
   * Persistent module homes using non-volatile memory [`FileSystemHomingStorage`](src/main/include/utils/file_system_homing_storage.h)
 * 370-degree turret with full-field vision targeting [`ShooterSubsystem`](src/main/include/subsystems/shooter_subsystem.h)
   * Vibration feedback on target lock
   * Automatic hood angle and flywheel speed setpoints based on distance interpolation maps from 10 inches to 30 feet [`shooterRange`](src/main/include/constants/interpolation_maps.h)
   * Target range estimation using target pitch and perspective distortion adjustment [`ShooterSubsystem::AutoAim`](src/main/include/subsystems/shooter_subsystem.h)
   * Automatic hood homing [`HomeHoodCommand`](src/include/commands/home_hood_command.h)
   * Persistent turret home using non-volatile memory [`FSHomingStorage`](src/main/include/utils/homing_storage_interface.h)
 * Modular autonomous selection system [`AutoSelector`](src/main/include/utils/auto_selector.h)
   * Autonomous routines for all starting positions with 1-ball, 2-ball, 5-ball, and 2-ball with defensive shots [commands/autonomous](src/main/include/commands/autonomous)
   * Support for separate red and blue alliance position setpoints to account for field assembly variance
   * Common absolute field position configuration for drive paths to reduce duplication [`field_points`](src/main/include/constants/field_points.h)

## Commissioning A Robot

When commissioning a new robot, you should set the instance type to either "Competition" or "Practice" by creating a text file readable by `lvuser` on the RoboRIO at path `/home/lvuser/robotInstance`.  The content of this file should be just the text `Competition` or `Practice` with no whitespace preceding.  If no valid instance is found at runtime, competition instance will be used and an error will be generated.

### Vision

See [vision readme](vision/README.md) for information on which pipelines to use and which indices to install these pipelines on.

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

  Make sure to run `pip install <package>` commands in an administrator terminal if installing in windows

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
| DPad Up         | Unused|
| DPad Right      | Move Pre-Climb Forward One Position |
| DPad Down       | Confirm climb |
| DPad Left       | Move Pre-Climb Backward One Position |
| A               | Home Swerve (hold with <kbd>B</kbd> and <kbd>X</kbd>) |
| B               | Home Swerve (hold with <kbd>A</kbd> and <kbd>X</kbd>) |
| X               | Home Swerve (hold with <kbd>A</kbd> and <kbd>B</kbd>) |
| Y               | Field Home (hold) |
| LB              | Hold For Robot Centric |
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
| Y               | Home Hood (hold) |
| DPad Up         | Front Close Up Shot |
| DPad Right      | Right Close Up Shot |
| DPad Down       | Back Close Up Shot |
| DPad Left       | Left Close Up Shot |
| LB              | Move Pre-Climb Backward One Position |
| RB              | Move Pre-Climb Forward One Position |
| LT              | Unused |
| RT              | Vision Target |
| Back            | Swap (hold with <kbd>Start</kbd>) |
| Start           | Swap (hold with <kbd>Back</kbd>) |
| Left JS Button  | Unused |
| Right JS Button | Unused |

## Software Checkout
1. Check turret position
2. Check wheel tread
3. Drive and Turn
    1. Drive forward
    2. Drive backward
    3. Drive left
    4. Drive right
    5. Turn left
    6. Turn right
    7. Drive forward
4. Intake
    1. Intake a ball
    2. Outtake a ball
5. Shooter
    1. Move turret left and right manually
    2. Move hood up and down manually
    3. Check the four setpoints (using the d-pad on the operator controller) for the proper turret positions
    4. Aim (vision; make sure camera is working)
    5. Intake and shoot a ball (this also checks the elevator functionality)
6. Climber
    1. Move the climb arms up and down
    2. Move the hooks up and down
    3. Go through the climb sequence
7. Wait for air
8. Change battery

## Software Versions

As of the end of the 2022 season, we're using the following dependencies:

 * [CTRE Phoenix 5.21.1](https://github.com/CrossTheRoadElec/Phoenix-Releases/releases/tag/v5.21.1.0)
 * [Limelight 2022.2.3](https://docs.limelightvision.io/en/latest/software_change_log.html)
 * [Playing With Fusion 2022.03.03](https://www.playingwithfusion.com/docview.php?docid=1205&catid=9012)
 * [REVLib 2022.1.1](https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information#changelog)
 * [WPILib 2022.4.1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2022.4.1)

## Special Thanks

 * Our sponsors for the 2022 season.  Thank you for your continued support
   * [Caterpillar](https://www.caterpillar.com/)
   * [J. H. Benedict Co](https://www.jhbenedict.com/)
   * [Limestone Community High School](https://www.limestone310.org/)
   * [Playing With Fusion](https://www.playingwithfusion.com/)
   * [Wadi Powder Coating](https://www.facebook.com/Wadipowdercoating/)
 * [Doxygen Awesome](https://jothepro.github.io/doxygen-awesome-css/) - for making our [documentation](https://frc1756-argos.github.io/2022-Robot/) look great

## License
This software is licensed under the [BSD 3-clause license](https://opensource.org/licenses/BSD-3-Clause). If you would like to use this software under the terms of a different license agreement, please [contact us](mailto:1756argos1756@limestone310.org).
