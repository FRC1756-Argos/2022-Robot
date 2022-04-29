# Vision Pipelines

This directory contains vision pipelines to detect the 2022 vision target in order to determine target distances and direction.

We initially explored using [PhotonVision](https://photonvision.org/) using [SnakeEyes](https://www.playingwithfusion.com/productview.php?pdid=134&catid=1009), but we had various issues with that platform and switched to [Limelight](https://limelightvision.io/).

The pipelines are detailed below.

## Limelight

In the `limelight` directory, we have three pipelines:
 * `Driver.vpr` - This pipeline should be installed on pipeline index 1.  It turns off the LEDs and allows the operator to manually aim without blinding anyone
 * `Targeting.vpr` - This pipeline should be installed on pipeline index 0.  It outputs a track at the center of the vision target group and a bounding box.
 * `customArgos.py` - This is an experimental custom pipeline that is unused.  The goal was for this pipeline to account for perspective distortion, but results were better doing this on the RoboRIO.

## SnakeEyes

In the `snakeEyes_red` directory, we have:
 * `pipelines`
   * `redTest.json` - This pipeline tracks all tape pieces on the vision target using the far-red Snake Eyes board.
 * `config.json` - This is the general configuration for PhotonVision
 * `drivermode.json` - This is the configuration for driver mode with LEDs off.
