# Plan B: record on the lab robot's Pi (borrowed, read-mostly)

Use only if a moving route is longer than the extension cord. We borrow the
robot and its Pi to record camera data; we never change the team's code,
workspace, services, `~/.bashrc` or network settings.

## Ask the team first

1. Permission to plug in our D435i, SSH in, install two apt packages, and
   record into a new folder in the home directory. May the packages stay
   installed afterwards?
2. Output of `lsb_release -ds`, `echo $ROS_DISTRO`, `df -h ~`, and whether
   `ros-$ROS_DISTRO-realsense2-camera` / librealsense is already installed
   (from apt or built from source — a source build must not be replaced).
3. Does their stack use a camera, auto-start ROS nodes at boot, or depend on a
   particular `ROS_DOMAIN_ID`? Is a USB 3 (blue) port free?
4. SSH user and IP on the lab Wi-Fi. **They type the sudo password themselves**
   (agents never enter passwords); ideally they run the install line below.
5. Who drives or pushes the robot, and is the robot battery fine with ~2 W extra.

## Access (decided 2026-10-23)

- Preferred: leave the team's microSD in the robot's Pi, power the robot, and
  SSH over the lab Wi-Fi with the Mac on the same network. No card swapping.
- If the card must go into our Pi: inspect it first **read-only** through a
  USB card reader while our Pi boots from its own card; boot from their card
  only for installing/recording, then put it back in the robot.
- Login: the owner or team runs `ssh-copy-id <user>@<ip>` and types the
  password themselves; Claude 2 then uses key login. That adds one line to
  their `~/.ssh/authorized_keys`; remove it at cleanup if the team wants.

## Install (team runs, once; leave installed for later sessions)

```bash
sudo apt update && sudo apt install -y ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-rosbag2-storage-mcap
```

If `rs-enumerate-devices` cannot open the camera, the udev rule is missing;
the team decides whether to add Intel's `99-realsense-libusb.rules`.
Never run `realsense-viewer` firmware update prompts; no firmware change.

## What Claude 2 does over SSH

1. Read-only checks: OS/ROS versions, disk space, `lsusb -t` (5000M), running
   ROS processes, installed RealSense packages.
2. Copy `borrowed_pi_capture.sh` into `~/wbr-capture-20261024/`.
3. A 10 s test: `bash borrowed_pi_capture.sh rgbd phase-0-test 10`; check topic
   counts in `bag-info.txt` before the real runs.
4. Real runs with the same IDs as the checklist (M1 stereo, M2 RGB-D, ...).
   The script uses ROS domain 87, so it is invisible to the robot's own nodes.
5. Copy each run folder to the Mac with `rsync`, then verify `SHA256SUMS`.

## Cleanup

- After the checksums match on the Mac, ask the owner before deleting
  `~/wbr-capture-20261024/` from the borrowed Pi (only our folder).
- Leave the two apt packages installed (owner's choice, for later sessions);
  tell the team exactly what was added. To remove later:
  `sudo apt remove ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-rosbag2-storage-mcap`.
- Nothing else is changed, so nothing else needs undoing.
