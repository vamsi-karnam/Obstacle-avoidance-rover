# Obstacle Avoidance Rover

A Raspberry Pi–powered robot that uses OpenCV and Machine learning to detect obstacles and plot a course to avoid accidents. 
The setup uses a Raspberry Pi 3B, Pi camera, L298N motor driver, a couple of DC motors, and rechargeable batteries wired under a custom chassis.


## Software Requirements

* **Raspberry Pi OS (Buster or Bullseye)**
* **Python 3.x**
* **Libraries**:

  * `opencv-python`
  * `numpy`
  * `RPi.GPIO`

Dependencies:

```bash
sudo apt-get update
sudo apt-get install -y python3-opencv python3-rpi.gpio
pip3 install numpy
```

## Build and usage

1. **Clone the repo**

2. **Make script executable**

   ```bash
   chmod +x robot.py
   ```

3. **Run in interactive mode**

   * Connect a monitor/keyboard to see debug windows.
   * Press **Q** to quit.

   ```bash
   ./obstacle_avoider.py
   ```

4. **Run headless (no display)**
   Comment out or remove any `cv2.imshow()` and `cv2.waitKey()` lines in the script, then:

   ```bash
   ./obstacle_avoider.py
   ```


## Tuning Parameters

Within `obstacle_avoider.py`, you can adjust:

* `FRAME_WIDTH` & `FRAME_HEIGHT` – camera resolution
* `BLUR_KERNEL` – Gaussian blur strength
* `THRESHOLD_VALUE` – binary threshold cutoff
* `MIN_CONTOUR_AREA` – minimum pixels to treat as an obstacle
* PWM **speed** values in `forward()`, `turn_left()`, etc.

Experiment to find the best settings for your lighting and obstacle sizes.


## Working principle

1. **Capture** a frame from the Pi Camera.
2. **Grayscale + blur** to reduce noise.
3. **Threshold** to create a binary mask of obstacles.
4. **Find contours** and select the largest.
5. **Compute centroid**; if it’s left of center → turn right, else turn left.
6. **No obstacle** → drive forward.


## Troubleshoot common issues

* **“Could not open camera”**

  * Ensure camera interface is enabled: run `raspi-config` → Interface Options → Camera → Enable.
* **Motors not spinning**

  * Verify power supply voltage/current.
  * Check GPIO pin assignments and wiring.
* **Erratic behavior in low light**

  * Increase `BLUR_KERNEL` size or lower `THRESHOLD_VALUE`.
  * Add external illumination (LED strip).

