# SJTU JiaoLong RM2018 Armor Detection

## State Machine

- Fast Explore(default state): Explore the armor when the robot is moving fast
- Fast Track: Track the armor detected in Fast Explore
- Slow Explore: After the robot slows down, explore the armor again
- Slow Track: Track the armor detected in Slow Explore

## Explore Algorithm

- Find the light region(for Gray Camera) or the blue/red region(for Color Camera)
- Use `findContour` to find the contours
- Find the thin and long contours, which may be the side light of armor
- Pair the lights to find the armor(length, angle and so on)

## Track Algorithm

- KCFTracker: the balance of speed(60 fps) and accuracy.

## Speedup Algorithm

- OpenMP: Process the image and fetch next image at the same time.

## Ways to improve

- Use Machine Learning to pair the lights
- Use Object Detection to explore the armor
- Recognize the digit in the center

## PS

- We use a special camera with **Global Shutter**, so the driver is special. It is wrapped in `include/GlobalCamera.h`.

- We find a kcftracker of cpp version instead of the one in the OpenCV, because it runs much faster. It is wrapped in `src/kcftracker.cpp`.

- To distinguish with the PC platform and MiniPC platform, there is a macro in the `precom.h`. If the CPU is ARM architecture, then it is regarded as the MiniPC platform.

- There are some macros in the `precom.h` to switch the **OpenMP**, **Show Image** and **Record Videos**. And when the OpenMP is on, there is no way to show the image.

- To improve the performance of RGB camera, we process the raw data with BAYER format. To learning more, search the Bayer. It is opened by macro `BAYER_HACKING`.