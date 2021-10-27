# pbs-project
https://docs.google.com/presentation/d/1uvZ8ysqVHNmmRGEixD8TgCCD3PRpIat2T0_aYzaLbSk/edit#slide=id.gb16a76d00a_0_0

Authors: Elham Amin Mansour and Akshan Naraya

This version of the PBS framework has been modified to run a computationally demanding rigid body simulation without a GUI, and can therefore be run on systems such as ETH's Euler cluster.

Set up (assuming current directory is `pbs-project`):
```
mkdir build
cd build
cmake ..
```
On Windows, use the CMake gui with the buttons Configure and Generate.

Build the `build/PBS.sln` using either `make` (Ubuntu) or an IDE of choice.

Run the compiled executable from the command line:
```
bowling_alley.exe <number-of-frames-to-record> <number-of-frames-to-ignore-at-beginning>
```
e.g. `bowling_alley.exe 1000 10` would ignore the first 10 frames of computation, and then record 1000 frames after that (each frame corresponds to a time step).

Generate PNGs using Blender:
```
<path-to-Blender-executable> -b euler.blend -P createPNG.py <number-of-objects-in-scene> <number-of-frames-to-record> <number-of-frames-to-ignore-at-beginning>
```
e.g. `/Applications/Blender\ 2.app/Contents/MacOS/Blender -b scene.blend -P createPNG.py -- 25 450 10`

Generate video using [ffmpeg](https://ffmpeg.org/) CLI:
```
ffmpeg -r 1/3 -i img_%01d.png -c:v libx264 -vf fps=25 -pix_fmt yuv420p out.mp4
```

The video is often quite long, so the following command can be used to speed it up:
```
ffmpeg -i out.mp4 -filter:v "setpts=0.01*PTS" output.mp4
```
