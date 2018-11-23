# sequence_publisher
Image sequence publisher package for ASL image sequences

**Author:** [David Zuñiga-Noël](http://mapir.isa.uma.es/mapirwebsite/index.php/people/270)

**License:**  [GPLv3](LICENSE.txt)

## 1. Dependencies (tested)

* CMake (3.5.1-1ubuntu1)
   ```
   sudo apt install cmake
   ```
* ROS (Kinetic Kame with catkin workspace)

   See [ROS installation](http://wiki.ros.org/kinetic/Installation) and [catkin documentation](http://wiki.ros.org/catkin).

* Boost (1.58.0.1ubuntu1)
   ```
   sudo apt install libboost-all-dev
   ```
* Gflags (2.1.2-3)
   ```
   sudo apt install libgflags-dev
   ```
* OpenCV (from ROS)
   
## 2. Build

Once all dependencies are installed, proceed to build the source code within a catkin workspace (see [catkin tutorials](http://wiki.ros.org/catkin/Tutorials)).

## 3. Data Format

The input timestamp association file should be in CSV format, specified by (ASL format):
```
timestamp,filename
```
with timestamp in nanoseconds.

## 4. Usage

The `sequence_publisher` tool can invoked as follows:
```
rosrun sequence_publisher sequence_publisher [options] <images_directory> <timestamps_file> <topic>
```
where the available options are:

* `-o sequence_offset`: to specify a starting sequence index

* `-s sequence_skip`: to decimate the sequence by skiping frames

* `-n sequence_length`: to set the maximum of frames published (0 for unlimited)

* `--rate=frame_rate`: to set the (minimum) frame publishing rate

