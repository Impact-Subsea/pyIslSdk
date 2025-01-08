# Impact Subsea Python Sdk

The Software Development Kit (SDK) is an open source software library that simplifies the development of an application intended to communicate with Impact Subsea products. The SDK provides the following:<br>
- Ability to open, close and configure the computer’s serial and network ports.
- Interact with Serial over LAN (SOL) or (network to serial adapters).
- Automatically discover Impact Subsea devices connected to the computer.
- Automatically detect NMEA devices, such as a GPS connected to the computer serial ports.
- Dynamically creates objects to manage each discovered device and allow the user’s application to interact with the device via this object.
- Log all connected Impact Subsea devices and GPS data to a log file.
- Play back of log files.

The SDK is written in C++17 and cmake is used for compiling the project. It has been written in a cross-platform way with embedded credit card computers in mind, such as the Raspberry Pi and Beagle Bone type systems. The minimum hardware specification is:<br>
- 100Mhz 32-bit processor with FPU (Floating Point Unit)
- 32MB RAM

This Python wapper project wraps the C++ SDK using the open source project pybind11.

## Compiling

### Using Microsoft Visual Studios with cmake
1. Open Microsoft Visual Studios and click `Continue without code`.
2. Open the `CmakeLists.txt` file from the menu `File->Open->CMake..`
3. Select the startup object from the dropdown green compile and run button.
4. Click the Compile / Run button.

### Linux, Windows using cmake

1. Make sure you have `cmake` and `python` installed. If not already installed with python you'll need the python-dev package

    ```bash
    $ apt-get install python-dev
    ```

2. Clone the git repo onto your local storage.

3. Change into root repo directory:

    ```bash
    $ cd pyIslSdk
    ```

4. Clone the git submodules.

    ```bash
    $ git submodule update --init --recursive --remote
    ```

5. Create a new build directory and change into it:

    ```bash
    $ mkdir build
    $ cd build
    ```

6. Run cmake on the parent directory to generate makefile:

    ```bash
    $ cmake -DCMAKE_BUILD_TYPE=Debug ..
    or
    $ cmake -DCMAKE_BUILD_TYPE=Release ..
    ```

7. Run make on the generated makefile to generate the python pyd library:

    ```bash
    $ make
    ```
    If using windows
    ```bash
    $ cmake --build . --config Debug
    or
    $ cmake --build . --config Release

8. Copy the .pyd file from the folder pyIslSdk/build/Debug/ or pyIslSdk/build/Release/ into your python project and use import pyIslSdk in your python project.
