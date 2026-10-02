# Caboose

**Keep good data loggers working.**

Caboose is a Linux utility for communicating with legacy Onset H07/H08 data loggers. It provides three command-line tools and a GTK3 graphical application for identifying loggers, retrieving measurements, and starting new deployments.

## Why this project exists

A manufacturer's decision to declare a product obsolete should not have to end its useful life. These loggers remain serviceable, useful instruments. They are good hardware! When the hardware still works, we should be able to continue using it—even after the original software and manufacturer support have disappeared.

That is why I embarked on Caboose: to restore a practical way to use equipment that still has work to do, rather than retire it because its software belongs to an earlier computing era.

I chose Linux for development for two reasons: I use it all the time, and it makes interacting with a serial port and testing communications easy. Direct access to the serial device, small diagnostic programs, and familiar C development tools made Linux a natural place to investigate the interface and build a replacement utility.

Caboose was developed by the author as a hobby project on my spare time.  I have recently used it to launch H08 external voltage loggers to test rechargeable AA Li-Ion batteries.  I hope others find it useful.  

## Hardware and scope

Development centered on the H08-006-04 four-channel external voltage logger. The GUI also includes support for H07 event loggers, which record a timestamp for each contact closure, such as provided by a reed switch on a tipping-bucket rain gauge. The command-line launch and read utilities are oriented toward H8 sampled-data loggers; the GUI's H07 support should not be taken to imply equivalent H07 support in every command-line tool.

You need a compatible logger interface cable with 1/8" stereo headphone jack and a serial port or suitable USB-to-serial adapter. Communication normally uses **1200 baud, 8 data bits, no parity, and 1 stop bit (8N1)**, with DTR and RTS asserted. The default Linux device is `/dev/ttyUSB0`.

Compatibility with every H7/H8 model, memory size, sensor, and adapter has not been established. Some header interpretations and timing details remain inferred; the source identifies such assumptions. Preserve raw memory images so they can be decoded again as support improves.

> **Before connecting a running logger:** Caboose may send a three-second serial BREAK to make it respond. This ends the current deployment. Reading a running logger is therefore a stop and offload operation, not a way to monitor it while it continues logging. Launching a new deployment starts at the beginning of memory and overwrites the previous deployment; retrieve and preserve old data first. Even a launch dry run can wake and stop a running logger.

### The data cable is part of the operating sequence

**Unplugging and re-plugging the data cable at the logger is an operating signal, not merely a way to connect a peripheral.** These legacy loggers use the cable connection as part of the transition between communication and deployment. Complete a launch by removing the data cable from the logger; reconnect it when you are ready to communicate with the logger and offload the deployment.

Here, **unplug** means remove the data-cable plug from the **logger itself**. Closing a program, closing the serial port, or unplugging the USB adapter from the computer is not a substitute for that physical action. The USB-to-serial adapter can remain connected to the computer throughout.

Use this sequence:

1. Insert the data cable fully into the logger before running a CLI operation.
2. Keep it connected until the command has completed. Do not interrupt a header write or memory offload by removing the cable.
3. After a successful launch, **unplug the cable from the logger** to complete the transition to deployment. Do not immediately reconnect it to check the launch: communication may wake and stop the new deployment.
4. When the deployment is ready to be retrieved, **re-plug the cable into the logger**, then run `h8_read`. The program supplies the wake/BREAK sequence needed to communicate with a running logger; this ends the deployment.
5. Wait until the offload has finished and the files have been saved, then unplug the cable from the logger. To start another deployment, reconnect it and perform a new launch.

The cable transition and the serial protocol work together. Re-plugging is the physical step for returning to communication; the software's wake/BREAK sequence is also significant. Unplugging after an offload does **not** constitute a new launch or resume the deployment that was stopped.

The GUI prompts for these actions. With the CLI utilities, you must perform them yourself; command completion alone does not replace the unplug/re-plug sequence.

## Building on Linux

The command-line tools require a C99 compiler and `make`. The GUI additionally requires GTK3 development headers and libraries, their GLib dependencies, and `pkg-config`.

From the directory containing the sources and Makefile:

```sh
make all       # Build h8_probe, h8_read, and h8_launch
make gui       # Build the Caboose GTK3 application
make clean     # Remove executables and object files
```

Plain `make` builds the command-line tools. The Makefile defaults to GCC with `-std=c99 -Wall -Wextra -O2`. Compiler settings can be overridden, for example:

```sh
make all CC=gcc CFLAGS='-std=c99 -Wall -Wextra -O0 -g'
```

Check that GTK3 is available before building the GUI:

```sh
pkg-config --modversion gtk+-3.0
```

The Makefile builds in place; it does not install the programs. Run them with `./program_name`, or place the executables somewhere on your `PATH`.

Your account must have permission to access the serial device. On many Linux systems this is managed through the `dialout` group; the applicable group and procedure depend on the distribution. Also ensure no other application has the port open.

## Command-line utilities

Each utility provides its current usage description through `--help`.

### h8_launch — start an H8 deployment

```sh
./h8_launch -i INTERVAL_S -c CHANNELS [options]
```

By default, this is a **dry run**: it reads the existing header, saves `h8_header_<serial>_<unixtime>.bin` in the current directory, and reports the bytes it would change. It writes launch settings only when `--commit` is supplied. The header backup is not a backup of the complete measurement record.

| Argument | Meaning |
| --- | --- |
| `-i`, `--interval=SECONDS` | Required sampling interval: 0.5–32767.5 seconds, in multiples of 0.5 seconds. |
| `-c`, `--channels=LIST` | Required comma-separated list of channels 1–4, such as `1` or `1,2,3,4`. |
| `-t`, `--description=TEXT` | Deployment description, printable ASCII, at most 40 characters. Default: `Linux launch`. |
| `-U`, `--utc` | Store launch time as UTC. Default: local standard time, without daylight saving adjustment. |
| `-C`, `--commit` | Write the launch settings and start the deployment. |
| `-R`, `--readback` | Read and compare the header after launch. Disabled by default because waking the newly launched logger may stop its deployment. |
| `-d`, `--device=PATH` | Serial device. Default: `/dev/ttyUSB0`. |
| `-h`, `--help` | Show help and exit. |

Preview a five-second, single-channel deployment, then launch it:

```sh
./h8_launch -d /dev/ttyUSB0 -i 5 -c 1 -t 'Battery test'
./h8_launch -d /dev/ttyUSB0 -i 5 -c 1 -t 'Battery test' -C
```

**Cable handling for this command:** connect the logger before the dry run or committed launch and leave it connected until the program exits. After the committed launch succeeds, **remove the data cable from the logger** and leave it disconnected during the deployment. Reconnect it when you are ready to end the deployment and retrieve the data with `h8_read`. Avoid `-R` for normal deployment: its immediate readback may stop the logger before you have unplugged it. A dry run does not arm a new deployment; unplugging after a dry run does not turn it into a committed launch.

Local standard time can be one hour behind the wall clock during daylight saving time. Keep a record of the time convention used for each deployment; do not assume a decoded timestamp is automatically converted between UTC and local time.

### h8_read — offload memory or decode a saved image

```sh
./h8_read [options]
./h8_read -f IMAGE.bin [options]
```

A live offload writes `PREFIX.bin`, preserving the bytes received from the logger, and `PREFIX.csv`, containing record index, elapsed seconds, timestamps, and raw 8-bit counts for the recorded channels. It prints header information and a checksum check to the terminal. A full 32 KB offload takes about six minutes at 1200 baud.

| Argument | Meaning |
| --- | --- |
| `-o`, `--output=PREFIX` | Output filename prefix. Default: `h8`. |
| `-p`, `--pages=N` | Read N 256-byte pages; accepted range 1–512. Default: 128 pages, or 32 KB. Choose a count appropriate to the logger's memory. |
| `-f`, `--file=IMAGE.bin` | Decode an existing memory image without contacting a logger. The input image is not rewritten. |
| `-c`, `--channels=N` | Override channel interpretation with consecutive channels 1 through N. Otherwise use the enabled-channel bits in the header. |
| `-d`, `--device=PATH` | Serial device. Default: `/dev/ttyUSB0`. |
| `-h`, `--help` | Show help and exit. |

```sh
./h8_read -d /dev/ttyUSB0 -o deployment_001
./h8_read -p 2 -o short_test
./h8_read -f deployment_001.bin -o decoded_again
```

**Cable handling for a live offload:** re-plug the data cable into the logger, run `h8_read`, and keep the cable connected until the program finishes writing the output files. Then unplug it from the logger. If you next intend to launch it again, re-plug the cable before running `h8_launch`, and unplug again after the successful launch. Offline decoding with `-f` requires no logger or cable connection.

A partial read contains only the requested pages and may omit later records. The channel override specifies a count, not an arbitrary channel list: `-c 2` means channels 1 and 2. Use distinct output prefixes to avoid replacing existing files. Keep the `.bin` image as the primary archive; a CSV is a decoded interpretation of that image.

### h8_probe — inspect serial communications

```sh
./h8_probe [options] TOKEN [TOKEN ...]
```

This diagnostic tool sends tokens in order and prints received bytes in hexadecimal and ASCII. It is useful for checking an adapter, investigating response timing, and examining protocol replies.

| Token | Meaning |
| --- | --- |
| `D`, `E`, or other letters | Send ASCII bytes. Separate tokens allow replies to be collected between commands; `DE` sends both bytes together. |
| `0xNN` | Send one hexadecimal byte, such as `0x45`. |
| `sleepNNNN` | Pause NNNN milliseconds. |
| `breakNNNN` | Hold serial BREAK for NNNN milliseconds. |

| Argument | Meaning |
| --- | --- |
| `-i`, `--idle=MS` | Stop collecting a reply after this much silence. Default: 500 ms. |
| `-w`, `--wait=MS` | Pause after each send before listening. Default: 0 ms. |
| `-t`, `--timing` | Print arrival timing for received chunks. |
| `-n`, `--no-handshake` | De-assert DTR and RTS; both are asserted by default. |
| `-2`, `--two-stop-bits` | Use 8N2 for comparison testing instead of 8N1. |
| `-d`, `--device=PATH` | Serial device. Default: `/dev/ttyUSB0`. |
| `-h`, `--help` | Show help and exit. |

```sh
./h8_probe D sleep500 D
./h8_probe -t D sleep100 D A
./h8_probe break3000 sleep300 D
```

**Cable handling for probing:** plug the cable fully into the logger before sending tokens, and keep it connected throughout the probe. For a fresh connection test, unplug and re-plug at the logger before running the next probe. Do not probe a deployment you intend to leave running: the connection and any wake/BREAK sequence belong to the communication workflow and can disturb the deployment. A successful probe is not a launch; removing the cable afterward does not start a new deployment.

`D` wakes the logger and resets the memory-page pointer; `A` requests identification; `E` requests the next 256-byte memory page. These commands do not write launch settings, but a BREAK can still end a running deployment. Do not experiment with `B`, `F`, `H`, `I`, or `M`: they can change settings, start a deployment, or change the baud rate. The probe sends the requested bytes; it does not protect against inappropriate commands.

## The Caboose GUI

Build with `make gui`, then start:

```sh
./caboose
```

The GUI uses `h8_protocol.c` directly; it does not need to invoke the three command-line executables. Serial operations run in a worker thread so the window remains responsive during an offload.

After a successful launch or offload, the GUI prominently instructs **UNPLUG THE LOGGER**. Follow that instruction at the logger end of the cable before continuing to another operation or deployment. When returning to retrieve a deployment, reconnect the logger as directed.

The interface guides the user through connecting a logger, inspecting its identity and settings, launching an idle logger, or offloading an existing deployment. H8 launch controls include sampling interval, channel selection, description, and time convention. For an H07 event logger, interval and channel controls are hidden because the instrument records contact-closure timestamps instead of regularly sampled analog records.

Offloads can be saved as a raw binary image, CSV, or both. For analog data, the GUI includes an adjustable linear conversion:

```text
volts = counts × volts_per_count + offset
```

Settings—including serial device, output folder, calibration, and the last interval—are saved in `caboose.conf` under GLib's user configuration directory, normally `~/.config/caboose.conf` on Linux.

### Viewing data with tsviewer

The **View data** button launches `tsviewer` with the saved CSV filename. Install it separately and ensure the executable is on your `PATH`:

**[tsviewer — source and build instructions](https://github.com/fred-ogden/tsviewer)**

Like Caboose, tsviewer was written with GTK3, natively on Linux. It can also be compiled on Windows using MinGW. It is an optional companion: Caboose can offload and save data without it. The configured executable name is the `DATA_VIEWER_PROGRAM` constant in `caboose.c`.

## Hardware-dependent defaults and assumptions

Many settings can be changed without recompiling. Others are constants that should be reviewed when adapting Caboose to a different logger or platform.

| Setting | Where to adjust it |
| --- | --- |
| Serial device, initially `/dev/ttyUSB0` | CLI `-d` option or GUI device setting. To change the compiled default, edit `DEFAULT_SERIAL_DEVICE_PATH` in the relevant program. |
| CLI memory read size, initially 128 pages / 32 KB | `h8_read -p N`; review memory handling for a different model. |
| Recorded-channel interpretation | Launch `-c LIST`; read `-c N` only when an override is appropriate. GUI selections depend on decoded hardware information. |
| Analog voltage calibration | GUI calibration controls. Defaults in `h8_protocol.h` are 0.00994 V/count and 0 V offset; verify them for your sensor and logger. The CLI reader exports raw counts. |
| Serial speed and line control | Serial setup routines in the CLI sources and `h8_protocol.c`. These are protocol/interface settings, not arbitrary performance options. |
| BREAK duration, timeouts, retries, and polling | Named constants near the tops of the relevant source files. Review if an adapter behaves differently. |
| Viewer program | `DATA_VIEWER_PROGRAM` in `caboose.c`, initially `tsviewer`. |
| Header formats, memory offsets, and timestamp encoding | Model-specific decoding in the reader and shared protocol source; confirm against hardware before extending support. |

A different serial device name alone does not require source changes. A different logger protocol or sensor calibration may. Avoid changing fixed protocol values simply because an offload seems slow.

## Prospects for Windows and MinGW

A Windows version is a reasonable future goal. GTK3 provides a viable route for the interface, and the Windows/MinGW build of tsviewer is an encouraging example for the plotting companion.

**Caboose itself is not yet a verified native Windows build.** Its serial backend currently uses POSIX/Linux facilities such as `termios`, `ioctl`, file descriptors, and serial BREAK controls. MinGW does not automatically translate these into Windows serial operations. A native port would need a Windows COM-port backend that implements reads, writes, timeouts, DTR/RTS, and BREAK with equivalent behavior. Calendar/time functions such as `timegm` and the `timezone` variable also need a portability review.

With those changes, building the GTK3 frontend using MinGW should be feasible. The build would also need suitable GTK3/GLib development packages, a Windows-aware Makefile, and the required runtime DLLs. Serial testing on physical loggers would remain essential, particularly for the BREAK sequence and launch timing. Merely changing `/dev/ttyUSB0` to a COM-port name is not sufficient.

Contributions toward that port are welcome.

## Protocol provenance and acknowledgments

Caboose is an independently written C implementation intended to restore operation of legacy hardware. Protocol knowledge was reconstructed using published information, experiments with physical loggers, examination of logger memory images, and analysis of legacy software behavior. Some interpretations remain inferred rather than independently confirmed on every supported model.

Tonu Samuel and Andrei Errapart's published work (2002) contributed information about serial settings and the D/E/C/H commands. No code from their program is used in this implementation. Caboose's source describes the hardware protocol rather than reproducing the internal architecture of earlier applications.

The author received a lot of very useful help from OpenAI ChatGPT GPT-6.1 Sol Light that accelerated the development, and helped produce this documentation.  

## Disclaimer

Caboose is an independent personal hobby project not associated with the Author's employment. It is not affiliated with, endorsed by, or supported by Onset or any hardware manufacturer. Manufacturer names and model numbers are used solely to identify compatible equipment.

The software is provided **as is**, without warranty. Users are responsible for checking compatibility, calibration, timestamps, and decoded results before relying on measurements. Preserve existing data before launching a new deployment, and validate unfamiliar hardware with a short test deployment before collecting important observations. The applicable warranty and liability terms are those of the license.

## License

Copyright 2026 Fred Ogden.

Caboose is licensed under the **Apache License, Version 2.0**, as identified by the `SPDX-License-Identifier: Apache-2.0` notices in the source files. See the [full Apache License 2.0 text](https://www.apache.org/licenses/LICENSE-2.0) for permissions, conditions, and limitations. Redistributed source and binaries must comply with that license, including its notice requirements.
