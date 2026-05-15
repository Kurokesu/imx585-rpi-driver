# IMX585 kernel driver for Raspberry Pi

[![Build](https://github.com/Kurokesu/imx585-rpi-driver/actions/workflows/build-rpi.yml/badge.svg)](https://github.com/Kurokesu/imx585-rpi-driver/actions/workflows/build-rpi.yml)
[![Raspberry Pi OS Bookworm](https://img.shields.io/badge/Raspberry_Pi_OS-Bookworm-blue?logo=raspberrypi)](https://www.debian.org/releases/bookworm/)
[![Raspberry Pi OS Trixie](https://img.shields.io/badge/Raspberry_Pi_OS-Trixie-blue?logo=raspberrypi)](https://www.debian.org/releases/trixie/)
[![Kernel 6.12+](https://img.shields.io/badge/kernel-6.12%2B-blue?logo=raspberrypi)](https://github.com/raspberrypi/linux/tree/rpi-6.12.y)

Raspberry Pi kernel driver for Sony IMX585, an 8.3 MP STARVIS 2 back-side illuminated CMOS sensor optimised for low-light and 4K applications.

- 2-lane and 4-lane MIPI CSI-2 (up to 1782 Mbps/lane)
- 10-bit and 12-bit RAW output
- 3856×2180 @ up to 60 fps, 4-lane 12-bit
- ClearHDR mode for high-dynamic-range capture
- Mono variant support
- Three sync modes for multi-camera setups

## Setup

Install required tools:

```bash
sudo apt install -y git
sudo apt install -y --no-install-recommends dkms
```

Clone this repository:

```bash
cd ~
git clone https://github.com/Kurokesu/imx585-rpi-driver.git
cd imx585-rpi-driver/
```

Run setup script:

```bash
sudo ./setup.sh
```

Edit boot configuration:

```bash
sudo nano /boot/firmware/config.txt
```

Make two changes:

1. Find `camera_auto_detect` near the top and set it to `0`:

```ini
camera_auto_detect=0
```

2. Add `dtoverlay=imx585` under the `[all]` section at the bottom of the file:

```ini
[all]
dtoverlay=imx585
```

Save and exit. Reboot for changes to take effect.

> [!IMPORTANT]
> Stock `libcamera` does not support IMX585. You must build a patched version for camera to function. See [Build libcamera](#build-libcamera) below.

## dtoverlay options

`imx585` overlay supports comma-separated options to override defaults:

| option | description | default |
|--------|-------------|---------|
| [`cam0`](#cam0) | Use cam0 port instead of cam1 | cam1 |
| [`2lane`](#2lane) | Use 2-lane MIPI CSI-2 | 4 lanes |
| [`mono`](#mono) | Enable monochrome sensor variant | off |
| [`always-on`](#always-on) | Keep regulator powered (prevents runtime PM power-off) | off |
| [`link-frequency=<Hz>`](#link-frequency) | Set MIPI CSI-2 link frequency (Hz) | 720000000 |
| [`sync-mode=<mode>`](#sync-modes) | Multi-camera synchronization mode | internal-leader |

### cam0

If camera is connected to cam0 port, append `,cam0`:

```ini
dtoverlay=imx585,cam0
```

### 2lane

To use 2-lane MIPI CSI-2 instead of the default 4-lane, append `,2lane`:

```ini
dtoverlay=imx585,2lane
```

> [!NOTE]
> Maximum framerate is roughly halved on 2-lane compared to 4-lane at the same link frequency. See the [link-frequency](#link-frequency) table for exact values.

### mono

For the monochrome sensor variant, append `,mono`:

```ini
dtoverlay=imx585,mono
```

### always-on

`always-on` keeps camera regulator permanently enabled, preventing kernel from powering off the sensor during runtime PM suspend. Useful for debugging hardware issues, since it forces `CAM_GPIO` high constantly.

```ini
dtoverlay=imx585,always-on
```

### link-frequency

Default link frequency is 720 MHz (1440 Mbps/lane). Other values trade MIPI bandwidth against maximum framerate.

To change link frequency, append `,link-frequency=<Hz>`:

```ini
dtoverlay=imx585,link-frequency=891000000
```

| Frequency (Hz) | Mbps/lane | Max FPS @ 4K 12-bit 4-lane | Max FPS @ 4K 12-bit 2-lane |
|---|---|---|---|
| 297000000 | 594 | 20.8 fps | 10.4 fps |
| 360000000 | 720 | 25.0 fps | 12.5 fps |
| 445500000 | 891 | 30.0 fps | 15.0 fps |
| 594000000 | 1188 | 41.7 fps | 20.8 fps |
| 720000000 (default) | 1440 | 50.0 fps | 25.0 fps |
| 891000000 | 1782 | 60.0 fps | 30.0 fps |
| 1039500000 | 2079 | 75.0 fps | 37.5 fps |

> [!NOTE]
> RPi5/RP1 has a 400 Mpix/s processing limit. Without overclocking RP1 (the Camera Frontend), effective framerate is capped at ~43.8 fps @ 4K regardless of the link frequency configured here.

> [!NOTE]
> ClearHDR halves the framerate. 1080p 2×2 binned mode doubles it.

> [!WARNING]
> The driver also accepts 1188 MHz (2376 Mbps/lane), but RPi4 does not support this rate and RPi5 exhibits frame drops. Not recommended for production use.

### Sync modes

The driver exposes three sync modes for multi-camera setups, selectable via the `sync-mode` overlay option:

| Mode | Description |
|---|---|
| `internal-leader` (default) | Sensor runs from its own internal clock and outputs both `XVS` (vertical sync) and `XHS` (horizontal sync). Other cameras can lock onto these signals. |
| `internal-follower` | Sensor still uses its own clock, but takes in an external `XVS` signal. It aligns vertical sync to this input by adding or subtracting a horizontal sync pulse. |
| `external` | Sensor clock and timing are fully driven by external `XVS` and `XHS` signals. Both syncs are inputs, no outputs are generated. |

```ini
dtoverlay=imx585,sync-mode=internal-follower
```

See the [IMX585 Camera Clock Synchronization Guide](https://github.com/will127534/StarlightEye/wiki/IMX585-Camera-Clock-Synchronization-Guide) for hardware wiring and timing details.

> [!TIP]
> Options can be combined. Example (mono, always-on, cam0, 297 MHz link):
>
> ```ini
> dtoverlay=imx585,mono,always-on,cam0,link-frequency=297000000
> ```

## Build libcamera

Main `libcamera` repository does not support IMX585. A fork with necessary modifications is available.

On Raspberry Pi, `libcamera` and `rpicam-apps` must be rebuilt together. Detailed instructions are available [here](https://www.raspberrypi.com/documentation/computers/camera_software.html#advanced-rpicam-apps), but for convenience, here is a shorter version.

Remove pre-installed `rpicam-apps`:

```bash
sudo apt remove --purge rpicam-apps
```

### libcamera

Install dependencies:

```bash
sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
sudo apt install -y meson cmake
sudo apt install -y python3-yaml python3-ply
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
```

Clone Kurokesu's `libcamera` fork with IMX585 support:

```bash
cd ~
git clone https://github.com/Kurokesu/libcamera.git --branch imx585
cd libcamera/
```

Configure with `meson`:

```bash
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=enabled -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
```

Build:

```bash
ninja -C build
```

Install:

```bash
sudo ninja -C build install
```

> [!TIP]
> On devices with 1 GB of memory or less, build may exceed available memory. Append `-j 1` to limit to a single process.

> [!WARNING]
> `libcamera` does not yet have a stable binary interface. Always build `rpicam-apps` after building `libcamera`.

### rpicam-apps

Install dependencies:

```bash
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install -y libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev
sudo apt install -y libepoxy-dev libpng-dev
```

Clone Raspberry Pi's `rpicam-apps` repository:

```bash
cd ~
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
```

Configure with `meson` (libav enabled by default):

```bash
meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
```

> [!IMPORTANT]
> On Raspberry Pi OS **Bookworm**, packaged `libav*` is **too old** for `rpicam-apps` newer than v1.9.0.

<details>
<summary>Bookworm libav workaround</summary>

Bookworm ships `libavcodec` **59.x** while newer `rpicam-apps` expects **libavcodec >= 60**, causing build errors like "libavcodec API version is too old" (see [Raspberry Pi forum thread](https://forums.raspberrypi.com/viewtopic.php?t=392649)).

- **Keep libav** by checking out `rpicam-apps` **v1.9.0** before running `meson setup`:
  ```bash
  git checkout v1.9.0
  ```
- **Disable libav** if building `rpicam-apps` > v1.9.0:
  ```bash
  meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
  ```

</details>

Build:

```bash
meson compile -C build
```

Install:

```bash
sudo meson install -C build
```

> [!TIP]
> This should automatically update `ldconfig` cache. If you have trouble accessing your new build, update manually:
>
> ```bash
> sudo ldconfig
> ```

### Verify rpicam-apps build

Verify `rpicam-apps` was rebuilt correctly:

```bash
rpicam-hello --version
```

Expected output (build date will differ):

```
rpicam-apps build: v1.12.0 ea1bbcbea049 14-05-2026 (08:13:45)
rpicam-apps capabilites: egl:1 qt:1 drm:1 libav:1
libcamera build: v0.7.1+rpt20260429+1-ebac948d
```

### Verify that `imx585` is detected

Do not forget to reboot!

```bash
sudo reboot
```

List available cameras:

```bash
rpicam-hello --list-cameras
```

Expected output (varies by link frequency and lane configuration):

```
Available cameras
-----------------
0 : imx585 [3840x2160 12-bit RGGB] (/base/axi/pcie@1000120000/rp1/i2c@80000/imx585@1a)
    Modes: 'SRGGB12_CSI2P' : 1928x1090 [50.00 fps - (0, 0)/3840x2160 crop]
                             3856x2180 [43.80 fps - (0, 0)/3840x2160 crop]
```

## Special thanks

- [Will Whang](https://github.com/will127534) for the original IMX585 driver ([imx585-v4l2-driver](https://github.com/will127534/imx585-v4l2-driver)).
- Soho Enterprises for additional register information (passed through from Will's repo).
