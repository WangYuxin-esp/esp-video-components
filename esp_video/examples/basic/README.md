| Supported Targets | ESP32-P4 | ESP32-S3 | ESP32-C3 | ESP32-C6 | ESP32-C61 | ESP32-C5 |
| ----------------- | -------- | -------- | -------- | -------- | --------- | -------- |

# Basic Capture Example

(See the [README.md](../README.md) file in the upper level [examples](../) directory for more information about examples.)

This example demonstrates a minimal V4L2 capture workflow:

1. Open the camera video device
2. Enumerate supported formats with `VIDIOC_ENUM_FMT` and print the information
3. Set the first supported format with `VIDIOC_S_FMT`
4. Request buffers, start streaming, and dequeue one frame every second with `VIDIOC_DQBUF`

## Example Output

```
I (1458) main_task: Calling app_main()
I (1458) example_init_video: MIPI-CSI camera sensor I2C port=0, scl_pin=8, sda_pin=7, freq=100000
I (1468) sc2336: Detected Camera sensor PID=0xcb3a
I (1538) basic: Enumerating supported formats:
I (1538) basic:   [0] pixelformat=0x31384142 (BA81), description=RAW8 BGGR, flags=0x0
I (1538) basic:        size[0]: 1920x1080
I (1538) basic:   [1] pixelformat=0x50424752 (RGBP), description=RGB 5-6-5 LE, flags=0x0
I (1548) basic:        size[0]: 1920x1080
I (1548) basic:   [2] pixelformat=0x33424752 (RGB3), description=RGB 8-8-8, flags=0x0
I (1558) basic:        size[0]: 1920x1080
I (1558) basic:   [3] pixelformat=0x32315559 (YU12), description=YUV 4:2:0, flags=0x0
I (1568) basic:        size[0]: 1920x1080
I (1568) basic:   [4] pixelformat=0x59565955 (UYVY), description=YUV 4:2:2 UYVY, flags=0x0
I (1578) basic:        size[0]: 1920x1080
I (1588) basic:   [5] pixelformat=0x30314742 (BG10), description=RAW10 BGGR, flags=0x0
I (1588) basic:        size[0]: 1920x1080
I (1598) basic:   [6] pixelformat=0x59555956 (VYUY), description=YUV 4:2:2 VYUY, flags=0x0
I (1608) basic:        size[0]: 1920x1080
I (1608) basic:   [7] pixelformat=0x55595659 (YVYU), description=YUV 4:2:2 YVYU, flags=0x0
I (1618) basic:        size[0]: 1920x1080
I (1618) basic: Set format: BA81, 1920x1080
I (2628) basic: frame done
I (2628) basic: frame head (16 bytes): 10 11 10 11 10 11 10 11 10 11 10 11 10 11 10 11 
I (3628) basic: frame done
I (3628) basic: frame head (16 bytes): 11 11 11 11 11 11 10 10 11 11 10 11 10 11 10 11 
I (4628) basic: frame done
I (4628) basic: frame head (16 bytes): 10 11 10 10 10 11 10 11 11 10 11 10 11 11 10 11 
I (5628) basic: frame done
I (5628) basic: frame head (16 bytes): 11 11 10 11 11 11 10 11 10 11 11 11 10 11 11 11 
I (6628) basic: frame done
...
```

## Build and Flash

```
idf.py -p PORT flash monitor
```
