# FatFS Library

This directory contains the FatFS embedded FAT filesystem module by ChaN.

## Obtaining FatFS

Download FatFS R0.15 from: http://elm-chan.org/fsw/ff/arc/ff15.zip

Extract and copy these files to this directory:
- `ff.c` - Core module
- `ff.h` - Header file

The following files are project-specific and already provided:
- `ffconf.h` - Configuration (customized for Hive)
- `diskio.h` - Disk I/O interface header
- `diskio.c` - Disk I/O implementation (SPI SD)

## Quick Setup

```bash
cd lib/fatfs
curl -LO http://elm-chan.org/fsw/ff/arc/ff15.zip
unzip ff15.zip
cp source/ff.c source/ff.h .
rm -rf ff15.zip source/ documents/
```

## Configuration

Key settings in `ffconf.h`:
- Read-only: No (we write telemetry)
- Code page: 437 (US ASCII)
- Long file names: Disabled (saves ~4KB flash)
- Multiple volumes: No
- Relative paths: Disabled

## License

FatFS is free software licensed under FatFS license (BSD-style).
See http://elm-chan.org/fsw/ff/doc/appnote.html#license

## References

- FatFS homepage: http://elm-chan.org/fsw/ff/00index_e.html
- Application note: http://elm-chan.org/fsw/ff/doc/appnote.html
