#!/bin/bash
# Fetch FatFS library from elm-chan.org
#
# Usage: ./fetch.sh
#
# Downloads FatFS R0.15 and extracts the core files.

set -e

FATFS_URL="http://elm-chan.org/fsw/ff/arc/ff15.zip"
FATFS_ZIP="ff15.zip"

cd "$(dirname "$0")"

echo "Downloading FatFS..."
curl -LO "$FATFS_URL"

echo "Extracting..."
unzip -o "$FATFS_ZIP"

echo "Copying core files..."
cp source/ff.c .
cp source/ff.h .

echo "Cleaning up..."
rm -rf "$FATFS_ZIP" source/ documents/

echo "Done. FatFS files:"
ls -la ff.c ff.h
