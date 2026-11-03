# Publish scan

`publish_check.py` scans every file in this project tree. It checks for thesis source files, private home paths, contact details, IP addresses, secret-like strings, internal workflow terms and identifiers, excluded camera captures, raster images, and compressed numeric dumps.

Result: passed after scanning the project files. The only media file is the selected MP4 simulation clip. Reference data are CSV text files. CAD meshes were omitted because a redistribution license was not verified.

Run with `python publish_check.py` before publishing.
