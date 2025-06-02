# scripts
Scripts for creating header files with git commit and tag version information
and to create binaries for stm32 sensor firmware updates via an uSD card.

## Version information
- create-git-info-header.py inserts the git commit and tag version information
into a c header for the compile process and into the toml files which are used
to create firmware images.

## Creating binaries for uSD card updates
The script pack.py creates the binary files which can be used via uSD card to
update the sensors stm32 firmware. Pass the optional parametern LEGACY to create
a firmware binary which can be used to update older sensor firmware versions.
up to 0.4.0.

- 1st build the stm32 firmware using the cubeide. Ensure that the *.elf binary
in the Release directory is created.

- invoke python3 scripts/pack.py    or    python3 scripts/pack.py LEGACY to
create an firmware image.

