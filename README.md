# MPC Audio Renderer with Atmos to Windows Spatial API

Audio renderer for MPC video players (fork of MPC Audio Renderer): bitstreams and decodes TrueHD Atmos and E-AC-3 Atmos, then sends them as objects through Windows Spatial Sound (for use with Atmos for Headphones, DTS:X), with ReClock-style audio sync to the display refresh.

**Install**

1. Unzip the folder somewhere permanent.
2. Right-click `Installer.bat` → Run as administrator.
3. MPC-BE: Options → Audio → Audio renderer: "MPC SpatialClock (external)".
4. MPC-BE audio decoder: TrueHD and E-AC-3 pass-through ON, DTS and AC-3 OFF. Note: For MPC-HC with LAV audio decoder, disable PCM fallback when bitstreaming is unavailable.
5. Windows: HDMI output → Spatial sound → "Dolby Atmos for home theater".

To remove it: `Uninstaller.bat`, as administrator.

Windows 10/11 x64. Source: this repository (GPL-3.0). Includes OpenJOC (Apache-2.0).
