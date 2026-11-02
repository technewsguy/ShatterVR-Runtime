# ShatterVR Runtime

An unofficial PC runtime that plays the **Smash Hit** Gear VR build in a PC VR headset through OpenXR, or flat in a
window with a mouse.

**This download contains no game.** No levels, music, art or other game files are included. You need your own copy of
the Gear VR APK, and this project can't provide one. Please don't post or link the APK, or ask for it, in issues.

ShatterVR Runtime is an independent project. It is not affiliated with, endorsed by or supported by Mediocre, who made
Smash Hit. Smash Hit is their game and their name; it is used here only to say what this software works with.

## Play

You need:
- Windows 10 or 11, 64-bit, and a graphics card with OpenGL support
- [Microsoft Visual C++ Redistributable (x64)](https://aka.ms/vs/17/release/vc_redist.x64.exe), if it isn't installed
- For VR: an OpenXR runtime, such as SteamVR, set as the active runtime
- Your own Smash Hit Gear VR APK

Steps:
1. Download the zip from [Releases](../../releases) and extract it to a folder.
2. Copy your APK into that folder and rename it `game.apk`.
3. Start your VR software, then double-click `Play VR.bat`. For a flat window, use `Play Flat.bat`.

## Controls

| Action | VR controller | Mouse and keyboard |
|---|---|---|
| Throw a ball, click menus | Trigger | Left click |
| Rapid fire, during the green power-up | Hold the trigger | Hold left click |
| Back, pause | B or menu button | Esc |
| Recentre the view | Thumbstick or trackpad click, or C on the keyboard | Not needed |

To aim with your head as the original did, add `--gaze` to the command in `Play VR.bat`.

## What has been tested

Played on one PC with SteamVR and Valve Index controllers. Other headsets and controllers have bindings but have not
been tried, and not every level has been played. Water, used only by a room that normal play can't reach, is not
implemented. Reports from other setups are welcome: say which headset, controllers and VR runtime you used.

Your progress is saved in `%APPDATA%\ShatterVR\ShatterVR`.

## Support the project

Donations are entirely optional: [Ko-fi](https://ko-fi.com/technewsguy). They support the time spent on this
independent project. They don't buy the game, any game content or any extra features, and everything here stays free.

## Build from source

Needs VS 2022 Build Tools (C++ workload), CMake 3.25+ and vcpkg:

```powershell
$env:VCPKG_ROOT = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\vcpkg"
cmake --preset default
cmake --build --preset release
ctest --preset release
```

## Licence

The code in this repository is under the [MIT licence](LICENSE). That covers this code only and gives no rights to
Smash Hit or its content. Third-party libraries keep their own licences: see
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
