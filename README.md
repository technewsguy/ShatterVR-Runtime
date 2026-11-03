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
- For VR: an OpenXR runtime, such as SteamVR, set as the active runtime
- Your own Smash Hit Gear VR APK

Steps:
1. Download the zip from [Releases](../../releases) and extract it to a folder.
2. Copy your APK into that folder and rename it `game.apk`.
3. Start your VR software, then double-click `ShatterVR.exe`. It starts in your headset, or in a window if no VR
   runtime is ready.

Windows may show "Windows protected your PC" because the program isn't code-signed. To run it, choose **More info**,
then **Run anyway**.

To always play in a window, run `ShatterVR.exe game.apk` from a terminal in that folder.

## Controls

| Action | VR controller | Mouse and keyboard |
|---|---|---|
| Throw a ball, click menus | Trigger | Left click |
| Rapid fire, during the green power-up | Hold the trigger | Hold left click |
| Back, pause | B or menu button | Esc |
| Recentre the view | Thumbstick or trackpad click, or C on the keyboard | Not needed |
| Switch between hand and head aiming | A or X (grip on Vive and Windows MR), or G on the keyboard | Not needed |

Hand aiming is the default. Head aiming works as the original did: you aim and pick menu items by looking, and either
trigger throws. Your choice is remembered.

## Game modes

Choose a mode with the button at the bottom left of the menu. This build's data turned that button off; ShatterVR
turns it back on without changing your APK.

- **Classic** and **Training** play the campaign normally.
- **Mayhem** plays the campaign with its harder settings. Its menu card says "boss challenge", but this build's data
  has no boss rooms in any level, so there are no bosses.
- **Zen** plays without using up balls. Its own levels aren't in this build's data, so Zen plays the campaign's rooms
  instead. If your APK does include them, ShatterVR uses them.

## What has been tested

Played on one PC with SteamVR and Valve Index controllers. Other headsets and controllers have bindings but have not
been tried, and not every level has been played. Water, used only by a room that normal play can't reach, is not
implemented, and the Zen, Training and Mayhem menus have no music of their own in this data. Reports from other setups are welcome: say which headset, controllers and VR runtime you used.

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
