cmake_minimum_required(VERSION 3.25)
project(shattervr LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# One self-contained ShatterVR.exe: the static C++ runtime and static libraries (the x64-windows-static triplet).
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

if(MSVC)
  add_compile_options(/W4 /permissive- /utf-8)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
endif()

find_package(miniz CONFIG REQUIRED)
find_package(pugixml CONFIG REQUIRED)
find_package(SDL3 CONFIG REQUIRED)
find_package(OpenXR CONFIG REQUIRED)
find_path(STB_INCLUDE_DIR stb_image.h REQUIRED)

# Lua 5.1.4, the exact version the game embeds (stock, double numbers). Fetched by hash at configure time
# from lua.org, so no third-party source is kept in this repo. lua.c / luac.c / print.c are the tools.
include(FetchContent)
FetchContent_Declare(lua51
  URL https://www.lua.org/ftp/lua-5.1.4.tar.gz
  URL_HASH SHA256=b038e225eaf2a5b57c9bcc35cd13aa8c6c8288ef493d52970c9545074098af3a)
FetchContent_MakeAvailable(lua51)
enable_language(C)
file(GLOB LUA51_SOURCES ${lua51_SOURCE_DIR}/src/*.c)
list(REMOVE_ITEM LUA51_SOURCES ${lua51_SOURCE_DIR}/src/lua.c ${lua51_SOURCE_DIR}/src/luac.c
     ${lua51_SOURCE_DIR}/src/print.c)
add_library(lua51 STATIC ${LUA51_SOURCES})
target_include_directories(lua51 PUBLIC ${lua51_SOURCE_DIR}/src)
if(MSVC)
  target_compile_options(lua51 PRIVATE /w)
endif()

# Asset layer: APK (zip) access and decoders for the original's container formats.
add_library(sh_assets
  src/assets/apk_archive.cpp
  src/assets/asset_decode.cpp
  src/assets/asset_pack.cpp
  src/assets/segment_def.cpp
  src/assets/segment_mesh.cpp
  src/assets/stb_impl.cpp
)
target_include_directories(sh_assets PUBLIC src)
target_include_directories(sh_assets PRIVATE ${STB_INCLUDE_DIR})
target_link_libraries(sh_assets PUBLIC miniz::miniz PRIVATE pugixml::pugixml)

# Render layer: GL loader and helpers. No windowing dependency; the caller supplies GetProcAddress.
add_library(sh_render
  src/render/gl.cpp
  src/render/shader_program.cpp
)
target_include_directories(sh_render PUBLIC src)

# Physics: port of the original's td* rigid body layer.
add_library(sh_physics
  src/physics/body.cpp
  src/physics/body_joint.cpp
  src/physics/breakage.cpp
  src/physics/debris.cpp
  src/physics/explosion.cpp
  src/physics/broadphase.cpp
  src/physics/collision.cpp
  src/physics/contact.cpp
  src/physics/convex_hull.cpp
  src/physics/joint.cpp
  src/physics/mass_properties.cpp
  src/physics/pairs.cpp
  src/physics/physics_world.cpp
  src/physics/polyhedron.cpp
  src/physics/rigid_body.cpp
  src/physics/solver.cpp
)
target_include_directories(sh_physics PUBLIC src)

# Script layer: the Lua host (the original's QiScript) and the mg* bindings.
add_library(sh_script
  src/script/level_script.cpp
  src/script/material_table.cpp
  src/script/script_host.cpp
)
target_include_directories(sh_script PUBLIC src)
target_link_libraries(sh_script PUBLIC lua51 sh_physics PRIVATE pugixml::pugixml)

# Game layer: levels, rooms and the game loop.
add_library(sh_game
  src/game/display.cpp
  src/game/effects.cpp
  src/game/level.cpp
  src/game/level_def.cpp
  src/game/menu_backdrop.cpp
  src/game/obstacle.cpp
  src/game/particles.cpp
  src/game/player_records.cpp
  src/game/powerup.cpp
  src/game/property_bag.cpp
  src/game/render_level.cpp
  src/game/scoring.cpp
  src/game/sound_bank.cpp
  src/game/viewport.cpp
  src/game/room.cpp
)
target_include_directories(sh_game PUBLIC src)
target_link_libraries(sh_game PUBLIC sh_script PRIVATE pugixml::pugixml)

add_library(sh_ui
  src/ui/font.cpp
  src/ui/ui_script.cpp
)
target_include_directories(sh_ui PUBLIC src)
target_link_libraries(sh_ui PUBLIC sh_script PRIVATE pugixml::pugixml)

add_library(sh_audio
  src/audio/audio_engine.cpp
  src/audio/level_music.cpp
  src/audio/room_effects.cpp
  src/audio/pcm.cpp
  src/audio/stb_vorbis_impl.cpp
)
target_include_directories(sh_audio PUBLIC src PRIVATE ${STB_INCLUDE_DIR})
target_link_libraries(sh_audio PUBLIC SDL3::SDL3)
if(MSVC)
  set_source_files_properties(src/audio/stb_vorbis_impl.cpp PROPERTIES COMPILE_OPTIONS /w)
endif()

add_executable(sh_play tools/play/main.cpp tools/play/xr.cpp)
target_include_directories(sh_play PRIVATE tools)
target_link_libraries(sh_play PRIVATE sh_assets sh_render sh_game sh_audio sh_ui SDL3::SDL3 OpenXR::openxr_loader opengl32)
# A windowed program (no console) named ShatterVR.exe, with its icon and version details.
target_sources(sh_play PRIVATE packaging/shattervr.rc)
set_target_properties(sh_play PROPERTIES OUTPUT_NAME ShatterVR WIN32_EXECUTABLE ON)


enable_testing()
target_include_directories(sh_assets PUBLIC ${STB_INCLUDE_DIR})
foreach(pair "asset_decode;sh_assets" "segment_mesh;sh_assets" "physics_solver;sh_physics" "audio;sh_audio"
             "ui_script;sh_ui" "menu_backdrop;sh_game" "player_records;sh_game")
  list(GET pair 0 name)
  list(GET pair 1 lib)
  add_executable(test_${name} tests/test_${name}.cpp)
  target_link_libraries(test_${name} PRIVATE ${lib})
  add_test(NAME ${name} COMMAND test_${name})
endforeach()
