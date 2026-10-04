# SARPBC-C

A from-scratch C99 + raylib recreation of SARPBC's Soccar mode. At runtime it loads the car exports from `../export_c/cars/` (or from `assets/cars/` next to the exe). No game assets are compiled into the executable.

```
powershell -ExecutionPolicy Bypass -File build.ps1
.\sarpbc.exe [octane|backfire|scarab|aftershock|renegade|zippy|marauder]   # run directly
.\run.ps1 [args]             # or in-process via runner (if Windows Smart App Control is active)
.\run.ps1 --test             # headless physics numbers
.\run.ps1 --shot             # skip the menu, auto-drive, save shot.png, exit
.\run.ps1 --menushot         # save menu.png of the main menu, exit
.\run.ps1 --carshot          # save car_0..3.png turntable views of the car, exit
```

## Menus and settings
- **Main menu:** Play, Car (left/right swaps the car live), Skin (toggle between Team default and the car's custom livery), Mode (Free play / 1v1 / 2v2 / 3v3 vs bots), Bots (Rookie / Pro / All-Star), Settings, Quit.
- **Bots:** drive with the same physics and inputs as you. They split into attacker / defender / support roles (your position counts on your team), line up shots behind the ball, front-flip into it, jump at bouncing balls, and recover to land on their wheels. Rookies don't boost or jump; All-Stars also double-jump and boost in the air. Each bot gets a different car.
- **Animation:** wheels are split out of each car mesh at load time. They roll with ground speed, the fronts steer with the real wheel angle, and each wheel follows its suspension. The boost flame has two flickering layers. Orange-team cars use the game's orange livery.
- **Pause** (Esc / P / gamepad Start): Resume, Restart match, Settings, Main menu, Quit.
- **Settings:** camera distance, camera height, field of view, boost FOV kick, match length (3 / 5 / 10 min / unlimited), invert air pitch, fullscreen (borderless), show FPS, show controls, shadows, bloom, use original camera.
- Settings and the chosen car are saved to `settings.ini` next to the exe.
- Menu input: arrow keys / WASD / stick / d-pad to move, Enter / Space / A to select, left-right (or left/right click) to change a value, Esc / B to go back.

## Rendering (Authentic 2008 UE3 SARPBC Look)
- The scene renders into an RGBA16F HDR target: warm golden daytime stadium sunlight (extracted from `MP_SOCCARARENA.XXX`), daytime blue sky with horizon haze, modulated shadow ambient floor (`mix(vec3(0.32, 0.30, 0.28), vec3(0.48, 0.54, 0.65), ...)`), clearcoat specular reflection, genuine `Tech_Ball_D.tga` ball texture, up to 16 point lights (goal glows, live boost pads, team-colored rocket boost flames), and two-cascade PCF shadow maps.
- Post: 6-mip bloom chain (`Bloom_Scale=0.25` from `PS3-TAGame.ini`), authentic UE3 extended Reinhard tonemapping, DisplayGamma=2.2, supersonic radial speed motion blur when boosting (instead of modern chromatic aberration), clean arcade saturation, and FXAA anti-aliasing. HUD and menus are drawn on top.
- **Shadows Off** falls back to blob shadows. **Bloom Off** skips the bloom chain, but tonemapping and FXAA still run.

## Controls
| Action | Keyboard / mouse | Gamepad |
|---|---|---|
| Throttle / reverse | W / S | RT / LT |
| Steer, pitch | A D, W S | Left stick |
| Jump / dodge (on your roof: roll back over) | Space, LMB | A |
| Boost | Left Shift, RMB | B |
| Powerslide / air roll | Left Ctrl | X |
| Air roll left / right | Q / E | LB / RB |
| Ball cam | C | Y |
| Camera closer / farther | [ / ] | |
| Reset kickoff | R | Back |
| Pause menu | Esc / P | Start |

## Where the physics numbers come from
- **From the PS3 config files** (`PS3-TAGame.ini`, `TAComponents.ini`): gravity, jump impulse/force/time, dodge impulse curves and flip time, boost force/amount/min time/speed cap, air torque and damping, air throttle, match length.
- **From the cooked packages** (class defaults in `TAGAME.XXX`, read with `export_c/tools/PkgDump.cs`):
  - Car DrawScale of 1.2. Cars render and collide at 1.2× mesh size.
  - `MaxSteerAngleCurve` (wheel angle vs speed) and `SteerSpeed` (100°/s), run through a bicycle model using each car's wheelbase.
  - Powerslide grip: `HandbrakeLatSlipFactor` 0.1 and `HandbrakeLongSlipFactor` 0.5.
  - `TireFrictionDisableTimeWhenLanding` (0.3 s): grip ramps back in after a landing.
  - `DownwardForceNormal` (sticky force).
  - `UprightTime` / `UprightLiftStrength`: used only for the recovery jump. There's no automatic self-righting; if you land on your roof or side, press jump to pop up and roll back onto your wheels.
  - The original camera (540 uu back, 60 uu up, 90° horizontal FOV). This is the "Use original camera" setting.
- **Assumed:** a car mass of 40, used to turn the config impulses into velocities.
- **Estimated as Rocket League values × 1.75** (SARPBC is built at about 1.75× RL scale): throttle curve, brake/coast, ball size and bounce, and the hit impulse. The original throttle is a PhysX drivetrain (torque, gear ratios), which doesn't map cleanly. The ball's physics material wasn't found in the packages.
- **Arena dimensions** come from the real collision meshes (CM_Ground01 + CM_Glass01). The constants at the top of `src/main.c` are only used for spawns, scoring, and the plane fallback.

## Arena (from your own extracted files)
The game loads `../export_c/arena/` (or `assets/arena/` next to the exe). If that folder is missing, it falls back to a procedural box arena. To regenerate it:
```
umodel_64.exe -export -ps3 -path=<...>\COOKEDPS3 MP_SOCCARARENA                 # -> exported_assets\MP_SOCCARARENA
umodel_64.exe -export -dds -ps3 -path=<...>\COOKEDPS3 MP_SOCCARARENA Texture2D  # raw DXT dumps -> Texture2D_dds
cd ..\export_c\tools
csc /r:System.Drawing.dll /out:ArenaTool.exe ArenaExport.cs
ArenaTool.exe export <MP_SOCCARARENA>\StaticMesh3 <MP_SOCCARARENA>\Texture2D ..\arena <MP_SOCCARARENA>\Texture2D_dds
```
If Windows Application Control blocks the freshly compiled exe, you can run the exporter in-process from PowerShell instead:
```
Add-Type -Path ArenaExport.cs -ReferencedAssemblies System.Drawing
[Program]::Main(@("export","<MP_SOCCARARENA>\StaticMesh3","<MP_SOCCARARENA>\Texture2D","..\arena","<MP_SOCCARARENA>\Texture2D_dds"))
```
- SkyTrim01 (the beam ring under the glass ceiling edge) has no exported placement, so the exporter lifts it to just under the glass. FactoryWalls01's level-wide floor is sunk 6 cm to stop it z-fighting with the pavement.
- Brick01 (DXT1) is decoded from the DDS dump, because UModel's TGA output for PS3 DXT1 is byte-swapped.
- SideWalk01 (PF_G8) still comes out scrambled, so it's drawn as flat concrete. GrayTiles, StreetLine, and MetalGarage only have normal maps, so they're flat colours.
- Not placed yet: rafters, buildings, statue, train, and the sky dome. These need actor positions from the level.

## Custom Skins & Skin Generator
Each car model has a custom skin built from scratch using the vehicle's UV mask channels (red = C1, green = C2, blue = C3, black = bare chassis metal):
- **Octane**: *Synthwave* (electric cyan body, micro-woven carbon fiber panels, hot neon magenta wing/decals).
- **Backfire**: *Hellfire* (gloss obsidian black, ember crimson trim, blazing flame gradient).
- **Scarab**: *Gold Rush* (radiant polished gold with metallic lustre, pearl ivory accents, royal velvet onyx).
- **Aftershock**: *Stealth Jet* (radar dark navy with tactical hex panels, titanium grey trim, flight neon amber).
- **Renegade**: *Desert Camo* (multi-tone digital camouflage body, tactical black trim, stencil white decals).
- **Zippy**: *Cyberpunk* (midnight violet body, laser turquoise speed claw stripes, high-tech white hood).
- **Marauder**: *Urban Hazard* (battleship slate grey body, gunmetal trim, 45-degree yellow & black hazard stripes).

The generator tool lives in [`export_c/tools/SkinGen.cs`](file:///e:/Sarpbc/export_c/tools/SkinGen.cs). To rebuild and bake skins:
```
cd ..\export_c\tools
C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe /r:System.Drawing.dll /out:SkinGen.exe SkinGen.cs
.\SkinGen.exe
```
This writes uncompressed 32bpp BGRA `body_custom.tga` and a visual `skin_preview.png` directly into each car's asset directory (`export_c/cars/<name>/`). In-game, toggle the **SKIN** option in the main menu to switch between Team default and your car's custom livery.
