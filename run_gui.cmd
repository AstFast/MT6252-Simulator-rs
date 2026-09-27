@echo off
REM Interactive run of the MT6252 firmware simulator (release build).
REM
REM MT6252_PROBES=none is NOT cosmetic: with the profile's probes installed the guest drops to
REM ~5 slices/s and the standby screen takes ~25 minutes to appear. With no probes, eight
REM measured runs on 2026-09-27 (release, 15000-28000 slices each, MT6252_FB_PNG writing one
REM PNG per frame) landed at **50-124 slices/s, median ~90** -- so standby (around slice 11000)
REM takes roughly 90-220 seconds depending on host load. Treat any single "slices/s" figure as
REM load-dependent, not a property of the emulator; the previous "105-165 / 60-90 s" line here
REM was measured on a quieter machine state and understated the spread.
REM Behavioral patches (set_reg / poke / read_override) are kept either way -- only the
REM count/log probes are filtered out.
REM
REM The standby clock advances with the emulated phone's own time. Calibration note measured
REM 2026-09-27 from the countdown app: **one firmware second = 278.6 slices** (two independent
REM samples: 1675 slices / 6 s and 1393 slices / 5 s), so at the 50-124 slices/s above the UI
REM runs at roughly 0.18-0.45x real time -- "about a quarter speed" is the median, not a
REM constant. (The nominal EMU_SLICE_US=2000 scale would say 500 slices/s; that gap is tracked
REM as a known deviation, see README "已知偏差" / task #44.) Set MT6252_RTC=host to seed the
REM clock from the host wall clock instead (default is a fixed constant so runs stay
REM reproducible).
REM
REM Do NOT set MT6252_NO_UI (that suppresses the window) or MT6252_MAX_SLICES (self-limits).
set MT6252_PROBES=none

if exist "%~dp0target\release\mt6252-sim.exe" (
  "%~dp0target\release\mt6252-sim.exe"
) else (
  echo release build not found, run: cargo build --release --bin mt6252-sim
)
