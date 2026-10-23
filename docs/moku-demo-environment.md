# Installed Moku Demo And Packaged Host Checks

Date: 2026-10-23. No physical Moku:Go or piezo was used in these checks.

## Result

The installed official Moku application opens its Moku:Go demo, displays
simulated Phasemeter measurements and accepts a three-slot Multi-Instrument
Mode layout. This verifies access to the vendor's demonstration interface. It
does not execute this repository's compiled controller, verify analog phase
measurement, or establish 20 Hz / 500 nm / 50 nm performance.

The separately packaged LaserAI host also starts, connects its own simulator
and closes cleanly. Its simulator is a project test backend; it is not connected
to the official Moku demo and must not be described as a vendor device test.

## Installed Environment Evidence

| Check | Observed result |
| --- | --- |
| Application | `/Applications/Moku.app` |
| `CFBundleIdentifier` | `com.liquidinstruments.moku` |
| `CFBundleShortVersionString` | `4.3.0a` |
| Resource `appconfig.ini` | `ReleaseChannel=Production` |
| Bundled instrument resource build | `21047`, including `mokugo` resources |
| Bundled firmware/API-server resource directory | `647` |
| Separate `mokucli` inside application bundle | No matching file found |
| Moku process listening TCP sockets | None observed before or during the Phasemeter demo |

Firmware/API-server resource directory names are package contents, not the
version of a connected device. No real device identity, firmware or license
status was read.

Commands used for the read-only inventory:

```sh
plutil -p /Applications/Moku.app/Contents/Info.plist
cat /Applications/Moku.app/Contents/Resources/appconfig.ini
rg --files /Applications/Moku.app/Contents
ps -ax -o pid,comm,args
lsof -nP -a -p <Moku-process-id> -iTCP -sTCP:LISTEN
```

`open -a /Applications/Moku.app` brought the existing application forward.
macOS accessibility inspection and local mouse events navigated only the demo
interface. Screenshots capture the Moku window, not the whole desktop. No
firmware update, account login, external message or physical output was used.

## Official Demo Checks

1. **Device selection:** the app showed no physical devices and offered
   Moku:Lab, Moku:Pro, Moku:Go and Moku:Delta demo tiles. The visible version was
   `v4.3.0a`, matching `Info.plist`.
2. **Go instrument selection:** the Go demo listed Phasemeter, PID Controller,
   Datalogger and Multi-Instrument Mode, among other instruments.
3. **Standalone Phasemeter:** opened successfully with two simulated inputs.
   The interface showed carrier frequencies near 1 MHz, changing phase in
   cycles and amplitude in dBm, a displayed acquisition rate of 119 Sa/s and
   PLL bandwidth of 1 kHz. These are demonstration readings and UI settings.
4. **Three-slot MiM:** selected the displayed `3 slots, 62.5 MSa/s` option,
   placed a Phasemeter in slot 1 and applied the demonstration layout.
5. **MiM Phasemeter inspection:** its slot diagram exposed Input A / Output A.
   Opening the actual slot instrument showed only Input A in the channel
   panel, with no Input B enable control. Its advanced panel showed phase wrap
   and auto-reset, with no visible single-input-mode toggle.

The last observation is a capability discrepancy requiring resolution before
hardware deployment: standalone Go's two inputs must not be assumed to imply
two independent Phasemeter channels in every MiM configuration. These checks
did not execute an API request against a real Go, so they do not prove that a
second MiM channel is impossible. They also do not establish that setting a
generic API parameter will create one. Verify actual capabilities and two
independently driven phase paths before accepting the proposed three-slot
controller architecture.

The generic API documentation lists Go channels 1 and 2 for
[`enable_input`](https://apis.liquidinstruments.com/api/reference/phasemeter/enable_input.html)
and [`generate_output`](https://apis.liquidinstruments.com/api/reference/phasemeter/generate_output.html).
[`enable_single_input`](https://apis.liquidinstruments.com/api/reference/phasemeter/enable_single_input.html)
defaults to `true` and fans the first input into the Phasemeter channels. The
host therefore explicitly disables this mode when attempting independent REF
and MEAS acquisition. The documentation's generic channel list is not a
successful device capability test.

Screenshots:

- [Device menu](images/moku-demo-device-menu.png)
- [Go instrument list](images/moku-demo-go-instruments.png)
- [Standalone Go Phasemeter](images/moku-demo-go-phasemeter.png)
- [Three-slot layout](images/moku-demo-go-mim-three-slots.png)
- [Applied Phasemeter slot diagram](images/moku-demo-go-mim-phasemeter.png)
- [Opened MiM Phasemeter, Input A only](images/moku-demo-go-mim-phasemeter-channel.png)

## Demo API Limits

No API endpoint or TCP listener was observed for the running official demo.
The official vendor forum's
[demo API feature request](https://forum.liquidinstruments.com/t/instrument-demo-mode-api-access/972)
contains a June 2024 staff response describing API access as a requested
feature requiring a virtual server. This is historical evidence, not a claim
about every future release. The installed application inventory and current
documentation search did not establish a supported demo REST server or a
vendor simulator able to load this custom bitstream.

Consequently the project did not substitute `localhost` for a physical Moku,
invent a demo IP address or report SDK mock calls as successful official API
operations. Official demo visuals, project simulation, HDL simulation, cloud
synthesis and physical measurements are distinct validation levels.

## Native Packaged Host Smoke Test

The native macOS build produced `dist/LaserAI.app`. This command completed with
exit code 0:

```sh
dist/LaserAI.app/Contents/MacOS/LaserAI --simulation --diagnose
```

It returned `simulation: true`, `telemetry_kind: simulated_full_rate`, build ID
`python-fixed-reference-v1`, and no output blockers. Its diagnostics explicitly
state that offline simulation does not validate physical hardware.

The packaged GUI was opened with:

```sh
open -n dist/LaserAI.app --args --simulation
```

The window title was `LaserAI | Moku:Go`. Clicking Connect changed its status to
the simulation-connected state. The normal close button then shut down its
process; a subsequent process query found no remaining process for that PID.
The screenshot is [the packaged host](images/laserai-packaged-macos.png).
This checks packaged startup, imported backend availability, connection and
normal Qt shutdown. It does not test Windows/Linux packaging or physical I/O.

## Remaining Acceptance Work

Obtain a connected Go's actual firmware and MiM channel capabilities. Resolve
the observed single-channel Phasemeter slot before choosing the final routing.
Compile the custom source for that exact target and verify the manifest/hash,
timing and resource report. Then perform signal conditioning, phase scaling,
wrapping, latency, loss-of-lock and direct-drive qualification, followed by
low-amplitude tests and repeated PID/Preview experiments. The official GUI demo
cannot replace those measurements.
