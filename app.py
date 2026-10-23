"""Cross-platform Moku:Go supervisor entrypoint."""

import argparse
import json
from pathlib import Path
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description="Moku:Go board control supervisor")
    parser.add_argument("--profile", type=Path, help="Hardware profile JSON")
    parser.add_argument("--simulation", "--simulate", action="store_true", help="Use simulated device")
    parser.add_argument("--diagnose", action="store_true", help="Print device diagnostics as JSON and exit")
    args = parser.parse_args(argv)
    if args.diagnose:
        from laserai.config import load_local_settings, load_profile, simulation_profile
        from laserai.device import MokuDevice, SimulatedDevice

        device = None
        try:
            profile = (simulation_profile() if args.simulation and not args.profile else load_profile(
                args.profile or Path.home() / ".laserai" / "hardware.json"))
            if args.simulation:
                profile["simulation"] = True
            device = (SimulatedDevice(profile) if args.simulation else
                      MokuDevice(profile, cli_path=load_local_settings().get("cli_path")))
            info = device.connect()
            print(json.dumps({"device": info, "diagnostics": device.diagnose()}, indent=2, ensure_ascii=False, default=str))
            return 0
        except Exception as exc:
            print(json.dumps({"error": str(exc)}, ensure_ascii=False))
            return 1
        finally:
            if device is not None:
                try:
                    device.close()
                except Exception as exc:
                    print(json.dumps({"close_error": str(exc)}, ensure_ascii=False), file=sys.stderr)
    from laserai.gui import run

    return run(args.profile, args.simulation)


if __name__ == "__main__":
    raise SystemExit(main())
