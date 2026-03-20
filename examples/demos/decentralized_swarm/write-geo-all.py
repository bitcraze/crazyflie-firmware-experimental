#!/usr/bin/env python3
"""
Write lighthouse geometry to a Crazyflie swarm.

Reads drone configurations from drones_config.yaml and writes a lighthouse
YAML file to each selected drone in sequence. Optionally verifies that all
drones have identical lighthouse configurations afterwards.
"""

import argparse
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml
from colorama import Fore, Style, init

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.localization import LighthouseConfigWriter
except ImportError as e:
    print(f"Error: cflib not found ({e}). Please install it:")
    print("  pip install cflib")
    sys.exit(1)

from cflib.crazyflie.mem.lighthouse_memory import LighthouseMemHelper

init(autoreset=True)


@dataclass
class DroneConfig:
    id: int
    uri: str
    platform: str
    app_type: str


def load_drones(config_file: str) -> List[DroneConfig]:
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"{Fore.RED}Error: Configuration file not found: {config_file}")
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"{Fore.RED}Error parsing YAML config: {e}")
        sys.exit(1)

    drones = []
    for drone in config.get('drones', []):
        drones.append(DroneConfig(
            id=drone['id'],
            uri=drone['uri'],
            platform=drone['platform'],
            app_type=drone['app_type']
        ))
    return sorted(drones, key=lambda d: d.id)


def filter_drones(
    drones: List[DroneConfig],
    ids: Optional[List[int]] = None,
    id_range: Optional[Tuple[int, int]] = None,
    app_type: Optional[str] = None,
) -> List[DroneConfig]:
    selected = drones
    if ids is not None:
        selected = [d for d in selected if d.id in ids]
    if id_range is not None:
        start, end = id_range
        selected = [d for d in selected if start <= d.id <= end]
    if app_type is not None:
        selected = [d for d in selected if d.app_type == app_type]
    return selected


def validate_lighthouse_file(file_name: str) -> bool:
    """Check that the file is a solved lighthouse configuration, not raw estimation data."""
    try:
        with open(file_name, 'r') as f:
            content = f.read()
    except OSError as e:
        print(f"{Fore.RED}Cannot read file '{file_name}': {e}")
        return False

    if 'geos:' not in content:
        print(f"{Fore.RED}Error: '{file_name}' does not look like a lighthouse configuration file.")
        print(f"{Fore.YELLOW}Expected a file with 'geos:' and 'calibs:' sections, exported from the")
        print(f"{Fore.YELLOW}Crazyflie client after running geometry estimation (not the raw data file).")
        return False

    # Detect raw estimation input files (e.g. exported from the geometry estimator tool)
    if '!Lh' in content or 'LhGeo' in content or 'LhCf' in content:
        print(f"{Fore.RED}Error: '{file_name}' is a raw geometry estimation file, not a solved configuration.")
        print(f"{Fore.YELLOW}Open the Crazyflie client, run the lighthouse geometry estimator, then use")
        print(f"{Fore.YELLOW}'Save system config' to export the correct file.")
        return False

    return True


def write_one(file_name: str, uri: str) -> bool:
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            writer = LighthouseConfigWriter(scf.cf)
            writer.write_and_store_config_from_file(None, file_name)
            time.sleep(2)
        return True
    except Exception as e:
        print(f"{Fore.RED}    Error: {e}")
        return False


def _round_vec(v, decimals=4):
    return [round(x, decimals) for x in v]


def read_geo(uri: str) -> Optional[Dict]:
    """Read lighthouse geometry from a drone.

    Returns a dict of {bs_id: {'origin': [...], 'rotation': [[...], [...], [...]]}}
    for each valid base station, or None on connection failure.
    Data is read from CF memory as 32-bit floats; values are rounded to 4 decimal
    places to make comparisons robust against float representation noise.
    """
    result = {}
    event = threading.Event()

    def geo_cb(geo_data):
        for bs_id, geo in geo_data.items():
            if geo.valid:
                result[bs_id] = {
                    'origin': _round_vec(geo.origin),
                    'rotation': [_round_vec(row) for row in geo.rotation_matrix],
                }
        event.set()

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            helper = LighthouseMemHelper(scf.cf)
            helper.read_all_geos(geo_cb)
            if not event.wait(timeout=10):
                return None
    except Exception as e:
        print(f"{Fore.RED}    Error: {e}")
        return None

    return result if result else None


def write_all(drones: List[DroneConfig], file_name: str) -> Tuple[int, int]:
    print(f"\n{Fore.CYAN}{'='*60}")
    print(f"{Fore.CYAN}Writing lighthouse geometry to {len(drones)} drone(s)")
    print(f"{Fore.CYAN}  File: {file_name}")
    print(f"{Fore.CYAN}{'='*60}\n")

    successful = 0
    failed = 0

    for drone in drones:
        print(f"{Fore.YELLOW}Writing to {Fore.RED}ID {drone.id:02d}"
              f"{Fore.YELLOW} ({drone.platform}, {drone.app_type}) at {drone.uri}...",
              end=' ', flush=True)
        if write_one(file_name, drone.uri):
            print(f"{Fore.GREEN}✓")
            successful += 1
        else:
            print(f"{Fore.RED}✗ Failed")
            failed += 1

    print(f"\n{Fore.CYAN}{'='*60}")
    print(f"{Fore.GREEN}Successfully written: {successful}/{len(drones)}")
    if failed > 0:
        print(f"{Fore.RED}Failed: {failed}/{len(drones)}")
    print(f"{Fore.CYAN}{'='*60}\n")

    return successful, failed


def verify_all(drones: List[DroneConfig]) -> bool:
    print(f"\n{Fore.CYAN}{'='*60}")
    print(f"{Fore.CYAN}Verifying lighthouse geometry on {len(drones)} drone(s)")
    print(f"{Fore.CYAN}{'='*60}\n")

    geo_data: Dict[int, Optional[Dict]] = {}

    for drone in drones:
        print(f"{Fore.YELLOW}Reading from {Fore.RED}ID {drone.id:02d}"
              f"{Fore.YELLOW} ({drone.platform}, {drone.app_type}) at {drone.uri}...",
              end=' ', flush=True)
        data = read_geo(drone.uri)
        if data is None:
            print(f"{Fore.RED}✗ Failed to read")
        else:
            print(f"{Fore.GREEN}✓ ({len(data)} valid BS)")
        geo_data[drone.id] = data

    # Pick the first successful read as reference
    ref_id, ref_data = next(
        ((d.id, geo_data[d.id]) for d in drones if geo_data[d.id] is not None),
        (None, None)
    )

    if ref_data is None:
        print(f"\n{Fore.RED}No data could be read from any drone.")
        return False

    print(f"\n{Fore.CYAN}Comparison (reference: ID {ref_id:02d}, "
          f"{len(ref_data)} base station(s)):")

    all_match = True
    for drone in drones:
        data = geo_data[drone.id]
        if data is None:
            print(f"  {Fore.RED}ID {drone.id:02d}: ✗  could not read")
            all_match = False
        elif data == ref_data:
            bs_ids = sorted(data.keys())
            print(f"  {Fore.GREEN}ID {drone.id:02d}: ✓  matches  "
                  f"(BS ids: {bs_ids})")
        else:
            print(f"  {Fore.RED}ID {drone.id:02d}: ✗  MISMATCH")
            for bs_id in sorted(set(data) | set(ref_data)):
                r = ref_data.get(bs_id)
                g = data.get(bs_id)
                if r is None:
                    print(f"           BS {bs_id}: missing in reference, got origin={g['origin']}")
                elif g is None:
                    print(f"           BS {bs_id}: missing on this drone, ref origin={r['origin']}")
                elif r != g:
                    print(f"           BS {bs_id}: origin ref={r['origin']} got={g['origin']}")
                    if r['rotation'] != g['rotation']:
                        print(f"                    rotation differs")
            all_match = False

    print(f"\n{Fore.CYAN}{'='*60}")
    if all_match:
        print(f"{Fore.GREEN}All drones have matching lighthouse configuration. ✓")
    else:
        print(f"{Fore.RED}WARNING: Not all drones have a matching lighthouse configuration!")
    print(f"{Fore.CYAN}{'='*60}\n")

    return all_match


def parse_range(range_str: str) -> Tuple[int, int]:
    try:
        start, end = range_str.split('-')
        return int(start), int(end)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"Invalid range format: {range_str}. Expected format: 'start-end' (e.g., '1-6')"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Write lighthouse geometry to a Crazyflie swarm.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Write to all pilot drones and verify
  %(prog)s lighthouse.yaml --all

  # Write to specific drone IDs and verify
  %(prog)s lighthouse.yaml --ids 1 2 3

  # Write to a range of drones and verify
  %(prog)s lighthouse.yaml --range 1-6

  # Only verify without writing
  %(prog)s --verify --all

  # Only verify specific drone IDs
  %(prog)s --verify --ids 1 2 3

  # Only verify a range of drones
  %(prog)s --verify --range 1-6

Sniffer drones are always excluded automatically.
        """,
    )

    parser.add_argument(
        'file',
        nargs='?',
        help='Lighthouse geometry YAML file to write',
    )
    parser.add_argument(
        '--config',
        default='config/drones_config.yaml',
        help='Path to drones configuration YAML (default: config/drones_config.yaml)',
    )

    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--all', action='store_true', help='Select all drones')
    selection.add_argument('--ids', nargs='+', type=int, metavar='ID',
                           help='Select specific drone IDs (e.g., --ids 1 2 3)')
    selection.add_argument('--range', type=parse_range, metavar='START-END',
                           help='Select a range of drone IDs (e.g., --range 1-6)')

    parser.add_argument('--verify', action='store_true',
                        help='Only verify, do not write')

    args = parser.parse_args()

    verify_only = args.verify
    if not verify_only and not args.file:
        parser.error("A lighthouse geometry file is required unless using --verify")

    if not (args.all or args.ids or args.range):
        parser.error("Please specify which drones to use (--all, --ids, or --range)")

    import logging
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    all_drones = load_drones(args.config)
    selected = filter_drones(
        all_drones,
        ids=args.ids,
        id_range=args.range,
        app_type='pilot',  # sniffers are always excluded
    ) if not args.all else filter_drones(all_drones, app_type='pilot')

    if not selected:
        print(f"{Fore.YELLOW}No pilot drones match the selection criteria.")
        sys.exit(0)

    print(f"\n{Fore.CYAN}Selected drones:")
    for drone in selected:
        print(f"  {Fore.YELLOW}ID {drone.id:02d}: {drone.platform} ({drone.app_type}) - {drone.uri}")

    action = "verify" if verify_only else "write and verify"
    try:
        response = input(f"\n{Fore.YELLOW}Proceed with {action}? [y/N]: {Style.RESET_ALL}")
        if response.lower() not in ['y', 'yes']:
            print(f"{Fore.YELLOW}Cancelled.")
            sys.exit(0)
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Cancelled.")
        sys.exit(0)

    if not verify_only:
        if not validate_lighthouse_file(args.file):
            sys.exit(1)
        successful, failed = write_all(selected, args.file)
        if failed > 0:
            sys.exit(1)

    ok = verify_all(selected)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
