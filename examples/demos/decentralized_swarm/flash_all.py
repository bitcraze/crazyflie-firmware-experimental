#!/usr/bin/env python3
"""
Mass flashing script for Crazyflie swarm.

This script reads drone configurations from drones_config.yaml and flashes
selected drones with the appropriate firmware (pilot or sniffer app).
"""

import argparse
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import yaml
from colorama import Fore, Style, init
from tqdm import tqdm

# Initialize colorama for colored terminal output
init(autoreset=True)

# Add cflib path if not already in system path
try:
    import cflib.crtp
    from cflib.bootloader import Bootloader
except ImportError:
    print(f"{Fore.RED}Error: cflib not found. Please install it:")
    print(f"{Fore.YELLOW}  pip install cflib")
    sys.exit(1)


@dataclass
class DroneConfig:
    """Configuration for a single drone."""
    id: int
    uri: str
    platform: str
    app_type: str


@dataclass
class PlatformConfig:
    """Build configuration for a platform."""
    defconfig: str
    make_flags: List[str]
    app_config: str


class SwarmFlasher:
    """Manages building and flashing firmware to Crazyflie swarm."""

    def __init__(self, config_file: str, firmware_base: Optional[str] = None):
        """Initialize the swarm flasher.

        Args:
            config_file: Path to the drones_config.yaml file
            firmware_base: Override firmware base directory from config
        """
        self.config_file = config_file
        self.config = self._load_config()
        self.firmware_base = Path(firmware_base) if firmware_base else Path(self.config.get('firmware_base', '../../../'))
        self.build_jobs = self.config.get('build_jobs', 8)
        self.platforms = self._parse_platforms()
        self.drones = self._parse_drones()
        self.built_firmware = {}  # Cache: (platform, app_type) -> firmware_path

    def _load_config(self) -> dict:
        """Load the YAML configuration file."""
        try:
            with open(self.config_file, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            print(f"{Fore.RED}Error: Configuration file not found: {self.config_file}")
            sys.exit(1)
        except yaml.YAMLError as e:
            print(f"{Fore.RED}Error parsing YAML config: {e}")
            sys.exit(1)

    def _parse_platforms(self) -> dict:
        """Parse platform configurations."""
        platforms = {}
        for name, config in self.config.get('platforms', {}).items():
            platforms[name] = PlatformConfig(
                defconfig=config['defconfig'],
                make_flags=config.get('make_flags', []),
                app_config=config.get('app_config', '')
            )
        return platforms

    def _parse_drones(self) -> List[DroneConfig]:
        """Parse drone configurations."""
        drones = []
        for drone in self.config.get('drones', []):
            drones.append(DroneConfig(
                id=drone['id'],
                uri=drone['uri'],
                platform=drone['platform'],
                app_type=drone['app_type']
            ))
        return sorted(drones, key=lambda d: d.id)

    def get_drones(self, ids: Optional[List[int]] = None,
                   id_range: Optional[Tuple[int, int]] = None,
                   platform: Optional[str] = None,
                   app_type: Optional[str] = None) -> List[DroneConfig]:
        """Filter drones based on selection criteria.

        Args:
            ids: List of specific drone IDs to include
            id_range: Tuple of (start, end) IDs (inclusive)
            platform: Filter by platform (cf2, cf21bl)
            app_type: Filter by app type (pilot, sniffer)

        Returns:
            List of matching DroneConfig objects
        """
        selected = self.drones

        if ids is not None:
            selected = [d for d in selected if d.id in ids]

        if id_range is not None:
            start, end = id_range
            selected = [d for d in selected if start <= d.id <= end]

        if platform is not None:
            selected = [d for d in selected if d.platform == platform]

        if app_type is not None:
            selected = [d for d in selected if d.app_type == app_type]

        return selected

    def build_firmware(self, platform: str, app_type: str, extra_flags: Optional[List[str]] = None) -> str:
        """Build firmware for a specific platform and app type.

        Args:
            platform: Platform identifier (cf2, cf21bl)
            app_type: App type (pilot, sniffer)
            extra_flags: Additional make flags

        Returns:
            Path to the built firmware binary

        Raises:
            RuntimeError: If build fails
        """
        cache_key = (platform, app_type)
        if cache_key in self.built_firmware:
            print(f"{Fore.GREEN}Using cached firmware for {platform} ({app_type})")
            return self.built_firmware[cache_key]

        print(f"{Fore.YELLOW}Building firmware for {Fore.RED}{platform.upper()}{Fore.YELLOW} with {Fore.CYAN}{app_type.upper()}{Fore.YELLOW} app...")

        platform_config = self.platforms[platform]
        firmware_dir = self.firmware_base.resolve()

        # Step 1: Run defconfig
        print(f"{Fore.CYAN}  → Running {platform_config.defconfig}...")
        try:
            subprocess.run(
                ['make', platform_config.defconfig],
                cwd=firmware_dir,
                check=True,
                capture_output=True,
                text=True
            )
        except subprocess.CalledProcessError as e:
            print(f"{Fore.RED}Error running defconfig: {e.stderr}")
            raise RuntimeError(f"Failed to run defconfig for {platform}")

        # Step 2: Build firmware with appropriate flags
        make_flags = platform_config.make_flags.copy()

        # Add app type flag
        if app_type == 'pilot':
            make_flags.append('CFLAGS=-DBUILD_PILOT_APP')
        elif app_type == 'sniffer':
            make_flags.append('CFLAGS=-DBUILD_SNIFFER_APP')

        if extra_flags:
            make_flags.extend(extra_flags)

        print(f"{Fore.CYAN}  → Building with flags: {' '.join(make_flags)}")

        build_cmd = ['make', f'-j{self.build_jobs}'] + make_flags
        try:
            result = subprocess.run(
                build_cmd,
                cwd=firmware_dir,
                check=True,
                capture_output=True,
                text=True
            )
            print(f"{Fore.GREEN}  ✓ Build successful!")
        except subprocess.CalledProcessError as e:
            print(f"{Fore.RED}Error building firmware:")
            print(f"{Fore.RED}{e.stderr}")
            raise RuntimeError(f"Failed to build firmware for {platform} ({app_type})")

        # The firmware binary is built in the current directory's build folder
        # Binary name matches the platform (cf2.bin for cf2, cf21bl.bin for cf21bl)
        binary_name = f'{platform}.bin'
        build_binary_path = Path.cwd() / 'build' / binary_name
        if not build_binary_path.exists():
            raise RuntimeError(f"Firmware binary not found at {build_binary_path}")

        # Copy the binary to a unique name including app_type to avoid overwrites
        # when building multiple variants of the same platform
        import shutil
        unique_binary_name = f'{platform}_{app_type}.bin'
        unique_binary_path = Path.cwd() / 'build' / unique_binary_name
        shutil.copy2(build_binary_path, unique_binary_path)
        print(f"{Fore.CYAN}  → Saved as {unique_binary_name}")

        self.built_firmware[cache_key] = str(unique_binary_path)
        return str(unique_binary_path)

    def flash_drone(self, drone: DroneConfig, firmware_path: str, retry_count: int = 2) -> bool:
        """Flash a single drone with firmware.

        Args:
            drone: DroneConfig object
            firmware_path: Path to the firmware binary
            retry_count: Number of retries on failure

        Returns:
            True if successful, False otherwise
        """
        print(f"{Fore.YELLOW}Flashing {Fore.RED}ID {drone.id:02d}{Fore.YELLOW} ({drone.platform.upper()}, {drone.app_type}) at {drone.uri}...")

        for attempt in range(retry_count + 1):
            if attempt > 0:
                print(f"{Fore.YELLOW}  → Retry attempt {attempt}/{retry_count}")
                time.sleep(2)

            try:
                # Initialize cflib
                cflib.crtp.init_drivers()

                # Create bootloader instance
                bootloader = Bootloader(drone.uri)

                # Create progress bar
                pbar = tqdm(total=100, desc=f"  {Fore.CYAN}Flashing{Style.RESET_ALL}",
                           bar_format='{desc}: {percentage:3.0f}%|{bar}| {n_fmt}/{total_fmt}',
                           ncols=80, leave=False)

                last_percent = 0

                # Define progress callback
                def progress_cb(msg: str, percent: int):
                    nonlocal last_percent
                    if percent > last_percent:
                        pbar.update(percent - last_percent)
                        last_percent = percent
                    # Update description with current phase
                    if 'Starting' in msg or 'done' in msg or 'Restarting' in msg:
                        pbar.set_description(f"  {Fore.CYAN}{msg.split('(')[0].strip()}{Style.RESET_ALL}")

                # Flash the firmware
                from cflib.bootloader import Target
                targets = [Target('cf2', 'stm32', 'fw', [], [])]

                bootloader.flash_full(
                    cf=None,
                    filename=firmware_path,
                    warm=True,
                    targets=targets,
                    progress_cb=progress_cb
                )

                pbar.close()
                print(f"{Fore.GREEN}  ✓ Successfully flashed ID {drone.id:02d}")
                return True

            except Exception as e:
                if 'pbar' in locals():
                    pbar.close()
                print(f"{Fore.RED}  ✗ Error flashing ID {drone.id:02d}: {str(e)}")
                if attempt < retry_count:
                    continue
                else:
                    print(f"{Fore.RED}  ✗ Failed after {retry_count + 1} attempts")
                    return False

        return False

    def flash_all(self, drones: List[DroneConfig], extra_flags: Optional[List[str]] = None) -> Tuple[int, int]:
        """Flash all selected drones.

        Args:
            drones: List of DroneConfig objects to flash
            extra_flags: Additional make flags for building

        Returns:
            Tuple of (successful_count, failed_count)
        """
        if not drones:
            print(f"{Fore.YELLOW}No drones selected to flash.")
            return 0, 0

        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{Fore.CYAN}Flashing {len(drones)} drone(s)")
        print(f"{Fore.CYAN}{'='*60}\n")

        # Identify unique (platform, app_type) combinations to build
        build_configs = set((d.platform, d.app_type) for d in drones)

        print(f"{Fore.YELLOW}Building {len(build_configs)} firmware variant(s)...\n")

        # Build all required firmware variants
        for platform, app_type in build_configs:
            try:
                self.build_firmware(platform, app_type, extra_flags)
                print()
            except RuntimeError as e:
                print(f"{Fore.RED}Build failed: {e}")
                return 0, len(drones)

        # Flash each drone
        print(f"{Fore.YELLOW}Starting flashing process...\n")
        successful = 0
        failed = 0

        for drone in drones:
            firmware_path = self.built_firmware[(drone.platform, drone.app_type)]
            if self.flash_drone(drone, firmware_path):
                successful += 1
            else:
                failed += 1
            print()  # Add spacing between drones

        # Print summary
        print(f"{Fore.CYAN}{'='*60}")
        print(f"{Fore.GREEN}Successfully flashed: {successful}/{len(drones)}")
        if failed > 0:
            print(f"{Fore.RED}Failed: {failed}/{len(drones)}")
        print(f"{Fore.CYAN}{'='*60}\n")

        return successful, failed


def parse_range(range_str: str) -> Tuple[int, int]:
    """Parse a range string like '1-6' into (start, end) tuple."""
    try:
        start, end = range_str.split('-')
        return int(start), int(end)
    except ValueError:
        raise argparse.ArgumentTypeError(f"Invalid range format: {range_str}. Expected format: 'start-end' (e.g., '1-6')")


def main():
    parser = argparse.ArgumentParser(
        description="Mass flash Crazyflie swarm drones with pilot or sniffer firmware.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Flash all drones
  %(prog)s --all

  # Flash specific drones by ID
  %(prog)s --ids 1 2 3

  # Flash a range of drones
  %(prog)s --range 1-6

  # Flash only sniffer drones
  %(prog)s --app-type sniffer

  # Flash only brushless drones
  %(prog)s --platform cf21bl

  # Combine filters: flash pilot drones in range 1-6
  %(prog)s --range 1-6 --app-type pilot
        """
    )

    parser.add_argument(
        '--config',
        default='config/drones_config.yaml',
        help='Path to drones configuration YAML file (default: config/drones_config.yaml)'
    )

    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        '--all',
        action='store_true',
        help='Flash all drones in the configuration'
    )
    selection.add_argument(
        '--ids',
        nargs='+',
        type=int,
        metavar='ID',
        help='Flash specific drone IDs (e.g., --ids 1 2 3)'
    )
    selection.add_argument(
        '--range',
        type=parse_range,
        metavar='START-END',
        help='Flash a range of drone IDs (e.g., --range 1-6)'
    )

    parser.add_argument(
        '--platform',
        choices=['cf2', 'cf21bl'],
        help='Filter by platform type'
    )
    parser.add_argument(
        '--app-type',
        choices=['pilot', 'sniffer'],
        help='Filter by app type'
    )
    parser.add_argument(
        '--firmware-base',
        help='Override firmware base directory from config'
    )
    parser.add_argument(
        '--make-flags',
        nargs='+',
        metavar='FLAG',
        help='Additional make flags to pass during build (e.g., --make-flags DEBUG=1)'
    )

    args = parser.parse_args()

    # Check if at least one selection method is specified
    if not (args.all or args.ids or args.range or args.platform or args.app_type):
        parser.error("Please specify which drones to flash (--all, --ids, --range, --platform, or --app-type)")

    # Initialize flasher
    flasher = SwarmFlasher(args.config, args.firmware_base)

    # Get selected drones
    if args.all:
        selected_drones = flasher.get_drones()
    else:
        selected_drones = flasher.get_drones(
            ids=args.ids,
            id_range=args.range,
            platform=args.platform,
            app_type=args.app_type
        )

    if not selected_drones:
        print(f"{Fore.YELLOW}No drones match the selection criteria.")
        sys.exit(0)

    # Display selected drones
    print(f"\n{Fore.CYAN}Selected drones:")
    for drone in selected_drones:
        print(f"  {Fore.YELLOW}ID {drone.id:02d}: {drone.platform} ({drone.app_type}) - {drone.uri}")

    # Confirm before flashing
    try:
        response = input(f"\n{Fore.YELLOW}Proceed with flashing? [y/N]: {Style.RESET_ALL}")
        if response.lower() not in ['y', 'yes']:
            print(f"{Fore.YELLOW}Flashing cancelled.")
            sys.exit(0)
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Flashing cancelled.")
        sys.exit(0)

    # Flash the drones
    successful, failed = flasher.flash_all(selected_drones, args.make_flags)

    # Exit with appropriate code
    sys.exit(0 if failed == 0 else 1)


if __name__ == '__main__':
    main()
